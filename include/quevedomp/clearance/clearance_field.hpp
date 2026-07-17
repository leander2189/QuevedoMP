// clearance/ClearanceField — voxel signed-distance field of the STATIC environment (roadmap R3,
// ADR-018). A separate type from CollisionScene on purpose: the scene is exact and
// boolean-authoritative; this field is approximate (voxel-resolution), gradient-bearing, and
// exists to serve optimization (clearance-aware smoothing/refinement, R4) plus visualization —
// mixing those semantics into one interface is how contracts rot. The exact backend remains the
// only collision certificate.
//
// Build (one-time, quasi-static assumption — the same one the GAS design exploits):
//   1. Environment triangles gathered in world space; primitives ride the SAME closed
//      tessellations as the OptiX backend (Task 3.3d P2 — one geometry story everywhere).
//   2. SEED: voxels near the surface get exact point-to-triangle distances (atomic 64-bit
//      min-encode, parallel over triangles).
//   3. JFA: jump-flooding propagates nearest-seed ids across the grid — O(V·log N), runs as
//      plain CUDA kernels when compiled with QUEVEDOMP_WITH_CUDA and a device is usable at
//      runtime, with a bit-identical OpenMP fallback otherwise (built_on_gpu() tells which ran).
//   4. SIGN: even-odd column parity per watertight object (the ADR-012 stance: non-watertight
//      meshes contribute UNSIGNED distance only, logged once).
//
// Queries are batched and trilinear; gradients by central differences. Outside the grid the
// distance is the clamped-border value plus the Euclidean gap to the border (approximate,
// documented — keep the margin generous enough that the robot works inside the grid).
#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp" // MeshSources
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::clearance {

struct ClearanceFieldOptions {
  double resolution = 0.01; // voxel edge (m)
  double margin = 0.20;     // grid padding beyond the environment AABB (m)
  // Hard cap on voxel count (memory guard): build() throws with a suggested resolution instead
  // of silently allocating tens of GB. 96M voxels ≈ 384 MB of float distance.
  std::size_t max_voxels = std::size_t{96} << 20;
  bool use_gpu = true; // try CUDA JFA first; silent CPU fallback when unavailable
};

class ClearanceField {
public:
  // Build the field over `env`. Throws on an empty environment or a grid over max_voxels.
  static ClearanceField build(const collision::SceneDescription &env,
                              const ClearanceFieldOptions &options = {});

  // Signed distance (m) at a world point: positive outside, negative inside watertight solids.
  [[nodiscard]] double distance(const Eigen::Vector3d &p) const;
  // Central-difference gradient of the field (≈ unit direction away from the nearest surface).
  [[nodiscard]] Eigen::Vector3d gradient(const Eigen::Vector3d &p) const;
  // Batched query — the shape R4's optimizer consumes. `gradients` may be empty to skip them.
  void query(std::span<const Eigen::Vector3d> points, std::span<double> distances,
             std::span<Eigen::Vector3d> gradients) const;

  // Grid metadata (tests, slicing, viz). data() is row-major, x fastest: index = (z*ny + y)*nx + x.
  [[nodiscard]] Eigen::Vector3d origin() const noexcept;
  [[nodiscard]] double resolution() const noexcept;
  [[nodiscard]] Eigen::Vector3i dims() const noexcept;
  [[nodiscard]] const std::vector<float> &data() const noexcept;
  [[nodiscard]] bool built_on_gpu() const noexcept;
  [[nodiscard]] double build_seconds() const noexcept;

  ClearanceField(ClearanceField &&) noexcept;
  ClearanceField &operator=(ClearanceField &&) noexcept;
  ~ClearanceField();

private:
  ClearanceField();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Conservative sphere cover of the robot's collision geometry (link frames): per geometry, N
// centers along the vertex cloud's principal axis, N grown until the cover radius (max vertex
// distance to its nearest center) drops to `target_radius` or `max_spheres_per_geometry` is hit.
// Conservative by construction — every collision vertex lies inside some sphere — so
// field(center) − radius NEVER overestimates clearance.
struct RobotSpheres {
  struct Sphere {
    int link = -1;          // index into RobotModel::links()
    Eigen::Vector3d center; // link frame
    double radius = 0.0;
  };
  std::vector<Sphere> spheres;
};

[[nodiscard]] RobotSpheres decompose_robot(const RobotModel &model,
                                           const collision::MeshSources &meshes = {},
                                           double target_radius = 0.05,
                                           int max_spheres_per_geometry = 8);

// Per-config clearance: min over spheres of (field(world center) − radius). Negative ⇒ some
// sphere (conservatively) penetrates. FK once per config; OpenMP across configs.
[[nodiscard]] std::vector<double> clearance_batch(const ClearanceField &field,
                                                  const RobotModel &model,
                                                  const RobotSpheres &spheres,
                                                  std::span<const JointPosition> configs);

} // namespace quevedomp::clearance
