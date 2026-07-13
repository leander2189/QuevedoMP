// collision/containment — ADR-012 parity-ray containment check, shared by both backends.
//
// Both boolean strategies (FCL BVH triangle-triangle, OptiX surface rays) are surface-intersection
// tests: a robot link entirely INSIDE a closed obstacle produces no surface crossing and reports
// free — the worst failure class (a false-free). This closes that gap on the CPU: per robot mesh
// link, test one precomputed interior point against the environment. Primitive env solids
// (box/sphere/cylinder) use an exact analytic inside-test; watertight meshes use a parity ray (odd
// intersection count => inside). Non-watertight meshes are excluded and logged once (ADR-012 #2).
//
// Applies to robot MESH links only: a primitive robot link is convex, so FCL's GJK already detects
// containment; OptiX doesn't handle primitive robot links at all.
#pragma once

#include <array>
#include <span>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/core/types.hpp"

namespace quevedomp::collision {

class EnvContainment {
public:
  EnvContainment() = default;
  explicit EnvContainment(const SceneDescription &env);

  // Any environment solid to be contained-by? (If not, inside() is always false.)
  bool any() const noexcept { return has_solids_; }

  // True if `p` (world frame) lies inside any environment solid. `skip`, if non-empty, is indexed
  // by the object's position in the SceneDescription this was built from; a nonzero entry excludes
  // that object (Task 3.3d P4: ACM-allowed robot-link × env-object pairs must not report
  // containment either).
  bool inside(const Eigen::Vector3d &p, std::span<const std::uint8_t> skip = {}) const;

private:
  struct BoxSolid {
    int object; // index into the source SceneDescription::objects
    Transform inv_pose;
    Eigen::Vector3d he;
  };
  struct SphereSolid {
    int object;
    Eigen::Vector3d c;
    double r;
  };
  struct CylSolid {
    int object;
    Transform inv_pose;
    double r;
    double half_len;
  };
  // One watertight mesh object: parity is counted PER OBJECT (a combined count would flip to
  // "outside" for a point inside two overlapping solids — and per-pair skipping needs the split
  // anyway).
  struct MeshSolid {
    int object;
    std::vector<std::array<Eigen::Vector3d, 3>> tris; // world-space
  };
  std::vector<BoxSolid> boxes_;
  std::vector<SphereSolid> spheres_;
  std::vector<CylSolid> cyls_;
  std::vector<MeshSolid> meshes_;
  bool has_solids_ = false;
};

// Centroid of a link's collision-mesh vertices (link frame) — the interior point ADR-012 casts
// from. Adequate for the roughly-convex robot links in scope; refine (push inside along a normal)
// if a concave link's centroid ever falls outside.
Eigen::Vector3d mesh_centroid(const std::vector<Eigen::Vector3d> &verts);

} // namespace quevedomp::collision
