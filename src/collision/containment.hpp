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

  // True if `p` (world frame) lies inside any environment solid.
  bool inside(const Eigen::Vector3d &p) const;

private:
  struct BoxSolid {
    Transform inv_pose;
    Eigen::Vector3d he;
  };
  struct SphereSolid {
    Eigen::Vector3d c;
    double r;
  };
  struct CylSolid {
    Transform inv_pose;
    double r;
    double half_len;
  };
  std::vector<BoxSolid> boxes_;
  std::vector<SphereSolid> spheres_;
  std::vector<CylSolid> cyls_;
  std::vector<std::array<Eigen::Vector3d, 3>> mesh_tris_; // world-space, watertight meshes only
  bool has_solids_ = false;
};

// Centroid of a link's collision-mesh vertices (link frame) — the interior point ADR-012 casts
// from. Adequate for the roughly-convex robot links in scope; refine (push inside along a normal)
// if a concave link's centroid ever falls outside.
Eigen::Vector3d mesh_centroid(const std::vector<Eigen::Vector3d> &verts);

} // namespace quevedomp::collision
