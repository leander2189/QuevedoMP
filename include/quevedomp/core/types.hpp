// core/types — foundational value types (Eigen-only, zero inter-module dependencies).
// Spec §6 Phase 1 "Core types". These are deliberately small and dependency-free so every
// other module (robot/, kinematics/, collision/, planning/) can build on them without cycles.
#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace quevedomp {

// Joint-space vectors are dynamically sized (DOF known only at runtime, from the URDF).
using JointPosition = Eigen::VectorXd;
using JointVelocity = Eigen::VectorXd;

struct JointState {
  JointPosition pos;
  JointVelocity vel;
  // Joint acceleration (rad/s² / m/s²); filled by trajectory parameterization (Task 3.4).
  // Empty ⇒ not computed (older producers).
  JointVelocity acc;
};

// SE(3) rigid-body transform — a thin, explicit wrapper over Eigen::Isometry3d so the rest
// of the codebase speaks `Transform` rather than raw Eigen types (spec §6). Composition is
// `operator*`; applying it to a Vector3d transforms a point.
class Transform {
public:
  Transform() noexcept : iso_(Eigen::Isometry3d::Identity()) {}
  explicit Transform(const Eigen::Isometry3d &iso) noexcept : iso_(iso) {}

  static Transform Identity() noexcept { return Transform{}; }
  static Transform from_translation(const Eigen::Vector3d &t) noexcept;
  static Transform from_rotation(const Eigen::Quaterniond &q) noexcept;
  static Transform from_parts(const Eigen::Vector3d &t, const Eigen::Quaterniond &q) noexcept;

  // Accessors. `isometry()` exposes the underlying Eigen object for interop.
  const Eigen::Isometry3d &isometry() const noexcept { return iso_; }
  Eigen::Matrix4d matrix() const noexcept { return iso_.matrix(); }
  Eigen::Vector3d translation() const noexcept { return iso_.translation(); }
  Eigen::Matrix3d rotation() const noexcept { return iso_.rotation(); }

  Transform inverse() const noexcept;
  Transform operator*(const Transform &rhs) const noexcept;               // compose: this ∘ rhs
  Eigen::Vector3d operator*(const Eigen::Vector3d &point) const noexcept; // apply to a point

  // Absolute (Frobenius-norm) closeness of the 4×4 matrices; `tol` is an absolute bound.
  bool is_approx(const Transform &other, double tol = 1e-12) const noexcept;

private:
  Eigen::Isometry3d iso_;
};

struct Pose {
  Transform tf;
  double pos_tol = 1e-3; // metres
  double rot_tol = 1e-2; // radians
};

struct Sphere {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

struct Box {
  Transform tf;
  Eigen::Vector3d half_extents = Eigen::Vector3d::Zero();
};

// Triangle mesh. Indices are triangles (each row = 3 vertex indices into `vertices`).
// Task 1.4b (assimp loading) populates these; kept simple here so the type exists for the
// collision/robot modules to reference.
struct Mesh {
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
};

struct Waypoint {
  JointState state;
  double time = 0.0; // seconds from trajectory start
};

using Trajectory = std::vector<Waypoint>;

} // namespace quevedomp
