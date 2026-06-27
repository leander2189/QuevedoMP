#include "quevedomp/core/types.hpp"

namespace quevedomp {

Transform Transform::from_translation(const Eigen::Vector3d &t) noexcept {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() = t;
  return Transform{iso};
}

Transform Transform::from_rotation(const Eigen::Quaterniond &q) noexcept {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = q.normalized().toRotationMatrix();
  return Transform{iso};
}

Transform Transform::from_parts(const Eigen::Vector3d &t, const Eigen::Quaterniond &q) noexcept {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = q.normalized().toRotationMatrix();
  iso.translation() = t;
  return Transform{iso};
}

Transform Transform::inverse() const noexcept { return Transform{iso_.inverse()}; }

Transform Transform::operator*(const Transform &rhs) const noexcept {
  return Transform{iso_ * rhs.iso_};
}

Eigen::Vector3d Transform::operator*(const Eigen::Vector3d &point) const noexcept {
  return iso_ * point;
}

bool Transform::is_approx(const Transform &other, double tol) const noexcept {
  return (iso_.matrix() - other.iso_.matrix()).norm() <= tol;
}

} // namespace quevedomp
