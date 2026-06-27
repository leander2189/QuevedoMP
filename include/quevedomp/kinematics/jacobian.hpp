// kinematics/jacobian — geometric Jacobian of a link (Task 1.6, spec §6).
#pragma once

#include <string>

#include <Eigen/Core>

#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp {

// Geometric Jacobian (6 × model.dof()) of `link`, expressed in the base frame.
// Row layout: rows 0..2 = linear-velocity map, rows 3..5 = angular-velocity map, so
// [v; w] = J · q̇. Column k belongs to the movable joint with dof_index == k; columns for
// joints not on the base→link path are zero.
// Throws std::runtime_error if the link is unknown or q.size() != model.dof().
Eigen::MatrixXd jacobian(const RobotModel &model, const JointPosition &q, const std::string &link);

} // namespace quevedomp
