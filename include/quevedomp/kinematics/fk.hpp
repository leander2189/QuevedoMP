// kinematics/fk — forward kinematics over a RobotModel (Task 1.5, spec §6).
//
// Pure functions: given a configuration q (one value per movable joint, indexed by
// Joint::dof_index, size == model.dof()), compute link poses in the robot's base frame.
#pragma once

#include <string>
#include <vector>

#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp {

// Base-frame pose of every link, indexed the same as RobotModel::links().
// Throws std::runtime_error if q.size() != model.dof().
std::vector<Transform> fk_all(const RobotModel &model, const JointPosition &q);

// Base-frame pose of a single link. Throws std::runtime_error if the link is unknown or
// q.size() != model.dof().
Transform fk(const RobotModel &model, const JointPosition &q, const std::string &link);

} // namespace quevedomp
