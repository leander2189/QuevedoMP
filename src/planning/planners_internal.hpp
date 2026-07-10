// planning — internal factory declarations for the concrete planners the registry wires up.
// NOT a public header (lives under src/): callers select a planner through make_planner() by id.
#pragma once

#include <memory>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace quevedomp::planning::detail {

// Batched bidirectional RRT-Connect (Task 3.2). Registered under "rrt_connect".
std::unique_ptr<Planner> make_rrt_connect(const PlannerParams &params,
                                          std::shared_ptr<const RobotInstance> robot,
                                          std::shared_ptr<const collision::CollisionScene> scene);

} // namespace quevedomp::planning::detail
