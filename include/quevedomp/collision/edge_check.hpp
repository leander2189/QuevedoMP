// collision/edge_check — the RRT primitive (spec §4.2/§4.4).
#pragma once

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace quevedomp::collision {

struct EdgeResult {
  bool valid = false;
  float first_contact_t = 1.0f; // t in [0,1]; 1.0 if valid
};

// q0 -> q1 is sub-sampled at `resolution` (rad, max joint step) and checked as ONE batch.
// Continuous/swept checking is a future swap-in behind this exact signature — callers never
// change. (Implemented in Task 2a.4.)
[[nodiscard]] EdgeResult check_edge(const CollisionScene &scene, const RobotInstance &robot,
                                    const JointPosition &q0, const JointPosition &q1,
                                    float resolution, const QueryOptions &opts, Workspace &ws);

// Same check under an EdgeDiscretization policy (Task 3.3d P3): Cartesian-bounded stepping when
// the policy enables it, else the uniform per-joint resolution above.
[[nodiscard]] EdgeResult check_edge(const CollisionScene &scene, const RobotInstance &robot,
                                    const JointPosition &q0, const JointPosition &q1,
                                    const EdgeDiscretization &disc, const QueryOptions &opts,
                                    Workspace &ws);

} // namespace quevedomp::collision
