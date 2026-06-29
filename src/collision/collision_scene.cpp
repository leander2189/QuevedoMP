#include "quevedomp/collision/collision_scene.hpp"

#include <array>
#include <span>

namespace quevedomp::collision {

CollisionResult CollisionScene::query(const RobotInstance &robot, const JointPosition &q,
                                      const QueryOptions &opts, Workspace &ws) const {
  const std::array<JointPosition, 1> one{q};
  const BatchResult batch = query_batch(robot, std::span<const JointPosition>(one), opts, ws);

  CollisionResult r;
  if (!batch.in_collision.empty())
    r.in_collision = batch.in_collision.front() != 0;
  if (!batch.min_distance.empty())
    r.min_distance = batch.min_distance.front();
  if (!batch.witnesses.empty())
    r.witness = batch.witnesses.front();
  return r;
}

} // namespace quevedomp::collision
