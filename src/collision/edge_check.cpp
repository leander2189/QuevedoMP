// collision/edge_check — the RRT edge primitive (Task 2a.4, spec §4.2/§4.4).
//
// q0 -> q1 is sub-sampled at `resolution` (rad, max per-joint step) and checked as ONE batch, so
// the backend sees the whole edge at once (a GPU batch is exactly this). Returns the first
// colliding parameter t ∈ [0,1]; 1.0 when the edge is free. Endpoints are included, so a q0 that
// already collides yields first_contact_t == 0. Continuous/swept checking is a future swap-in
// behind this exact signature (spec §9.3) — callers never change.
#include "quevedomp/collision/edge_check.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"

namespace quevedomp::collision {

EdgeResult check_edge(const CollisionScene &scene, const RobotInstance &robot,
                      const JointPosition &q0, const JointPosition &q1, float resolution,
                      const QueryOptions &opts, Workspace &ws) {
  if (q0.size() != q1.size())
    throw std::runtime_error("check_edge: q0 and q1 have different sizes");
  if (!(resolution > 0.0f))
    throw std::runtime_error("check_edge: resolution must be > 0");

  const JointPosition delta = q1 - q0;
  const double max_step = delta.size() > 0 ? delta.cwiseAbs().maxCoeff() : 0.0;
  const int n = std::max(1, static_cast<int>(std::ceil(max_step / resolution)));

  // n+1 samples including both endpoints, uniformly in t = k/n along the segment.
  std::vector<JointPosition> samples;
  samples.reserve(static_cast<std::size_t>(n) + 1);
  for (int k = 0; k <= n; ++k)
    samples.push_back(q0 + (static_cast<double>(k) / n) * delta);

  const BatchResult batch = scene.query_batch(robot, samples, opts, ws);
  for (std::size_t k = 0; k < batch.in_collision.size(); ++k) {
    if (batch.in_collision[k] != 0)
      return {false, static_cast<float>(static_cast<double>(k) / n)};
  }
  return {true, 1.0f};
}

} // namespace quevedomp::collision
