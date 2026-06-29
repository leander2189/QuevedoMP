// collision/CollisionScene — the backend-agnostic collision interface (spec §4.2/§4.3).
// Batch-first: query_batch is the primitive; query is a one-element convenience. A per-thread
// Workspace owns all mutable scratch, so concurrent const queries are lock-free.
#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/types.hpp" // Transform, JointPosition
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::collision {

// Opaque, backend-specific scratch. FCL's is ~trivial; OptiX's owns a CUDA stream, device
// buffers, pinned staging, SBT and params. Never crosses the API as a concrete type.
class Workspace {
public:
  virtual ~Workspace() = default;
};

using SceneHandle = std::uint32_t;

class CollisionScene {
public:
  virtual ~CollisionScene() = default;

  // Static-environment editing (typically once at setup).
  virtual SceneHandle add_object(std::string id, const Geometry &geom, const Transform &pose) = 0;
  virtual void remove_object(SceneHandle handle) = 0;
  virtual void move_object(SceneHandle handle, const Transform &pose) = 0;

  // Each querying thread owns one workspace ⇒ lock-free concurrent queries.
  [[nodiscard]] virtual std::unique_ptr<Workspace> make_workspace() const = 0;

  // Primary query. The scene FKs the robot at each q (scene-internal FK, ADR-015), poses its
  // collision geometry, and tests robot-vs-environment and robot-vs-self (per the ACM).
  [[nodiscard]] virtual BatchResult query_batch(const RobotInstance &robot,
                                                std::span<const JointPosition> qs,
                                                const QueryOptions &opts, Workspace &ws) const = 0;

  // One-element convenience over query_batch, defined once in the base class.
  [[nodiscard]] CollisionResult query(const RobotInstance &robot, const JointPosition &q,
                                      const QueryOptions &opts, Workspace &ws) const;
};

enum class BackendHint { Auto, ForceCpuFcl, ForceOptix };

// Build a scene over a static environment. The robot model is needed for FK + collision geometry.
// (FCL backend lands in Task 2a.2; OptiX in Phase 2b.)
[[nodiscard]] std::unique_ptr<CollisionScene>
make_static_scene(std::shared_ptr<const RobotModel> robot, const SceneDescription &environment,
                  BackendHint hint = BackendHint::Auto);

} // namespace quevedomp::collision
