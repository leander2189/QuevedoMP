// collision/CollisionScene — the backend-agnostic collision interface (spec §4.2/§4.3).
// Batch-first: query_batch is the primitive; query is a one-element convenience. A per-thread
// Workspace owns all mutable scratch, so concurrent const queries are lock-free.
#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
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

// How to resolve a robot's URDF mesh collision URIs into loadable files. URDF references geometry
// by URI (commonly `package://<pkg>/...`), and there is no global package index — the caller
// supplies the {package name → directory} map (and an optional base dir for relative paths). A
// robot whose collision geometry is all primitives needs none of this (leave it default). Mirrors
// resolve_mesh_uri()'s parameters; see robot/mesh_resolver.hpp.
struct MeshSources {
  std::unordered_map<std::string, std::string> package_dirs;
  std::string base_dir;
};

// Build a scene over a static environment. The robot model is needed for FK + collision geometry;
// `meshes` resolves the robot's mesh collision links (Task 2a.2b). Throws if a robot mesh URI
// cannot be resolved or loaded (it is never silently skipped). `BackendHint::ForceOptix` builds the
// GPU backend when this build includes it (see optix_available()), else throws.
[[nodiscard]] std::unique_ptr<CollisionScene>
make_static_scene(std::shared_ptr<const RobotModel> robot, const SceneDescription &environment,
                  BackendHint hint = BackendHint::Auto, const MeshSources &meshes = {});

// True if this build includes the OptiX GPU backend (Phase 2b). When false,
// make_static_scene(ForceOptix) throws.
bool optix_available() noexcept;

} // namespace quevedomp::collision
