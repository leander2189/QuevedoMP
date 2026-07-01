// capture/serialize — binary serializers for the capturable inputs (Task 2a.5, spec §5.3).
//
// The capture bundle (§5.3) records everything needed to re-run a plan; its serialized robot
// (URDF inlined), robot state (ACM), and scene (objects + poses) come from these functions. They
// are also reused by the collision differential harness. This task builds the per-object
// serializers; the MCAP container + zstd compression (ADR-007) is Phase 3.
//
// Format: a compact, self-describing binary blob (4-byte magic + version). Round-trip is exact on
// the same architecture (same endianness); cross-endian portability is out of scope for v0.
// deserialize_* throw std::runtime_error on a bad magic/version or truncated data.
#pragma once

#include <memory>
#include <string>

#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::capture {

// RobotModel <-> its source (URDF + optional tool YAML), re-parsed on load (lossless).
std::string serialize_robot_model(const RobotModel &model);
std::shared_ptr<const RobotModel> deserialize_robot_model(const std::string &blob);

// RobotInstance = the model blob + the ACM's allowed pairs.
std::string serialize_robot_instance(const RobotInstance &robot);
RobotInstance deserialize_robot_instance(const std::string &blob);

// SceneDescription = the environment's objects (id, geometry, pose).
std::string serialize_scene(const collision::SceneDescription &scene);
collision::SceneDescription deserialize_scene(const std::string &blob);

} // namespace quevedomp::capture
