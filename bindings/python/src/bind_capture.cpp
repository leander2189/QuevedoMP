// bindings/python/bind_capture — Task 2a.5 serializers (bound for Task 4a.6, ADR-016).
//
// quevedomp-studio saves/loads its robot + scene state through these, so a Phase 3b capture
// bundle will open in the studio without a second format. The C++ API speaks std::string
// BLOBS (binary, 4-byte magic + version) — they cross the boundary as Python `bytes`, never
// `str` (the default string caster would mangle non-UTF-8 payloads).
#include <string>

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "quevedomp/capture/serialize.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace quevedomp;

namespace {

std::string to_blob(const nb::bytes &b) { return {b.c_str(), b.size()}; }
nb::bytes to_bytes(const std::string &s) { return nb::bytes(s.data(), s.size()); }

} // namespace

void bind_capture(nb::module_ &m) {
  m.def(
      "serialize_robot_model",
      [](const RobotModel &model) { return to_bytes(capture::serialize_robot_model(model)); },
      "model"_a, "RobotModel -> blob (URDF + optional YAML inlined; re-parsed on load).");
  m.def(
      "deserialize_robot_model",
      [](const nb::bytes &blob) { return capture::deserialize_robot_model(to_blob(blob)); },
      "blob"_a);

  m.def(
      "serialize_robot_instance",
      [](const RobotInstance &robot) { return to_bytes(capture::serialize_robot_instance(robot)); },
      "robot"_a, "RobotInstance -> blob (model + the ACM's allowed pairs).");
  m.def(
      "deserialize_robot_instance",
      [](const nb::bytes &blob) { return capture::deserialize_robot_instance(to_blob(blob)); },
      "blob"_a);

  m.def(
      "serialize_scene",
      [](const collision::SceneDescription &scene) {
        return to_bytes(capture::serialize_scene(scene));
      },
      "scene"_a, "SceneDescription -> blob (objects: id, geometry, pose).");
  m.def(
      "deserialize_scene",
      [](const nb::bytes &blob) { return capture::deserialize_scene(to_blob(blob)); }, "blob"_a);
}
