// collision/geometry — environment collision geometry and scene description (spec §4.2).
// Shapes are pose-free; their world placement is the Transform passed to add_object /
// SceneObject::pose (so the same shape can be reused at many poses).
#pragma once

#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/core/types.hpp" // quevedomp::Mesh, quevedomp::Transform

namespace quevedomp::collision {

struct BoxShape {
  Eigen::Vector3d half_extents = Eigen::Vector3d::Zero();
};
struct SphereShape {
  double radius = 0.0;
};
struct CylinderShape { // axis along +Z, centered at origin
  double radius = 0.0;
  double length = 0.0;
};

// An environment shape: a primitive or a triangle mesh (quevedomp::Mesh, pose-free).
using Geometry = std::variant<BoxShape, SphereShape, CylinderShape, Mesh>;

struct SceneObject {
  std::string id;
  Geometry geometry;
  Transform pose; // object frame in the world/base frame
};

// The static environment handed to make_static_scene.
struct SceneDescription {
  std::vector<SceneObject> objects;
};

} // namespace quevedomp::collision
