// bindings/python/bind_collision — collision scene + queries (Task 4a.4, ADR-016).
//
// The batch-first contract crosses the boundary intact: query_batch takes an (N, dof) numpy
// array and returns BatchResult whose in_collision/min_distance are zero-copy views over the
// C++ vectors. GIL released on the blocking calls: make_static_scene (BVH builds), query,
// query_batch, check_edge. One Workspace per Python thread (ADR-005), exactly as in C++.
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

// (N, dof) numpy batch -> the vector<JointPosition> the C++ API speaks. An O(batch) copy of a
// few tens of KB — noise next to the query itself (ADR-016).
std::vector<JointPosition>
to_configs(const nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig> &qs) {
  std::vector<JointPosition> out(qs.shape(0));
  for (std::size_t i = 0; i < qs.shape(0); ++i)
    out[i] = Eigen::Map<const Eigen::VectorXd>(qs.data() + i * qs.shape(1),
                                               static_cast<Eigen::Index>(qs.shape(1)));
  return out;
}

// Zero-copy 1-D view over a std::vector, with `owner` keeping the BatchResult alive.
template <typename T> auto view_1d(std::vector<T> &v, nb::handle owner) {
  return nb::ndarray<T, nb::numpy, nb::shape<-1>>(v.empty() ? nullptr : v.data(), {v.size()},
                                                  owner);
}

} // namespace

void bind_collision(nb::module_ &m) {
  // ---- Environment geometry ------------------------------------------------------------------
  nb::class_<BoxShape>(m, "BoxShape")
      .def(
          "__init__", [](BoxShape *s, const Eigen::Vector3d &he) { new (s) BoxShape{he}; },
          "half_extents"_a)
      .def_rw("half_extents", &BoxShape::half_extents);

  nb::class_<SphereShape>(m, "SphereShape")
      .def(
          "__init__", [](SphereShape *s, double r) { new (s) SphereShape{r}; }, "radius"_a)
      .def_rw("radius", &SphereShape::radius);

  nb::class_<CylinderShape>(m, "CylinderShape", "Axis along +Z, centered at the origin.")
      .def(
          "__init__",
          [](CylinderShape *s, double r, double l) {
            new (s) CylinderShape{r, l};
          },
          "radius"_a, "length"_a)
      .def_rw("radius", &CylinderShape::radius)
      .def_rw("length", &CylinderShape::length);

  nb::class_<SceneObject>(m, "SceneObject")
      .def(
          "__init__",
          [](SceneObject *o, std::string id, Geometry geometry, const Transform &pose) {
            new (o) SceneObject{std::move(id), std::move(geometry), pose};
          },
          "id"_a, "geometry"_a, "pose"_a = Transform{})
      .def_rw("id", &SceneObject::id)
      .def_rw("geometry", &SceneObject::geometry)
      .def_rw("pose", &SceneObject::pose);

  nb::class_<SceneDescription>(m, "SceneDescription",
                               "The static environment handed to make_static_scene. NOTE: "
                               "`objects` crosses the boundary by value — assign a whole list "
                               "or use add(); mutating the returned list is a no-op.")
      .def(nb::init<>())
      .def_rw("objects", &SceneDescription::objects)
      .def(
          "add",
          [](SceneDescription &s, std::string id, Geometry geometry, const Transform &pose) {
            s.objects.push_back({std::move(id), std::move(geometry), pose});
          },
          "id"_a, "geometry"_a, "pose"_a = Transform{});

  // ---- Query options + results -----------------------------------------------------------------
  nb::class_<PaddingMap>(m, "PaddingMap",
                         "Per link/object-pair collision padding override (metres, ADR-013).")
      .def(nb::init<>())
      .def("set", &PaddingMap::set, "a"_a, "b"_a, "padding"_a)
      .def("get", &PaddingMap::get, "a"_a, "b"_a, "fallback"_a = 0.0f)
      .def("empty", &PaddingMap::empty);

  nb::class_<QueryOptions>(m, "QueryOptions")
      .def(nb::init<>())
      .def_rw("distance", &QueryOptions::distance)
      .def_rw("safety_margin", &QueryOptions::safety_margin)
      .def_rw("robot_padding", &QueryOptions::robot_padding)
      .def_rw("max_distance", &QueryOptions::max_distance)
      .def_rw("check_self_collision", &QueryOptions::check_self_collision)
      // optional<> setters must explicitly allow None (nanobind rejects it otherwise).
      .def_rw("per_pair_padding", &QueryOptions::per_pair_padding,
              nb::for_setter(nb::arg("value").none()));

  nb::class_<CollisionPair>(m, "CollisionPair", "Witness pair, for debug/visualization.")
      .def_ro("a", &CollisionPair::a)
      .def_ro("b", &CollisionPair::b)
      .def_ro("point_a", &CollisionPair::point_a)
      .def_ro("point_b", &CollisionPair::point_b);

  nb::class_<CollisionResult>(m, "CollisionResult")
      .def_ro("in_collision", &CollisionResult::in_collision)
      .def_ro("min_distance", &CollisionResult::min_distance)
      .def_ro("witness", &CollisionResult::witness)
      .def("__repr__", [](const CollisionResult &r) {
        return nb::str("CollisionResult(in_collision={}, min_distance={})")
            .format(r.in_collision, r.min_distance);
      });

  nb::class_<BatchResult>(m, "BatchResult",
                          "Per-config results. `in_collision` (N,) uint8 and `min_distance` "
                          "(N,) float32 are zero-copy views over the C++ buffers.")
      .def_prop_ro("in_collision",
                   [](BatchResult &r) { return view_1d(r.in_collision, nb::find(&r)); })
      .def_prop_ro("min_distance",
                   [](BatchResult &r) { return view_1d(r.min_distance, nb::find(&r)); })
      .def_ro("witnesses", &BatchResult::witnesses);

  nb::class_<EdgeResult>(m, "EdgeResult")
      .def_ro("valid", &EdgeResult::valid)
      .def_ro("first_contact_t", &EdgeResult::first_contact_t);

  // ---- Scene -----------------------------------------------------------------------------------
  nb::class_<Workspace>(m, "Workspace",
                        "Opaque per-thread query scratch. One per Python thread (ADR-005); "
                        "create via CollisionScene.make_workspace().");

  nb::class_<CollisionScene>(m, "CollisionScene")
      .def("add_object", &CollisionScene::add_object, "id"_a, "geometry"_a, "pose"_a,
           "Add a static object; returns its handle.")
      .def("remove_object", &CollisionScene::remove_object, "handle"_a)
      .def("move_object", &CollisionScene::move_object, "handle"_a, "pose"_a)
      .def("make_workspace", &CollisionScene::make_workspace)
      .def("query", &CollisionScene::query, "robot"_a, "q"_a, "options"_a, "workspace"_a,
           nb::call_guard<nb::gil_scoped_release>(),
           "Collision state of one configuration (convenience over query_batch).")
      .def(
          "query_batch",
          [](const CollisionScene &scene, const RobotInstance &robot,
             nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig> qs,
             const QueryOptions &opts, Workspace &ws) {
            const auto configs = to_configs(qs);
            return scene.query_batch(robot, configs, opts, ws);
          },
          "robot"_a, "qs"_a, "options"_a, "workspace"_a, nb::call_guard<nb::gil_scoped_release>(),
          "Collision state of an (N, dof) batch of configurations — the primary query.");

  m.def("check_edge",
        nb::overload_cast<const CollisionScene &, const RobotInstance &, const JointPosition &,
                          const JointPosition &, float, const QueryOptions &, Workspace &>(
            &check_edge),
        "scene"_a, "robot"_a, "q0"_a, "q1"_a, "resolution"_a, "options"_a, "workspace"_a,
        nb::call_guard<nb::gil_scoped_release>(),
        "Validate the straight joint-space edge q0 -> q1, sub-sampled at `resolution` (rad) "
        "and checked as ONE batch. The RRT primitive.");

  // ---- Construction ----------------------------------------------------------------------------
  nb::enum_<BackendHint>(m, "BackendHint")
      .value("Auto", BackendHint::Auto)
      .value("ForceCpuFcl", BackendHint::ForceCpuFcl)
      .value("ForceOptix", BackendHint::ForceOptix);

  nb::class_<MeshSources>(m, "MeshSources",
                          "How to resolve the robot's URDF mesh URIs: {package -> directory} "
                          "plus an optional base dir for relative paths.")
      .def(nb::init<>())
      .def(
          "__init__",
          [](MeshSources *s, std::unordered_map<std::string, std::string> package_dirs,
             std::string base_dir) {
            new (s) MeshSources{std::move(package_dirs), std::move(base_dir)};
          },
          "package_dirs"_a, "base_dir"_a = "")
      .def_rw("package_dirs", &MeshSources::package_dirs)
      .def_rw("base_dir", &MeshSources::base_dir);

  m.def("make_static_scene", &make_static_scene, "model"_a, "environment"_a,
        "hint"_a = BackendHint::Auto, "meshes"_a = MeshSources{},
        nb::call_guard<nb::gil_scoped_release>(),
        "Build a collision scene over a static environment (BVH build; GIL released).");

  // (After the MeshSources binding above: nanobind casts default args at def time.)
  m.def("cartesian_lever_weights", &cartesian_lever_weights, "model"_a, "meshes"_a = MeshSources{},
        nb::call_guard<nb::gil_scoped_release>(),
        "Per-dof lever weights (m/rad; 1 for prismatic): upper bound on how far any collision-"
        "geometry point moves per unit joint motion — feed PlannerParams/SmootherParams."
        "lever_weights for Cartesian-bounded edge stepping (max_link_sweep).");

  m.def("optix_available", &optix_available, "True if this build includes the OptiX GPU backend.");
}
