// bindings/python/bind_robot — robot model + kinematics + mesh loading (Task 4a.3, ADR-016).
//
// RobotModel is immutable and shared: from_urdf returns shared_ptr<const RobotModel>, and
// nanobind's shared_ptr caster lets Python objects feed the C++ factories (RobotInstance,
// make_numerical_ik, make_static_scene) while Python's refcount keeps the model alive.
// GIL released on the blocking calls: from_urdf (XML parse), load_mesh (assimp I/O),
// InverseKinematics::solve (iterative DLS).
#include <memory>
#include <string>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/kinematics/jacobian.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace quevedomp;

void bind_robot(nb::module_ &m) {
  // ---- Model data (read-only: the model is immutable once parsed) ---------------------------
  nb::enum_<JointType>(m, "JointType")
      .value("Fixed", JointType::Fixed)
      .value("Revolute", JointType::Revolute)
      .value("Continuous", JointType::Continuous)
      .value("Prismatic", JointType::Prismatic);

  nb::enum_<GeometryType>(m, "GeometryType")
      .value("Mesh", GeometryType::Mesh)
      .value("Box", GeometryType::Box)
      .value("Sphere", GeometryType::Sphere)
      .value("Cylinder", GeometryType::Cylinder);

  nb::class_<JointLimits>(m, "JointLimits")
      .def_ro("lower", &JointLimits::lower)
      .def_ro("upper", &JointLimits::upper)
      .def_ro("velocity", &JointLimits::velocity)
      .def_ro("effort", &JointLimits::effort)
      .def_ro("acceleration", &JointLimits::acceleration)
      .def_ro("jerk", &JointLimits::jerk)
      .def_ro("has_position_limit", &JointLimits::has_position_limit);

  nb::class_<Joint>(m, "Joint")
      .def_ro("name", &Joint::name)
      .def_ro("type", &Joint::type)
      .def_ro("parent_link", &Joint::parent_link)
      .def_ro("child_link", &Joint::child_link)
      .def_ro("axis", &Joint::axis)
      .def_ro("origin", &Joint::origin)
      .def_ro("limits", &Joint::limits)
      .def_ro("dof_index", &Joint::dof_index)
      .def("is_movable", &Joint::is_movable);

  nb::class_<CollisionGeometry>(m, "CollisionGeometry")
      .def_ro("type", &CollisionGeometry::type)
      .def_ro("origin", &CollisionGeometry::origin)
      .def_ro("mesh_filename", &CollisionGeometry::mesh_filename)
      .def_ro("mesh_scale", &CollisionGeometry::mesh_scale)
      .def_ro("box_half_extents", &CollisionGeometry::box_half_extents)
      .def_ro("sphere_radius", &CollisionGeometry::sphere_radius)
      .def_ro("cylinder_radius", &CollisionGeometry::cylinder_radius)
      .def_ro("cylinder_length", &CollisionGeometry::cylinder_length);

  nb::class_<Link>(m, "Link")
      .def_ro("name", &Link::name)
      .def_ro("parent_joint", &Link::parent_joint)
      .def_ro("child_joints", &Link::child_joints)
      .def_ro("collisions", &Link::collisions);

  nb::class_<KinematicChain>(m, "KinematicChain")
      .def_ro("base_link", &KinematicChain::base_link)
      .def_ro("tip_link", &KinematicChain::tip_link)
      .def_ro("joints", &KinematicChain::joints);

  nb::class_<RobotModel>(m, "RobotModel",
                         "Immutable robot description parsed from URDF. Construct via "
                         "RobotModel.from_urdf(urdf_xml, yaml_extension=None).")
      .def_static("from_urdf", &RobotModel::from_urdf, "urdf_xml"_a,
                  "yaml_extension"_a = nb::none(), nb::call_guard<nb::gil_scoped_release>())
      .def_prop_ro("name", &RobotModel::name)
      .def_prop_ro("root_link", &RobotModel::root_link)
      .def_prop_ro("dof", &RobotModel::dof)
      .def_prop_ro("num_links", &RobotModel::num_links)
      .def_prop_ro("num_joints", &RobotModel::num_joints)
      .def_prop_ro("links", &RobotModel::links)
      .def_prop_ro("joints", &RobotModel::joints)
      .def_prop_ro("source_urdf", &RobotModel::source_urdf)
      .def_prop_ro("source_yaml", &RobotModel::source_yaml)
      .def("find_link", &RobotModel::find_link, "name"_a, nb::rv_policy::reference_internal,
           "The Link named `name`, or None.")
      .def("find_joint", &RobotModel::find_joint, "name"_a, nb::rv_policy::reference_internal,
           "The Joint named `name`, or None.")
      .def("chain_to", &RobotModel::chain_to, "tip_link"_a,
           "Base-to-tip serial chain ending at `tip_link` (raises on unknown link).");

  // ---- Instance (mutable per-owner state: the ACM) -------------------------------------------
  nb::class_<AllowedCollisionMatrix>(m, "AllowedCollisionMatrix",
                                     "Order-independent set of link/object id pairs whose "
                                     "mutual collisions are ignored.")
      .def(nb::init<>())
      .def("allow", &AllowedCollisionMatrix::allow, "a"_a, "b"_a)
      .def("disallow", &AllowedCollisionMatrix::disallow, "a"_a, "b"_a)
      .def("is_allowed", &AllowedCollisionMatrix::is_allowed, "a"_a, "b"_a)
      .def("pairs", &AllowedCollisionMatrix::pairs, "The normalized allowed pairs (a <= b).")
      .def("__len__", &AllowedCollisionMatrix::size);

  nb::class_<RobotInstance>(m, "RobotInstance",
                            "A queryable robot: the shared immutable model plus this owner's "
                            "mutable AllowedCollisionMatrix.")
      .def(nb::init<std::shared_ptr<const RobotModel>>(), "model"_a)
      .def_prop_ro("model", &RobotInstance::model, nb::rv_policy::reference_internal)
      .def_prop_ro(
          "acm", [](RobotInstance &r) -> AllowedCollisionMatrix & { return r.acm(); },
          nb::rv_policy::reference_internal);

  // ---- Kinematics (free functions over the model) --------------------------------------------
  m.def("fk_all", &fk_all, "model"_a, "q"_a,
        "Base-frame pose of every link, indexed like model.links.");
  m.def("fk", &fk, "model"_a, "q"_a, "link"_a, "Base-frame pose of one link.");
  m.def("jacobian", &jacobian, "model"_a, "q"_a, "link"_a,
        "Geometric Jacobian (6, dof) of `link` in the base frame; rows 0..2 linear, 3..5 "
        "angular.");

  // ---- IK ------------------------------------------------------------------------------------
  nb::class_<IkOptions>(m, "IkOptions")
      .def(nb::init<>())
      .def_rw("pos_tol", &IkOptions::pos_tol)
      .def_rw("rot_tol", &IkOptions::rot_tol)
      .def_rw("max_iters", &IkOptions::max_iters)
      .def_rw("max_restarts", &IkOptions::max_restarts)
      .def_rw("stall_iters", &IkOptions::stall_iters)
      .def_rw("stall_eps", &IkOptions::stall_eps)
      .def_rw("damping", &IkOptions::damping)
      .def_rw("max_step", &IkOptions::max_step)
      .def_rw("seed", &IkOptions::seed);

  nb::class_<IkResult>(m, "IkResult")
      .def_ro("success", &IkResult::success)
      .def_ro("q", &IkResult::q)
      .def_ro("iterations", &IkResult::iterations)
      .def_ro("restarts", &IkResult::restarts)
      .def_ro("pos_error", &IkResult::pos_error)
      .def_ro("rot_error", &IkResult::rot_error)
      .def("__repr__", [](const IkResult &r) {
        return nb::str("IkResult(success={}, iterations={}, restarts={}, pos_error={}, "
                       "rot_error={})")
            .format(r.success, r.iterations, r.restarts, r.pos_error, r.rot_error);
      });

  nb::class_<InverseKinematics>(m, "InverseKinematics")
      .def("solve", &InverseKinematics::solve, "link"_a, "target"_a, "seed"_a = JointPosition(),
           nb::call_guard<nb::gil_scoped_release>(),
           "Solve for a configuration placing `link` at `target` (base frame). A seed of "
           "size model.dof seeds the first attempt; further attempts re-seed randomly.");

  m.def("make_numerical_ik", &make_numerical_ik, "model"_a, "options"_a = IkOptions{},
        "Damped-least-squares numerical IK with multi-seed restart.");

  // ---- Mesh loading (the IDE renders link meshes through these) ------------------------------
  m.def("load_mesh", &load_mesh, "path"_a, nb::call_guard<nb::gil_scoped_release>(),
        "Load a triangle mesh (STL/DAE/OBJ/...) normalized to metres.");
  m.def("resolve_mesh_uri", &resolve_mesh_uri, "uri"_a, "package_dirs"_a, "base_dir"_a = "",
        "Resolve a URDF mesh URI (package://, file://, absolute, relative) to a filesystem "
        "path.");
}
