// bindings/python/bind_types — core vocabulary types (Task 4a.2, ADR-016).
//
// `JointPosition`/`JointVelocity` are NOT classes here: nanobind's Eigen caster maps them
// to/from 1-D float64 numpy arrays (a small per-call copy — accepted by ADR-016; Python never
// sits in a hot loop). Mesh vertex/triangle buffers are the large arrays: they are exposed as
// zero-copy (N,3) numpy views over the underlying std::vector storage.
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/operators.h>

#include "quevedomp/core/types.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace quevedomp;

namespace {

// Quaternions cross the boundary as (4,) arrays in [w, x, y, z] order, normalized on entry
// (gizmo/UI floats drift; Eigen expects unit quaternions for rotations).
Eigen::Quaterniond quat_from_wxyz(const Eigen::Vector4d &q) {
  return Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized();
}

Eigen::Vector4d quat_to_wxyz(const Eigen::Quaterniond &q) { return {q.w(), q.x(), q.y(), q.z()}; }

// Zero-copy (N,3) view over a std::vector of Eigen 3-vectors; `owner` (the bound Mesh's Python
// object, via nb::find) keeps the C++ storage alive as long as the array is referenced.
template <typename Scalar, typename Vec> auto view_n3(std::vector<Vec> &v, nb::handle owner) {
  return nb::ndarray<Scalar, nb::numpy, nb::shape<-1, 3>>(
      v.empty() ? nullptr : reinterpret_cast<Scalar *>(v.data()), {v.size(), 3}, owner);
}

} // namespace

void bind_types(nb::module_ &m) {
  // ---- Transform ---------------------------------------------------------------------------
  nb::class_<Transform>(m, "Transform",
                        "SE(3) rigid-body transform. Compose with `*`; apply to a (3,) point "
                        "with `*`. Quaternions are (4,) [w, x, y, z], normalized on entry.")
      .def(nb::init<>(), "Identity transform.")
      .def_static("identity", &Transform::Identity)
      .def_static("from_translation", &Transform::from_translation, "t"_a)
      .def_static(
          "from_rotation",
          [](const Eigen::Vector4d &q) { return Transform::from_rotation(quat_from_wxyz(q)); },
          "q_wxyz"_a)
      .def_static(
          "from_parts",
          [](const Eigen::Vector3d &t, const Eigen::Vector4d &q) {
            return Transform::from_parts(t, quat_from_wxyz(q));
          },
          "t"_a, "q_wxyz"_a)
      .def_static(
          "from_matrix",
          [](const Eigen::Matrix4d &mat) {
            Eigen::Isometry3d iso;
            iso.matrix() = mat;
            return Transform(iso);
          },
          "matrix"_a, "Build from a 4x4 homogeneous matrix (rotation part must be orthonormal).")
      .def("matrix", &Transform::matrix, "4x4 homogeneous matrix.")
      .def("translation", &Transform::translation)
      .def("rotation", &Transform::rotation, "3x3 rotation matrix.")
      .def(
          "quaternion",
          [](const Transform &t) { return quat_to_wxyz(Eigen::Quaterniond(t.rotation())); },
          "Rotation as (4,) [w, x, y, z].")
      .def("inverse", &Transform::inverse)
      .def(
          "__mul__", [](const Transform &a, const Transform &b) { return a * b; },
          nb::is_operator())
      .def(
          "__mul__", [](const Transform &a, const Eigen::Vector3d &p) { return a * p; },
          nb::is_operator())
      .def("is_approx", &Transform::is_approx, "other"_a, "tol"_a = 1e-12)
      .def("__repr__", [](const Transform &t) {
        const Eigen::Vector3d p = t.translation();
        const Eigen::Vector4d q = quat_to_wxyz(Eigen::Quaterniond(t.rotation()));
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "Transform(t=[%.4g, %.4g, %.4g], q_wxyz=[%.4g, %.4g, %.4g, %.4g])", p[0],
                      p[1], p[2], q[0], q[1], q[2], q[3]);
        return nb::str(buf);
      });

  // ---- Pose --------------------------------------------------------------------------------
  nb::class_<Pose>(m, "Pose", "A target transform plus position/rotation tolerances.")
      .def(nb::init<>())
      .def(
          "__init__",
          [](Pose *p, const Transform &tf, double pos_tol, double rot_tol) {
            new (p) Pose{tf, pos_tol, rot_tol};
          },
          "tf"_a, "pos_tol"_a = 1e-3, "rot_tol"_a = 1e-2)
      .def_rw("tf", &Pose::tf)
      .def_rw("pos_tol", &Pose::pos_tol)
      .def_rw("rot_tol", &Pose::rot_tol);

  // ---- Mesh --------------------------------------------------------------------------------
  nb::class_<Mesh>(m, "Mesh",
                   "Triangle mesh. `vertices` (N,3) float64 and `triangles` (M,3) int32 are "
                   "zero-copy views over the C++ buffers: in-place numpy edits mutate the mesh.")
      .def(nb::init<>())
      .def(
          "__init__",
          [](Mesh *m_, nb::ndarray<const double, nb::shape<-1, 3>, nb::c_contig> v,
             nb::ndarray<const std::int32_t, nb::shape<-1, 3>, nb::c_contig> t) {
            new (m_) Mesh();
            m_->vertices.resize(v.shape(0));
            m_->triangles.resize(t.shape(0));
            if (v.shape(0) > 0)
              std::memcpy(m_->vertices.data(), v.data(), v.shape(0) * 3 * sizeof(double));
            if (t.shape(0) > 0)
              std::memcpy(m_->triangles.data(), t.data(), t.shape(0) * 3 * sizeof(std::int32_t));
          },
          "vertices"_a, "triangles"_a)
      .def_prop_ro("vertices", [](Mesh &m_) { return view_n3<double>(m_.vertices, nb::find(&m_)); })
      .def_prop_ro("triangles",
                   [](Mesh &m_) { return view_n3<std::int32_t>(m_.triangles, nb::find(&m_)); })
      .def("__repr__", [](const Mesh &m_) {
        return nb::str("Mesh(vertices={}, triangles={})")
            .format(m_.vertices.size(), m_.triangles.size());
      });

  // ---- Joint-space state + trajectory samples ----------------------------------------------
  nb::class_<JointState>(m, "JointState")
      .def(nb::init<>())
      .def_rw("pos", &JointState::pos)
      .def_rw("vel", &JointState::vel);

  nb::class_<Waypoint>(m, "Waypoint", "One timed trajectory sample (seconds from start).")
      .def(nb::init<>())
      .def_rw("state", &Waypoint::state)
      .def_rw("time", &Waypoint::time);
}
