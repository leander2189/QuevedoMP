// bindings/python/bind_clearance — roadmap R3: ClearanceField + robot sphere cover.
//
// build()/query()/clearance_batch release the GIL (heavy grid/FK work). `data` is a zero-copy
// (nz, ny, nx) float32 view over the field's grid — the studio slices it directly for heatmaps.
#include <cstddef>
#include <vector>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "quevedomp/clearance/clearance_field.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace quevedomp;
using namespace quevedomp::clearance;

void bind_clearance(nb::module_ &m) {
  nb::class_<ClearanceFieldOptions>(m, "ClearanceFieldOptions")
      .def(nb::init<>())
      .def_rw("resolution", &ClearanceFieldOptions::resolution, "Voxel edge (m).")
      .def_rw("margin", &ClearanceFieldOptions::margin, "Grid padding beyond the env AABB (m).")
      .def_rw("max_voxels", &ClearanceFieldOptions::max_voxels)
      .def_rw("use_gpu", &ClearanceFieldOptions::use_gpu,
              "Try the CUDA JFA first; silent CPU (OpenMP) fallback when no device.");

  nb::class_<ClearanceField>(m, "ClearanceField",
                             "Voxel SDF of the STATIC environment (R3, ADR-018): approximate, "
                             "gradient-bearing, for optimization/visualization — the exact "
                             "collision backend remains the only collision certificate.")
      .def_static("build", &ClearanceField::build, "environment"_a,
                  "options"_a = ClearanceFieldOptions{}, nb::call_guard<nb::gil_scoped_release>(),
                  "One-time build over a SceneDescription (quasi-static assumption).")
      .def("distance", &ClearanceField::distance, "p"_a,
           "Signed distance (m): positive outside, negative inside watertight solids.")
      .def("gradient", &ClearanceField::gradient, "p"_a)
      .def(
          "query",
          [](const ClearanceField &f,
             nb::ndarray<const double, nb::shape<-1, 3>, nb::c_contig> pts) {
            const std::size_t n = pts.shape(0);
            std::vector<Eigen::Vector3d> points(n);
            for (std::size_t i = 0; i < n; ++i) {
              points[i] = Eigen::Vector3d(pts(i, 0), pts(i, 1), pts(i, 2));
            }
            auto *d = new double[n];
            auto *g = new double[n * 3];
            {
              nb::gil_scoped_release release;
              std::vector<double> dist(n);
              std::vector<Eigen::Vector3d> grad(n);
              f.query(points, dist, grad);
              for (std::size_t i = 0; i < n; ++i) {
                d[i] = dist[i];
                g[3 * i] = grad[i].x();
                g[3 * i + 1] = grad[i].y();
                g[3 * i + 2] = grad[i].z();
              }
            }
            nb::capsule od(d, [](void *p) noexcept { delete[] static_cast<double *>(p); });
            nb::capsule og(g, [](void *p) noexcept { delete[] static_cast<double *>(p); });
            return nb::make_tuple(nb::ndarray<double, nb::numpy, nb::shape<-1>>(d, {n}, od),
                                  nb::ndarray<double, nb::numpy, nb::shape<-1, 3>>(g, {n, 3}, og));
          },
          "points"_a, "Batched (N,3) query -> (distances (N,), gradients (N,3)).")
      .def_prop_ro("origin", &ClearanceField::origin)
      .def_prop_ro("resolution", &ClearanceField::resolution)
      .def_prop_ro("dims", &ClearanceField::dims, "(nx, ny, nz)")
      .def_prop_ro("built_on_gpu", &ClearanceField::built_on_gpu)
      .def_prop_ro("build_seconds", &ClearanceField::build_seconds)
      .def_prop_ro(
          "data",
          [](const ClearanceField &f) {
            const Eigen::Vector3i d = f.dims();
            return nb::ndarray<const float, nb::numpy>(f.data().data(),
                                                       {static_cast<std::size_t>(d.z()),
                                                        static_cast<std::size_t>(d.y()),
                                                        static_cast<std::size_t>(d.x())},
                                                       nb::find(&f));
          },
          "Zero-copy (nz, ny, nx) float32 view of the signed-distance grid.");

  nb::class_<RobotSpheres::Sphere>(m, "ClearanceSphere")
      .def_ro("link", &RobotSpheres::Sphere::link)
      .def_ro("center", &RobotSpheres::Sphere::center)
      .def_ro("radius", &RobotSpheres::Sphere::radius);

  nb::class_<RobotSpheres>(m, "RobotSpheres").def_ro("spheres", &RobotSpheres::spheres);

  m.def("decompose_robot", &decompose_robot, "model"_a, "meshes"_a = collision::MeshSources{},
        "target_radius"_a = 0.05, "max_spheres_per_geometry"_a = 8,
        nb::call_guard<nb::gil_scoped_release>(),
        "Conservative sphere cover of the robot's collision geometry (link frames).");

  m.def(
      "clearance_batch",
      [](const ClearanceField &field, const RobotModel &model, const RobotSpheres &spheres,
         nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig> qs) {
        std::vector<JointPosition> configs(qs.shape(0));
        for (std::size_t i = 0; i < qs.shape(0); ++i) {
          configs[i] = Eigen::Map<const Eigen::VectorXd>(qs.data() + i * qs.shape(1),
                                                         static_cast<Eigen::Index>(qs.shape(1)));
        }
        nb::gil_scoped_release release;
        return clearance_batch(field, model, spheres, configs);
      },
      "field"_a, "model"_a, "spheres"_a, "qs"_a,
      "Per-config min clearance: min over spheres of field(center) - radius. Negative => a "
      "sphere (conservatively) penetrates.");
}
