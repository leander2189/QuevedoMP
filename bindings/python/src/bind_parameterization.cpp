// bindings/python/bind_parameterization — Task 3.4: PathSpline + time parameterization.
//
// parametrize() and fit_collision_free() release the GIL (pure C++ number-crunching /
// batch collision). PathSpline is move-only in C++; Python holds it by value semantics via
// the returned objects.
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include "quevedomp/parameterization/parameterize.hpp"
#include "quevedomp/parameterization/path_spline.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace quevedomp;
using namespace quevedomp::parameterization;

void bind_parameterization(nb::module_ &m) {
  nb::class_<PathSpline>(m, "PathSpline",
                         "C4 (degree-5 B-spline) geometric path through waypoints, s in [0,1]. "
                         "Jerk limits need at least C3 — never feed the parameterizer a "
                         "polyline when jerk is on.")
      .def_static("fit", &PathSpline::fit, "waypoints"_a,
                  "Interpolate the waypoints (>= 2 distinct); short lists are densified along "
                  "the polyline so degree 5 is always available.")
      .def("eval", &PathSpline::eval, "s"_a)
      .def("d1", &PathSpline::d1, "s"_a)
      .def("d2", &PathSpline::d2, "s"_a)
      .def("d3", &PathSpline::d3, "s"_a)
      .def_prop_ro("dof", &PathSpline::dof)
      .def_prop_ro("waypoint_params", &PathSpline::waypoint_params);

  nb::class_<SplineFitResult>(m, "SplineFitResult")
      .def_ro("success", &SplineFitResult::success)
      .def_ro("message", &SplineFitResult::message)
      // PathSpline is move-only; expose the optional as a nullable internal reference.
      .def_prop_ro(
          "spline",
          [](const SplineFitResult &r) -> const PathSpline * {
            return r.spline.has_value() ? &*r.spline : nullptr;
          },
          nb::rv_policy::reference_internal)
      .def_ro("rounds", &SplineFitResult::rounds)
      .def_ro("checked_samples", &SplineFitResult::checked_samples);

  nb::class_<collision::EdgeDiscretization>(m, "EdgeDiscretization",
                                            "Edge/curve sampling policy (P3): uniform "
                                            "joint_resolution, or Cartesian-bounded when "
                                            "max_link_sweep > 0 with lever_weights set.")
      .def(nb::init<>())
      .def_rw("joint_resolution", &collision::EdgeDiscretization::joint_resolution)
      .def_rw("max_link_sweep", &collision::EdgeDiscretization::max_link_sweep)
      .def_rw("lever_weights", &collision::EdgeDiscretization::lever_weights);

  m.def("fit_collision_free", &fit_collision_free, "waypoints"_a, "scene"_a, "robot"_a, "disc"_a,
        "options"_a, "workspace"_a, "max_rounds"_a = 4, nb::call_guard<nb::gil_scoped_release>(),
        "Fit a PathSpline and re-validate it against the scene at edge fidelity (ONE "
        "query_batch per round); densifies along the polyline and refits on collision.");

  nb::class_<Limits>(m, "ParameterizationLimits")
      .def(nb::init<>())
      .def_rw("max_velocity", &Limits::max_velocity)
      .def_rw("max_acceleration", &Limits::max_acceleration)
      .def_rw("max_jerk", &Limits::max_jerk)
      .def_rw("tip_linear_velocity", &Limits::tip_linear_velocity,
              "m/s, Euclidean norm; 0 = off. Time-optimal profiles ride this bound (near-"
              "constant tool speed along the stroke).")
      .def_rw("tip_angular_velocity", &Limits::tip_angular_velocity)
      .def_rw("tip_linear_acceleration", &Limits::tip_linear_acceleration,
              "m/s^2, applied per axis as limit/sqrt(3) (box inscribed in the sphere); 0 = off.")
      .def_rw("tip_angular_acceleration", &Limits::tip_angular_acceleration)
      .def_rw("tip_link", &Limits::tip_link);

  m.def("limits_from_model", &limits_from_model, "model"_a, "task"_a = planning::TaskLimits{},
        "default_acceleration"_a = 10.0,
        "Limits from the URDF (+ yaml accel/jerk extension) and the problem's TaskLimits.");

  nb::enum_<ParameterizationOptions::Mode>(m, "ParameterizationMode")
      .value("ConvexOnly", ParameterizationOptions::Mode::ConvexOnly)
      .value("JerkLimited", ParameterizationOptions::Mode::JerkLimited);

  nb::class_<ParameterizationOptions>(m, "ParameterizationOptions")
      .def(nb::init<>())
      .def_rw("nodes", &ParameterizationOptions::nodes)
      .def_rw("eps", &ParameterizationOptions::eps)
      .def_rw("max_jerk_passes", &ParameterizationOptions::max_jerk_passes,
              "JerkLimited: velocity-reduction kernel pass budget.")
      .def_rw("jerk_tolerance", &ParameterizationOptions::jerk_tolerance,
              "JerkLimited: accepted relative violation (certified max |jerk|/j_max - 1).")
      .def_rw("mode", &ParameterizationOptions::mode);

  nb::class_<ParameterizationResult>(m, "ParameterizationResult")
      .def_ro("success", &ParameterizationResult::success)
      .def_ro("message", &ParameterizationResult::message)
      .def_ro("trajectory", &ParameterizationResult::trajectory,
              "Timed waypoints: state.pos/vel/acc + time (seconds).")
      .def_ro("duration", &ParameterizationResult::duration)
      .def_ro("s", &ParameterizationResult::s)
      .def_ro("beta", &ParameterizationResult::beta)
      .def_ro("jerk_passes", &ParameterizationResult::jerk_passes)
      .def_ro("max_jerk_violation", &ParameterizationResult::max_jerk_violation)
      .def("__repr__", [](const ParameterizationResult &r) {
        return nb::str("ParameterizationResult(success={}, duration={:.4f}s, nodes={})")
            .format(r.success, r.duration, r.trajectory.size());
      });

  m.def("parametrize", &parametrize, "model"_a, "path"_a, "limits"_a,
        "options"_a = ParameterizationOptions{}, nb::call_guard<nb::gil_scoped_release>(),
        "Time-optimal parameterization of a PathSpline under joint vel/acc(/jerk) and tip "
        "vel/acc limits. Rest-to-rest; deterministic; no collision checks (validate the path "
        "first — see fit_collision_free).");
}
