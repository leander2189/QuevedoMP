// bindings/python/bind_planning — planner + smoother (Task 4a.5, ADR-016).
//
// The verb-level boundary of the whole slice: Planner.plan and Smoother.smooth are single C++
// calls that release the GIL, so an IDE can plan on a worker thread while the UI renders
// (Planner::plan is const + reentrant per the planner contract). Goals are polymorphic
// (shared_ptr<const Goal>) so a Python JointGoal/PoseGoal/MultiGoal plugs into
// PlanningProblem.goal directly.
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include "quevedomp/clearance/clearance_field.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/refiner.hpp"
#include "quevedomp/planning/smoother.hpp"
#include "quevedomp/planning/types.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace quevedomp;
using namespace quevedomp::planning;

void bind_planning(nb::module_ &m) {
  // ---- Goals ---------------------------------------------------------------------------------
  nb::class_<Goal>(m, "Goal", "Polymorphic goal description; see JointGoal/PoseGoal/MultiGoal.");

  nb::class_<JointGoal, Goal>(m, "JointGoal",
                              "Reach a target configuration within a per-joint tolerance.")
      .def(nb::init<JointPosition, double>(), "target"_a, "tolerance"_a = 1e-3)
      .def_rw("target", &JointGoal::target)
      .def_rw("tolerance", &JointGoal::tolerance)
      .def("satisfies", &JointGoal::satisfies, "q"_a);

  nb::class_<PoseGoal, Goal>(m, "PoseGoal",
                             "Reach a Cartesian pose of `tip_link` (empty = the model tip).")
      .def(nb::init<Pose, std::string>(), "target"_a, "tip_link"_a = std::string())
      .def_rw("target", &PoseGoal::target)
      .def_rw("tip_link", &PoseGoal::tip_link);

  nb::class_<MultiGoal, Goal>(m, "MultiGoal", "Disjunction: satisfied when ANY sub-goal is.")
      .def(nb::init<std::vector<std::shared_ptr<const Goal>>>(), "goals"_a)
      .def_rw("goals", &MultiGoal::goals);

  // ---- Problem -------------------------------------------------------------------------------
  nb::class_<Constraints>(m, "Constraints")
      .def(nb::init<>())
      .def_rw("joint_bounds", &Constraints::joint_bounds,
              "Optional per-joint (lower, upper) pairs, intersected with the URDF limits; "
              "empty = URDF limits unchanged.");

  nb::class_<TaskLimits>(m, "TaskLimits", "TCP Cartesian limits for parameterization; 0 = off.")
      .def(nb::init<>())
      .def_rw("max_linear_velocity", &TaskLimits::max_linear_velocity)
      .def_rw("max_linear_acceleration", &TaskLimits::max_linear_acceleration)
      .def_rw("max_angular_velocity", &TaskLimits::max_angular_velocity)
      .def_rw("max_angular_acceleration", &TaskLimits::max_angular_acceleration)
      .def_rw("frame", &TaskLimits::frame);

  nb::class_<PlanningProblem>(m, "PlanningProblem")
      .def(nb::init<>())
      .def_rw("start", &PlanningProblem::start)
      .def_rw("goal", &PlanningProblem::goal)
      .def_rw("constraints", &PlanningProblem::constraints)
      .def_rw("task_limits", &PlanningProblem::task_limits)
      .def_rw("collision", &PlanningProblem::collision)
      .def_rw("timeout", &PlanningProblem::timeout)
      // optional<> setters must explicitly allow None (nanobind rejects it otherwise).
      .def_rw("seed", &PlanningProblem::seed, nb::for_setter(nb::arg("value").none()),
              "Fixed seed for determinism; None = auto-generated.");

  m.def("validate", &validate, "problem"_a, "model"_a,
        "Structural check; returns a human-readable reason if ill-formed, else None.");

  // ---- Result + stats ------------------------------------------------------------------------
  nb::enum_<PlanningStatus>(m, "PlanningStatus")
      .value("Success", PlanningStatus::Success)
      .value("Timeout", PlanningStatus::Timeout)
      .value("NoSolution", PlanningStatus::NoSolution)
      .value("InvalidProblem", PlanningStatus::InvalidProblem);

  nb::class_<PlanningStats>(m, "PlanningStats")
      .def_ro("collision_queries", &PlanningStats::collision_queries)
      .def_ro("collision_configs", &PlanningStats::collision_configs)
      .def_ro("batch_size_histogram", &PlanningStats::batch_size_histogram)
      .def_ro("iterations", &PlanningStats::iterations)
      .def_ro("refiner_mode", &PlanningStats::refiner_mode,
              "Which R4 refiner mode ran ('refiner' | 'standalone'); empty for sampling planners.")
      .def_ro("time_collision", &PlanningStats::time_collision)
      .def_ro("time_planner", &PlanningStats::time_planner)
      .def_ro("time_smoothing", &PlanningStats::time_smoothing)
      .def_ro("time_parameterization", &PlanningStats::time_parameterization)
      .def_ro("time_first_solution", &PlanningStats::time_first_solution)
      .def_ro("time_total", &PlanningStats::time_total);

  nb::class_<TreeSnapshot>(m, "TreeSnapshot",
                           "One search tree, snapshotted at plan exit (record_tree). Parents "
                           "precede children; parent -1 marks a root.")
      .def_ro("nodes", &TreeSnapshot::nodes)
      .def_ro("parents", &TreeSnapshot::parents);

  nb::class_<PlanningResult>(m, "PlanningResult")
      .def_ro("status", &PlanningResult::status)
      .def_ro("path", &PlanningResult::path, "Joint-space waypoints as a list of (dof,) arrays.")
      .def_ro("stats", &PlanningResult::stats)
      .def_ro("used_seed", &PlanningResult::used_seed)
      .def_ro("message", &PlanningResult::message)
      .def_ro("trees", &PlanningResult::trees,
              "[start, goal] TreeSnapshots when PlannerParams.record_tree was set, else empty.")
      .def("ok", &PlanningResult::ok)
      .def(
          "path_array",
          [](const PlanningResult &r) {
            const std::size_t n = r.path.size();
            const std::size_t dof = n ? static_cast<std::size_t>(r.path[0].size()) : 0;
            double *data = new double[n * dof];
            for (std::size_t i = 0; i < n; ++i)
              for (std::size_t j = 0; j < dof; ++j)
                data[i * dof + j] = r.path[i][static_cast<Eigen::Index>(j)];
            nb::capsule owner(data, [](void *p) noexcept { delete[] static_cast<double *>(p); });
            return nb::ndarray<double, nb::numpy, nb::shape<-1, -1>>(data, {n, dof}, owner);
          },
          "The path as one (N, dof) float64 array (a copy).")
      .def("__repr__", [](const PlanningResult &r) {
        return nb::str("PlanningResult(status={}, waypoints={}, used_seed={}, message='{}')")
            .format(to_string(r.status), r.path.size(), r.used_seed, r.message);
      });

  // ---- Planner -------------------------------------------------------------------------------
  nb::class_<PlannerParams>(m, "PlannerParams")
      .def(nb::init<>())
      .def_rw("algorithm", &PlannerParams::algorithm)
      .def_rw("edge_resolution", &PlannerParams::edge_resolution)
      .def_rw("max_link_sweep", &PlannerParams::max_link_sweep,
              "P3: > 0 (metres) bounds how far any robot point moves between edge samples, "
              "replacing edge_resolution; needs lever_weights (see cartesian_lever_weights).")
      .def_rw("lever_weights", &PlannerParams::lever_weights,
              "Per-dof lever weights for max_link_sweep; empty = computed by make_planner "
              "(precompute for robots whose mesh URIs need package dirs).")
      .def_rw("max_extension", &PlannerParams::max_extension)
      .def_rw("goal_bias", &PlannerParams::goal_bias)
      .def_rw("batch_size", &PlannerParams::batch_size)
      .def_rw("max_iterations", &PlannerParams::max_iterations)
      .def_rw("record_tree", &PlannerParams::record_tree,
              "Copy the final search trees into PlanningResult.trees at plan exit (one copy, "
              "zero growth-loop cost). Debug/visualization; off by default.");

  nb::class_<Planner>(m, "Planner",
                      "plan() is const + reentrant: distinct Python threads may plan "
                      "concurrently on one planner (one plan per thread).")
      .def("plan", &Planner::plan, "problem"_a, nb::call_guard<nb::gil_scoped_release>());

  m.def("make_planner", &make_planner, "params"_a, "robot"_a, "scene"_a,
        "Build the planner named by params.algorithm (raises on an unknown id — no fallback).");
  m.def("registered_planners", &registered_planners);

  // ---- Smoother ------------------------------------------------------------------------------
  nb::class_<SmootherParams>(m, "SmootherParams")
      .def(nb::init<>())
      .def_rw("edge_resolution", &SmootherParams::edge_resolution)
      .def_rw("max_link_sweep", &SmootherParams::max_link_sweep,
              "Match the planner's max_link_sweep so shortcuts validate at the same fidelity.")
      .def_rw("lever_weights", &SmootherParams::lever_weights)
      .def_rw("max_iterations", &SmootherParams::max_iterations)
      .def_rw("batch_size", &SmootherParams::batch_size,
              "Disjoint candidate chords validated per collision round (ONE query_batch); "
              "1 = the classic serial shortcut.")
      .def_rw("time_budget", &SmootherParams::time_budget,
              "Wall-clock polish budget in seconds, checked per round; 0 = unlimited.")
      .def_rw("seed", &SmootherParams::seed)
      .def_rw("collision", &SmootherParams::collision);

  nb::class_<Smoother>(m, "Smoother")
      .def("smooth", &Smoother::smooth, "path"_a, nb::call_guard<nb::gil_scoped_release>(),
           "Shortcut the path; output is collision-free and never longer than the input.");

  m.def("make_shortcut_smoother", &make_shortcut_smoother, "params"_a, "robot"_a, "scene"_a);

  // ---- Refiner (roadmap R4) ------------------------------------------------------------------
  nb::class_<RefinerParams>(m, "RefinerParams",
                            "CHOMP/TrajOpt refiner config. seed non-empty = refiner mode "
                            "(polish a feasible trajectory); empty = standalone (straight-line "
                            "guess to a resolved goal).")
      .def(nb::init<>())
      .def_rw("seed", &RefinerParams::seed,
              "Feasible trajectory to polish (list of (dof,) arrays); empty = standalone mode. "
              "Its endpoints are the fixed start/goal of the optimization.")
      .def_rw("waypoints", &RefinerParams::waypoints,
              "Optimized trajectory length; the seed/straight line is resampled to this many.")
      .def_rw("max_iterations", &RefinerParams::max_iterations)
      .def_rw("smoothness_weight", &RefinerParams::smoothness_weight)
      .def_rw("clearance_weight", &RefinerParams::clearance_weight)
      .def_rw("clearance_epsilon", &RefinerParams::clearance_epsilon,
              "CHOMP hinge width (m): obstacle cost is 0 beyond this clearance.")
      .def_rw("step_size", &RefinerParams::step_size,
              "Update step on the A^-1-preconditioned gradient (the metric that avoids kinks).")
      .def_rw("convergence_tol", &RefinerParams::convergence_tol)
      .def_rw("edge_resolution", &RefinerParams::edge_resolution)
      .def_rw("max_link_sweep", &RefinerParams::max_link_sweep)
      .def_rw("lever_weights", &RefinerParams::lever_weights)
      .def_rw("collision", &RefinerParams::collision)
      .def_rw("rng_seed", &RefinerParams::rng_seed,
              "Seed for standalone goal IK; CHOMP is RNG-free.");

  m.def("make_refiner", &make_refiner, "params"_a, "robot"_a, "scene"_a, "field"_a, "spheres"_a,
        "Build the R4 optimization refiner over a ClearanceField (R3) + robot sphere cover. The "
        "output is CERTIFIED collision-free by the exact scene backend — the field is never the "
        "certificate. A first-class Planner: call plan(problem).");
}
