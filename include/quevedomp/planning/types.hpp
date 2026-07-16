// planning/types — value types crossing the planning boundary (Task 3.1, spec §6 Phase 3).
//
// Pure data + a small amount of self-contained behavior (JointGoal satisfaction, problem
// validation). Model-coupled logic (PoseGoal satisfaction via FK, goal sampling via IK) lives in
// the planner (Task 3.2), so these types stay dependency-light: they lean only on core/types,
// robot_model (for validation), and collision::QueryOptions (the collision options a problem
// carries). No planner, smoother, or backend type appears here.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "quevedomp/collision/types.hpp" // QueryOptions carried by a PlanningProblem
#include "quevedomp/core/types.hpp"      // JointPosition, Pose, Transform
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::planning {

// A joint-space path: an ordered list of configurations WITHOUT timing. This is what a planner
// produces and a smoother consumes; TOPP-RA (Task 3.4) turns it into a timed core::Trajectory.
using Path = std::vector<JointPosition>;

// ---- Goals -------------------------------------------------------------------------------
// A polymorphic goal specification (spec §6: "Goal base with JointGoal, PoseGoal, MultiGoal").
// Goals carry only the DESCRIPTION of the target region; a planner interprets them (JointGoal is
// self-contained; PoseGoal needs FK; MultiGoal is a disjunction). Immutable once built — held by
// shared_ptr<const Goal> in a PlanningProblem so copies (for capture) are cheap.
enum class GoalType { Joint, Pose, Multi };

class Goal {
public:
  virtual ~Goal() = default;
  [[nodiscard]] virtual GoalType type() const noexcept = 0;
  [[nodiscard]] virtual std::unique_ptr<Goal> clone() const = 0;
};

// Reach a target configuration within a per-joint tolerance (infinity-norm). Self-contained:
// satisfaction needs no robot model, so the planner can test it directly.
class JointGoal : public Goal {
public:
  JointGoal() = default;
  explicit JointGoal(JointPosition target_, double tolerance_ = 1e-3)
      : target(std::move(target_)), tolerance(tolerance_) {}

  JointPosition target;    // size must equal model.dof()
  double tolerance = 1e-3; // max |q_i - target_i| for every joint (rad / m)

  [[nodiscard]] GoalType type() const noexcept override { return GoalType::Joint; }
  [[nodiscard]] std::unique_ptr<Goal> clone() const override {
    return std::make_unique<JointGoal>(*this);
  }
  // True iff q has the same size as target and stays within `tolerance` on every joint.
  [[nodiscard]] bool satisfies(const JointPosition &q) const noexcept;
};

// Reach a Cartesian pose of `tip_link` (empty ⇒ the model's tip). The Pose carries its own
// position/orientation tolerances. Satisfaction/sampling need FK/IK, so they live in the planner.
class PoseGoal : public Goal {
public:
  PoseGoal() = default;
  explicit PoseGoal(Pose target_, std::string tip_link_ = {})
      : target(std::move(target_)), tip_link(std::move(tip_link_)) {}

  Pose target;          // desired pose of tip_link in the base frame (with pos/rot tolerances)
  std::string tip_link; // link whose frame the pose constrains; empty ⇒ model tip link

  [[nodiscard]] GoalType type() const noexcept override { return GoalType::Pose; }
  [[nodiscard]] std::unique_ptr<Goal> clone() const override {
    return std::make_unique<PoseGoal>(*this);
  }
};

// A disjunction: satisfied when ANY sub-goal is. Used for multi-target reaching (e.g. several IK
// solutions of one pose, or alternative grasp configs).
class MultiGoal : public Goal {
public:
  MultiGoal() = default;
  explicit MultiGoal(std::vector<std::shared_ptr<const Goal>> goals_) : goals(std::move(goals_)) {}

  std::vector<std::shared_ptr<const Goal>> goals; // sub-goals share ownership (immutable)

  [[nodiscard]] GoalType type() const noexcept override { return GoalType::Multi; }
  [[nodiscard]] std::unique_ptr<Goal> clone() const override {
    return std::make_unique<MultiGoal>(*this); // shallow: sub-goals are const, sharing is safe
  }
};

// ---- Task-space limits -------------------------------------------------------------------
// TCP Cartesian limits (spec §6: "TCP vel/acc + frame"). Consumed by TOPP-RA (Task 3.4) to bound
// the end-effector, on top of the per-joint limits already in RobotModel. 0 ⇒ unconstrained.
struct TaskLimits {
  double max_linear_velocity = 0.0;      // m/s
  double max_linear_acceleration = 0.0;  // m/s²
  double max_angular_velocity = 0.0;     // rad/s
  double max_angular_acceleration = 0.0; // rad/s²
  std::string frame;                     // link whose origin is the TCP; empty ⇒ model tip link
};

// ---- Path constraints --------------------------------------------------------------------
// Constraints that hold along the whole path (spec §6). v0 supports sampling-space narrowing:
// per-joint bounds tighter than the URDF limits (intersected with them). Cartesian path
// constraints (keep-upright, TCP box) are [DEFERRED] — the struct is the extension point.
struct Constraints {
  // Optional per-joint [lower, upper] override. Empty ⇒ use the model's URDF limits unchanged.
  // If non-empty, size must equal model.dof(); each bound is intersected with the URDF limit.
  std::vector<std::pair<double, double>> joint_bounds;
};

// ---- Problem -----------------------------------------------------------------------------
struct PlanningProblem {
  JointPosition start;               // size must equal model.dof()
  std::shared_ptr<const Goal> goal;  // required; a disjunction/pose/joint target
  Constraints constraints;           // path constraints (v0: joint-bound narrowing)
  TaskLimits task_limits;            // TCP limits for parameterization
  collision::QueryOptions collision; // options passed through to every collision query
  double timeout = 1.0;              // wall-clock budget in seconds (> 0)
  std::optional<std::uint64_t> seed; // fixed seed for determinism; nullopt ⇒ auto-generated
};

// ---- Result + stats ----------------------------------------------------------------------
enum class PlanningStatus { Success, Timeout, NoSolution, InvalidProblem };

[[nodiscard]] const char *to_string(PlanningStatus) noexcept;

// Attribution stats every planner populates alongside its result (Task 3.1, 2026-07-08 note).
// The performance contract is unenforceable without this: when a planner is slow, the batch-size
// histogram shows at a glance whether it starved the GPU with thin batches or burned the budget
// in its own logic. Cheap to carry, painful to retrofit.
struct PlanningStats {
  std::uint64_t collision_queries = 0; // number of query_batch / check_edge calls issued
  std::uint64_t collision_configs = 0; // total configurations checked (sum of batch sizes)
  // batch size → how many queries used that size (the histogram the contract is measured on).
  std::map<std::size_t, std::uint64_t> batch_size_histogram;
  std::uint64_t iterations = 0; // nodes expanded (sampling) or iterations (optimization)

  // Time split, seconds. planner logic / collision / smoothing / parameterization are disjoint;
  // total is end-to-end wall time; first_solution is wall time to the first feasible path.
  double time_collision = 0.0;
  double time_planner = 0.0;
  double time_smoothing = 0.0;
  double time_parameterization = 0.0;
  double time_first_solution = 0.0;
  double time_total = 0.0;

  // Record one collision batch (keeps `collision_queries`, `collision_configs`, and the
  // histogram consistent — the single place planners bump collision accounting).
  void record_batch(std::size_t batch_size) noexcept {
    ++collision_queries;
    collision_configs += batch_size;
    ++batch_size_histogram[batch_size];
  }
};

// One planner search tree, snapshotted AFTER planning when PlannerParams::record_tree is set
// (roadmap R2). Data export, not instrumentation: a single copy at exit, zero hot-loop cost —
// deliberately NOT the deferred PlanningTrace (live per-iteration tracing stays out of the core).
struct TreeSnapshot {
  std::vector<JointPosition> nodes; // configs, in insertion order (parents precede children)
  std::vector<int> parents;         // parallel to nodes; -1 marks a root
};

struct PlanningResult {
  PlanningStatus status = PlanningStatus::NoSolution;
  Path path;                   // joint-space waypoints (untimed); empty unless Success
  PlanningStats stats;         // attribution for the run
  std::uint64_t used_seed = 0; // ALWAYS populated, passed or auto-generated (spec §5.2)
  std::string message;         // human-readable detail (esp. why a problem was InvalidProblem)

  // Filled only when PlannerParams::record_tree: [start tree, goal tree] for RRT-Connect
  // (empty on pre-search failures — invalid problem, colliding start, IK-less goal).
  std::vector<TreeSnapshot> trees;

  [[nodiscard]] bool ok() const noexcept { return status == PlanningStatus::Success; }
};

// ---- Validation --------------------------------------------------------------------------
// Structural check of a problem against a model (DOF, joint limits, goal well-formedness,
// timeout). Returns a human-readable reason if ill-formed, or std::nullopt if valid. Planners
// call this first; a failing problem yields PlanningStatus::InvalidProblem WITHOUT running the
// search (spec §6 exit: "invalid-problem detection"). Cheap, side-effect-free.
[[nodiscard]] std::optional<std::string> validate(const PlanningProblem &problem,
                                                  const RobotModel &model);

} // namespace quevedomp::planning
