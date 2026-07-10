// planning/types — JointGoal satisfaction, status naming, and problem validation (Task 3.1).
#include "quevedomp/planning/types.hpp"

#include <cmath>
#include <cstddef>

namespace quevedomp::planning {

bool JointGoal::satisfies(const JointPosition &q) const noexcept {
  if (q.size() != target.size()) {
    return false;
  }
  return (q - target).cwiseAbs().maxCoeff() <= tolerance;
}

const char *to_string(PlanningStatus s) noexcept {
  switch (s) {
  case PlanningStatus::Success:
    return "Success";
  case PlanningStatus::Timeout:
    return "Timeout";
  case PlanningStatus::NoSolution:
    return "NoSolution";
  case PlanningStatus::InvalidProblem:
    return "InvalidProblem";
  }
  return "Unknown";
}

namespace {

// Per-dof position limits, indexed by Joint::dof_index (size == model.dof()). Fixed joints carry
// no dof index and are skipped; a joint without a position limit (Continuous) gets has_limit=false.
struct DofLimits {
  double lower = 0.0;
  double upper = 0.0;
  bool has_limit = false;
};

std::vector<DofLimits> dof_limits(const RobotModel &model) {
  std::vector<DofLimits> limits(model.dof());
  for (const auto &joint : model.joints()) {
    if (!joint.is_movable() || joint.dof_index < 0) {
      continue;
    }
    const auto idx = static_cast<std::size_t>(joint.dof_index);
    if (idx < limits.size()) {
      limits[idx] = {joint.limits.lower, joint.limits.upper, joint.limits.has_position_limit};
    }
  }
  return limits;
}

// Small slack so a configuration sitting exactly on a limit (a common IK/goal case) validates
// despite floating-point round-off.
constexpr double kLimitEps = 1e-9;

// Verify a config has the right size and lies within the per-dof position limits. Returns a
// reason (with the `what` label) if not, else nullopt.
std::optional<std::string> check_config(const JointPosition &q, const std::vector<DofLimits> &lim,
                                        const char *what) {
  if (static_cast<std::size_t>(q.size()) != lim.size()) {
    return std::string(what) + " has " + std::to_string(q.size()) + " values but the model has " +
           std::to_string(lim.size()) + " DOF";
  }
  for (std::size_t i = 0; i < lim.size(); ++i) {
    if (!lim[i].has_limit) {
      continue;
    }
    if (q[static_cast<Eigen::Index>(i)] < lim[i].lower - kLimitEps ||
        q[static_cast<Eigen::Index>(i)] > lim[i].upper + kLimitEps) {
      return std::string(what) + " joint " + std::to_string(i) + " = " +
             std::to_string(q[static_cast<Eigen::Index>(i)]) + " is outside limits [" +
             std::to_string(lim[i].lower) + ", " + std::to_string(lim[i].upper) + "]";
    }
  }
  return std::nullopt;
}

std::optional<std::string> validate_goal(const Goal &goal, const RobotModel &model,
                                         const std::vector<DofLimits> &lim) {
  switch (goal.type()) {
  case GoalType::Joint: {
    const auto &jg = static_cast<const JointGoal &>(goal);
    if (jg.tolerance < 0.0) {
      return "JointGoal tolerance is negative";
    }
    return check_config(jg.target, lim, "JointGoal target");
  }
  case GoalType::Pose: {
    const auto &pg = static_cast<const PoseGoal &>(goal);
    if (!pg.tip_link.empty() && model.find_link(pg.tip_link) == nullptr) {
      return "PoseGoal tip_link '" + pg.tip_link + "' is not a link of the model";
    }
    return std::nullopt;
  }
  case GoalType::Multi: {
    const auto &mg = static_cast<const MultiGoal &>(goal);
    if (mg.goals.empty()) {
      return "MultiGoal has no sub-goals";
    }
    for (std::size_t i = 0; i < mg.goals.size(); ++i) {
      if (mg.goals[i] == nullptr) {
        return "MultiGoal sub-goal " + std::to_string(i) + " is null";
      }
      if (auto reason = validate_goal(*mg.goals[i], model, lim)) {
        return "MultiGoal sub-goal " + std::to_string(i) + ": " + *reason;
      }
    }
    return std::nullopt;
  }
  }
  return "unknown goal type";
}

} // namespace

std::optional<std::string> validate(const PlanningProblem &problem, const RobotModel &model) {
  const auto lim = dof_limits(model);

  if (auto reason = check_config(problem.start, lim, "start")) {
    return reason;
  }

  if (problem.goal == nullptr) {
    return "problem has no goal";
  }
  if (auto reason = validate_goal(*problem.goal, model, lim)) {
    return reason;
  }

  if (!problem.constraints.joint_bounds.empty()) {
    if (problem.constraints.joint_bounds.size() != lim.size()) {
      return "constraints.joint_bounds has " +
             std::to_string(problem.constraints.joint_bounds.size()) +
             " entries but the model has " + std::to_string(lim.size()) + " DOF";
    }
    for (std::size_t i = 0; i < lim.size(); ++i) {
      const auto &[lo, hi] = problem.constraints.joint_bounds[i];
      if (lo > hi) {
        return "constraints.joint_bounds[" + std::to_string(i) + "] is empty (lower > upper)";
      }
    }
  }

  if (!(problem.timeout > 0.0)) {
    return "timeout must be positive";
  }

  return std::nullopt;
}

} // namespace quevedomp::planning
