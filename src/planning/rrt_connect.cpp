// planning/RrtConnectPlanner — batched bidirectional RRT-Connect (Task 3.2, spec §6 Phase 3).
//
// v0 sampling planner, chosen to validate the Planner interface + performance contract and give
// the Task 3.5 goal gate a reference number (NOT a commitment to RRT). It obeys the contract:
//   • Batch-first collision: every collision goes through ONE CollisionScene::query_batch per
//     growth/connect step — `batch_size` candidate extensions are discretized and concatenated
//     into a single batch (contract item 1), never a per-config query().
//   • Determinism per seed: a single Rng(used_seed) drives all sampling single-threaded, so the
//     same problem + seed yields the same tree and path (contract item 3).
//   • No hidden mutable state: plan() owns its trees + a per-call Workspace (contract item 4).
//
// Design vs. classic serial RRT-Connect: growth validates a *batch* of candidate extensions at
// once, and "connect" validates the full bridge edge (new node → nearest node of the other tree)
// at edge_resolution — a collision-free bridge IS a successful greedy connect. Both keep the
// collision batches fat enough to cross the hybrid-Auto GPU threshold.
#include "planners_internal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <span>
#include <utility>
#include <vector>

#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/rng.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/planning/types.hpp"

namespace quevedomp::planning::detail {
namespace {

using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

// Per-dof sampling bounds: the URDF position limits (or ±π for a continuous joint), intersected
// with any tighter per-joint bounds from Constraints.
struct Bounds {
  JointPosition lo, hi;
};

Bounds compute_bounds(const RobotModel &model, const Constraints &c) {
  const std::size_t dof = model.dof();
  Bounds b;
  b.lo = JointPosition::Constant(static_cast<Eigen::Index>(dof), -kPi);
  b.hi = JointPosition::Constant(static_cast<Eigen::Index>(dof), kPi);
  for (const auto &joint : model.joints()) {
    if (!joint.is_movable() || joint.dof_index < 0) {
      continue;
    }
    const auto i = static_cast<Eigen::Index>(joint.dof_index);
    if (i >= 0 && i < static_cast<Eigen::Index>(dof) && joint.limits.has_position_limit) {
      b.lo[i] = joint.limits.lower;
      b.hi[i] = joint.limits.upper;
    }
  }
  if (!c.joint_bounds.empty() && c.joint_bounds.size() == dof) {
    for (std::size_t k = 0; k < dof; ++k) {
      const auto i = static_cast<Eigen::Index>(k);
      b.lo[i] = std::max(b.lo[i], c.joint_bounds[k].first);
      b.hi[i] = std::min(b.hi[i], c.joint_bounds[k].second);
    }
  }
  return b;
}

// Steer from `from` toward `to`, clamped to at most `max_ext` in joint-space L2.
JointPosition steer(const JointPosition &from, const JointPosition &to, double max_ext) {
  const JointPosition d = to - from;
  const double n = d.norm();
  if (n <= max_ext || n == 0.0) {
    return to;
  }
  return from + (max_ext / n) * d;
}

// A single-rooted or multi-rooted tree over joint configs. `parent[i] == -1` marks a root.
struct Tree {
  std::vector<JointPosition> q;
  std::vector<int> parent;

  int add(const JointPosition &cfg, int par) {
    q.push_back(cfg);
    parent.push_back(par);
    return static_cast<int>(q.size()) - 1;
  }
  // Nearest node by joint-space L2 (linear scan — fine for v0 tree sizes).
  int nearest(const JointPosition &cfg) const {
    int best = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(q.size()); ++i) {
      const double d = (q[i] - cfg).squaredNorm();
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    return best;
  }
};

// One edge to validate.
struct EdgeReq {
  JointPosition q0, q1;
};

// Validate a set of edges with ONE query_batch, returning per-edge first-contact t (1.0 = free).
// Each edge is discretized exactly as collision::check_edge (n = ceil(max|Δq|/res), n+1 inclusive
// samples); all edges' samples are concatenated so the backend sees one fat batch. Bumps the
// collision stats + timer. Empty input ⇒ no query issued.
std::vector<float> batch_check(std::span<const EdgeReq> edges,
                               const collision::CollisionScene &scene, const RobotInstance &robot,
                               const collision::QueryOptions &opts, collision::Workspace &ws,
                               double resolution, PlanningStats &stats, double &t_collision) {
  std::vector<float> result(edges.size(), 1.0f);
  if (edges.empty()) {
    return result;
  }

  std::vector<JointPosition> samples;
  std::vector<std::pair<std::size_t, int>> slices; // (offset, n) per edge
  slices.reserve(edges.size());
  for (const auto &e : edges) {
    const JointPosition delta = e.q1 - e.q0;
    const double max_step = delta.size() > 0 ? delta.cwiseAbs().maxCoeff() : 0.0;
    const int n = std::max(1, static_cast<int>(std::ceil(max_step / resolution)));
    slices.emplace_back(samples.size(), n);
    for (int k = 0; k <= n; ++k) {
      samples.push_back(e.q0 + (static_cast<double>(k) / n) * delta);
    }
  }

  const auto t0 = Clock::now();
  const collision::BatchResult batch = scene.query_batch(robot, samples, opts, ws);
  t_collision += std::chrono::duration<double>(Clock::now() - t0).count();
  stats.record_batch(samples.size());

  for (std::size_t e = 0; e < edges.size(); ++e) {
    const auto [off, n] = slices[e];
    for (int k = 0; k <= n; ++k) {
      if (batch.in_collision[off + static_cast<std::size_t>(k)] != 0) {
        result[e] = static_cast<float>(static_cast<double>(k) / n);
        break;
      }
    }
  }
  return result;
}

// Resolve a goal into concrete goal configurations (the roots of the goal tree). JointGoal is its
// target; PoseGoal is solved by numerical IK (a few deterministic seeds); MultiGoal unions its
// sub-goals. May be empty (e.g. IK failed) ⇒ the caller reports NoSolution.
std::vector<JointPosition> resolve_goal(const Goal &goal, const RobotModel &model,
                                        std::shared_ptr<const RobotModel> model_ptr,
                                        std::uint64_t seed) {
  std::vector<JointPosition> out;
  switch (goal.type()) {
  case GoalType::Joint:
    out.push_back(static_cast<const JointGoal &>(goal).target);
    break;
  case GoalType::Pose: {
    const auto &pg = static_cast<const PoseGoal &>(goal);
    const std::string tip = pg.tip_link.empty() ? model.links().back().name : pg.tip_link;
    // A handful of seeded IK attempts; collect distinct successes (redundant arms have many).
    for (int attempt = 0; attempt < 4; ++attempt) {
      IkOptions io;
      io.seed = seed + static_cast<std::uint64_t>(attempt) * 0x9E3779B97F4A7C15ULL + 1;
      io.pos_tol = pg.target.pos_tol;
      io.rot_tol = pg.target.rot_tol;
      const auto ik = make_numerical_ik(model_ptr, io);
      const IkResult r = ik->solve(tip, pg.target.tf);
      if (r.success) {
        const bool dup = std::any_of(out.begin(), out.end(), [&](const JointPosition &q) {
          return q.size() == r.q.size() && (q - r.q).cwiseAbs().maxCoeff() < 1e-3;
        });
        if (!dup) {
          out.push_back(r.q);
        }
      }
    }
    break;
  }
  case GoalType::Multi:
    for (const auto &sub : static_cast<const MultiGoal &>(goal).goals) {
      if (sub) {
        auto sub_cfgs = resolve_goal(*sub, model, model_ptr, seed);
        out.insert(out.end(), sub_cfgs.begin(), sub_cfgs.end());
      }
    }
    break;
  }
  return out;
}

// Root-ward walk: [i, parent(i), ..., root].
std::vector<JointPosition> ascend(const Tree &t, int i) {
  std::vector<JointPosition> v;
  while (i >= 0) {
    v.push_back(t.q[i]);
    i = t.parent[i];
  }
  return v;
}

// Stitch start_tree[s_idx]…start_root (reversed) with goal_tree[g_idx]…goal_root; the bridge edge
// s_idx↔g_idx is already validated collision-free by the caller.
Path build_path(const Tree &start_tree, int s_idx, const Tree &goal_tree, int g_idx) {
  Path start_side = ascend(start_tree, s_idx);        // s_idx..start_root
  std::reverse(start_side.begin(), start_side.end()); // start_root..s_idx
  const Path goal_side = ascend(goal_tree, g_idx);    // g_idx..goal_root
  start_side.insert(start_side.end(), goal_side.begin(), goal_side.end());
  return start_side;
}

class RrtConnectPlanner final : public Planner {
public:
  RrtConnectPlanner(PlannerParams params, std::shared_ptr<const RobotInstance> robot,
                    std::shared_ptr<const collision::CollisionScene> scene)
      : params_(std::move(params)), robot_(std::move(robot)), scene_(std::move(scene)) {}

  PlanningResult plan(const PlanningProblem &problem) const override {
    const auto t_begin = Clock::now();
    const RobotModel &model = robot_->model();

    PlanningResult result;
    // Per-call scratch (locals ⇒ plan() carries no state between calls; reentrant, contract item
    // 4).
    std::uint64_t iterations = 0;
    double t_collision = 0.0;

    const std::uint64_t used_seed = problem.seed.value_or(
        static_cast<std::uint64_t>(std::random_device{}()) << 32 | std::random_device{}());
    result.used_seed = used_seed;

    auto finish = [&](PlanningStatus status, std::string msg) {
      result.status = status;
      result.message = std::move(msg);
      result.stats.iterations = iterations;
      result.stats.time_collision = t_collision;
      result.stats.time_total = std::chrono::duration<double>(Clock::now() - t_begin).count();
      result.stats.time_planner = std::max(0.0, result.stats.time_total - t_collision);
      if (status == PlanningStatus::Success) {
        result.stats.time_first_solution = result.stats.time_total;
      }
      return result;
    };

    if (auto reason = validate(problem, model)) {
      return finish(PlanningStatus::InvalidProblem, *reason);
    }

    auto goals = resolve_goal(*problem.goal, model, robot_->model_ptr(), used_seed);
    if (goals.empty()) {
      return finish(PlanningStatus::NoSolution, "no goal configuration (IK failed for the goal)");
    }

    const Bounds bounds = compute_bounds(model, problem.constraints);
    const auto ws = scene_->make_workspace();
    const collision::QueryOptions &opts = problem.collision;
    Rng rng(used_seed);

    // Feasibility probe: start + all goal configs in one batch. A colliding start ⇒ NoSolution;
    // colliding goals are dropped. (query_batch, not query — still batch-first.)
    {
      std::vector<JointPosition> probe;
      probe.reserve(goals.size() + 1);
      probe.push_back(problem.start);
      for (const auto &g : goals) {
        probe.push_back(g);
      }
      const auto t0 = Clock::now();
      const collision::BatchResult br = scene_->query_batch(*robot_, probe, opts, *ws);
      t_collision += std::chrono::duration<double>(Clock::now() - t0).count();
      result.stats.record_batch(probe.size());
      if (br.in_collision[0] != 0) {
        return finish(PlanningStatus::NoSolution, "start configuration is in collision");
      }
      std::vector<JointPosition> free_goals;
      for (std::size_t i = 0; i < goals.size(); ++i) {
        if (br.in_collision[i + 1] == 0) {
          free_goals.push_back(goals[i]);
        }
      }
      if (free_goals.empty()) {
        return finish(PlanningStatus::NoSolution, "all goal configurations are in collision");
      }
      goals = std::move(free_goals);
    }

    // Trees: start tree (single root) and goal tree (one root per goal config).
    Tree start_tree;
    start_tree.add(problem.start, -1);
    Tree goal_tree;
    for (const auto &g : goals) {
      goal_tree.add(g, -1);
    }

    const double res = params_.edge_resolution;
    auto check = [&](std::span<const EdgeReq> edges) {
      return batch_check(edges, *scene_, *robot_, opts, *ws, res, result.stats, t_collision);
    };

    // Quick win: direct start→goal edges.
    {
      std::vector<EdgeReq> direct;
      direct.reserve(goals.size());
      for (const auto &g : goals) {
        direct.push_back({problem.start, g});
      }
      const auto ts = check(direct);
      for (std::size_t i = 0; i < ts.size(); ++i) {
        if (ts[i] >= 1.0f) {
          result.path = {problem.start, goals[i]};
          return finish(PlanningStatus::Success, "solved by direct connection");
        }
      }
    }

    const double timeout = problem.timeout;
    bool active_is_start = true;
    while (std::chrono::duration<double>(Clock::now() - t_begin).count() < timeout &&
           iterations < params_.max_iterations) {
      ++iterations;
      Tree &active = active_is_start ? start_tree : goal_tree;
      Tree &other = active_is_start ? goal_tree : start_tree;

      // Grow the active tree with a batch of candidate extensions.
      std::vector<EdgeReq> grow;
      std::vector<int> parent;
      std::vector<JointPosition> newc;
      grow.reserve(params_.batch_size);
      for (std::size_t j = 0; j < params_.batch_size; ++j) {
        JointPosition target;
        if (rng.uniform(0.0, 1.0) < params_.goal_bias) {
          // Bias toward the other side's roots (a goal config when growing start, else start).
          if (active_is_start) {
            const auto idx =
                static_cast<std::size_t>(rng.uniform(0.0, static_cast<double>(goals.size())));
            target = goals[std::min(idx, goals.size() - 1)];
          } else {
            target = problem.start;
          }
        } else {
          target = rng.sample_in_box(bounds.lo, bounds.hi);
        }
        const int near = active.nearest(target);
        grow.push_back({active.q[near], steer(active.q[near], target, params_.max_extension)});
        parent.push_back(near);
        newc.push_back(grow.back().q1);
      }
      const auto gts = check(grow);

      std::vector<int> added;
      for (std::size_t j = 0; j < grow.size(); ++j) {
        if (gts[j] >= 1.0f) {
          added.push_back(active.add(newc[j], parent[j]));
        }
      }
      if (added.empty()) {
        active_is_start = !active_is_start;
        continue;
      }

      // Connect: bridge each new node to the nearest node of the other tree.
      std::vector<EdgeReq> bridges;
      std::vector<std::pair<int, int>> pair_idx; // (active_idx, other_idx)
      bridges.reserve(added.size());
      for (const int a_idx : added) {
        const int o_idx = other.nearest(active.q[a_idx]);
        bridges.push_back({active.q[a_idx], other.q[o_idx]});
        pair_idx.emplace_back(a_idx, o_idx);
      }
      const auto cts = check(bridges);
      for (std::size_t m = 0; m < cts.size(); ++m) {
        if (cts[m] >= 1.0f) {
          const auto [a_idx, o_idx] = pair_idx[m];
          result.path = active_is_start ? build_path(start_tree, a_idx, goal_tree, o_idx)
                                        : build_path(start_tree, o_idx, goal_tree, a_idx);
          return finish(PlanningStatus::Success, "solved");
        }
      }
      active_is_start = !active_is_start;
    }

    const bool timed_out = std::chrono::duration<double>(Clock::now() - t_begin).count() >= timeout;
    return finish(timed_out ? PlanningStatus::Timeout : PlanningStatus::NoSolution,
                  timed_out ? "planning timed out" : "iteration budget exhausted");
  }

private:
  PlannerParams params_;
  std::shared_ptr<const RobotInstance> robot_;
  std::shared_ptr<const collision::CollisionScene> scene_;
};

} // namespace

std::unique_ptr<Planner> make_rrt_connect(const PlannerParams &params,
                                          std::shared_ptr<const RobotInstance> robot,
                                          std::shared_ptr<const collision::CollisionScene> scene) {
  return std::make_unique<RrtConnectPlanner>(params, std::move(robot), std::move(scene));
}

} // namespace quevedomp::planning::detail
