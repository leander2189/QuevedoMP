// planning/PrmPlanner — PRM-flavored multi-query roadmap planner (roadmap R5). See roadmap.hpp
// for the interface + the build-once/query-many rationale (ADR-020).
//
// Build (once, at construction — the fat-batch phase where the GPU wins outright):
//   1. Sample `num_nodes` configs in the (bounded) free space; validate them in fat batches and
//      keep the collision-free ones as roadmap nodes.
//   2. For every node, gather its k nearest neighbours (+ any within connection_radius) as
//      candidate edges; validate them all in fat batches; keep the collision-free ones with
//      weight = joint-space L2. The kept adjacency is the roadmap.
//
// Query (plan(), cheap + reentrant):
//   • resolve the goal to configs, validate start + goals free;
//   • connect start and each goal to their k nearest roadmap nodes (+ direct start↔goal), one
//     batch of connection edges;
//   • A* over (roadmap ∪ connection edges) with a joint-space straight-line heuristic;
//   • P6 shortcut-smooth the extracted path. Every edge on the path was validated collision-free
//     (roadmap at build, connections at query), so the output is collision-free by construction.
//
// Determinism: the roadmap is a pure function of (params, robot, scene, seed); a query is
// deterministic per problem.seed (goal IK + smoother). A* breaks ties by node index.
#include "quevedomp/planning/roadmap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/core/rng.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/planning/smoother.hpp"

namespace quevedomp::planning {
namespace {

using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Bounds {
  JointPosition lo, hi;
};

// URDF position limits (±π for continuous joints), intersected with Constraints::joint_bounds.
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

using Segment = std::pair<JointPosition, JointPosition>;

// Validate a set of segments (q0→q1), returning per-segment collision-free flags. Samples are
// concatenated into batches kept under `chunk_cfg` configs each (0 ⇒ one batch), so a huge roadmap
// validates as a few fat query_batch calls rather than one oversized allocation. Bumps the config
// counter + collision timer. A segment free ⇔ none of its inclusive samples collide.
std::vector<char>
validate_segments(const std::vector<Segment> &segs, const collision::CollisionScene &scene,
                  const RobotInstance &robot, const collision::QueryOptions &opts,
                  collision::Workspace &ws, const collision::EdgeDiscretization &disc,
                  std::size_t chunk_cfg, std::uint64_t &configs, double &t_collision) {
  std::vector<char> free(segs.size(), 1);
  std::vector<JointPosition> samples;
  std::vector<std::pair<std::size_t, int>> slice(segs.size(), {0, 0}); // (offset-in-chunk, n)
  std::vector<std::size_t> in_chunk; // segment ids in the current chunk

  const auto flush = [&] {
    if (samples.empty()) {
      return;
    }
    const auto t0 = Clock::now();
    const collision::BatchResult br = scene.query_batch(robot, samples, opts, ws);
    t_collision += std::chrono::duration<double>(Clock::now() - t0).count();
    configs += samples.size();
    for (const std::size_t s : in_chunk) {
      const auto [off, n] = slice[s];
      for (int k = 0; k <= n; ++k) {
        if (br.in_collision[off + static_cast<std::size_t>(k)] != 0) {
          free[s] = 0;
          break;
        }
      }
    }
    samples.clear();
    in_chunk.clear();
  };

  for (std::size_t s = 0; s < segs.size(); ++s) {
    const JointPosition delta = segs[s].second - segs[s].first;
    const int n = disc.steps(delta);
    if (chunk_cfg > 0 && !samples.empty() &&
        samples.size() + static_cast<std::size_t>(n) + 1 > chunk_cfg) {
      flush();
    }
    slice[s] = {samples.size(), n};
    in_chunk.push_back(s);
    for (int k = 0; k <= n; ++k) {
      samples.push_back(segs[s].first + (static_cast<double>(k) / n) * delta);
    }
  }
  flush();
  return free;
}

// The k nearest indices to `from` among `nodes` (joint-space L2), excluding `exclude` (-1 = none).
// Linear scan + partial_sort — fine for v0 roadmap sizes (a kd-tree is the scale follow-up).
std::vector<int> k_nearest(const std::vector<JointPosition> &nodes, const JointPosition &from,
                           std::size_t k, double radius, int exclude) {
  std::vector<std::pair<double, int>> d;
  d.reserve(nodes.size());
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    if (i == exclude) {
      continue;
    }
    d.emplace_back((nodes[static_cast<std::size_t>(i)] - from).squaredNorm(), i);
  }
  const std::size_t kk = std::min(k, d.size());
  std::partial_sort(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(kk), d.end());
  std::vector<int> out;
  const double r2 = radius * radius;
  for (std::size_t i = 0; i < d.size(); ++i) {
    if (i < kk || (radius > 0.0 && d[i].first <= r2)) {
      out.push_back(d[i].second);
    } else {
      break; // sorted: once past k and outside radius, done
    }
  }
  return out;
}

// Goal → candidate configs (JointGoal target; PoseGoal via seeded IK; MultiGoal union). Mirrors
// the RRT/refiner resolver.
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
        auto s = resolve_goal(*sub, model, model_ptr, seed);
        out.insert(out.end(), s.begin(), s.end());
      }
    }
    break;
  }
  return out;
}

class PrmPlanner final : public Planner {
public:
  PrmPlanner(PrmParams params, std::shared_ptr<const RobotInstance> robot,
             std::shared_ptr<const collision::CollisionScene> scene, PrmBuildStats *out_stats)
      : params_(std::move(params)), robot_(std::move(robot)), scene_(std::move(scene)),
        disc_(collision::make_edge_discretization(params_.edge_resolution, params_.max_link_sweep,
                                                  params_.lever_weights, robot_->model())) {
    build(out_stats);
  }

  PlanningResult plan(const PlanningProblem &problem) const override {
    const auto t_begin = Clock::now();
    const RobotModel &model = robot_->model();
    PlanningResult result;
    double t_collision = 0.0;
    double t_smoothing = 0.0;
    std::uint64_t configs = 0;

    const std::uint64_t used_seed = problem.seed.value_or(
        static_cast<std::uint64_t>(std::random_device{}()) << 32 | std::random_device{}());
    result.used_seed = used_seed;

    auto finish = [&](PlanningStatus status, std::string msg) {
      result.status = status;
      result.message = std::move(msg);
      result.stats.time_collision = t_collision;
      result.stats.time_smoothing = t_smoothing;
      result.stats.collision_configs = configs;
      result.stats.time_total = std::chrono::duration<double>(Clock::now() - t_begin).count();
      result.stats.time_planner =
          std::max(0.0, result.stats.time_total - t_collision - t_smoothing);
      if (status == PlanningStatus::Success) {
        result.stats.time_first_solution = result.stats.time_total;
      }
      return result;
    };

    if (auto reason = validate(problem, model)) {
      return finish(PlanningStatus::InvalidProblem, *reason);
    }
    if (nodes_.empty()) {
      return finish(PlanningStatus::NoSolution, "empty roadmap (no free samples at build)");
    }

    auto goals = resolve_goal(*problem.goal, model, robot_->model_ptr(), used_seed);
    if (goals.empty()) {
      return finish(PlanningStatus::NoSolution, "no goal configuration (IK failed for the goal)");
    }

    const auto ws = scene_->make_workspace();
    const collision::QueryOptions &opts = params_.collision; // the roadmap fixes the semantics
    auto count_batch = [&](std::size_t n) { result.stats.record_batch(n); };

    // Feasibility: start + all goal configs in one batch.
    {
      std::vector<JointPosition> probe;
      probe.push_back(problem.start);
      for (const auto &g : goals) {
        probe.push_back(g);
      }
      const auto t0 = Clock::now();
      const collision::BatchResult br = scene_->query_batch(*robot_, probe, opts, *ws);
      t_collision += std::chrono::duration<double>(Clock::now() - t0).count();
      configs += probe.size();
      count_batch(probe.size());
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

    // Graph indexing: [0, N) roadmap nodes, N = start, N+1.. = goals.
    const int N = static_cast<int>(nodes_.size());
    const int start_idx = N;
    const int goal0 = N + 1;
    const int total = N + 1 + static_cast<int>(goals.size());
    const auto config_at = [&](int idx) -> const JointPosition & {
      if (idx < N) {
        return nodes_[static_cast<std::size_t>(idx)];
      }
      if (idx == start_idx) {
        return problem.start;
      }
      return goals[static_cast<std::size_t>(idx - goal0)];
    };

    // Connection edges: start↔its k-nearest nodes, each goal↔its k-nearest nodes, and direct
    // start↔goal (the trivial win). Validate them as ONE batch.
    std::vector<std::pair<int, int>> conn; // index pairs (both endpoints in graph indexing)
    std::vector<Segment> segs;
    const auto add_conn = [&](int a, int b) {
      conn.emplace_back(a, b);
      segs.emplace_back(config_at(a), config_at(b));
    };
    for (int nb :
         k_nearest(nodes_, problem.start, params_.k_neighbors, params_.connection_radius, -1)) {
      add_conn(start_idx, nb);
    }
    for (std::size_t g = 0; g < goals.size(); ++g) {
      for (int nb :
           k_nearest(nodes_, goals[g], params_.k_neighbors, params_.connection_radius, -1)) {
        add_conn(goal0 + static_cast<int>(g), nb);
      }
      add_conn(start_idx, goal0 + static_cast<int>(g));
    }
    // Query-local extra adjacency (bidirectional) layered over the immutable roadmap.
    std::unordered_map<int, std::vector<std::pair<int, double>>> extra;
    if (!segs.empty()) {
      const std::vector<char> free =
          validate_segments(segs, *scene_, *robot_, opts, *ws, disc_, params_.edge_batch_configs,
                            configs, t_collision);
      result.stats.collision_queries += 1; // the connection-edge validation (chunked internally)
      for (std::size_t e = 0; e < conn.size(); ++e) {
        if (!free[e]) {
          continue;
        }
        const auto [a, b] = conn[e];
        const double w = (config_at(a) - config_at(b)).norm();
        extra[a].emplace_back(b, w);
        extra[b].emplace_back(a, w);
      }
    }

    // A* from start to the nearest reachable goal. Heuristic = min straight-line joint distance to
    // any goal (admissible: edge weights are exact joint distances).
    const auto heuristic = [&](int idx) {
      double h = kInf;
      for (std::size_t g = 0; g < goals.size(); ++g) {
        h = std::min(h, (config_at(idx) - goals[g]).norm());
      }
      return h;
    };
    std::vector<double> g_cost(static_cast<std::size_t>(total), kInf);
    std::vector<int> came_from(static_cast<std::size_t>(total), -1);
    std::vector<char> is_goal(static_cast<std::size_t>(total), 0);
    for (std::size_t g = 0; g < goals.size(); ++g) {
      is_goal[static_cast<std::size_t>(goal0) + g] = 1;
    }
    using QNode = std::tuple<double, double, int>; // (f, g, idx) — min-heap, tie-break by idx
    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;
    g_cost[static_cast<std::size_t>(start_idx)] = 0.0;
    open.emplace(heuristic(start_idx), 0.0, start_idx);

    const auto neighbors = [&](int idx, auto &&visit) {
      if (idx < N) {
        for (const auto &[to, w] : adj_[static_cast<std::size_t>(idx)]) {
          visit(to, w);
        }
      }
      const auto it = extra.find(idx);
      if (it != extra.end()) {
        for (const auto &[to, w] : it->second) {
          visit(to, w);
        }
      }
    };

    int reached = -1;
    while (!open.empty()) {
      const auto [f, gc, idx] = open.top();
      open.pop();
      if (gc > g_cost[static_cast<std::size_t>(idx)]) {
        continue; // stale heap entry
      }
      if (is_goal[static_cast<std::size_t>(idx)]) {
        reached = idx;
        break;
      }
      neighbors(idx, [&](int to, double w) {
        const double ng = gc + w;
        if (ng < g_cost[static_cast<std::size_t>(to)]) {
          g_cost[static_cast<std::size_t>(to)] = ng;
          came_from[static_cast<std::size_t>(to)] = idx;
          open.emplace(ng + heuristic(to), ng, to);
        }
      });
    }
    result.stats.iterations = 0;

    if (reached < 0) {
      return finish(PlanningStatus::NoSolution,
                    "start and goal are not connected through the roadmap");
    }

    // Reconstruct + optionally smooth.
    Path path;
    for (int idx = reached; idx >= 0; idx = came_from[static_cast<std::size_t>(idx)]) {
      path.push_back(config_at(idx));
    }
    std::reverse(path.begin(), path.end());

    if (params_.smooth && path.size() > 2) {
      SmootherParams sp;
      sp.edge_resolution = params_.edge_resolution;
      sp.max_link_sweep = params_.max_link_sweep;
      sp.lever_weights = params_.lever_weights;
      sp.collision = opts;
      sp.seed = used_seed;
      const auto t0 = Clock::now();
      path = make_shortcut_smoother(sp, robot_, scene_)->smooth(path);
      t_smoothing += std::chrono::duration<double>(Clock::now() - t0).count();
    }

    result.path = std::move(path);
    return finish(PlanningStatus::Success, "solved via roadmap");
  }

private:
  void build(PrmBuildStats *out_stats) {
    const auto t0 = Clock::now();
    const RobotModel &model = robot_->model();
    const Bounds bounds = compute_bounds(model, params_.constraints);
    const collision::QueryOptions &opts = params_.collision;
    const auto ws = scene_->make_workspace();
    Rng rng(params_.seed);
    PrmBuildStats stats;
    stats.node_candidates = params_.num_nodes;

    // --- Nodes: sample num_nodes configs, validate in fat batches, keep the free ones. ---
    std::vector<JointPosition> candidates;
    candidates.reserve(params_.num_nodes);
    for (std::size_t i = 0; i < params_.num_nodes; ++i) {
      candidates.push_back(rng.sample_in_box(bounds.lo, bounds.hi));
    }
    for (std::size_t off = 0; off < candidates.size();) {
      const std::size_t chunk = params_.edge_batch_configs > 0
                                    ? std::min(candidates.size() - off, params_.edge_batch_configs)
                                    : candidates.size() - off;
      std::vector<JointPosition> batch(candidates.begin() + static_cast<std::ptrdiff_t>(off),
                                       candidates.begin() +
                                           static_cast<std::ptrdiff_t>(off + chunk));
      const collision::BatchResult br = scene_->query_batch(*robot_, batch, opts, *ws);
      stats.collision_configs += batch.size();
      for (std::size_t i = 0; i < batch.size(); ++i) {
        if (br.in_collision[i] == 0) {
          nodes_.push_back(batch[i]);
        }
      }
      off += chunk;
    }

    // --- Edges: k-nearest (+radius) candidate edges, deduped, validated in fat batches. ---
    if (nodes_.size() >= 2) {
      std::unordered_set<long long> seen;
      std::vector<std::pair<int, int>> cand;
      const auto key = [](int a, int b) { return static_cast<long long>(a) * 1000000003LL + b; };
      for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        for (int j : k_nearest(nodes_, nodes_[static_cast<std::size_t>(i)], params_.k_neighbors,
                               params_.connection_radius, i)) {
          const int a = std::min(i, j), b = std::max(i, j);
          if (seen.insert(key(a, b)).second) {
            cand.emplace_back(a, b);
          }
        }
      }
      stats.edge_candidates = cand.size();
      std::vector<Segment> segs;
      segs.reserve(cand.size());
      for (const auto &[a, b] : cand) {
        segs.emplace_back(nodes_[static_cast<std::size_t>(a)], nodes_[static_cast<std::size_t>(b)]);
      }
      double t_collision = 0.0;
      const std::vector<char> free =
          validate_segments(segs, *scene_, *robot_, opts, *ws, disc_, params_.edge_batch_configs,
                            stats.collision_configs, t_collision);
      adj_.assign(nodes_.size(), {});
      for (std::size_t e = 0; e < cand.size(); ++e) {
        if (!free[e]) {
          continue;
        }
        const auto [a, b] = cand[e];
        const double w =
            (nodes_[static_cast<std::size_t>(a)] - nodes_[static_cast<std::size_t>(b)]).norm();
        adj_[static_cast<std::size_t>(a)].emplace_back(b, w);
        adj_[static_cast<std::size_t>(b)].emplace_back(a, w);
        ++stats.edges;
      }
    } else {
      adj_.assign(nodes_.size(), {});
    }

    stats.nodes = nodes_.size();
    stats.build_seconds = std::chrono::duration<double>(Clock::now() - t0).count();
    if (out_stats != nullptr) {
      *out_stats = stats;
    }
  }

  PrmParams params_;
  std::shared_ptr<const RobotInstance> robot_;
  std::shared_ptr<const collision::CollisionScene> scene_;
  collision::EdgeDiscretization disc_;
  std::vector<JointPosition> nodes_;
  std::vector<std::vector<std::pair<int, double>>> adj_; // parallel to nodes_
};

} // namespace

std::unique_ptr<Planner> make_prm_planner(PrmParams params,
                                          std::shared_ptr<const RobotInstance> robot,
                                          std::shared_ptr<const collision::CollisionScene> scene,
                                          PrmBuildStats *out_stats) {
  if (robot == nullptr) {
    throw std::runtime_error("make_prm_planner: robot instance is null");
  }
  if (scene == nullptr) {
    throw std::runtime_error("make_prm_planner: collision scene is null");
  }
  return std::make_unique<PrmPlanner>(std::move(params), std::move(robot), std::move(scene),
                                      out_stats);
}

} // namespace quevedomp::planning
