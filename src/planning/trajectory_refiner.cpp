// planning/TrajectoryRefiner — CHOMP/TrajOpt-flavored optimization refiner (roadmap R4, Task
// 3.3c). See refiner.hpp for the interface and the mode/plumbing rationale (ADR-019).
//
// The optimizer moves a fixed-endpoint trajectory ξ = [q₀ … q_{M-1}] (q₀, q_{M-1} clamped) to
// minimize U(ξ) = w_s·F_smooth(ξ) + w_obs·F_obs(ξ):
//
//   • F_smooth = ½ Σ ‖q_{i-1} − 2q_i + q_{i+1}‖²  (summed squared finite-difference acceleration).
//     Its gradient is the discrete biharmonic gₛ[j] = a_{j-1} − 2a_j + a_{j+1} = (A·Q + b)[j],
//     where a_i is the acceleration at i and A = KᵀK is the pentadiagonal smoothness metric.
//   • F_obs sums a CHOMP hinge over the robot's conservative sphere cover against the R3
//     ClearanceField: cost 0 beyond ε clearance, quadratic in [0, ε], linear when penetrating.
//     The workspace gradient (cost′·∇dist) is mapped to joint space by each sphere-center's
//     position Jacobian Jₚ = J_v − [x−o]×·J_w.
//
// CHOMP update (the A⁻¹ preconditioning is what keeps local obstacle pushes from kinking the
// path): Q ← Q − step·A⁻¹·∇U. A depends only on the interior count, so it is factored once.
//
// Contract: the per-iteration clearance/gradient lookup is ONE fat ClearanceField::query over all
// (interior waypoint × sphere) points — GPU/OpenMP-friendly by construction. Determinism per seed:
// CHOMP is RNG-free (only standalone goal IK draws, seeded); per-waypoint gradients are computed
// independently with a fixed intra-waypoint summation order, so the result is bit-identical across
// thread counts. The exact CollisionScene backend CERTIFIES the output — the field is never trusted
// as the certificate (ADR-018).
#include "quevedomp/planning/refiner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/kinematics/jacobian.hpp"

namespace quevedomp::planning {
namespace {

using Clock = std::chrono::steady_clock;

// Per-dof clamp bounds: the URDF position limits (continuous joints stay unclamped), intersected
// with any tighter Constraints::joint_bounds. Parallel to the trajectory's dof.
struct Bounds {
  JointPosition lo, hi;
  std::vector<char> active; // 1 ⇒ this dof is clamped
};

Bounds compute_bounds(const RobotModel &model, const Constraints &c) {
  const auto dof = static_cast<Eigen::Index>(model.dof());
  Bounds b;
  b.lo = JointPosition::Constant(dof, -std::numeric_limits<double>::infinity());
  b.hi = JointPosition::Constant(dof, std::numeric_limits<double>::infinity());
  b.active.assign(static_cast<std::size_t>(dof), 0);
  for (const auto &joint : model.joints()) {
    if (!joint.is_movable() || joint.dof_index < 0) {
      continue;
    }
    const auto i = static_cast<Eigen::Index>(joint.dof_index);
    if (i >= 0 && i < dof && joint.limits.has_position_limit) {
      b.lo[i] = joint.limits.lower;
      b.hi[i] = joint.limits.upper;
      b.active[static_cast<std::size_t>(i)] = 1;
    }
  }
  if (!c.joint_bounds.empty() && static_cast<Eigen::Index>(c.joint_bounds.size()) == dof) {
    for (std::size_t k = 0; k < c.joint_bounds.size(); ++k) {
      const auto i = static_cast<Eigen::Index>(k);
      b.lo[i] = std::max(b.lo[i], c.joint_bounds[k].first);
      b.hi[i] = std::min(b.hi[i], c.joint_bounds[k].second);
      b.active[k] = 1;
    }
  }
  return b;
}

void clamp_to_bounds(JointPosition &q, const Bounds &b) {
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    if (b.active[static_cast<std::size_t>(i)]) {
      q[i] = std::clamp(q[i], b.lo[i], b.hi[i]);
    }
  }
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d s;
  s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return s;
}

// Resample a polyline to exactly `n` configs by uniform arc-length (joint-space), endpoints
// preserved. n >= 2; a single-config input is replicated.
Path resample(const Path &in, std::size_t n) {
  if (in.empty()) {
    return {};
  }
  if (in.size() == 1) {
    return Path(n, in.front());
  }
  std::vector<double> cum(in.size(), 0.0);
  for (std::size_t i = 1; i < in.size(); ++i) {
    cum[i] = cum[i - 1] + (in[i] - in[i - 1]).norm();
  }
  const double total = cum.back();
  Path out;
  out.reserve(n);
  if (total <= 0.0) { // degenerate (all coincident) — replicate the endpoint
    return Path(n, in.front());
  }
  std::size_t seg = 0;
  for (std::size_t k = 0; k < n; ++k) {
    const double s = total * static_cast<double>(k) / static_cast<double>(n - 1);
    while (seg + 1 < in.size() - 1 && cum[seg + 1] < s) {
      ++seg;
    }
    const double seg_len = cum[seg + 1] - cum[seg];
    const double a = seg_len > 0.0 ? (s - cum[seg]) / seg_len : 0.0;
    out.push_back((1.0 - a) * in[seg] + a * in[seg + 1]);
  }
  out.front() = in.front(); // exact endpoints (guard against float drift)
  out.back() = in.back();
  return out;
}

// Resolve a goal into candidate configurations (standalone mode). Mirrors the RRT planner's
// resolver: JointGoal is its target; PoseGoal is numerical IK over a few deterministic seeds;
// MultiGoal unions its sub-goals. May be empty (IK failed) ⇒ caller reports NoSolution.
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
        auto sub_cfgs = resolve_goal(*sub, model, model_ptr, seed);
        out.insert(out.end(), sub_cfgs.begin(), sub_cfgs.end());
      }
    }
    break;
  }
  return out;
}

class TrajectoryRefiner final : public Planner {
public:
  TrajectoryRefiner(RefinerParams params, std::shared_ptr<const RobotInstance> robot,
                    std::shared_ptr<const collision::CollisionScene> scene,
                    std::shared_ptr<const clearance::ClearanceField> field,
                    clearance::RobotSpheres spheres)
      : params_(std::move(params)), robot_(std::move(robot)), scene_(std::move(scene)),
        field_(std::move(field)), spheres_(std::move(spheres)),
        disc_(collision::make_edge_discretization(params_.edge_resolution, params_.max_link_sweep,
                                                  params_.lever_weights, robot_->model())) {
    // Link name per sphere (jacobian() takes a name) — resolved once.
    const auto &links = robot_->model().links();
    for (const auto &s : spheres_.spheres) {
      sphere_link_name_.push_back(links[static_cast<std::size_t>(s.link)].name);
    }
  }

  PlanningResult plan(const PlanningProblem &problem) const override {
    const auto t_begin = Clock::now();
    const RobotModel &model = robot_->model();
    PlanningResult result;
    double t_collision = 0.0;
    std::uint64_t iterations = 0;
    std::string mode;

    auto finish = [&](PlanningStatus status, std::string msg) {
      result.status = status;
      result.message = std::move(msg);
      result.stats.iterations = iterations;
      result.stats.refiner_mode = mode;
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

    // ---- Build the initial full trajectory (fixed endpoints + interior) --------------------
    const std::size_t M = std::max<std::size_t>(3, params_.waypoints);
    Path init;
    if (!params_.seed.empty()) {
      mode = "refiner";
      if (params_.seed.size() < 2) {
        return finish(PlanningStatus::InvalidProblem, "refiner seed needs >= 2 waypoints");
      }
      init = resample(params_.seed, M);
    } else {
      mode = "standalone";
      auto goals = resolve_goal(*problem.goal, model, robot_->model_ptr(), params_.rng_seed);
      if (goals.empty()) {
        return finish(PlanningStatus::NoSolution, "no goal configuration (IK failed for the goal)");
      }
      // Straight line to the goal config nearest the start (most likely to refine cleanly).
      const JointPosition *best = &goals.front();
      double best_d = (goals.front() - problem.start).squaredNorm();
      for (const auto &g : goals) {
        const double d = (g - problem.start).squaredNorm();
        if (d < best_d) {
          best_d = d;
          best = &g;
        }
      }
      init = resample(Path{problem.start, *best}, M);
    }

    const auto dof = static_cast<Eigen::Index>(model.dof());
    const std::size_t n = M - 2; // interior waypoint count (optimization variables)
    const Bounds bounds = compute_bounds(model, problem.constraints);

    // ---- Smoothness metric A = KᵀK (n×n), factored once. K = tridiag(-2; +1). ---------------
    Eigen::MatrixXd K =
        Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(n));
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(n); ++i) {
      K(i, i) = -2.0;
      if (i > 0) {
        K(i, i - 1) = 1.0;
      }
      if (i + 1 < static_cast<Eigen::Index>(n)) {
        K(i, i + 1) = 1.0;
      }
    }
    const Eigen::MatrixXd A = K.transpose() * K;
    const Eigen::LLT<Eigen::MatrixXd> A_llt(A);

    // Interior configs as an (n × dof) matrix; endpoints held separately.
    Eigen::MatrixXd Q(static_cast<Eigen::Index>(n), dof);
    for (std::size_t i = 0; i < n; ++i) {
      Q.row(static_cast<Eigen::Index>(i)) = init[i + 1].transpose();
    }
    const JointPosition q_start = init.front();
    const JointPosition q_goal = init.back();

    const std::size_t S = spheres_.spheres.size();

    // ---- CHOMP iterations ------------------------------------------------------------------
    for (std::size_t it = 0; it < params_.max_iterations; ++it) {
      if (std::chrono::duration<double>(Clock::now() - t_begin).count() >= problem.timeout) {
        break;
      }
      ++iterations;

      // Full trajectory row accessor (interior from Q, endpoints fixed).
      const auto row = [&](std::size_t i) -> JointPosition {
        if (i == 0) {
          return q_start;
        }
        if (i == M - 1) {
          return q_goal;
        }
        return Q.row(static_cast<Eigen::Index>(i - 1)).transpose();
      };

      // (1) FK per interior waypoint (cached: reused for both point-gather and Jacobian origins).
      std::vector<std::vector<Transform>> poses(n);
#ifdef _OPENMP
#pragma omp parallel for schedule(static) if (n >= 8)
#endif
      for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(n); ++i) {
        poses[static_cast<std::size_t>(i)] = fk_all(model, row(static_cast<std::size_t>(i) + 1));
      }

      // (2) Gather every sphere-center world point — ONE fat clearance batch (contract).
      std::vector<Eigen::Vector3d> points(n * S);
      for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t s = 0; s < S; ++s) {
          const auto &sph = spheres_.spheres[s];
          points[i * S + s] = poses[i][static_cast<std::size_t>(sph.link)] * sph.center;
        }
      }
      std::vector<double> dist(n * S);
      std::vector<Eigen::Vector3d> grad(n * S);
      field_->query(points, dist, grad);

      // (3) Obstacle gradient in joint space, per interior waypoint (independent ⇒ deterministic).
      Eigen::MatrixXd g_obs = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(n), dof);
      const double eps = params_.clearance_epsilon;
#ifdef _OPENMP
#pragma omp parallel for schedule(static) if (n >= 8)
#endif
      for (std::ptrdiff_t ii = 0; ii < static_cast<std::ptrdiff_t>(n); ++ii) {
        const auto i = static_cast<std::size_t>(ii);
        const JointPosition qi = row(i + 1);
        Eigen::VectorXd gi = Eigen::VectorXd::Zero(dof);
        // Jacobian per unique sphere link, computed on demand and cached within this waypoint.
        for (std::size_t s = 0; s < S; ++s) {
          const auto &sph = spheres_.spheres[s];
          const double c = dist[i * S + s] - sph.radius; // clearance
          double dcost = 0.0;                            // ∂cost/∂c (CHOMP hinge)
          if (c < 0.0) {
            dcost = -1.0;
          } else if (c < eps) {
            dcost = (c - eps) / eps;
          } else {
            continue; // beyond ε: no obstacle force from this sphere
          }
          const Eigen::Vector3d gw = dcost * grad[i * S + s]; // workspace gradient
          const Eigen::Vector3d x = points[i * S + s];
          const Eigen::Vector3d o = poses[i][static_cast<std::size_t>(sph.link)].translation();
          const Eigen::MatrixXd J6 = jacobian(model, qi, sphere_link_name_[s]); // 6×dof
          const Eigen::MatrixXd Jp =
              J6.topRows(3) - skew(x - o) * J6.bottomRows(3); // 3×dof position Jacobian
          gi.noalias() += Jp.transpose() * gw;
        }
        g_obs.row(static_cast<Eigen::Index>(i)) = gi.transpose();
      }

      // (4) Smoothness gradient gₛ[j] = a_{j-1} − 2a_j + a_{j+1} (= (A·Q + b)[j]).
      //     a_i = q_{i-1} − 2q_i + q_{i+1} over the FULL trajectory (real endpoints included).
      std::vector<JointPosition> accel(M); // accel[i] valid for i = 1..M-2
      for (std::size_t i = 1; i + 1 < M; ++i) {
        accel[i] = row(i - 1) - 2.0 * row(i) + row(i + 1);
      }
      Eigen::MatrixXd g_smooth(static_cast<Eigen::Index>(n), dof);
      for (std::size_t j = 1; j + 1 < M; ++j) { // full index; interior variable = j-1
        JointPosition g = -2.0 * accel[j];
        if (j >= 2) {
          g += accel[j - 1];
        }
        if (j + 2 < M) {
          g += accel[j + 1];
        }
        g_smooth.row(static_cast<Eigen::Index>(j - 1)) = g.transpose();
      }

      // (5) Preconditioned CHOMP step: Q ← Q − step·A⁻¹·(w_s·gₛ + w_obs·g_obs).
      const Eigen::MatrixXd G =
          params_.smoothness_weight * g_smooth + params_.clearance_weight * g_obs;
      const Eigen::MatrixXd step = A_llt.solve(G);
      Q.noalias() -= params_.step_size * step;

      // (6) Project onto joint limits + measure the largest per-waypoint move (convergence).
      double max_update = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        JointPosition qi = Q.row(static_cast<Eigen::Index>(i)).transpose();
        clamp_to_bounds(qi, bounds);
        Q.row(static_cast<Eigen::Index>(i)) = qi.transpose();
        max_update =
            std::max(max_update, params_.step_size * step.row(static_cast<Eigen::Index>(i)).norm());
      }
      if (max_update < params_.convergence_tol) {
        break;
      }
    }

    // ---- Assemble the optimized full trajectory --------------------------------------------
    Path optimized;
    optimized.reserve(M);
    optimized.push_back(q_start);
    for (std::size_t i = 0; i < n; ++i) {
      optimized.push_back(Q.row(static_cast<Eigen::Index>(i)).transpose());
    }
    optimized.push_back(q_goal);

    // ---- CERTIFY through the exact backend (ADR-018: the field is never the certificate) ----
    const auto ws = scene_->make_workspace();
    if (certify(optimized, problem.collision, *ws, result.stats, t_collision)) {
      result.path = std::move(optimized);
      return finish(PlanningStatus::Success, mode + " mode: certified collision-free in " +
                                                 std::to_string(iterations) + " iterations");
    }

    // Optimized path failed certification. In refiner mode, the caller handed us a feasible seed;
    // never return something worse than that input — fall back to the certified (resampled) seed
    // if it holds. In standalone mode there is no seed: honest failure (local minimum).
    if (mode == "refiner") {
      Path seed_traj = init; // the resampled seed (its endpoints already exact)
      if (certify(seed_traj, problem.collision, *ws, result.stats, t_collision)) {
        result.path = std::move(seed_traj);
        return finish(PlanningStatus::Success,
                      "refiner mode: refinement rejected by certificate; returning certified seed");
      }
      return finish(PlanningStatus::NoSolution,
                    "refiner mode: neither the refined path nor the seed certified collision-free");
    }
    return finish(PlanningStatus::NoSolution,
                  "standalone mode: refined path did not certify collision-free (local minimum)");
  }

private:
  // Validate every consecutive edge of `traj` as ONE query_batch (samples concatenated exactly as
  // collision::check_edge discretizes them). Returns true iff all edges are collision-free.
  bool certify(const Path &traj, const collision::QueryOptions &opts, collision::Workspace &ws,
               PlanningStats &stats, double &t_collision) const {
    if (traj.size() < 2) {
      return true;
    }
    std::vector<JointPosition> samples;
    std::vector<std::pair<std::size_t, int>> slices; // (offset, n) per edge
    slices.reserve(traj.size() - 1);
    for (std::size_t e = 0; e + 1 < traj.size(); ++e) {
      const JointPosition delta = traj[e + 1] - traj[e];
      const int steps = disc_.steps(delta);
      slices.emplace_back(samples.size(), steps);
      for (int k = 0; k <= steps; ++k) {
        samples.push_back(traj[e] + (static_cast<double>(k) / steps) * delta);
      }
    }
    const auto t0 = Clock::now();
    const collision::BatchResult batch = scene_->query_batch(*robot_, samples, opts, ws);
    t_collision += std::chrono::duration<double>(Clock::now() - t0).count();
    stats.record_batch(samples.size());
    return std::none_of(batch.in_collision.begin(), batch.in_collision.end(),
                        [](std::uint8_t c) { return c != 0; });
  }

  RefinerParams params_;
  std::shared_ptr<const RobotInstance> robot_;
  std::shared_ptr<const collision::CollisionScene> scene_;
  std::shared_ptr<const clearance::ClearanceField> field_;
  clearance::RobotSpheres spheres_;
  std::vector<std::string> sphere_link_name_;
  collision::EdgeDiscretization disc_;
};

} // namespace

std::unique_ptr<Planner> make_refiner(RefinerParams params,
                                      std::shared_ptr<const RobotInstance> robot,
                                      std::shared_ptr<const collision::CollisionScene> scene,
                                      std::shared_ptr<const clearance::ClearanceField> field,
                                      clearance::RobotSpheres spheres) {
  if (robot == nullptr) {
    throw std::runtime_error("make_refiner: robot instance is null");
  }
  if (scene == nullptr) {
    throw std::runtime_error("make_refiner: collision scene is null");
  }
  if (field == nullptr) {
    throw std::runtime_error("make_refiner: clearance field is null");
  }
  if (spheres.spheres.empty()) {
    throw std::runtime_error("make_refiner: robot sphere cover is empty (decompose_robot first)");
  }
  return std::make_unique<TrajectoryRefiner>(std::move(params), std::move(robot), std::move(scene),
                                             std::move(field), std::move(spheres));
}

} // namespace quevedomp::planning
