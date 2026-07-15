// parameterization/parametrize — Phase A: the convex (jerk-free) time-optimal profile, solved
// exactly by the TOPP-RA backward/forward structure (see parameterize.hpp).
//
// Everything is expressed as rows  lo ≤ a·u + b·β ≤ hi  evaluated at an interval endpoint, with
// u = (β_{k+1} − β_k)/(2Δ) (the exact C1 recursion for piecewise-constant u). The backward pass
// computes the controllable interval K_k = {β_k : ∃β_{k+1} ∈ K_{k+1} with all rows feasible} —
// a projection of a 2-variable polygon onto the β_k axis, solved by exact vertex enumeration
// (≈60 halfplanes per node; no external solver). The forward pass then greedily maximizes
// β_{k+1} inside K_{k+1}, which for velocity/acceleration-type constraints is the provably
// time-optimal profile (TOPP-RA). Deterministic by construction.
#include "quevedomp/parameterization/parameterize.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "quevedomp/kinematics/jacobian.hpp"

namespace quevedomp::parameterization {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kBetaCap = 1e12; // keeps the LPs bounded when no velocity row binds a node

// One acceleration-type halfplane in (x, y) = (β_k, β_{k+1}):  px·x + py·y ≤ h.
struct Halfplane {
  double px, py, h;
};

// Exact 2-variable LP by vertex enumeration: maximize cx·x + cy·y subject to `rows` (which must
// include box rows, so the feasible set is a bounded polygon). Returns false if infeasible.
bool lp2(const std::vector<Halfplane> &rows, double cx, double cy, double &out_x, double &out_y) {
  constexpr double kTol = 1e-7;
  const auto feasible = [&](double x, double y) {
    for (const Halfplane &r : rows) {
      const double scale = std::max({std::abs(r.px), std::abs(r.py), std::abs(r.h), 1.0});
      if (r.px * x + r.py * y - r.h > kTol * scale) {
        return false;
      }
    }
    return true;
  };

  bool found = false;
  double best = -kInf;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    for (std::size_t j = i + 1; j < rows.size(); ++j) {
      const double det = rows[i].px * rows[j].py - rows[i].py * rows[j].px;
      const double scale = std::max({std::abs(rows[i].px), std::abs(rows[i].py),
                                     std::abs(rows[j].px), std::abs(rows[j].py), 1.0});
      if (std::abs(det) < 1e-12 * scale * scale) {
        continue; // parallel
      }
      const double x = (rows[i].h * rows[j].py - rows[i].py * rows[j].h) / det;
      const double y = (rows[i].px * rows[j].h - rows[i].h * rows[j].px) / det;
      if (!std::isfinite(x) || !std::isfinite(y) || !feasible(x, y)) {
        continue;
      }
      const double v = cx * x + cy * y;
      if (!found || v > best) {
        best = v;
        out_x = x;
        out_y = y;
        found = true;
      }
    }
  }
  return found;
}

// Per-node constraint data: acceleration-type rows (a·u + b·β ∈ [lo, hi] with β at THIS node)
// and the velocity-type MVC bound β ≤ beta_max.
struct NodeRows {
  std::vector<double> a, b, lo, hi;
  double beta_max = kBetaCap;

  void add(double a_, double b_, double lo_, double hi_) {
    a.push_back(a_);
    b.push_back(b_);
    lo.push_back(lo_);
    hi.push_back(hi_);
  }
};

// ---- Stage 2: jerk via the velocity-reduction kernel (ADR-017 as amended) ------------------
//
// Discrete node jerk: q⃛_i(s_k) = √β_k·(q'_i·u'_k + 3·q''_i·u_k + q'''_i·β_k), with u derived
// from β by the exact C1 recursion and u' by finite difference. Every term is homogeneous under
// a UNIFORM scaling β → α²β (√β·α, u·α², u'·α²), so node jerk scales by EXACTLY α³ — the
// certified terminal fallback. The kernel exploits the same law LOCALLY: dip β by a smooth
// envelope where the exactly-evaluated jerk exceeds its limit, then re-evaluate exactly (no
// linearization anywhere). Envelope ramps inject their own u/u', so a candidate is accepted only
// if the acceleration-type rows still hold; otherwise the ramps are widened — at the width limit
// the envelope degenerates to the uniform scaling, which shrinks every constraint. Deterministic,
// O(N·dof) per pass.
struct KernelInput {
  int N = 0;
  double delta = 0.0;
  const std::vector<NodeRows> *nodes = nullptr; // acc-type rows (symmetric) + MVC per node
  const std::vector<JointPosition> *d1 = nullptr, *d2 = nullptr, *d3 = nullptr;
  JointPosition j_max; // per joint; entries <= 0 are unconstrained
};

// Per-node jerk ratio max_i |q⃛_i|/j_i (1 = at the limit) + the profile-wide worst.
struct JerkRatios {
  std::vector<double> node; // size N+1; endpoints 0 (no u' there)
  double worst = 0.0;
};

JerkRatios jerk_ratios(const std::vector<double> &beta, const KernelInput &in) {
  const double inv2d = 1.0 / (2.0 * in.delta);
  JerkRatios out;
  out.node.assign(beta.size(), 0.0);
  for (int k = 1; k < in.N; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    const double u_k = (beta[ki + 1] - beta[ki]) * inv2d;
    const double u_km1 = (beta[ki] - beta[ki - 1]) * inv2d;
    const double du = (u_k - u_km1) / in.delta;
    const double sq = std::sqrt(std::max(beta[ki], 0.0));
    double ratio = 0.0;
    for (Eigen::Index i = 0; i < in.j_max.size(); ++i) {
      if (in.j_max[i] <= 0.0) {
        continue;
      }
      const double h =
          sq * ((*in.d1)[ki][i] * du + 3.0 * (*in.d2)[ki][i] * u_k + (*in.d3)[ki][i] * beta[ki]);
      ratio = std::max(ratio, std::abs(h) / in.j_max[i]);
    }
    out.node[ki] = ratio;
    out.worst = std::max(out.worst, ratio);
  }
  return out;
}

// Worst acceleration-type row ratio |a·u + b·β| / bound over both interval endpoints (the exact
// Phase A constraint set; all rows are symmetric, hi > 0).
double acc_ratio(const std::vector<double> &beta, const KernelInput &in) {
  const double inv2d = 1.0 / (2.0 * in.delta);
  double worst = 0.0;
  for (int k = 0; k < in.N; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    const double u = (beta[ki + 1] - beta[ki]) * inv2d;
    const NodeRows &nk = (*in.nodes)[ki];
    for (std::size_t r = 0; r < nk.a.size(); ++r) {
      worst = std::max(worst, std::abs(nk.a[r] * u + nk.b[r] * beta[ki]) / nk.hi[r]);
    }
    const NodeRows &nk1 = (*in.nodes)[ki + 1];
    for (std::size_t r = 0; r < nk1.a.size(); ++r) {
      worst = std::max(worst, std::abs(nk1.a[r] * u + nk1.b[r] * beta[ki + 1]) / nk1.hi[r]);
    }
  }
  return worst;
}

struct KernelOutcome {
  int passes = 0;
  double max_violation = 0.0; // worst ratio − 1 at the returned profile (≤ tolerance ⇒ certified)
  bool converged = false;
};

KernelOutcome jerk_limit_kernel(std::vector<double> &beta, const KernelInput &in,
                                const ParameterizationOptions &opt) {
  KernelOutcome out;
  const int N = in.N;
  const auto n_nodes = beta.size();
  constexpr double kTarget = 0.95; // dip aims 5% under the limit — headroom for ramp coupling
  int W = std::max(2, N / 50);     // ramp half-width (nodes); widens on acc-row violations

  JerkRatios jr = jerk_ratios(beta, in);
  while (out.passes < opt.max_jerk_passes && jr.worst > 1.0 + opt.jerk_tolerance) {
    ++out.passes;

    // Raw per-node cap from the α³ law, then a min-filter (width 2W+1) to widen each dip and a
    // box smooth (half-width W/2, support ≤ the plateau) so ramps are gentle but plateau centers
    // keep their target value.
    // Per-pass depth clamp: a 90×-violating node wants cap ≈ 0.05 at once, whose ramps are far
    // too steep for zero-headroom acceleration segments — reach deep dips over several passes
    // with gentle ramps instead (0.25 ⇒ ≤ 4× slowdown per pass, compounding across passes).
    constexpr double kMaxDipPerPass = 0.25;
    std::vector<double> cap(n_nodes, 1.0);
    for (std::size_t k = 0; k < n_nodes; ++k) {
      if (jr.node[k] > kTarget) {
        cap[k] = std::max(std::pow(kTarget / jr.node[k], 2.0 / 3.0), kMaxDipPerPass);
      }
    }
    std::vector<double> env(n_nodes, 1.0);
    for (std::size_t k = 0; k < n_nodes; ++k) {
      const std::size_t lo = k >= static_cast<std::size_t>(W) ? k - static_cast<std::size_t>(W) : 0;
      const std::size_t hi = std::min(n_nodes - 1, k + static_cast<std::size_t>(W));
      double m = 1.0;
      for (std::size_t j = lo; j <= hi; ++j) {
        m = std::min(m, cap[j]);
      }
      env[k] = m;
    }
    const int h = std::max(1, W / 2);
    for (int smooth_pass = 0; smooth_pass < 2; ++smooth_pass) {
      std::vector<double> sm(n_nodes);
      for (std::size_t k = 0; k < n_nodes; ++k) {
        const std::size_t lo =
            k >= static_cast<std::size_t>(h) ? k - static_cast<std::size_t>(h) : 0;
        const std::size_t hi = std::min(n_nodes - 1, k + static_cast<std::size_t>(h));
        double sum = 0.0;
        for (std::size_t j = lo; j <= hi; ++j) {
          sum += env[j];
        }
        sm[k] = sum / static_cast<double>(hi - lo + 1);
      }
      env = std::move(sm);
    }

    std::vector<double> candidate(n_nodes);
    for (std::size_t k = 0; k < n_nodes; ++k) {
      candidate[k] = beta[k] * env[k];
    }
    if (acc_ratio(candidate, in) > 1.0 + 1e-9) {
      // The envelope's ramps stole acceleration headroom: widen and rebuild from the SAME β.
      // At W ≥ N the envelope is constant (uniform scaling), which strictly shrinks every row.
      W = std::min(2 * W, N);
      continue;
    }
    beta = std::move(candidate);
    jr = jerk_ratios(beta, in);
  }

  if (jr.worst > 1.0 + opt.jerk_tolerance) {
    // Certified terminal fallback: uniform β·α² scales every node jerk by exactly α³ and
    // shrinks all acceleration/velocity/tip rows. Lands at (1 + tolerance/2) of the limit.
    const double alpha3 = (1.0 + 0.5 * opt.jerk_tolerance) / jr.worst;
    const double a2 = std::pow(alpha3, 2.0 / 3.0);
    for (double &b : beta) {
      b *= a2;
    }
    jr = jerk_ratios(beta, in);
  }
  out.max_violation = std::max(jr.worst - 1.0, 0.0);
  out.converged = jr.worst <= 1.0 + opt.jerk_tolerance;
  return out;
}

} // namespace

Limits limits_from_model(const RobotModel &model, const planning::TaskLimits &task,
                         double default_acceleration) {
  const auto dof = static_cast<Eigen::Index>(model.dof());
  Limits lim;
  lim.max_velocity = JointPosition::Constant(dof, 1e9);
  lim.max_acceleration = JointPosition::Constant(dof, default_acceleration);
  lim.max_jerk = JointPosition::Zero(dof);
  for (const Joint &j : model.joints()) {
    if (j.dof_index < 0) {
      continue;
    }
    if (j.limits.velocity > 0.0) {
      lim.max_velocity[j.dof_index] = j.limits.velocity;
    }
    if (j.limits.acceleration > 0.0) {
      lim.max_acceleration[j.dof_index] = j.limits.acceleration;
    }
    if (j.limits.jerk > 0.0) {
      lim.max_jerk[j.dof_index] = j.limits.jerk;
    }
  }
  lim.tip_linear_velocity = task.max_linear_velocity;
  lim.tip_angular_velocity = task.max_angular_velocity;
  lim.tip_linear_acceleration = task.max_linear_acceleration;
  lim.tip_angular_acceleration = task.max_angular_acceleration;
  lim.tip_link = task.frame;
  return lim;
}

ParameterizationResult parametrize(const RobotModel &model, const PathSpline &path,
                                   const Limits &limits, const ParameterizationOptions &options) {
  ParameterizationResult out;
  const auto dof = static_cast<Eigen::Index>(model.dof());
  const int N = options.nodes;
  if (N < 2) {
    out.message = "parametrize: nodes must be >= 2";
    return out;
  }
  if (limits.max_velocity.size() != dof || limits.max_acceleration.size() != dof) {
    out.message = "parametrize: max_velocity/max_acceleration must be size dof";
    return out;
  }
  if (static_cast<Eigen::Index>(path.dof()) != dof) {
    out.message = "parametrize: path dof != model dof";
    return out;
  }

  const double delta = 1.0 / N;
  const bool tip_vel = limits.tip_linear_velocity > 0.0 || limits.tip_angular_velocity > 0.0;
  const bool tip_acc =
      limits.tip_linear_acceleration > 0.0 || limits.tip_angular_acceleration > 0.0;
  const std::string tip = limits.tip_link.empty() ? model.links().back().name : limits.tip_link;

  // ---- Precompute per node: path derivatives, tip rows, the MVC, acceleration rows ----------
  std::vector<NodeRows> nodes(static_cast<std::size_t>(N) + 1);
  std::vector<JointPosition> qs(nodes.size()), d1s(nodes.size()), d2s(nodes.size());
  for (int k = 0; k <= N; ++k) {
    const double s = static_cast<double>(k) / N;
    const auto ki = static_cast<std::size_t>(k);
    qs[ki] = path.eval(s);
    d1s[ki] = path.d1(s);
    d2s[ki] = path.d2(s);
    NodeRows &node = nodes[ki];

    // Joint velocity → MVC; joint acceleration → rows.
    for (Eigen::Index i = 0; i < dof; ++i) {
      const double dq = d1s[ki][i];
      if (std::abs(dq) > 1e-12 && limits.max_velocity[i] < 1e9) {
        const double bound = limits.max_velocity[i] / std::abs(dq);
        node.beta_max = std::min(node.beta_max, bound * bound);
      }
      if (limits.max_acceleration[i] > 0.0) {
        node.add(dq, d2s[ki][i], -limits.max_acceleration[i], limits.max_acceleration[i]);
      }
    }

    if (tip_vel || tip_acc) {
      const Eigen::MatrixXd J = jacobian(model, qs[ki], tip);
      const Eigen::VectorXd g = J * d1s[ki]; // tip twist per unit ṡ
      if (limits.tip_linear_velocity > 0.0) {
        const double n2 = g.head<3>().squaredNorm();
        if (n2 > 1e-16) {
          node.beta_max =
              std::min(node.beta_max, limits.tip_linear_velocity * limits.tip_linear_velocity / n2);
        }
      }
      if (limits.tip_angular_velocity > 0.0) {
        const double n2 = g.tail<3>().squaredNorm();
        if (n2 > 1e-16) {
          node.beta_max = std::min(node.beta_max,
                                   limits.tip_angular_velocity * limits.tip_angular_velocity / n2);
        }
      }
      if (tip_acc) {
        // ẍ_tip = (J·q')·u + (J·q'' + J'·q')·β — J' by central finite difference over s.
        const double h = 0.5 * delta;
        const double s_lo = std::max(0.0, s - h), s_hi = std::min(1.0, s + h);
        const Eigen::MatrixXd J_lo = jacobian(model, path.eval(s_lo), tip);
        const Eigen::MatrixXd J_hi = jacobian(model, path.eval(s_hi), tip);
        const Eigen::MatrixXd Jp = (J_hi - J_lo) / (s_hi - s_lo);
        const Eigen::VectorXd hrow = J * d2s[ki] + Jp * d1s[ki];
        // Per-axis (box) form; a scalar limit inscribes the sphere: limit/√3 per axis.
        const double alin = limits.tip_linear_acceleration / std::sqrt(3.0);
        const double aang = limits.tip_angular_acceleration / std::sqrt(3.0);
        for (int axis = 0; axis < 3; ++axis) {
          if (limits.tip_linear_acceleration > 0.0) {
            node.add(g[axis], hrow[axis], -alin, alin);
          }
          if (limits.tip_angular_acceleration > 0.0) {
            node.add(g[3 + axis], hrow[3 + axis], -aang, aang);
          }
        }
      }
    }
  }

  // ---- Backward pass: controllable intervals K_k ---------------------------------------------
  // Build the (x, y) halfplane set for interval k and project onto each axis with lp2().
  std::vector<double> K_lo(nodes.size()), K_hi(nodes.size());
  K_lo[static_cast<std::size_t>(N)] = 0.0; // rest-to-rest: β_N = 0
  K_hi[static_cast<std::size_t>(N)] = 0.0;

  const double inv2d = 1.0 / (2.0 * delta);
  const auto interval_rows = [&](int k, double y_lo, double y_hi) {
    std::vector<Halfplane> rows;
    const NodeRows &nk = nodes[static_cast<std::size_t>(k)];
    const NodeRows &nk1 = nodes[static_cast<std::size_t>(k) + 1];
    rows.reserve(2 * (nk.a.size() + nk1.a.size()) + 8);
    // Box: 0 ≤ x ≤ βmax_k, y_lo ≤ y ≤ y_hi.
    rows.push_back({-1.0, 0.0, 0.0});
    rows.push_back({1.0, 0.0, nk.beta_max});
    rows.push_back({0.0, -1.0, -y_lo});
    rows.push_back({0.0, 1.0, y_hi});
    // Rows at node k (β = x):  lo ≤ a·(y−x)·inv2d + b·x ≤ hi.
    for (std::size_t r = 0; r < nk.a.size(); ++r) {
      const double px = -nk.a[r] * inv2d + nk.b[r], py = nk.a[r] * inv2d;
      rows.push_back({px, py, nk.hi[r]});
      rows.push_back({-px, -py, -nk.lo[r]});
    }
    // Rows at node k+1 (β = y).
    for (std::size_t r = 0; r < nk1.a.size(); ++r) {
      const double px = -nk1.a[r] * inv2d, py = nk1.a[r] * inv2d + nk1.b[r];
      rows.push_back({px, py, nk1.hi[r]});
      rows.push_back({-px, -py, -nk1.lo[r]});
    }
    return rows;
  };

  for (int k = N - 1; k >= 0; --k) {
    const auto rows = interval_rows(k, K_lo[static_cast<std::size_t>(k) + 1],
                                    K_hi[static_cast<std::size_t>(k) + 1]);
    double x, y;
    if (!lp2(rows, 1.0, 0.0, x, y)) {
      out.message = "parametrize: not controllable at node " + std::to_string(k) +
                    " (limits too tight for this path)";
      return out;
    }
    K_hi[static_cast<std::size_t>(k)] = std::max(0.0, x);
    if (!lp2(rows, -1.0, 0.0, x, y)) {
      out.message = "parametrize: not controllable at node " + std::to_string(k);
      return out;
    }
    K_lo[static_cast<std::size_t>(k)] = std::max(0.0, x);
  }
  if (K_lo[0] > 1e-9) {
    out.message = "parametrize: rest start (β_0 = 0) is outside the controllable set";
    return out;
  }

  // ---- Forward pass: greedy maximal profile (time-optimal for these constraint types) --------
  std::vector<double> beta(nodes.size(), 0.0);
  beta[0] = 0.0;
  for (int k = 0; k < N; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    double y_lo = K_lo[ki + 1], y_hi = std::min(K_hi[ki + 1], nodes[ki + 1].beta_max);
    const double x = beta[ki];
    const NodeRows &nk = nodes[ki];
    const NodeRows &nk1 = nodes[ki + 1];
    const auto clip = [&](double coeff, double rhs_lo, double rhs_hi) {
      // coeff·y ∈ [rhs_lo, rhs_hi]
      if (std::abs(coeff) < 1e-14) {
        return; // row does not involve y at this x (feasibility was ensured backward)
      }
      double lo = rhs_lo / coeff, hi = rhs_hi / coeff;
      if (coeff < 0.0) {
        std::swap(lo, hi);
      }
      y_lo = std::max(y_lo, lo);
      y_hi = std::min(y_hi, hi);
    };
    for (std::size_t r = 0; r < nk.a.size(); ++r) {
      const double c = nk.a[r] * inv2d;
      const double base = -nk.a[r] * inv2d * x + nk.b[r] * x;
      clip(c, nk.lo[r] - base, nk.hi[r] - base);
    }
    for (std::size_t r = 0; r < nk1.a.size(); ++r) {
      const double c = nk1.a[r] * inv2d + nk1.b[r];
      const double base = -nk1.a[r] * inv2d * x;
      clip(c, nk1.lo[r] - base, nk1.hi[r] - base);
    }
    if (y_hi < y_lo - 1e-9) {
      out.message = "parametrize: forward pass infeasible at node " + std::to_string(k + 1) +
                    " (numerical: tighten tolerances or raise nodes)";
      return out;
    }
    beta[ki + 1] = std::max(0.0, y_hi); // greedy: ride the maximal controllable profile
  }

  // ---- Stage 2: jerk via the velocity-reduction kernel ---------------------------------------
  const bool want_jerk = options.mode == ParameterizationOptions::Mode::JerkLimited &&
                         limits.max_jerk.size() == dof && (limits.max_jerk.array() > 0.0).any();
  bool jerk_ok = true;
  if (want_jerk) {
    std::vector<JointPosition> d3s(nodes.size());
    for (int k = 0; k <= N; ++k) {
      d3s[static_cast<std::size_t>(k)] = path.d3(static_cast<double>(k) / N);
    }
    KernelInput in;
    in.N = N;
    in.delta = delta;
    in.nodes = &nodes;
    in.d1 = &d1s;
    in.d2 = &d2s;
    in.d3 = &d3s;
    in.j_max = limits.max_jerk;
    const KernelOutcome kc = jerk_limit_kernel(beta, in, options);
    out.jerk_passes = kc.passes;
    out.max_jerk_violation = kc.max_violation;
    jerk_ok = kc.converged; // the uniform fallback certifies; false only on numerical surprise
  }

  // ---- Times + trajectory --------------------------------------------------------------------
  out.s.resize(nodes.size());
  out.beta = beta;
  out.trajectory.resize(nodes.size());
  double t = 0.0;
  for (int k = 0; k <= N; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    out.s[ki] = static_cast<double>(k) / N;
    if (k > 0) {
      const double denom = std::sqrt(beta[ki - 1]) + std::sqrt(beta[ki]);
      if (denom < options.eps) {
        out.message = "parametrize: profile stalls between nodes " + std::to_string(k - 1) +
                      " and " + std::to_string(k);
        out.trajectory.clear();
        return out;
      }
      t += 2.0 * delta / denom;
    }
    const double u =
        (k < N) ? (beta[ki + 1] - beta[ki]) * inv2d : (beta[ki] - beta[ki - 1]) * inv2d;
    Waypoint &w = out.trajectory[ki];
    w.time = t;
    w.state.pos = qs[ki];
    w.state.vel = d1s[ki] * std::sqrt(beta[ki]);
    w.state.acc = d1s[ki] * u + d2s[ki] * beta[ki];
  }
  out.duration = t;
  out.success = jerk_ok;
  if (!jerk_ok) {
    out.message = "jerk kernel did not certify: residual violation " +
                  std::to_string(out.max_jerk_violation * 100.0) + "% (numerical — raise nodes)";
  } else if (want_jerk) {
    out.message = "solved (convex + jerk-kernel phases)";
  } else if (options.mode == ParameterizationOptions::Mode::JerkLimited) {
    out.message = "solved (convex phase; no jerk limits set)";
  } else {
    out.message = "solved (convex phase)";
  }
  return out;
}

} // namespace quevedomp::parameterization
