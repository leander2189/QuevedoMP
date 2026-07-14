// parameterization/PathSpline — see path_spline.hpp.
#include "quevedomp/parameterization/path_spline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <unsupported/Eigen/Splines>

namespace quevedomp::parameterization {
namespace {

// One fixed-degree 1-D spline PER JOINT, sharing one knot vector. Mathematically identical to a
// joint n-D fit (B-spline interpolation is per-coordinate linear) — and necessary: Eigen 3.4's
// unsupported Spline module asserts inside operator() for a Dynamic-dimension spline
// (Replicate<..., Dynamic, 1> in the basis broadcast).
using Spline1d = Eigen::Spline<double, 1, 5>;

// Drop consecutive duplicates (they break the strictly-increasing chord-length knots).
planning::Path dedup(const planning::Path &in) {
  planning::Path out;
  for (const JointPosition &q : in) {
    if (out.empty() || (q - out.back()).norm() > 1e-12) {
      out.push_back(q);
    }
  }
  return out;
}

// Subdivide every polyline segment once (midpoints stay ON the collision-validated polyline).
planning::Path densify(const planning::Path &in) {
  planning::Path out;
  out.reserve(in.size() * 2);
  for (std::size_t i = 0; i + 1 < in.size(); ++i) {
    out.push_back(in[i]);
    out.push_back(0.5 * (in[i] + in[i + 1]));
  }
  out.push_back(in.back());
  return out;
}

} // namespace

struct PathSpline::Impl {
  std::vector<Spline1d> joints; // one spline per joint, shared knots
  std::vector<double> params;   // chord-length parameter of each interpolated waypoint
  std::size_t dof = 0;

  JointPosition derivative(double s, int order) const {
    JointPosition out(static_cast<Eigen::Index>(dof));
    for (std::size_t i = 0; i < dof; ++i) {
      out[static_cast<Eigen::Index>(i)] = joints[i].derivatives(s, order)(0, order);
    }
    return out;
  }
};

PathSpline::PathSpline() : impl_(std::make_unique<Impl>()) {}
PathSpline::PathSpline(PathSpline &&) noexcept = default;
PathSpline &PathSpline::operator=(PathSpline &&) noexcept = default;
PathSpline::~PathSpline() = default;

PathSpline PathSpline::fit(const planning::Path &waypoints) {
  planning::Path pts = dedup(waypoints);
  if (pts.size() < 2) {
    throw std::runtime_error("PathSpline::fit: need >= 2 distinct waypoints");
  }
  const auto dof = static_cast<std::size_t>(pts.front().size());
  for (const JointPosition &q : pts) {
    if (static_cast<std::size_t>(q.size()) != dof) {
      throw std::runtime_error("PathSpline::fit: inconsistent waypoint sizes");
    }
  }
  // Degree 5 (C⁴) needs >= 6 interpolation points; densify along the polyline until we have them
  // (added points lie ON the polyline, so they change nothing geometrically).
  while (pts.size() < 6) {
    pts = densify(pts);
  }

  // Chord-length parameterization, normalized to s ∈ [0, 1].
  std::vector<double> params(pts.size(), 0.0);
  for (std::size_t i = 1; i < pts.size(); ++i) {
    params[i] = params[i - 1] + (pts[i] - pts[i - 1]).norm();
  }
  const double total = params.back();
  if (!(total > 0.0)) {
    throw std::runtime_error("PathSpline::fit: zero-length path");
  }
  for (double &p : params) {
    p /= total;
  }

  Eigen::RowVectorXd knots(static_cast<Eigen::Index>(params.size()));
  for (std::size_t i = 0; i < params.size(); ++i) {
    knots[static_cast<Eigen::Index>(i)] = params[i];
  }

  PathSpline out;
  out.impl_->joints.reserve(dof);
  Eigen::RowVectorXd vals(static_cast<Eigen::Index>(pts.size()));
  for (std::size_t j = 0; j < dof; ++j) {
    for (std::size_t i = 0; i < pts.size(); ++i) {
      vals[static_cast<Eigen::Index>(i)] = pts[i][static_cast<Eigen::Index>(j)];
    }
    out.impl_->joints.push_back(Eigen::SplineFitting<Spline1d>::Interpolate(vals, 5, knots));
  }
  out.impl_->params = std::move(params);
  out.impl_->dof = dof;
  return out;
}

JointPosition PathSpline::eval(double s) const {
  return impl_->derivative(std::clamp(s, 0.0, 1.0), 0);
}
JointPosition PathSpline::d1(double s) const {
  return impl_->derivative(std::clamp(s, 0.0, 1.0), 1);
}
JointPosition PathSpline::d2(double s) const {
  return impl_->derivative(std::clamp(s, 0.0, 1.0), 2);
}
JointPosition PathSpline::d3(double s) const {
  return impl_->derivative(std::clamp(s, 0.0, 1.0), 3);
}
std::size_t PathSpline::dof() const noexcept { return impl_->dof; }
const std::vector<double> &PathSpline::waypoint_params() const noexcept { return impl_->params; }

SplineFitResult
fit_collision_free(const planning::Path &waypoints, const collision::CollisionScene &scene,
                   const RobotInstance &robot, const collision::EdgeDiscretization &disc,
                   const collision::QueryOptions &opts, collision::Workspace &ws, int max_rounds) {
  SplineFitResult result;
  planning::Path pts = waypoints;
  double first_bad_s = -1.0;

  for (int round = 1; round <= max_rounds; ++round) {
    PathSpline sp = PathSpline::fit(pts);

    // Sample the curve at edge fidelity: refine intervals until consecutive samples are within
    // one discretization step (the P3 guarantee planner edges get). Depth-bounded for safety.
    std::vector<double> ss;
    const auto refine = [&](auto &&self, double sa, const JointPosition &qa, double sb,
                            const JointPosition &qb, int depth) -> void {
      if (depth < 24 && disc.steps(qb - qa) > 1) {
        const double sm = 0.5 * (sa + sb);
        const JointPosition qm = sp.eval(sm);
        self(self, sa, qa, sm, qm, depth + 1);
        self(self, sm, qm, sb, qb, depth + 1);
        return;
      }
      ss.push_back(sb);
    };
    ss.push_back(0.0);
    const auto &wp = sp.waypoint_params();
    for (std::size_t i = 0; i + 1 < wp.size(); ++i) {
      refine(refine, wp[i], sp.eval(wp[i]), wp[i + 1], sp.eval(wp[i + 1]), 0);
    }

    std::vector<JointPosition> samples;
    samples.reserve(ss.size());
    for (const double s : ss) {
      samples.push_back(sp.eval(s));
    }
    const collision::BatchResult br = scene.query_batch(robot, samples, opts, ws);
    result.checked_samples += samples.size();

    first_bad_s = -1.0;
    for (std::size_t i = 0; i < ss.size(); ++i) {
      if (br.in_collision[i] != 0) {
        first_bad_s = ss[i];
        break;
      }
    }
    if (first_bad_s < 0.0) {
      result.success = true;
      result.spline.emplace(std::move(sp));
      result.rounds = round;
      return result;
    }
    // The spline cut a corner into an obstacle: densify the interpolation points along the
    // (collision-free) polyline and refit — the spline converges to the polyline as knots grow.
    pts = densify(pts);
  }

  result.message = "fit_collision_free: spline still collides after " + std::to_string(max_rounds) +
                   " densify rounds (first collision at s=" + std::to_string(first_bad_s) +
                   "); parameterize the polyline without jerk rows";
  return result;
}

} // namespace quevedomp::parameterization
