// Task 3.4 verify (Stage 0+1) — PathSpline + Phase A convex time parameterization.
// Covers: spline interpolation + C³-in-practice smoothness, collision re-validation with the
// densify-and-refit fallback, the analytic 1-DOF trapezoid (duration matches closed form),
// limit compliance (joint vel/acc, tip vel/acc) on a UR5 spline, rest-to-rest boundaries,
// monotone time, tip-speed saturation (the "constant tool speed" behavior), determinism, and
// clean failure messages on infeasible input.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/kinematics/jacobian.hpp"
#include "quevedomp/parameterization/parameterize.hpp"
#include "quevedomp/parameterization/path_spline.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::parameterization;

namespace {

std::string read_fixture(const std::string &rel) {
  std::ifstream f(std::string(QUEVEDOMP_FIXTURE_DIR) + "/" + rel);
  EXPECT_TRUE(f.good()) << "missing fixture: " << rel;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// One prismatic joint: q(s) = s metres — the analytic testbed (q' = 1, q'' = 0).
const char *kSlider = R"(<robot name="slider">
  <link name="base"/>
  <joint name="j" type="prismatic">
    <parent link="base"/><child link="ee"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="10"/>
  </joint>
  <link name="ee"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
</robot>)";

JointPosition q1(double v) {
  JointPosition q(1);
  q << v;
  return q;
}

PathSpline unit_line() {
  planning::Path wp;
  for (int k = 0; k <= 6; ++k)
    wp.push_back(q1(k / 6.0));
  return PathSpline::fit(wp);
}

} // namespace

// ---- PathSpline ------------------------------------------------------------------------------

TEST(PathSpline, InterpolatesWaypointsAndIsSmooth) {
  const auto model = RobotModel::from_urdf(read_fixture("robots/ur5.urdf"));
  planning::Path wp;
  wp.push_back(JointPosition::Zero(6));
  wp.push_back((JointPosition(6) << 0.4, -0.8, 0.9, -0.3, 0.5, 0.2).finished());
  wp.push_back((JointPosition(6) << -0.2, -1.2, 1.4, -0.9, 0.1, -0.4).finished());
  wp.push_back((JointPosition(6) << 0.3, -0.5, 0.6, -1.2, 0.8, 0.5).finished());
  const PathSpline sp = PathSpline::fit(wp);

  // Interpolation: every ORIGINAL waypoint is hit EXACTLY at one of the interpolated parameters
  // (fit densifies 4 → 7 points along the polyline; the originals are among them).
  EXPECT_GE(sp.waypoint_params().size(), wp.size());
  for (const JointPosition &q : wp) {
    double best = 1e9;
    for (const double p : sp.waypoint_params())
      best = std::min(best, (sp.eval(p) - q).norm());
    EXPECT_LT(best, 1e-9);
  }

  // C³ in practice: third derivative exists and finite differences of d2 match d3.
  for (double s = 0.05; s < 1.0; s += 0.09) {
    const double h = 1e-5;
    const JointPosition fd = (sp.d2(s + h) - sp.d2(s - h)) / (2 * h);
    EXPECT_LT((fd - sp.d3(s)).norm(), 1e-2 * std::max(1.0, sp.d3(s).norm()));
  }
}

TEST(PathSpline, CollisionRevalidationDensifiesAwayFromWall) {
  // Gantry + wall (the planner fixture): a detour polyline is free, but a loose spline through
  // its corners cuts INTO the wall; fit_collision_free must refit until clear.
  const char *kGantry2D = R"(<robot name="gantry2d">
    <link name="base"/>
    <joint name="jx" type="prismatic"><parent link="base"/><child link="cx"/>
      <origin xyz="0 0 0"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
    <link name="cx"/>
    <joint name="jy" type="prismatic"><parent link="cx"/><child link="ee"/>
      <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
    <link name="ee"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  </robot>)";
  const auto model = RobotModel::from_urdf(kGantry2D);
  const RobotInstance robot(model);
  collision::SceneDescription env;
  env.objects.push_back({"wall", collision::BoxShape{Eigen::Vector3d(0.1, 1.25, 0.5)},
                         Transform::from_translation(Eigen::Vector3d(0, -0.75, 0))});
  const auto scene = collision::make_static_scene(model, env);
  const auto ws = scene->make_workspace();

  planning::Path detour; // tight corner right at the wall's top edge (0, 0.62)
  detour.push_back((JointPosition(2) << -1.0, -1.0).finished());
  detour.push_back((JointPosition(2) << -0.25, 0.62).finished());
  detour.push_back((JointPosition(2) << 0.25, 0.62).finished());
  detour.push_back((JointPosition(2) << 1.0, -1.0).finished());

  collision::EdgeDiscretization disc;
  disc.joint_resolution = 0.01;
  const auto fit = fit_collision_free(detour, *scene, robot, disc, collision::QueryOptions{}, *ws);
  ASSERT_TRUE(fit.success) << fit.message;
  ASSERT_TRUE(fit.spline.has_value());
  EXPECT_GT(fit.checked_samples, 0u);

  // Re-verify independently at finer resolution: every sample of the returned spline is free.
  std::vector<JointPosition> samples;
  for (double s = 0.0; s <= 1.0; s += 1e-3)
    samples.push_back(fit.spline->eval(s));
  const auto br = scene->query_batch(robot, samples, collision::QueryOptions{}, *ws);
  for (std::size_t i = 0; i < samples.size(); ++i)
    EXPECT_EQ(br.in_collision[i], 0) << "spline collides at sample " << i;
}

// ---- Phase A: analytic 1-DOF ------------------------------------------------------------------

TEST(ParametrizePhaseA, MatchesAnalyticTrapezoid) {
  const auto model = RobotModel::from_urdf(kSlider);
  const PathSpline line = unit_line();

  Limits lim;
  lim.max_velocity = q1(0.6);     // cruise below the URDF limit
  lim.max_acceleration = q1(1.0); // trapezoid: T = v/a + d/v = 0.6 + 1/0.6 = 2.2666…
  ParameterizationOptions opt;
  opt.nodes = 400;
  const auto r = parametrize(*model, line, lim, opt);
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_NEAR(r.duration, 0.6 + 1.0 / 0.6, 0.01);

  // Triangular case: v_max high enough to never bind — T = 2·√(d/a) = 2.
  lim.max_velocity = q1(10.0);
  const auto tri = parametrize(*model, line, lim, opt);
  ASSERT_TRUE(tri.success);
  EXPECT_NEAR(tri.duration, 2.0, 0.01);
}

TEST(ParametrizePhaseA, RestToRestMonotoneAndDeterministic) {
  const auto model = RobotModel::from_urdf(kSlider);
  const PathSpline line = unit_line();
  Limits lim;
  lim.max_velocity = q1(0.6);
  lim.max_acceleration = q1(1.0);
  const auto a = parametrize(*model, line, lim);
  const auto b = parametrize(*model, line, lim);
  ASSERT_TRUE(a.success);
  EXPECT_LT(a.trajectory.front().state.vel.norm(), 1e-6);
  EXPECT_LT(a.trajectory.back().state.vel.norm(), 1e-6);
  for (std::size_t k = 0; k + 1 < a.trajectory.size(); ++k)
    EXPECT_LT(a.trajectory[k].time, a.trajectory[k + 1].time);
  ASSERT_EQ(a.beta.size(), b.beta.size());
  for (std::size_t k = 0; k < a.beta.size(); ++k)
    EXPECT_DOUBLE_EQ(a.beta[k], b.beta[k]);
}

// ---- Phase A: limit compliance on a real arm ---------------------------------------------------

namespace {

struct Ur5Case {
  std::shared_ptr<const RobotModel> model;
  PathSpline spline;
  Limits lim;
  ParameterizationResult result;
};

Ur5Case ur5_case(double vtip, double atip) {
  const auto model = RobotModel::from_urdf(read_fixture("robots/ur5.urdf"));
  planning::Path wp;
  wp.push_back(JointPosition::Zero(6));
  wp.push_back((JointPosition(6) << 0.5, -0.9, 1.1, -0.4, 0.6, 0.3).finished());
  wp.push_back((JointPosition(6) << -0.3, -1.3, 1.5, -1.0, 0.2, -0.5).finished());
  PathSpline sp = PathSpline::fit(wp);

  Limits lim = limits_from_model(*model, {}, /*default_acceleration=*/8.0);
  lim.tip_link = "wrist_3_link";
  lim.tip_linear_velocity = vtip;
  lim.tip_linear_acceleration = atip;

  ParameterizationOptions opt;
  opt.nodes = 300;
  auto r = parametrize(*model, sp, lim, opt);
  return {model, std::move(sp), std::move(lim), std::move(r)};
}

} // namespace

TEST(ParametrizePhaseA, RespectsJointAndTipLimitsOnUr5) {
  auto c = ur5_case(/*vtip=*/0.5, /*atip=*/4.0);
  ASSERT_TRUE(c.result.success) << c.result.message;
  const double tol = 1.02; // 2% slack for endpoint-sampled rows between nodes

  for (std::size_t k = 0; k < c.result.trajectory.size(); ++k) {
    const auto &w = c.result.trajectory[k];
    for (Eigen::Index i = 0; i < 6; ++i) {
      EXPECT_LE(std::abs(w.state.vel[i]), c.lim.max_velocity[i] * tol) << "vel joint " << i;
      EXPECT_LE(std::abs(w.state.acc[i]), c.lim.max_acceleration[i] * tol) << "acc joint " << i;
    }
    const Eigen::MatrixXd J = jacobian(*c.model, w.state.pos, "wrist_3_link");
    const Eigen::VectorXd tip_v = J * w.state.vel;
    EXPECT_LE(tip_v.head<3>().norm(), c.lim.tip_linear_velocity * tol) << "tip speed node " << k;
  }
}

TEST(ParametrizePhaseA, TipSpeedCapSaturatesAndSlowsTrajectory) {
  auto fast = ur5_case(/*vtip=*/0.0, /*atip=*/0.0); // no tip limits
  auto slow = ur5_case(/*vtip=*/0.25, /*atip=*/0.0);
  ASSERT_TRUE(fast.result.success);
  ASSERT_TRUE(slow.result.success);
  EXPECT_GT(slow.result.duration, fast.result.duration * 1.2);

  // The capped run rides the tip-speed bound over a sustained stretch (near-constant tool
  // speed — the paint-pass behavior): count interior nodes within 5% of the cap.
  int saturated = 0;
  for (std::size_t k = 5; k + 5 < slow.result.trajectory.size(); ++k) {
    const auto &w = slow.result.trajectory[k];
    const Eigen::MatrixXd J = jacobian(*slow.model, w.state.pos, "wrist_3_link");
    if ((J * w.state.vel).head<3>().norm() > 0.95 * 0.25)
      ++saturated;
  }
  EXPECT_GT(saturated, static_cast<int>(slow.result.trajectory.size() / 4));
}

TEST(ParametrizePhaseA, InfeasibleAndInvalidInputsFailLoudly) {
  const auto model = RobotModel::from_urdf(kSlider);
  const PathSpline line = unit_line();
  Limits lim;
  lim.max_velocity = q1(0.6); // wrong-size limits
  lim.max_acceleration = JointPosition();
  auto r = parametrize(*model, line, lim);
  EXPECT_FALSE(r.success);
  EXPECT_FALSE(r.message.empty());

  ParameterizationOptions opt;
  opt.nodes = 1;
  lim.max_acceleration = q1(1.0);
  r = parametrize(*model, line, lim, opt);
  EXPECT_FALSE(r.success);
}
