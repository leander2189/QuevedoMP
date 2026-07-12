// Task 3.3d P3 verify — Cartesian-bounded edge discretization. Covers: lever weights analytic on
// a planar 2R arm + a revolute-over-prismatic chain (incl. a geometry-free wrist getting weight
// 0), EdgeDiscretization::steps in both modes (fallback identical to the pre-P3 formula), the
// factory's validation, the SWEEP GUARANTEE itself (FK-tracked collision-geometry points move
// ≤ max_link_sweep between consecutive samples on random edges — the property the whole feature
// sells), and planner/smoother integration (sweep-mode plan solves, re-validates, deterministic).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/smoother.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using quevedomp::collision::EdgeDiscretization;

namespace {

// Planar 2R arm + a geometry-free "wrist" leaf: link1 is a 1 m box (centred at x=0.5), link2 a
// sphere (r=0.1) offset 0.5 m from joint2, joint3 carries nothing collidable. Analytic weights:
//   w3 = 0                                     (no distal geometry)
//   w2 = 0.5 + 0.1                     = 0.6   (sphere centre offset + radius)
//   w1 = max(0.5 + |(0.5,0.05,0.05)|,  1.0 + w2) = 1.6   (box radius vs joint2 offset + reach)
const char *kPlanar2R = R"(<robot name="planar2r">
  <link name="base"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="l1">
    <collision><origin xyz="0.5 0 0"/><geometry><box size="1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="l2">
    <collision><origin xyz="0.5 0 0"/><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="wrist"/>
    <origin xyz="0.6 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="wrist"/>
</robot>)";

// Revolute base carrying a prismatic slide (travel −0.5..1.5) that carries a sphere:
//   w_jp = 1 (prismatic: metres per metre), w_jr = max travel + sphere reach = 1.5 + 0.1.
const char *kRevoluteOverPrismatic = R"(<robot name="revprism">
  <link name="base"/>
  <joint name="jr" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="l1"/>
  <joint name="jp" type="prismatic">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="1.5" effort="10" velocity="1"/>
  </joint>
  <link name="l2">
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
</robot>)";

JointPosition qv(std::initializer_list<double> vals) {
  JointPosition q(static_cast<Eigen::Index>(vals.size()));
  Eigen::Index i = 0;
  for (const double v : vals)
    q[i++] = v;
  return q;
}

} // namespace

// ---- lever weights -------------------------------------------------------------------------

TEST(LeverWeights, Planar2RAnalytic) {
  const auto model = RobotModel::from_urdf(kPlanar2R);
  const JointPosition w = collision::cartesian_lever_weights(*model);
  ASSERT_EQ(w.size(), 3);
  EXPECT_NEAR(w[0], 1.6, 1e-12);
  EXPECT_NEAR(w[1], 0.6, 1e-12);
  EXPECT_DOUBLE_EQ(w[2], 0.0); // nothing collidable distal of the wrist
}

TEST(LeverWeights, PrismaticIsUnitAndAddsTravelToParent) {
  const auto model = RobotModel::from_urdf(kRevoluteOverPrismatic);
  const JointPosition w = collision::cartesian_lever_weights(*model);
  ASSERT_EQ(w.size(), 2);
  // Look dof indices up by name — urdfdom does not preserve declaration order of joints.
  EXPECT_NEAR(w[model->find_joint("jr")->dof_index], 1.6, 1e-12); // |travel 1.5| + sphere 0.1
  EXPECT_DOUBLE_EQ(w[model->find_joint("jp")->dof_index], 1.0);
}

// ---- steps ---------------------------------------------------------------------------------

TEST(EdgeDiscretizationSteps, FallbackMatchesPreP3Formula) {
  EdgeDiscretization d;
  d.joint_resolution = 0.05;
  EXPECT_EQ(d.steps(qv({0.12, -0.03})), 3); // ceil(0.12 / 0.05)
  EXPECT_EQ(d.steps(qv({0.0, 0.0})), 1);    // degenerate edge still yields one segment
}

TEST(EdgeDiscretizationSteps, SweepModeWeighsJointsByLever) {
  EdgeDiscretization d;
  d.max_link_sweep = 0.005;
  d.lever_weights = qv({1.6, 0.6});
  // Σ w·|Δq| = 0.176 + 0.12 = 0.296 m -> ceil(59.2) = 60 steps of ≤ 5 mm. (Deltas chosen off the
  // exact step boundary — ceil at a boundary is one ulp from flipping.)
  EXPECT_EQ(d.steps(qv({0.11, -0.2})), 60);
  // A pure low-lever move needs far fewer steps than the uniform rule would spend.
  EXPECT_EQ(d.steps(qv({0.0, 0.11})), 14); // ceil(13.2)
  EXPECT_EQ(d.steps(qv({0.0, 0.0})), 1);
}

TEST(EdgeDiscretizationSteps, SweepModeWrongWeightSizeThrows) {
  EdgeDiscretization d;
  d.max_link_sweep = 0.005;
  d.lever_weights = qv({1.0});
  EXPECT_THROW((void)d.steps(qv({0.1, 0.2})), std::runtime_error);
}

TEST(MakeEdgeDiscretization, ComputesWeightsValidatesSizes) {
  const auto model = RobotModel::from_urdf(kPlanar2R);
  const auto d =
      collision::make_edge_discretization(0.05, 0.005, JointPosition{}, *model); // auto weights
  ASSERT_EQ(d.lever_weights.size(), 3);
  EXPECT_NEAR(d.lever_weights[0], 1.6, 1e-12);

  EXPECT_THROW((void)collision::make_edge_discretization(0.05, 0.005, qv({1.0}), *model),
               std::runtime_error);
  EXPECT_THROW((void)collision::make_edge_discretization(0.0, 0.0, JointPosition{}, *model),
               std::runtime_error);
  // Sweep off: weights left empty, no computation attempted.
  EXPECT_EQ(
      collision::make_edge_discretization(0.05, 0.0, JointPosition{}, *model).lever_weights.size(),
      0);
}

// ---- the sweep guarantee -------------------------------------------------------------------

// The property the feature sells: between consecutive edge samples, NO tracked point of the
// robot's collision geometry moves more than max_link_sweep. Tracked points are the extreme
// points of each shape (box corners; sphere centre ± radius along each axis), rigidly attached —
// a rigid displacement over a convex shape is maximized at an extreme point, so these witness it.
TEST(SweepGuarantee, HoldsOnRandomEdges) {
  const auto model = RobotModel::from_urdf(kPlanar2R);
  EdgeDiscretization d;
  d.max_link_sweep = 0.02;
  d.lever_weights = collision::cartesian_lever_weights(*model);

  // (link name, point in link frame)
  std::vector<std::pair<std::string, Eigen::Vector3d>> points;
  for (const double sx : {-0.5, 0.5})
    for (const double sy : {-0.05, 0.05})
      for (const double sz : {-0.05, 0.05})
        points.emplace_back("l1", Eigen::Vector3d(0.5 + sx, sy, sz));
  points.emplace_back("l2", Eigen::Vector3d(0.5, 0, 0));
  for (int axis = 0; axis < 3; ++axis)
    for (const double s : {-0.1, 0.1})
      points.emplace_back("l2", Eigen::Vector3d(0.5, 0, 0) + s * Eigen::Vector3d::Unit(axis));

  std::mt19937 gen(7);
  std::uniform_real_distribution<double> u(-3.14, 3.14);
  for (int trial = 0; trial < 50; ++trial) {
    const JointPosition q0 = qv({u(gen), u(gen), u(gen)});
    const JointPosition q1 = qv({u(gen), u(gen), u(gen)});
    const JointPosition delta = q1 - q0;
    const int n = d.steps(delta);
    auto prev = fk_all(*model, q0);
    for (int k = 1; k <= n; ++k) {
      const auto curr = fk_all(*model, q0 + (static_cast<double>(k) / n) * delta);
      for (const auto &[link, p] : points) {
        const int li = static_cast<int>(model->find_link(link) - model->links().data());
        const double moved =
            (curr[static_cast<std::size_t>(li)] * p - prev[static_cast<std::size_t>(li)] * p)
                .norm();
        EXPECT_LE(moved, d.max_link_sweep * (1.0 + 1e-9))
            << "trial " << trial << " step " << k << " link " << link;
      }
      prev = curr;
    }
  }
}

// ---- planner + smoother integration ---------------------------------------------------------

namespace {

// The test_planner 2-DOF gantry: q = (x, y) is the sphere end-effector position, with a wall
// blocking x≈0 below y = 0.5 so the straight line at y=-1 fails but a detour over the top exists.
const char *kGantry2D = R"(<robot name="gantry2d">
  <link name="base"/>
  <joint name="jx" type="prismatic">
    <parent link="base"/><child link="cx"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
  <link name="cx"/>
  <joint name="jy" type="prismatic">
    <parent link="cx"/><child link="ee"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
  <link name="ee">
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
</robot>)";

} // namespace

TEST(PlannerSweepMode, SolvesRevalidatesAndIsDeterministic) {
  const auto model = RobotModel::from_urdf(kGantry2D);
  const auto robot = std::make_shared<const RobotInstance>(model);
  collision::SceneDescription env;
  collision::BoxShape wall;
  wall.half_extents = Eigen::Vector3d(0.1, 1.25, 0.5);
  env.objects.push_back({"wall", wall, Transform::from_translation(Eigen::Vector3d(0, -0.75, 0))});
  const std::shared_ptr<collision::CollisionScene> scene = collision::make_static_scene(model, env);

  planning::PlannerParams params;
  params.max_link_sweep = 0.01; // 1 cm sweep bound; both gantry weights are exactly 1 m/m
  params.goal_bias = 0.1;

  planning::PlanningProblem problem;
  problem.start = qv({-1.0, -1.0});
  problem.goal = std::make_shared<planning::JointGoal>(qv({1.0, -1.0}), 1e-3);
  problem.timeout = 10.0;
  problem.seed = 42;

  const auto planner = planning::make_planner(params, robot, scene);
  const auto r1 = planner->plan(problem);
  ASSERT_EQ(r1.status, planning::PlanningStatus::Success) << r1.message;

  // Re-validate through the check_edge policy overload (a separate code path from the planner's
  // internal batched checker).
  EdgeDiscretization disc =
      collision::make_edge_discretization(0.05, params.max_link_sweep, JointPosition{}, *model);
  const auto ws = scene->make_workspace();
  const collision::QueryOptions opts;
  for (std::size_t i = 0; i + 1 < r1.path.size(); ++i) {
    const auto e =
        collision::check_edge(*scene, *robot, r1.path[i], r1.path[i + 1], disc, opts, *ws);
    EXPECT_TRUE(e.valid) << "edge " << i << " collides at t=" << e.first_contact_t;
  }

  const auto r2 = planner->plan(problem);
  ASSERT_EQ(r2.status, planning::PlanningStatus::Success);
  ASSERT_EQ(r1.path.size(), r2.path.size());
  for (std::size_t i = 0; i < r1.path.size(); ++i)
    EXPECT_TRUE(r1.path[i].isApprox(r2.path[i]));

  // Smoother in the same mode: output stays collision-free and no longer than the input.
  planning::SmootherParams sp;
  sp.max_link_sweep = params.max_link_sweep;
  sp.seed = 42;
  const auto smoother = planning::make_shortcut_smoother(sp, robot, scene);
  const auto smoothed = smoother->smooth(r1.path);
  ASSERT_GE(smoothed.size(), 2u);
  EXPECT_LE(smoothed.size(), r1.path.size());
  for (std::size_t i = 0; i + 1 < smoothed.size(); ++i) {
    const auto e =
        collision::check_edge(*scene, *robot, smoothed[i], smoothed[i + 1], disc, opts, *ws);
    EXPECT_TRUE(e.valid) << "smoothed edge " << i;
  }
}
