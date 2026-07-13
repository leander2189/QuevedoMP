// Task 3.3 verify — ShortcutSmoother: output is still collision-free and no longer than the input
// (the build-plan Done-when), plus determinism per seed, endpoint preservation, actual shortening
// of a redundant path in free space, and null-arg / short-path handling.
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/planning/smoother.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::planning;

namespace {

// Same 2-DOF planar gantry as the planner test: q = (x, y) is the sphere end-effector position.
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

JointPosition q2(double x, double y) {
  JointPosition q(2);
  q << x, y;
  return q;
}

struct Fixture {
  std::shared_ptr<const RobotModel> model;
  std::shared_ptr<const RobotInstance> robot;
  std::shared_ptr<collision::CollisionScene> scene;
};

Fixture make_fixture(bool with_wall) {
  Fixture f;
  f.model = RobotModel::from_urdf(kGantry2D);
  f.robot = std::make_shared<const RobotInstance>(f.model);
  collision::SceneDescription env;
  if (with_wall) {
    collision::BoxShape wall;
    wall.half_extents = Eigen::Vector3d(0.1, 1.25, 0.5); // spans y∈[-2,0.5] at x≈0
    env.objects.push_back(
        {"wall", wall, Transform::from_translation(Eigen::Vector3d(0, -0.75, 0))});
  }
  f.scene = make_static_scene(f.model, env);
  return f;
}

double path_length(const Path &p) {
  double len = 0.0;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    len += (p[i + 1] - p[i]).norm();
  }
  return len;
}

void expect_collision_free(const Fixture &f, const Path &path, double res) {
  const auto ws = f.scene->make_workspace();
  collision::QueryOptions opts;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto e = collision::check_edge(*f.scene, *f.robot, path[i], path[i + 1],
                                         static_cast<float>(res), opts, *ws);
    EXPECT_TRUE(e.valid) << "edge " << i << " collides at t=" << e.first_contact_t;
  }
}

SmootherParams params(std::uint64_t seed) {
  SmootherParams p;
  p.edge_resolution = 0.05;
  p.max_iterations = 200;
  p.seed = seed;
  return p;
}

} // namespace

TEST(ShortcutSmoother, NullArgsThrow) {
  auto f = make_fixture(false);
  EXPECT_THROW(make_shortcut_smoother(params(1), nullptr, f.scene), std::runtime_error);
  EXPECT_THROW(make_shortcut_smoother(params(1), f.robot, nullptr), std::runtime_error);
}

TEST(ShortcutSmoother, ShortPathUnchanged) {
  auto f = make_fixture(false);
  const auto sm = make_shortcut_smoother(params(1), f.robot, f.scene);
  const Path two = {q2(-1, -1), q2(1, 1)};
  const Path out = sm->smooth(two);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_LT((out[0] - two[0]).norm(), 1e-12);
  EXPECT_LT((out[1] - two[1]).norm(), 1e-12);
}

TEST(ShortcutSmoother, CollapsesRedundantFreePath) {
  auto f = make_fixture(/*with_wall=*/false);
  const auto sm = make_shortcut_smoother(params(42), f.robot, f.scene);
  // A zig-zag in free space; the direct chord start→goal is free, so it should collapse hard.
  const Path in = {q2(-1, -1), q2(-1, 1), q2(1, 1), q2(1, -1)};
  const Path out = sm->smooth(in);

  ASSERT_GE(out.size(), 2u);
  EXPECT_LT(path_length(out), path_length(in)); // strictly shorter
  EXPECT_LE(path_length(out), path_length(in) + 1e-9);
  // Endpoints preserved.
  EXPECT_LT((out.front() - in.front()).norm(), 1e-12);
  EXPECT_LT((out.back() - in.back()).norm(), 1e-12);
  expect_collision_free(f, out, 0.05);
}

TEST(ShortcutSmoother, PreservesCollisionFreeAroundWall) {
  auto f = make_fixture(/*with_wall=*/true);
  const auto sm = make_shortcut_smoother(params(7), f.robot, f.scene);
  // A valid detour over the wall with a redundant node (-0.5,1) on the top leg.
  const Path in = {q2(-1, -1), q2(-1, 1), q2(-0.5, 1), q2(1, 1), q2(1, -1)};
  expect_collision_free(f, in, 0.05); // sanity: the input really is free
  const Path out = sm->smooth(in);

  EXPECT_LE(path_length(out), path_length(in) + 1e-9); // never longer
  EXPECT_LT((out.front() - in.front()).norm(), 1e-12);
  EXPECT_LT((out.back() - in.back()).norm(), 1e-12);
  expect_collision_free(f, out, 0.05); // still free — the shortcut across the wall must be rejected
}

// ---- Task 3.3d P6: batched rounds + time budget ----------------------------------------------

TEST(ShortcutSmoother, SingleChordModeStillShortens) {
  auto f = make_fixture(false);
  SmootherParams p = params(42);
  p.batch_size = 1; // the pre-P6 smoother, draw-for-draw
  const auto sm = make_shortcut_smoother(p, f.robot, f.scene);
  const Path in = {q2(-1, -1), q2(-1, 1), q2(1, 1), q2(1, -1)};
  const Path out = sm->smooth(in);
  EXPECT_LT(path_length(out), path_length(in));
  expect_collision_free(f, out, 0.05);
}

TEST(ShortcutSmoother, BatchedRoundAcceptsMultipleDisjointChords) {
  auto f = make_fixture(false);
  SmootherParams p = params(11);
  p.batch_size = 8;
  p.max_iterations = 8; // ONE round's worth of attempts — shortening must come from that round
  const auto sm = make_shortcut_smoother(p, f.robot, f.scene);
  // A long zig-zag in free space: plenty of disjoint shortcut opportunities in a single round.
  Path in;
  for (int k = 0; k <= 10; ++k) {
    in.push_back(q2(-1.0 + 0.2 * k, (k % 2 == 0) ? -1.0 : 1.0));
  }
  const Path out = sm->smooth(in);
  EXPECT_LT(out.size(), in.size()); // at least one chord accepted within the single round
  EXPECT_LT(path_length(out), path_length(in));
  EXPECT_LT((out.front() - in.front()).norm(), 1e-12);
  EXPECT_LT((out.back() - in.back()).norm(), 1e-12);
  expect_collision_free(f, out, 0.05);
}

TEST(ShortcutSmoother, TinyTimeBudgetReturnsInputUnchanged) {
  auto f = make_fixture(false);
  SmootherParams p = params(3);
  p.time_budget = 1e-12; // trips before the first round: anytime contract, zero polish
  const auto sm = make_shortcut_smoother(p, f.robot, f.scene);
  const Path in = {q2(-1, -1), q2(-1, 1), q2(1, 1), q2(1, -1)};
  const Path out = sm->smooth(in);
  ASSERT_EQ(out.size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    EXPECT_LT((out[i] - in[i]).norm(), 1e-12);
  }
}

TEST(ShortcutSmoother, DeterministicPerSeed) {
  auto f = make_fixture(false);
  const auto sm = make_shortcut_smoother(params(999), f.robot, f.scene);
  const Path in = {q2(-1, -1), q2(-1, 1), q2(0, 1), q2(1, 1), q2(1, -1), q2(0.5, -1)};
  const Path a = sm->smooth(in);
  const Path b = sm->smooth(in);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_LT((a[i] - b[i]).norm(), 1e-12) << "waypoint " << i << " differs";
  }
}
