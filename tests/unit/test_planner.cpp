// Task 3.2 verify — Planner interface + registry + RrtConnectPlanner. Covers: registry selection
// (unknown id / null args fail loudly), a known 2D solution found in bounded work, the returned
// path independently re-validated collision-free, determinism per seed, InvalidProblem / colliding
// start handling, and the performance-contract batch shape (fat collision batches).
//
// OMPL cross-check (build-plan Verify) is DEFERRED: OMPL is not an apt dependency (deviation D2)
// and pulling it in is a §12 decision, so correctness is checked self-containedly here instead.
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::planning;

namespace {

// A 2-DOF planar "gantry": two prismatic joints translate a sphere end-effector in x and y, so a
// config q = (x, y) IS the end-effector position — a clean point robot for 2D planning.
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

// Build the robot + a scene. `with_wall` inserts a vertical wall blocking x≈0 for y ≤ 0.5, leaving
// a gap above it — so the straight start→goal line at y=-1 is blocked but a detour exists.
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

PlannerParams default_params() {
  PlannerParams p;
  p.edge_resolution = 0.05;
  p.max_extension = 0.5;
  p.goal_bias = 0.1;
  p.batch_size = 64;
  return p;
}

PlanningProblem problem_to(const JointPosition &start, const JointPosition &goal, double timeout,
                           std::uint64_t seed) {
  PlanningProblem p;
  p.start = start;
  p.goal = std::make_shared<JointGoal>(goal, 1e-3);
  p.timeout = timeout;
  p.seed = seed;
  return p;
}

// Independently re-validate a path collision-free at `res` via collision::check_edge (a different
// code path than the planner's internal batched checker).
void expect_path_collision_free(const Fixture &f, const Path &path, double res) {
  ASSERT_GE(path.size(), 2u);
  const auto ws = f.scene->make_workspace();
  collision::QueryOptions opts;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto e = collision::check_edge(*f.scene, *f.robot, path[i], path[i + 1],
                                         static_cast<float>(res), opts, *ws);
    EXPECT_TRUE(e.valid) << "edge " << i << " collides at t=" << e.first_contact_t;
  }
}

} // namespace

// ---- registry ----------------------------------------------------------------------------

TEST(PlannerRegistry, ListsRrtConnect) {
  const auto ids = registered_planners();
  EXPECT_NE(std::find(ids.begin(), ids.end(), "rrt_connect"), ids.end());
}

TEST(PlannerRegistry, UnknownIdThrows) {
  auto f = make_fixture(false);
  PlannerParams p;
  p.algorithm = "no_such_planner";
  EXPECT_THROW(make_planner(p, f.robot, f.scene), std::runtime_error);
}

TEST(PlannerRegistry, NullArgsThrow) {
  auto f = make_fixture(false);
  PlannerParams p;
  EXPECT_THROW(make_planner(p, nullptr, f.scene), std::runtime_error);
  EXPECT_THROW(make_planner(p, f.robot, nullptr), std::runtime_error);
}

// ---- planning ----------------------------------------------------------------------------

TEST(RrtConnect, SolvesDirectFreePath) {
  auto f = make_fixture(/*with_wall=*/false);
  const auto planner = make_planner(default_params(), f.robot, f.scene);
  const auto r = planner->plan(problem_to(q2(-1, -1), q2(1, -1), 2.0, 1));
  ASSERT_EQ(r.status, PlanningStatus::Success) << r.message;
  EXPECT_TRUE(r.ok());
  EXPECT_EQ(r.used_seed, 1u);
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_LT((r.path.front() - q2(-1, -1)).norm(), 1e-9);
  EXPECT_LT((r.path.back() - q2(1, -1)).norm(), 1e-9);
  expect_path_collision_free(f, r.path, 0.05);
}

TEST(RrtConnect, SolvesAroundWall) {
  auto f = make_fixture(/*with_wall=*/true);
  const auto planner = make_planner(default_params(), f.robot, f.scene);
  const auto r = planner->plan(problem_to(q2(-1, -1), q2(1, -1), 5.0, 7));
  ASSERT_EQ(r.status, PlanningStatus::Success) << r.message;
  EXPECT_LT((r.path.front() - q2(-1, -1)).norm(), 1e-9);
  EXPECT_LT((r.path.back() - q2(1, -1)).norm(), 1e-9);
  expect_path_collision_free(f, r.path, 0.05);
  // A real detour: the straight-line solution was blocked, so the tree had to be grown.
  EXPECT_GT(r.stats.iterations, 0u);
}

TEST(RrtConnect, DeterministicPerSeed) {
  auto f = make_fixture(/*with_wall=*/true);
  const auto planner = make_planner(default_params(), f.robot, f.scene);
  const auto a = planner->plan(problem_to(q2(-1, -1), q2(1, -1), 5.0, 12345));
  const auto b = planner->plan(problem_to(q2(-1, -1), q2(1, -1), 5.0, 12345));
  ASSERT_EQ(a.status, PlanningStatus::Success);
  ASSERT_EQ(b.status, PlanningStatus::Success);
  ASSERT_EQ(a.path.size(), b.path.size());
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    EXPECT_LT((a.path[i] - b.path[i]).norm(), 1e-12) << "waypoint " << i << " differs";
  }
}

TEST(RrtConnect, BatchesAreFat) {
  // Performance contract item 1: collision goes out in batches fat enough to engage the GPU path.
  auto f = make_fixture(/*with_wall=*/true);
  const auto planner = make_planner(default_params(), f.robot, f.scene);
  const auto r = planner->plan(problem_to(q2(-1, -1), q2(1, -1), 5.0, 3));
  ASSERT_EQ(r.status, PlanningStatus::Success) << r.message;

  // The batched cross-edge growth emits at least one batch ≥ the hybrid Auto crossover (256),
  std::size_t max_batch = 0;
  std::uint64_t configs_in_fat = 0;
  for (const auto &[size, count] : r.stats.batch_size_histogram) {
    max_batch = std::max(max_batch, size);
    if (size >= 256) {
      configs_in_fat += static_cast<std::uint64_t>(size) * count;
    }
  }
  EXPECT_GE(max_batch, 256u);
  // and the bulk of all checked configs travel in those fat batches (not thin per-edge checks).
  EXPECT_GT(configs_in_fat, r.stats.collision_configs / 2);
}

// ---- rejection / edge cases --------------------------------------------------------------

TEST(RrtConnect, InvalidProblemDetected) {
  auto f = make_fixture(false);
  const auto planner = make_planner(default_params(), f.robot, f.scene);
  PlanningProblem p = problem_to(q2(-1, -1), q2(1, -1), 2.0, 5);
  p.start = JointPosition(3); // wrong DOF
  p.start.setZero();
  const auto r = planner->plan(p);
  EXPECT_EQ(r.status, PlanningStatus::InvalidProblem);
  EXPECT_FALSE(r.message.empty());
  EXPECT_EQ(r.used_seed, 5u); // used_seed populated even on rejection
}

TEST(RrtConnect, StartInCollisionIsNoSolution) {
  auto f = make_fixture(/*with_wall=*/true);
  const auto planner = make_planner(default_params(), f.robot, f.scene);
  // Start sits inside the wall (x=0, y=-1).
  const auto r = planner->plan(problem_to(q2(0, -1), q2(1, -1), 1.0, 5));
  EXPECT_EQ(r.status, PlanningStatus::NoSolution);
  EXPECT_FALSE(r.message.empty());
}

TEST(RrtConnect, GoalInCollisionIsNoSolution) {
  auto f = make_fixture(/*with_wall=*/true);
  const auto planner = make_planner(default_params(), f.robot, f.scene);
  const auto r = planner->plan(problem_to(q2(-1, -1), q2(0, -1), 1.0, 5)); // goal inside wall
  EXPECT_EQ(r.status, PlanningStatus::NoSolution);
}

// ---- R2: record_tree snapshot ---------------------------------------------------------------

TEST(RrtConnect, RecordTreeSnapshotsFinalTrees) {
  auto f = make_fixture(/*with_wall=*/true); // blocked straight line => a real search happens
  PlannerParams p = default_params();
  p.record_tree = true;
  const auto planner = make_planner(p, f.robot, f.scene);
  const auto r = planner->plan(problem_to(q2(-1, -1), q2(1, -1), 5.0, 11));
  ASSERT_TRUE(r.ok()) << r.message;

  ASSERT_EQ(r.trees.size(), 2u);                                    // [start, goal]
  EXPECT_LT((r.trees[0].nodes.at(0) - q2(-1, -1)).norm(), 1e-12);   // start tree rooted at start
  EXPECT_GT(r.trees[0].nodes.size() + r.trees[1].nodes.size(), 2u); // search actually grew
  for (const auto &t : r.trees) {
    ASSERT_EQ(t.nodes.size(), t.parents.size());
    for (std::size_t i = 0; i < t.parents.size(); ++i) {
      EXPECT_GE(t.parents[i], -1);
      EXPECT_LT(t.parents[i], static_cast<int>(i)); // parents precede children
    }
  }

  // Off by default: result carries no trees.
  const auto r2 = make_planner(default_params(), f.robot, f.scene)
                      ->plan(problem_to(q2(-1, -1), q2(1, -1), 5.0, 11));
  ASSERT_TRUE(r2.ok());
  EXPECT_TRUE(r2.trees.empty());
}
