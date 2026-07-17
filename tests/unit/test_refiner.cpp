// Roadmap R4 verify — TrajectoryRefiner (CHOMP/TrajOpt refiner over the R3 ClearanceField):
//   • certificate soundness: a Success path is independently collision-free (the exact backend is
//     the only certificate — ADR-018/019);
//   • refiner mode measurably raises min-clearance over a wall-hugging seed at equal (free)
//   success; • standalone mode solves the easy (obstacle-free) subset from a straight-line guess;
//   • determinism per seed; loud failure on the registry stub and null args.
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/clearance/clearance_field.hpp"
#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/refiner.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::planning;

namespace {

// 2-DOF planar gantry: q = (x, y) is the sphere end-effector position (same as the smoother /
// clearance fixtures).
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
  std::shared_ptr<const clearance::ClearanceField> field;
  clearance::RobotSpheres spheres;
};

Fixture make_fixture(bool with_wall) {
  Fixture f;
  f.model = RobotModel::from_urdf(kGantry2D);
  f.robot = std::make_shared<const RobotInstance>(f.model);
  collision::SceneDescription env;
  if (with_wall) {
    collision::BoxShape wall;
    wall.half_extents = Eigen::Vector3d(0.1, 1.25, 0.5); // x∈[-0.1,0.1], y∈[-2,0.5]
    env.objects.push_back(
        {"wall", wall, Transform::from_translation(Eigen::Vector3d(0, -0.75, 0))});
  } else {
    // ClearanceField needs a non-empty environment; put a wall far outside the workspace.
    collision::BoxShape far;
    far.half_extents = Eigen::Vector3d(0.1, 0.1, 0.1);
    env.objects.push_back({"far", far, Transform::from_translation(Eigen::Vector3d(10, 10, 10))});
  }
  f.scene = make_static_scene(f.model, env);
  clearance::ClearanceFieldOptions opts;
  opts.resolution = 0.02;
  f.field = std::make_shared<const clearance::ClearanceField>(
      clearance::ClearanceField::build(env, opts));
  f.spheres = clearance::decompose_robot(*f.model);
  return f;
}

RefinerParams params() {
  RefinerParams p;
  p.edge_resolution = 0.02;
  p.waypoints = 32;
  p.max_iterations = 120;
  p.step_size = 0.15;
  p.clearance_epsilon = 0.15;
  return p;
}

bool path_is_free(const Fixture &f, const Path &path, double res) {
  const auto ws = f.scene->make_workspace();
  collision::QueryOptions opts;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto e = collision::check_edge(*f.scene, *f.robot, path[i], path[i + 1],
                                         static_cast<float>(res), opts, *ws);
    if (!e.valid) {
      return false;
    }
  }
  return true;
}

// Min clearance along a path, densified so the estimate captures mid-edge dips (not just nodes).
double min_clearance(const Fixture &f, const Path &path, int per_seg = 12) {
  std::vector<JointPosition> dense;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    for (int k = 0; k < per_seg; ++k) {
      const double a = static_cast<double>(k) / per_seg;
      dense.push_back((1.0 - a) * path[i] + a * path[i + 1]);
    }
  }
  dense.push_back(path.back());
  const auto cs = clearance::clearance_batch(*f.field, *f.model, f.spheres, dense);
  double m = std::numeric_limits<double>::infinity();
  for (double c : cs) {
    m = std::min(m, c);
  }
  return m;
}

PlanningProblem problem(const JointPosition &start, const JointPosition &goal) {
  PlanningProblem prob;
  prob.start = start;
  prob.goal = std::make_shared<JointGoal>(goal, 1e-3);
  prob.timeout = 5.0;
  return prob;
}

} // namespace

TEST(TrajectoryRefiner, NullArgsAndEmptySpheresThrow) {
  auto f = make_fixture(false);
  EXPECT_THROW(make_refiner(params(), nullptr, f.scene, f.field, f.spheres), std::runtime_error);
  EXPECT_THROW(make_refiner(params(), f.robot, nullptr, f.field, f.spheres), std::runtime_error);
  EXPECT_THROW(make_refiner(params(), f.robot, f.scene, nullptr, f.spheres), std::runtime_error);
  EXPECT_THROW(make_refiner(params(), f.robot, f.scene, f.field, clearance::RobotSpheres{}),
               std::runtime_error);
}

TEST(TrajectoryRefiner, RegistryStubFailsLoudly) {
  auto f = make_fixture(false);
  // "chomp" is registered (discoverable) but building it through make_planner must throw a clear
  // directive error — never a silent fallback.
  const auto ids = registered_planners();
  EXPECT_NE(std::find(ids.begin(), ids.end(), "chomp"), ids.end());
  PlannerParams pp;
  pp.algorithm = "chomp";
  EXPECT_THROW(make_planner(pp, f.robot, f.scene), std::runtime_error);
}

TEST(TrajectoryRefiner, StandaloneSolvesObstacleFreeSubset) {
  auto f = make_fixture(false);
  const auto refiner = make_refiner(params(), f.robot, f.scene, f.field, f.spheres);
  const PlanningResult r = refiner->plan(problem(q2(-1, -1), q2(1, 1)));
  ASSERT_EQ(r.status, PlanningStatus::Success) << r.message;
  EXPECT_EQ(r.stats.refiner_mode, "standalone");
  EXPECT_GE(r.path.size(), 3u);
  EXPECT_TRUE(path_is_free(f, r.path, 0.02));
  // Endpoints preserved.
  EXPECT_LT((r.path.front() - q2(-1, -1)).norm(), 1e-9);
  EXPECT_LT((r.path.back() - q2(1, 1)).norm(), 1e-9);
}

TEST(TrajectoryRefiner, RefinerRaisesClearanceOverWallHuggingSeed) {
  auto f = make_fixture(true);
  // A feasible seed whose middle hugs the padded wall face (x≈0.25 ⇒ ~0.05 m clearance); the
  // endpoints sit well clear at x=1.
  Path seed = {q2(1, -1), q2(0.25, 0), q2(1, 1)};
  ASSERT_TRUE(path_is_free(f, seed, 0.02)) << "seed must be feasible to refine";

  RefinerParams p = params();
  p.seed = seed;
  p.clearance_weight = 4.0; // push hard for clearance on this deliberately tight seed
  const auto refiner = make_refiner(p, f.robot, f.scene, f.field, f.spheres);
  const PlanningResult r = refiner->plan(problem(q2(1, -1), q2(1, 1)));

  ASSERT_EQ(r.status, PlanningStatus::Success) << r.message;
  EXPECT_EQ(r.stats.refiner_mode, "refiner");
  EXPECT_TRUE(path_is_free(f, r.path, 0.02)); // certified free, independently re-checked

  const double seed_clear = min_clearance(f, seed);
  const double refined_clear = min_clearance(f, r.path);
  EXPECT_GT(refined_clear, seed_clear + 0.02) // measurably higher min-clearance (the Done-when)
      << "seed=" << seed_clear << " refined=" << refined_clear;
}

TEST(TrajectoryRefiner, DeterministicPerSeed) {
  auto f = make_fixture(true);
  RefinerParams p = params();
  p.seed = {q2(1, -1), q2(0.25, 0), q2(1, 1)};
  const auto refiner = make_refiner(p, f.robot, f.scene, f.field, f.spheres);
  const PlanningResult a = refiner->plan(problem(q2(1, -1), q2(1, 1)));
  const PlanningResult b = refiner->plan(problem(q2(1, -1), q2(1, 1)));
  ASSERT_EQ(a.path.size(), b.path.size());
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    EXPECT_LT((a.path[i] - b.path[i]).norm(), 1e-12) << "waypoint " << i << " differs";
  }
}
