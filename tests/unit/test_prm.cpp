// Roadmap R5 verify — PrmPlanner (multi-query roadmap planner):
//   • builds a roadmap over the static scene (nodes + edges), reported in PrmBuildStats;
//   • solves a start→goal that must route around a wall; the path is independently collision-free;
//   • ONE roadmap answers MANY queries (the multi-query point);
//   • determinism per (build seed, query seed); loud failure on the registry stub + null args.
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/roadmap.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::planning;

namespace {

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
    wall.half_extents = Eigen::Vector3d(0.1, 1.25, 0.5); // padded x∈[-0.2,0.2], y up to ~0.6
    env.objects.push_back(
        {"wall", wall, Transform::from_translation(Eigen::Vector3d(0, -0.75, 0))});
  }
  f.scene = make_static_scene(f.model, env);
  return f;
}

PrmParams params(std::uint64_t seed) {
  PrmParams p;
  p.num_nodes = 800;
  p.k_neighbors = 12;
  // Edges (and the smoother) validate at this step; the independent path_is_free checks below use
  // the SAME resolution, so the collision guarantee and the verification are consistent (a finer
  // check would flag grazes the planner never promised to avoid — the discretization gap).
  p.edge_resolution = 0.02;
  p.seed = seed;
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

PlanningProblem problem(const JointPosition &start, const JointPosition &goal, std::uint64_t seed) {
  PlanningProblem prob;
  prob.start = start;
  prob.goal = std::make_shared<JointGoal>(goal, 1e-3);
  prob.timeout = 5.0;
  prob.seed = seed;
  return prob;
}

} // namespace

TEST(PrmPlanner, NullArgsThrow) {
  auto f = make_fixture(false);
  EXPECT_THROW(make_prm_planner(params(1), nullptr, f.scene), std::runtime_error);
  EXPECT_THROW(make_prm_planner(params(1), f.robot, nullptr), std::runtime_error);
}

TEST(PrmPlanner, RegistryStubFailsLoudly) {
  auto f = make_fixture(false);
  const auto ids = registered_planners();
  EXPECT_NE(std::find(ids.begin(), ids.end(), "prm"), ids.end());
  PlannerParams pp;
  pp.algorithm = "prm";
  EXPECT_THROW(make_planner(pp, f.robot, f.scene), std::runtime_error);
}

TEST(PrmPlanner, BuildStatsPopulated) {
  auto f = make_fixture(true);
  PrmBuildStats stats;
  const auto prm = make_prm_planner(params(7), f.robot, f.scene, &stats);
  EXPECT_GT(stats.nodes, 100u); // most of 800 samples are free in this cell
  EXPECT_EQ(stats.node_candidates, 800u);
  EXPECT_GT(stats.edges, stats.nodes); // k-nearest connectivity ⇒ more edges than nodes
  EXPECT_GT(stats.collision_configs, 0u);
  EXPECT_GT(stats.build_seconds, 0.0);
}

TEST(PrmPlanner, SolvesAroundWall) {
  auto f = make_fixture(true);
  const auto prm = make_prm_planner(params(3), f.robot, f.scene);
  const PlanningResult r = prm->plan(problem(q2(-1, -1), q2(1, -1), 42));
  ASSERT_EQ(r.status, PlanningStatus::Success) << r.message;
  EXPECT_TRUE(path_is_free(f, r.path, 0.02));
  EXPECT_LT((r.path.front() - q2(-1, -1)).norm(), 1e-6);
  EXPECT_LT((r.path.back() - q2(1, -1)).norm(), 1e-6);
  // Routing around the wall means the path must rise above the padded top (y > 0.4 somewhere).
  const double max_y =
      std::max_element(r.path.begin(), r.path.end(),
                       [](const JointPosition &a, const JointPosition &b) { return a.y() < b.y(); })
          ->y();
  EXPECT_GT(max_y, 0.4);
}

TEST(PrmPlanner, OneRoadmapAnswersManyQueries) {
  auto f = make_fixture(true);
  const auto prm = make_prm_planner(params(5), f.robot, f.scene);
  for (const auto &[s, g] : std::vector<std::pair<JointPosition, JointPosition>>{
           {q2(-1, -1), q2(1, -1)}, {q2(-1.5, 0), q2(1.5, 0)}, {q2(-1, 1), q2(1, -1.5)}}) {
    const PlanningResult r = prm->plan(problem(s, g, 1));
    ASSERT_EQ(r.status, PlanningStatus::Success) << r.message;
    EXPECT_TRUE(path_is_free(f, r.path, 0.02));
  }
}

TEST(PrmPlanner, DeterministicPerSeed) {
  auto f = make_fixture(true);
  const auto a = make_prm_planner(params(9), f.robot, f.scene);
  const auto b = make_prm_planner(params(9), f.robot, f.scene); // same build seed ⇒ same roadmap
  const PlanningResult ra = a->plan(problem(q2(-1, -1), q2(1, -1), 100));
  const PlanningResult rb = b->plan(problem(q2(-1, -1), q2(1, -1), 100));
  ASSERT_EQ(ra.status, PlanningStatus::Success);
  ASSERT_EQ(ra.path.size(), rb.path.size());
  for (std::size_t i = 0; i < ra.path.size(); ++i) {
    EXPECT_LT((ra.path[i] - rb.path[i]).norm(), 1e-12) << "waypoint " << i << " differs";
  }
}
