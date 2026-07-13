// Task 3.3d P4 verify — ACM covers robot-link × environment-object pairs (the
// AllowedCollisionMatrix doc always promised "link/object ids"; the backends only honored
// self pairs). Covers, on the FCL backend: the boolean fast path, the distance/witness path,
// the safety-margin path, dynamic disallow, non-matching ids staying inert, and the ADR-012
// containment check (a link allowed to touch an object must not report "inside it" either).
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

// The test_planner 2-DOF gantry: q = (x, y) IS the sphere end-effector position.
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
  std::shared_ptr<RobotInstance> robot;
  std::unique_ptr<CollisionScene> scene;
  std::unique_ptr<Workspace> ws;
};

// Wall spanning y in [-2, 0.5] at x ~ 0; q = (0, -1) puts the ee sphere inside it.
Fixture make_fixture() {
  Fixture f;
  f.model = RobotModel::from_urdf(kGantry2D);
  f.robot = std::make_shared<RobotInstance>(f.model);
  SceneDescription env;
  env.objects.push_back({"wall", BoxShape{Eigen::Vector3d(0.1, 1.25, 0.5)},
                         Transform::from_translation(Eigen::Vector3d(0, -0.75, 0))});
  f.scene = make_static_scene(f.model, env, BackendHint::ForceCpuFcl);
  f.ws = f.scene->make_workspace();
  return f;
}

// A closed (watertight) axis-aligned cube mesh of the given half-size, centered at the origin.
Mesh box_mesh(double h) {
  Mesh m;
  for (int i = 0; i < 8; ++i)
    m.vertices.emplace_back(i & 1 ? h : -h, i & 2 ? h : -h, i & 4 ? h : -h);
  const int f[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                        {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  for (const auto &t : f)
    m.triangles.emplace_back(t[0], t[1], t[2]);
  return m;
}

} // namespace

TEST(AcmEnv, BooleanSkipsAllowedPair) {
  auto f = make_fixture();
  const JointPosition q = q2(0.0, -1.0); // inside the wall
  QueryOptions opts;

  EXPECT_TRUE(f.scene->query(*f.robot, q, opts, *f.ws).in_collision);

  f.robot->acm().allow("ee", "wall");
  EXPECT_FALSE(f.scene->query(*f.robot, q, opts, *f.ws).in_collision);

  // The pair is consulted live per batch: disallowing restores the collision.
  f.robot->acm().disallow("ee", "wall");
  EXPECT_TRUE(f.scene->query(*f.robot, q, opts, *f.ws).in_collision);

  // Pairs naming no env object, or the wrong link, stay inert.
  f.robot->acm().allow("ee", "no_such_object");
  f.robot->acm().allow("cx", "wall"); // cx has no collision geometry
  EXPECT_TRUE(f.scene->query(*f.robot, q, opts, *f.ws).in_collision);
}

TEST(AcmEnv, DistanceAndWitnessExcludeAllowedPair) {
  auto f = make_fixture();
  const JointPosition q = q2(0.0, -1.0);
  QueryOptions opts;
  opts.distance = true;

  const CollisionResult before = f.scene->query(*f.robot, q, opts, *f.ws);
  EXPECT_TRUE(before.in_collision);
  ASSERT_TRUE(before.witness.has_value());
  EXPECT_EQ(before.witness->b, "wall");

  f.robot->acm().allow("ee", "wall");
  const CollisionResult after = f.scene->query(*f.robot, q, opts, *f.ws);
  EXPECT_FALSE(after.in_collision);
  // The allowed pair is excluded from min-distance/witness entirely (exactly like self pairs):
  // with the only geometry pair allowed, nothing remains to witness.
  if (after.witness.has_value())
    EXPECT_NE(after.witness->b, "wall");
}

TEST(AcmEnv, SafetyMarginPathHonorsAllowedPair) {
  auto f = make_fixture();
  // Near the wall face (x = ±0.1 + sphere 0.1): at x = -0.25 the gap is 0.05 < margin.
  const JointPosition q = q2(-0.25, -1.0);
  QueryOptions opts;
  opts.safety_margin = 0.1f;

  EXPECT_TRUE(f.scene->query(*f.robot, q, opts, *f.ws).in_collision);
  f.robot->acm().allow("ee", "wall");
  EXPECT_FALSE(f.scene->query(*f.robot, q, opts, *f.ws).in_collision);
}

// ADR-012 containment: a robot mesh link fully inside a watertight env mesh is flagged by the
// parity-ray check — unless the ACM allows that (link, object) pair.
TEST(AcmEnv, ContainmentSkipsAllowedPair) {
  const std::string urdf_path = std::string(QUEVEDOMP_FIXTURE_DIR) + "/robots/ur5.urdf";
  std::ifstream in(urdf_path);
  ASSERT_TRUE(in.good()) << "missing: " << urdf_path;
  std::ostringstream ss;
  ss << in.rdbuf();
  const auto model = RobotModel::from_urdf(ss.str());
  RobotInstance robot(model);
  const MeshSources meshes{{{"example-robot-data", std::string(QUEVEDOMP_FIXTURE_DIR) +
                                                       "/robots/meshes/example-robot-data"}},
                           ""};

  SceneDescription cage;
  cage.objects.push_back({"cage", box_mesh(2.0), Transform::Identity()});
  const auto scene = make_static_scene(model, cage, BackendHint::ForceCpuFcl, meshes);
  const auto ws = scene->make_workspace();
  const JointPosition home = JointPosition::Zero(static_cast<int>(model->dof()));
  QueryOptions opts;
  opts.check_self_collision = false;

  EXPECT_TRUE(scene->query(robot, home, opts, *ws).in_collision);

  for (const Link &l : model->links())
    robot.acm().allow(l.name, "cage");
  EXPECT_FALSE(scene->query(robot, home, opts, *ws).in_collision);

  // One link disallowed again: its interior point is still inside => back to colliding.
  robot.acm().disallow("wrist_3_link", "cage");
  EXPECT_TRUE(scene->query(robot, home, opts, *ws).in_collision);
}
