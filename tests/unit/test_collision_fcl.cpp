// Task 2a.2 verify — the FCL CPU backend's boolean query_batch on closed-form cases with a known
// answer: sphere-sphere, sphere-box, box-box (overlap vs clear), an environment mesh, robot-vs-self
// honoring the ACM, and a multi-config batch. (Distance/witness is Task 2a.3.)
#include <gtest/gtest.h>

#include <memory>
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

// A one-link robot whose only collision shape is a sphere of `r` at the base origin.
const char *kSphereRobot = R"(<robot name="r">
  <link name="base"><collision><geometry><sphere radius="0.5"/></geometry></collision></link>
</robot>)";

// A one-link robot whose only collision shape is a unit box (half-extents 0.5) at the base origin.
const char *kBoxRobot = R"(<robot name="rb">
  <link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
</robot>)";

// Two spheres (r=0.3) joined by a prismatic joint along +x, child nominal at x=1.0. q slides the
// child toward the base: at q=0 centers are 1.0 apart (clear), at q=-0.5 they are 0.5 apart
// (touch).
const char *kPrismaticRobot = R"(<robot name="arm">
  <link name="base"><collision><geometry><sphere radius="0.3"/></geometry></collision></link>
  <link name="link1"><collision><geometry><sphere radius="0.3"/></geometry></collision></link>
  <joint name="j1" type="prismatic">
    <parent link="base"/><child link="link1"/>
    <origin xyz="1 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
</robot>)";

Transform at_x(double x) { return Transform::from_translation(Eigen::Vector3d(x, 0.0, 0.0)); }

// Returns boolean collision of the (DOF-free) robot against a single-object environment.
bool collide_env(const char *robot_urdf, const Geometry &env_geom, const Transform &env_pose) {
  const auto model = RobotModel::from_urdf(robot_urdf);
  const RobotInstance robot(model);
  SceneDescription env;
  env.objects.push_back({"obj", env_geom, env_pose});
  const auto scene = make_static_scene(model, env);
  const auto ws = scene->make_workspace();
  QueryOptions opts;
  opts.check_self_collision = false;
  return scene->query(robot, JointPosition(), opts, *ws).in_collision;
}

// A closed axis-aligned cube mesh of the given half-size, centered at the origin.
Mesh box_mesh(double h) {
  Mesh m;
  for (int i = 0; i < 8; ++i) {
    m.vertices.emplace_back((i & 1 ? h : -h), (i & 2 ? h : -h), (i & 4 ? h : -h));
  }
  const int faces[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                            {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  for (const auto &f : faces)
    m.triangles.emplace_back(f[0], f[1], f[2]);
  return m;
}

} // namespace

// ---- robot-vs-environment, closed form ---------------------------------------------------------

TEST(FclBackend, SphereSphere) {
  EXPECT_TRUE(collide_env(kSphereRobot, SphereShape{0.5}, at_x(0.8)));  // 0.8 < 0.5+0.5
  EXPECT_FALSE(collide_env(kSphereRobot, SphereShape{0.5}, at_x(1.2))); // 1.2 > 1.0
}

TEST(FclBackend, SphereBox) {
  EXPECT_TRUE(collide_env(kSphereRobot, BoxShape{Eigen::Vector3d(0.1, 0.1, 0.1)}, at_x(0.55)));
  EXPECT_FALSE(collide_env(kSphereRobot, BoxShape{Eigen::Vector3d(0.1, 0.1, 0.1)}, at_x(0.70)));
}

TEST(FclBackend, BoxBox) {
  EXPECT_TRUE(collide_env(kBoxRobot, BoxShape{Eigen::Vector3d(0.1, 0.1, 0.1)}, at_x(0.55)));
  EXPECT_FALSE(collide_env(kBoxRobot, BoxShape{Eigen::Vector3d(0.1, 0.1, 0.1)}, at_x(0.65)));
}

TEST(FclBackend, EnvironmentMesh) {
  EXPECT_TRUE(collide_env(kSphereRobot, box_mesh(0.1), at_x(0.55)));
  EXPECT_FALSE(collide_env(kSphereRobot, box_mesh(0.1), at_x(0.70)));
}

// ---- robot-vs-self + ACM -----------------------------------------------------------------------

TEST(FclBackend, SelfCollisionHonorsAcm) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  JointPosition q(1);
  q << -0.5; // child slides to x=0.5; the two r=0.3 spheres overlap (0.5 < 0.6)

  QueryOptions opts;
  EXPECT_TRUE(scene->query(robot, q, opts, *ws).in_collision);

  robot.acm().allow("base", "link1");
  EXPECT_FALSE(scene->query(robot, q, opts, *ws).in_collision);

  // With self-collision disabled, an overlap is also ignored.
  RobotInstance fresh(model);
  opts.check_self_collision = false;
  EXPECT_FALSE(scene->query(fresh, q, opts, *ws).in_collision);
}

// ---- batch -------------------------------------------------------------------------------------

TEST(FclBackend, BatchMixedResults) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  const RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  std::vector<JointPosition> qs;
  qs.emplace_back(1);
  qs.back() << 0.0; // clear: centers 1.0 apart
  qs.emplace_back(1);
  qs.back() << -1.0; // collide: child at base origin

  const BatchResult b = scene->query_batch(robot, qs, QueryOptions{}, *ws);
  ASSERT_EQ(b.in_collision.size(), 2u);
  EXPECT_EQ(b.in_collision[0], 0);
  EXPECT_EQ(b.in_collision[1], 1);
}

// ---- backend selection -------------------------------------------------------------------------

TEST(FclBackend, ForceOptixUnavailableThrows) {
  const auto model = RobotModel::from_urdf(kSphereRobot);
  EXPECT_THROW(make_static_scene(model, SceneDescription{}, BackendHint::ForceOptix),
               std::runtime_error);
}

// ---- dynamic environment editing ---------------------------------------------------------------

TEST(FclBackend, MoveAndRemoveObject) {
  const auto model = RobotModel::from_urdf(kSphereRobot);
  const RobotInstance robot(model);

  SceneDescription env;
  env.objects.push_back({"ball", SphereShape{0.5}, at_x(1.2)}); // initially clear
  auto scene = make_static_scene(model, env);
  const auto ws = scene->make_workspace();

  QueryOptions opts;
  opts.check_self_collision = false;
  EXPECT_FALSE(scene->query(robot, JointPosition(), opts, *ws).in_collision);

  const SceneHandle h = scene->add_object("ball2", SphereShape{0.5}, at_x(0.8));
  EXPECT_TRUE(scene->query(robot, JointPosition(), opts, *ws).in_collision);

  scene->move_object(h, at_x(1.5));
  EXPECT_FALSE(scene->query(robot, JointPosition(), opts, *ws).in_collision);

  const SceneHandle h2 = scene->add_object("ball3", SphereShape{0.5}, at_x(0.8));
  EXPECT_TRUE(scene->query(robot, JointPosition(), opts, *ws).in_collision);

  scene->remove_object(h2);
  EXPECT_FALSE(scene->query(robot, JointPosition(), opts, *ws).in_collision);
}
