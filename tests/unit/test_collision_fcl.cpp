// Task 2a.2 verify — the FCL CPU backend's boolean query_batch on closed-form cases with a known
// answer: sphere-sphere, sphere-box, box-box (overlap vs clear), an environment mesh, robot-vs-self
// honoring the ACM, and a multi-config batch. (Distance/witness is Task 2a.3.)
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_check.hpp"
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

// Full query result of the (DOF-free) robot against a single-object environment "obj".
CollisionResult query_env(const char *robot_urdf, const Geometry &env_geom,
                          const Transform &env_pose, QueryOptions opts) {
  const auto model = RobotModel::from_urdf(robot_urdf);
  const RobotInstance robot(model);
  SceneDescription env;
  env.objects.push_back({"obj", env_geom, env_pose});
  const auto scene = make_static_scene(model, env);
  const auto ws = scene->make_workspace();
  opts.check_self_collision = false;
  return scene->query(robot, JointPosition(), opts, *ws);
}

// Returns boolean collision of the (DOF-free) robot against a single-object environment.
bool collide_env(const char *robot_urdf, const Geometry &env_geom, const Transform &env_pose) {
  return query_env(robot_urdf, env_geom, env_pose, QueryOptions{}).in_collision;
}

// ---- fixture-robot (real meshes) helpers -------------------------------------------------------
std::string fixtures() { return std::string(QUEVEDOMP_FIXTURE_DIR); }
std::string mesh_root() { return fixtures() + "/robots/meshes"; }

std::string read_text(const std::string &path) {
  std::ifstream f(path);
  EXPECT_TRUE(f.good()) << "missing: " << path;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

MeshSources ur5_meshes() {
  return MeshSources{{{"example-robot-data", mesh_root() + "/example-robot-data"}}, ""};
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

// ---- signed distance + witness (Task 2a.3) -----------------------------------------------------

namespace {
QueryOptions dist_opts(float max_distance = 10.0f) {
  QueryOptions o;
  o.distance = true;
  o.max_distance = max_distance; // wide, so the true gap is reported (not clamped)
  return o;
}
} // namespace

// Sphere robot (r=0.5) at the origin vs an env sphere (r=0.5): separation = gap, witness on the two
// surfaces along +x.
TEST(FclDistance, Separation) {
  const auto r = query_env(kSphereRobot, SphereShape{0.5}, at_x(1.5), dist_opts());
  EXPECT_FALSE(r.in_collision);
  EXPECT_NEAR(r.min_distance, 0.5, 1e-3); // 1.5 - (0.5+0.5)
  ASSERT_TRUE(r.witness.has_value());
  EXPECT_NEAR(r.witness->point_a.x(), 0.5, 1e-3); // robot surface
  EXPECT_NEAR(r.witness->point_b.x(), 1.0, 1e-3); // env surface
  EXPECT_EQ(r.witness->a, "base");
  EXPECT_EQ(r.witness->b, "obj");
}

TEST(FclDistance, Penetration) {
  const auto r = query_env(kSphereRobot, SphereShape{0.5}, at_x(0.8), dist_opts());
  EXPECT_TRUE(r.in_collision);
  EXPECT_NEAR(r.min_distance, -0.2, 1e-2); // depth = (0.5+0.5) - 0.8
}

TEST(FclDistance, TouchingIsApproxZero) {
  const auto r = query_env(kSphereRobot, SphereShape{0.5}, at_x(1.0), dist_opts());
  EXPECT_NEAR(r.min_distance, 0.0, 1e-3);
}

TEST(FclDistance, ClampedAtMaxDistance) {
  QueryOptions o;
  o.distance = true; // default max_distance = 0.10
  const auto r = query_env(kSphereRobot, SphereShape{0.5}, at_x(1.5), o);
  EXPECT_FLOAT_EQ(r.min_distance, 0.10f); // true gap 0.5 clamped to max_distance
}

TEST(FclDistance, RobotPaddingOffsetsDistance) {
  QueryOptions o = dist_opts();
  o.robot_padding = 0.1f;
  const auto r = query_env(kSphereRobot, SphereShape{0.5}, at_x(1.5), o);
  EXPECT_NEAR(r.min_distance, 0.4, 1e-3); // 0.5 gap - 0.1 padding
}

// safety_margin makes a (physically clear) near-miss count as a collision — and forces the
// distance computation even though distance output was not requested (so it stays empty).
TEST(FclDistance, SafetyMarginGatesBooleanWithoutDistanceOutput) {
  QueryOptions clear;
  EXPECT_FALSE(query_env(kSphereRobot, SphereShape{0.5}, at_x(1.3), clear).in_collision);

  QueryOptions margin;
  margin.safety_margin = 0.4f; // gap is 0.3 < 0.4 -> "in collision"
  const auto r = query_env(kSphereRobot, SphereShape{0.5}, at_x(1.3), margin);
  EXPECT_TRUE(r.in_collision);
  EXPECT_FALSE(r.witness.has_value());   // distance output not requested
  EXPECT_FLOAT_EQ(r.min_distance, 0.0f); // unset
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

TEST(FclBackend, ForceOptixThrowsWhenUnavailable) {
  // When OptiX is present, actually constructing a GPU scene needs CUDA (and the ASan shadow-gap
  // suppression) — that path is covered by test_optix_backend. Here we only assert the CPU-build
  // contract: ForceOptix throws when the backend isn't compiled in.
  if (optix_available())
    GTEST_SKIP() << "OptiX backend present; construction is covered by test_optix_backend";
  const auto model = RobotModel::from_urdf(kSphereRobot);
  EXPECT_THROW(make_static_scene(model, SceneDescription{}, BackendHint::ForceOptix),
               std::runtime_error);
}

// ---- robot-link meshes (Task 2a.2b) ------------------------------------------------------------

// A real fixture robot (UR5) whose collision geometry is all meshes resolves, loads, and collides
// through the same FCL path as primitives.
TEST(FclMeshLinks, Ur5MeshRobotVsEnvironment) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const JointPosition home = JointPosition::Zero(model->dof());

  // A box enclosing the whole arm collides; the same box parked 100 m away does not.
  QueryOptions opts;
  opts.check_self_collision = false;

  SceneDescription enclosing;
  enclosing.objects.push_back({"cage", BoxShape{Eigen::Vector3d(2.0, 2.0, 2.0)}, at_x(0.0)});
  const auto hit_scene = make_static_scene(model, enclosing, BackendHint::Auto, ur5_meshes());
  const auto ws1 = hit_scene->make_workspace();
  EXPECT_TRUE(hit_scene->query(robot, home, opts, *ws1).in_collision);

  SceneDescription faraway;
  faraway.objects.push_back({"box", BoxShape{Eigen::Vector3d(0.5, 0.5, 0.5)}, at_x(100.0)});
  const auto clear_scene = make_static_scene(model, faraway, BackendHint::Auto, ur5_meshes());
  const auto ws2 = clear_scene->make_workspace();
  EXPECT_FALSE(clear_scene->query(robot, home, opts, *ws2).in_collision);
}

// A mesh robot built WITHOUT the means to resolve its mesh URIs fails loudly — never silently
// skipped (which would make a real robot uncollidable).
TEST(FclMeshLinks, MeshRobotWithoutSourcesThrows) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  EXPECT_THROW(make_static_scene(model, SceneDescription{}), std::runtime_error);
}

// ADR-012 containment: a robot mesh link fully inside a watertight environment MESH produces no
// triangle-triangle crossing, so FCL's surface test alone reports free — the parity ray catches it.
TEST(FclContainment, MeshRobotInsideWatertightMesh) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const JointPosition home = JointPosition::Zero(model->dof());
  QueryOptions opts;
  opts.check_self_collision = false;

  // A big watertight cube MESH enclosing the whole arm (no surface crossing anywhere).
  SceneDescription cage;
  cage.objects.push_back({"cage", box_mesh(2.0), Transform::Identity()});
  const auto in = make_static_scene(model, cage, BackendHint::ForceCpuFcl, ur5_meshes());
  const auto ws_in = in->make_workspace();
  EXPECT_TRUE(in->query(robot, home, opts, *ws_in).in_collision);

  // A tiny cube mesh far away contains nothing.
  SceneDescription far;
  far.objects.push_back({"pebble", box_mesh(0.05), at_x(5.0)});
  const auto out = make_static_scene(model, far, BackendHint::ForceCpuFcl, ur5_meshes());
  const auto ws_out = out->make_workspace();
  EXPECT_FALSE(out->query(robot, home, opts, *ws_out).in_collision);
}

// ---- check_edge (Task 2a.4) --------------------------------------------------------------------

namespace {
JointPosition qv(double x) {
  JointPosition q(1);
  q << x;
  return q;
}
} // namespace

// Prismatic robot, self-collision along the edge: the two r=0.3 spheres overlap once the child
// slides within 0.6 of the base (|1.0+q| < 0.6, i.e. q < -0.4).
TEST(FclEdge, FreeEdgeIsValid) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  const RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  const EdgeResult r = check_edge(*scene, robot, qv(0.0), qv(-0.3), 0.1f, QueryOptions{}, *ws);
  EXPECT_TRUE(r.valid); // closest approach 0.7 > 0.6, never collides
  EXPECT_FLOAT_EQ(r.first_contact_t, 1.0f);
}

TEST(FclEdge, CollidingEdgeReportsFirstContact) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  const RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  // q: 0 -> -1 at resolution 0.25 -> samples q = 0, -0.25, -0.5, -0.75, -1. First overlap at
  // q = -0.5 (t = 0.5).
  const EdgeResult r = check_edge(*scene, robot, qv(0.0), qv(-1.0), 0.25f, QueryOptions{}, *ws);
  EXPECT_FALSE(r.valid);
  EXPECT_FLOAT_EQ(r.first_contact_t, 0.5f);
}

TEST(FclEdge, CollidingStartIsContactAtZero) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  const RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  const EdgeResult r = check_edge(*scene, robot, qv(-1.0), qv(-0.9), 0.1f, QueryOptions{}, *ws);
  EXPECT_FALSE(r.valid);
  EXPECT_FLOAT_EQ(r.first_contact_t, 0.0f); // q0 itself already collides
}

// An edge that drives the child link through an environment obstacle. Boundary at q = 0.45
// (env sphere at x=2.05); at resolution 0.1 the first colliding sample is q = 0.5 (t = 0.5).
TEST(FclEdge, EdgeThroughEnvironmentObstacle) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  const RobotInstance robot(model);
  SceneDescription env;
  env.objects.push_back({"post", SphereShape{0.3}, at_x(2.05)});
  const auto scene = make_static_scene(model, env);
  const auto ws = scene->make_workspace();

  const EdgeResult r = check_edge(*scene, robot, qv(0.0), qv(1.0), 0.1f, QueryOptions{}, *ws);
  EXPECT_FALSE(r.valid);
  EXPECT_FLOAT_EQ(r.first_contact_t, 0.5f);
}

TEST(FclEdge, MismatchedSizesThrow) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  const RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  JointPosition q2(2);
  q2 << 0.0, 0.0;
  EXPECT_THROW(check_edge(*scene, robot, qv(0.0), q2, 0.1f, QueryOptions{}, *ws),
               std::runtime_error);
  EXPECT_THROW(check_edge(*scene, robot, qv(0.0), qv(1.0), 0.0f, QueryOptions{}, *ws),
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

// Task 3.3d P7a — the OpenMP-parallel batch path must agree config-for-config with serial
// single-config queries (which stay below the parallel threshold), for boolean AND distance.
TEST(FclBatchParallel, MatchesSerialSingles) {
  const auto model = RobotModel::from_urdf(kPrismaticRobot);
  const RobotInstance robot(model);
  SceneDescription env;
  env.objects.push_back({"obj", SphereShape{0.3}, at_x(1.8)});
  const auto scene = make_static_scene(model, env, BackendHint::ForceCpuFcl);
  const auto ws = scene->make_workspace();

  // Sweep q through self-colliding (q < -0.4), free, env-colliding (q ~ [0.2, 1.4]), free again.
  std::vector<JointPosition> qs;
  for (int i = 0; i < 96; ++i) {
    JointPosition q(1);
    q << -1.0 + 3.0 * i / 95.0;
    qs.push_back(q);
  }

  for (const bool with_distance : {false, true}) {
    QueryOptions opts;
    opts.distance = with_distance;
    opts.max_distance = 10.0f;
    const BatchResult batch = scene->query_batch(robot, qs, opts, *ws);
    ASSERT_EQ(batch.in_collision.size(), qs.size());
    for (std::size_t i = 0; i < qs.size(); ++i) {
      const CollisionResult single = scene->query(robot, qs[i], opts, *ws);
      EXPECT_EQ(batch.in_collision[i] != 0, single.in_collision)
          << "config " << i << " distance=" << with_distance;
      if (with_distance)
        EXPECT_FLOAT_EQ(batch.min_distance[i], single.min_distance) << "config " << i;
    }
  }
}
