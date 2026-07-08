// Task 2b.1 — the OptiX backend. Built + run only under the dev-optix preset
// (QUEVEDOMP_WITH_OPTIX).
//   1) the device toolchain works end-to-end (batched transformed-ray launch);
//   2) the OptiX boolean AGREES with FCL on a real mesh robot vs a surface-crossing obstacle.
#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace quevedomp::collision {
bool optix_selftest(std::string &err);
}

namespace {

std::string read_text(const std::string &path) {
  std::ifstream f(path);
  EXPECT_TRUE(f.good()) << "missing: " << path;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string fixtures() { return std::string(QUEVEDOMP_FIXTURE_DIR); }

MeshSources ur5_meshes() {
  return MeshSources{{{"example-robot-data", fixtures() + "/robots/meshes/example-robot-data"}},
                     ""};
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

TEST(OptixBackend, DeviceToolchainSelfTest) {
  std::string err;
  EXPECT_TRUE(optix_selftest(err)) << err;
}

// A primitive-only robot with an empty environment builds an OptiX scene (empty GAS + no rays) and
// queries free — the degenerate case make_static_scene(ForceOptix) must handle.
TEST(OptixBackend, BuildsPrimitiveRobotEmptyEnvironment) {
  const char *kSphereRobot = R"(<robot name="r">
    <link name="base"><collision><geometry><sphere radius="0.5"/></geometry></collision></link>
  </robot>)";
  const auto model = RobotModel::from_urdf(kSphereRobot);
  const RobotInstance robot(model);
  ASSERT_TRUE(optix_available());
  const auto scene = make_static_scene(model, SceneDescription{}, BackendHint::ForceOptix);
  const auto ws = scene->make_workspace();
  QueryOptions opts;
  opts.check_self_collision = false;
  EXPECT_FALSE(scene->query(robot, JointPosition(), opts, *ws).in_collision);
}

// GPU robot-vs-environment must match FCL config-for-config. A THIN wall (a slab) is used on
// purpose: a slab can never *contain* a link, so every collision is a surface crossing — exactly
// what the v0 edge-ray backend detects (full-containment is ADR-012's parity ray, deferred). We
// place the slab at the UR5 wrist's home position and rotate the shoulder to sweep the wrist into
// and out of it.
TEST(OptixBackend, AgreesWithFclOnSurfaceCrossing) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const int dof = static_cast<int>(model->dof());

  const Eigen::Vector3d wrist = fk(*model, JointPosition::Zero(dof), "wrist_3_link").translation();

  SceneDescription env;
  env.objects.push_back({"wall", BoxShape{Eigen::Vector3d(0.25, 0.25, 0.02)},
                         Transform::from_translation(wrist)}); // thin in z

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, ur5_meshes());
  const auto optix = make_static_scene(model, env, BackendHint::ForceOptix, ur5_meshes());
  const auto fcl_ws = fcl->make_workspace();
  const auto optix_ws = optix->make_workspace();

  QueryOptions opts;
  opts.check_self_collision = false; // isolate robot-vs-environment

  // Sweep shoulder_pan (joint 0) to swing the wrist through the slab.
  std::vector<JointPosition> qs;
  for (int k = 0; k <= 16; ++k) {
    JointPosition q = JointPosition::Zero(dof);
    q[0] = -0.8 + 1.6 * k / 16.0;
    qs.push_back(q);
  }

  const BatchResult f = fcl->query_batch(robot, qs, opts, *fcl_ws);
  const BatchResult o = optix->query_batch(robot, qs, opts, *optix_ws);

  ASSERT_EQ(f.in_collision.size(), qs.size());
  ASSERT_EQ(o.in_collision.size(), qs.size());

  int collisions = 0, frees = 0, agree = 0;
  for (std::size_t i = 0; i < qs.size(); ++i) {
    EXPECT_EQ(o.in_collision[i], f.in_collision[i]) << "config " << i << " (q0=" << qs[i][0] << ")";
    if (o.in_collision[i] == f.in_collision[i])
      ++agree;
    (f.in_collision[i] ? collisions : frees)++;
  }
  EXPECT_EQ(agree, static_cast<int>(qs.size())); // full agreement
  EXPECT_GT(collisions, 0) << "obstacle never engaged — test is trivial";
  EXPECT_GT(frees, 0) << "obstacle always engaged — test is trivial";
}

// Persistent/grown workspace buffers (ADR-014): one workspace reused across several query_batch
// calls with DIFFERENT batch sizes (grow then shrink) must keep agreeing with FCL config-for-config.
// This exercises the geometric-growth device/pinned buffers and the explicit-stream launch path —
// stale capacity, a wrong memset/copy length, or leftover results would surface here.
TEST(OptixBackend, ReusedWorkspaceVaryingBatchSizesAgreeWithFcl) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const int dof = static_cast<int>(model->dof());

  const Eigen::Vector3d wrist = fk(*model, JointPosition::Zero(dof), "wrist_3_link").translation();
  SceneDescription env;
  env.objects.push_back({"wall", BoxShape{Eigen::Vector3d(0.25, 0.25, 0.02)},
                         Transform::from_translation(wrist)});

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, ur5_meshes());
  const auto optix = make_static_scene(model, env, BackendHint::ForceOptix, ur5_meshes());
  const auto fcl_ws = fcl->make_workspace();
  const auto optix_ws = optix->make_workspace(); // reused across every batch below

  QueryOptions opts;
  opts.check_self_collision = false;

  auto sweep = [&](int count) {
    std::vector<JointPosition> qs;
    for (int k = 0; k < count; ++k) {
      JointPosition q = JointPosition::Zero(dof);
      q[0] = -0.8 + 1.6 * k / std::max(1, count - 1);
      qs.push_back(q);
    }
    return qs;
  };

  // Grow (32) → shrink (5) → grow again (20): the second batch must not see the first's capacity or
  // stale device results, and the reused pinned/device buffers must serve every size correctly.
  int total_collisions = 0, total_frees = 0;
  for (int count : {32, 5, 20}) {
    const std::vector<JointPosition> qs = sweep(count);
    const BatchResult f = fcl->query_batch(robot, qs, opts, *fcl_ws);
    const BatchResult o = optix->query_batch(robot, qs, opts, *optix_ws);
    ASSERT_EQ(o.in_collision.size(), qs.size());
    for (std::size_t i = 0; i < qs.size(); ++i) {
      EXPECT_EQ(o.in_collision[i], f.in_collision[i]) << "n=" << count << " config " << i;
      (f.in_collision[i] ? total_collisions : total_frees)++;
    }
  }
  EXPECT_GT(total_collisions, 0) << "obstacle never engaged — test is trivial";
  EXPECT_GT(total_frees, 0) << "obstacle always engaged — test is trivial";
}

// ---- BackendHint::Auto hybrid dispatch (Task 2b.3 item 1) ---------------------------------------

// Auto on a mesh robot must agree with ForceCpuFcl on BOTH sides of the batch-size threshold
// (small batches route to FCL, large to OptiX — same answers either way).
TEST(HybridAuto, AgreesWithFclAcrossTheThreshold) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const int dof = static_cast<int>(model->dof());

  const Eigen::Vector3d wrist = fk(*model, JointPosition::Zero(dof), "wrist_3_link").translation();
  SceneDescription env;
  env.objects.push_back({"wall", BoxShape{Eigen::Vector3d(0.25, 0.25, 0.02)},
                         Transform::from_translation(wrist)});

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, ur5_meshes());
  const auto hybrid = make_static_scene(model, env, BackendHint::Auto, ur5_meshes());
  const auto fcl_ws = fcl->make_workspace();
  const auto hybrid_ws = hybrid->make_workspace();

  QueryOptions opts;
  opts.check_self_collision = false;

  auto sweep = [&](int count) {
    std::vector<JointPosition> qs;
    for (int k = 0; k < count; ++k) {
      JointPosition q = JointPosition::Zero(dof);
      q[0] = -0.8 + 1.6 * k / std::max(1, count - 1);
      qs.push_back(q);
    }
    return qs;
  };

  // 8 stays under the default threshold (FCL route); 400 goes over it (OptiX route).
  int collisions = 0, frees = 0;
  for (int count : {8, 400}) {
    const std::vector<JointPosition> qs = sweep(count);
    const BatchResult f = fcl->query_batch(robot, qs, opts, *fcl_ws);
    const BatchResult h = hybrid->query_batch(robot, qs, opts, *hybrid_ws);
    ASSERT_EQ(h.in_collision.size(), qs.size());
    for (std::size_t i = 0; i < qs.size(); ++i) {
      EXPECT_EQ(h.in_collision[i], f.in_collision[i]) << "n=" << count << " config " << i;
      (f.in_collision[i] ? collisions : frees)++;
    }
  }
  EXPECT_GT(collisions, 0) << "obstacle never engaged — test is trivial";
  EXPECT_GT(frees, 0) << "obstacle always engaged — test is trivial";
}

// A robot with a PRIMITIVE collision link must not be routed to OptiX for env checks (its links
// cast no rays there -> false-free). Auto must detect the collision FCL detects, at any batch size.
TEST(HybridAuto, PrimitiveRobotFallsBackToFcl) {
  const char *kSphereRobot = R"(<robot name="r">
    <link name="base"><collision><geometry><sphere radius="0.5"/></geometry></collision></link>
  </robot>)";
  const auto model = RobotModel::from_urdf(kSphereRobot);
  const RobotInstance robot(model);

  SceneDescription env; // a box overlapping the sphere at the origin
  env.objects.push_back({"box", BoxShape{Eigen::Vector3d(0.4, 0.4, 0.4)}, Transform::Identity()});
  const auto scene = make_static_scene(model, env, BackendHint::Auto);
  const auto ws = scene->make_workspace();

  QueryOptions opts;
  opts.check_self_collision = false;

  // Batch larger than the threshold: if this were routed to OptiX it would false-free.
  const std::vector<JointPosition> qs(300, JointPosition());
  const BatchResult r = scene->query_batch(robot, qs, opts, *ws);
  ASSERT_EQ(r.in_collision.size(), qs.size());
  for (std::size_t i = 0; i < qs.size(); ++i)
    EXPECT_TRUE(r.in_collision[i]) << "config " << i;
}

// Post-build environment edits demote the hybrid to FCL-only routing (the v0 OptiX scene is
// static): a wall added AFTER construction must still be seen by large batches.
TEST(HybridAuto, EnvironmentEditDemotesToFcl) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const int dof = static_cast<int>(model->dof());

  const auto scene = make_static_scene(model, SceneDescription{}, BackendHint::Auto, ur5_meshes());
  const auto ws = scene->make_workspace();
  QueryOptions opts;
  opts.check_self_collision = false;

  const std::vector<JointPosition> qs(300, JointPosition::Zero(dof));
  const BatchResult before = scene->query_batch(robot, qs, opts, *ws);
  EXPECT_FALSE(before.in_collision[0]); // empty environment

  const Eigen::Vector3d wrist = fk(*model, JointPosition::Zero(dof), "wrist_3_link").translation();
  scene->add_object("late_wall", BoxShape{Eigen::Vector3d(0.25, 0.25, 0.02)},
                    Transform::from_translation(wrist));
  const BatchResult after = scene->query_batch(robot, qs, opts, *ws);
  for (std::size_t i = 0; i < qs.size(); ++i)
    EXPECT_TRUE(after.in_collision[i]) << "config " << i;
}

// Distance/margin queries route to FCL (ADR-013 — OptiX is boolean-only): Auto must serve them
// without throwing, at any batch size.
TEST(HybridAuto, DistanceQueriesRouteToFcl) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const int dof = static_cast<int>(model->dof());

  const Eigen::Vector3d wrist = fk(*model, JointPosition::Zero(dof), "wrist_3_link").translation();
  SceneDescription env;
  env.objects.push_back({"wall", BoxShape{Eigen::Vector3d(0.25, 0.25, 0.02)},
                         Transform::from_translation(wrist + Eigen::Vector3d(0, 0, 0.5))});

  const auto scene = make_static_scene(model, env, BackendHint::Auto, ur5_meshes());
  const auto ws = scene->make_workspace();

  QueryOptions opts;
  opts.check_self_collision = false;
  opts.distance = true;

  const std::vector<JointPosition> qs(300, JointPosition::Zero(dof)); // over the threshold
  BatchResult r;
  ASSERT_NO_THROW(r = scene->query_batch(robot, qs, opts, *ws));
  ASSERT_EQ(r.min_distance.size(), qs.size());
  EXPECT_GT(r.min_distance[0], 0.0f); // separated: wall is 0.5 m above the wrist
}

// ADR-012 containment: a robot fully inside a watertight mesh casts no surface ray that hits, so
// the parity-ray check (CPU, shared with FCL) is what flags it.
TEST(OptixBackend, ContainmentInsideWatertightMesh) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const JointPosition home = JointPosition::Zero(static_cast<int>(model->dof()));
  QueryOptions opts;
  opts.check_self_collision = false;

  SceneDescription cage;
  cage.objects.push_back({"cage", box_mesh(2.0), Transform::Identity()});
  const auto in = make_static_scene(model, cage, BackendHint::ForceOptix, ur5_meshes());
  EXPECT_TRUE(in->query(robot, home, opts, *in->make_workspace()).in_collision);

  SceneDescription far;
  far.objects.push_back(
      {"pebble", box_mesh(0.05), Transform::from_translation(Eigen::Vector3d(5, 0, 0))});
  const auto out = make_static_scene(model, far, BackendHint::ForceOptix, ur5_meshes());
  EXPECT_FALSE(out->query(robot, home, opts, *out->make_workspace()).in_collision);
}
