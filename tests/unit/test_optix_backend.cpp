// Task 2b.1 — the OptiX backend. Built + run only under the dev-optix preset
// (QUEVEDOMP_WITH_OPTIX).
//   1) the device toolchain works end-to-end (batched transformed-ray launch);
//   2) the OptiX boolean AGREES with FCL on a real mesh robot vs a surface-crossing obstacle.
#include <gtest/gtest.h>

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
