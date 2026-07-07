// DTC scene (Phase A/B of the DTC benchmark): the rbrobout robot — a UR10e arm on an Ewellix 900 mm
// vertical lift, with the ee_hilok end-effector baked onto the tool-changer tip — loads and resolves
// every collision mesh; the work-object environment builds; the SRDF allowed-collision matrix loads;
// and the FCL and OptiX backends agree config-for-config on robot-vs-environment. The benchmark
// (Phase C) and rerun visualization (Phase D) build on this same fixture via tests/support/dtc_scene.
#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

#include "dtc_scene.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {
std::string fixtures() { return std::string(QUEVEDOMP_FIXTURE_DIR); }

double collision_fraction(const std::vector<std::uint8_t> &v) {
  std::size_t c = 0;
  for (std::uint8_t x : v)
    c += x ? 1 : 0;
  return v.empty() ? 0.0 : static_cast<double>(c) / static_cast<double>(v.size());
}
} // namespace

TEST(DtcScene, RobotLoadsAndResolvesCollisionMeshes) {
  const auto model = dtc::load_robot(fixtures());

  // complete_chain = Ewellix prismatic lift (1) + UR10e revolute arm (6).
  EXPECT_EQ(model->dof(), 7u);

  int prismatic = 0, revolute = 0;
  for (const Joint &j : model->joints()) {
    if (j.type == JointType::Prismatic) {
      ++prismatic;
      EXPECT_TRUE(j.limits.has_position_limit);
      EXPECT_LT(j.limits.lower, j.limits.upper);
    } else if (j.type == JointType::Revolute) {
      ++revolute;
    }
  }
  EXPECT_EQ(prismatic, 1);
  EXPECT_EQ(revolute, 6);

  // Building the FCL scene resolves + loads EVERY robot collision mesh (UR10e + Ewellix + the baked
  // ee_hilok), throwing on any failure — the real gate.
  std::unique_ptr<CollisionScene> scene;
  ASSERT_NO_THROW(scene = make_static_scene(model, SceneDescription{}, BackendHint::ForceCpuFcl,
                                            dtc::meshes(fixtures())));
  const JointPosition q = JointPosition::Zero(static_cast<int>(model->dof()));
  EXPECT_EQ(fk_all(*model, q).size(), model->num_links());
}

TEST(DtcScene, SrdfAcmLoads) {
  const auto model = dtc::load_robot(fixtures());
  RobotInstance robot(model);
  const int n = dtc::load_srdf_acm(dtc::read_text(fixtures() + "/robots/rbrobout.srdf"), robot.acm());
  EXPECT_GT(n, 0);
  EXPECT_EQ(robot.acm().size(), static_cast<std::size_t>(n));
  // A representative adjacent pair from the SRDF must be allowed.
  EXPECT_TRUE(robot.acm().is_allowed("base_link", "ewellix_lift_base_link"));
}

TEST(DtcScene, WorkObjectEnvBuildsWithCollisionMix) {
  const auto model = dtc::load_robot(fixtures());
  const RobotInstance robot(model);
  const SceneDescription env = dtc::make_env(fixtures());
  EXPECT_EQ(env.objects.size(), 4u); // work object + 3 markers

  const auto scene = make_static_scene(model, env, BackendHint::ForceCpuFcl, dtc::meshes(fixtures()));
  const auto ws = scene->make_workspace();

  Rng rng(7);
  const std::vector<JointPosition> qs = dtc::sample_configs(*model, rng, 1000);
  QueryOptions opts;
  opts.check_self_collision = false; // isolate robot-vs-work-object
  const BatchResult r = scene->query_batch(robot, qs, opts, *ws);

  const double frac = collision_fraction(r.in_collision);
  // The work object must be reachable enough that random poses give a genuine mix (not all-free /
  // all-collide) — otherwise the benchmark scene is trivial.
  EXPECT_GT(frac, 0.0) << "no random pose hit the work object — scene unreachable";
  EXPECT_LT(frac, 1.0) << "every random pose collided — scene degenerate";
  std::printf("[DtcScene] work-object collision fraction over 1000 random poses: %.3f\n", frac);
}

// FCL and OptiX must agree config-for-config on robot-vs-environment for the real DTC cell. Runs
// only under the OptiX build (skipped otherwise).
TEST(DtcScene, FclOptixAgreeOnWorkObject) {
  if (!optix_available())
    GTEST_SKIP() << "OptiX backend not built";

  const auto model = dtc::load_robot(fixtures());
  const RobotInstance robot(model);
  const SceneDescription env = dtc::make_env(fixtures());
  const auto meshes = dtc::meshes(fixtures());

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, meshes);
  const auto optix = make_static_scene(model, env, BackendHint::ForceOptix, meshes);
  const auto fcl_ws = fcl->make_workspace();
  const auto optix_ws = optix->make_workspace();

  Rng rng(11);
  const std::vector<JointPosition> qs = dtc::sample_configs(*model, rng, 500);
  QueryOptions opts;
  opts.check_self_collision = false;

  const BatchResult f = fcl->query_batch(robot, qs, opts, *fcl_ws);
  const BatchResult o = optix->query_batch(robot, qs, opts, *optix_ws);
  ASSERT_EQ(f.in_collision.size(), qs.size());
  ASSERT_EQ(o.in_collision.size(), qs.size());

  int disagree = 0, collide = 0;
  for (std::size_t i = 0; i < qs.size(); ++i) {
    disagree += (f.in_collision[i] != o.in_collision[i]) ? 1 : 0;
    collide += f.in_collision[i] ? 1 : 0;
  }
  EXPECT_EQ(disagree, 0) << "FCL vs OptiX disagreed on " << disagree << "/" << qs.size();
  EXPECT_GT(collide, 0) << "no collisions in sample — agreement is trivial";
  EXPECT_LT(collide, static_cast<int>(qs.size())) << "all collided — agreement is trivial";
}

// ---- Inlet cell (bmt_9636: UR10e + 500 mm lift + dress-kits + jointA EE vs inlet_mesh_2) ---------

TEST(DtcScene, InletRobotLoadsAndResolvesCollisionMeshes) {
  const auto model = dtc::load_robot(fixtures(), dtc::Scene::Inlet);
  EXPECT_EQ(model->dof(), 7u); // 500 mm prismatic lift + UR10e revolute arm

  const RobotInstance robot(model);
  std::unique_ptr<CollisionScene> scene;
  ASSERT_NO_THROW(scene = make_static_scene(model, SceneDescription{}, BackendHint::ForceCpuFcl,
                                            dtc::meshes(fixtures(), dtc::Scene::Inlet)));
  RobotInstance r2(model);
  EXPECT_GT(dtc::load_srdf_acm(dtc::read_text(fixtures() + "/robots/rbrobout_inlet.srdf"), r2.acm()),
            0);
}

TEST(DtcScene, InletFclOptixAgreeOnWorkObject) {
  if (!optix_available())
    GTEST_SKIP() << "OptiX backend not built";

  const auto model = dtc::load_robot(fixtures(), dtc::Scene::Inlet);
  const RobotInstance robot(model);
  const SceneDescription env = dtc::make_env(fixtures(), dtc::Scene::Inlet);
  const auto meshes = dtc::meshes(fixtures(), dtc::Scene::Inlet);

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, meshes);
  const auto optix = make_static_scene(model, env, BackendHint::ForceOptix, meshes);
  const auto fcl_ws = fcl->make_workspace();
  const auto optix_ws = optix->make_workspace();

  Rng rng(13);
  const std::vector<JointPosition> qs = dtc::sample_configs(*model, rng, 500);
  QueryOptions opts;
  opts.check_self_collision = false;

  const BatchResult f = fcl->query_batch(robot, qs, opts, *fcl_ws);
  const BatchResult o = optix->query_batch(robot, qs, opts, *optix_ws);
  int disagree = 0;
  for (std::size_t i = 0; i < qs.size(); ++i)
    disagree += (f.in_collision[i] != o.in_collision[i]) ? 1 : 0;
  EXPECT_EQ(disagree, 0) << "inlet: FCL vs OptiX disagreed on " << disagree << "/" << qs.size();
  std::printf("[DtcScene] inlet work-object collision fraction (500 poses): %.3f\n",
              collision_fraction(f.in_collision));
}
