// DTC scene fixtures (Phase A of the DTC benchmark): the rbrobout robot — a UR10e arm on an Ewellix
// 900 mm vertical lift with a tool-changer tip — parses from the flattened URDF, resolves ALL of its
// collision meshes (UR10e collision STLs via package://ur_description, Ewellix lift STLs via
// package://dtc_test), and FKs across its prismatic + revolute chain. The work-object environment,
// the attached end-effector mesh, and the FCL-vs-OptiX benchmark land in later phases.
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

std::string fixtures() { return std::string(QUEVEDOMP_FIXTURE_DIR); }

std::string read_text(const std::string &path) {
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// The two mesh packages the flattened rbrobout URDF references.
MeshSources dtc_meshes() {
  const std::string m = fixtures() + "/robots/meshes/";
  return MeshSources{{{"ur_description", m + "ur_description"}, {"dtc_test", m + "dtc_test"}}, ""};
}

} // namespace

TEST(DtcScene, RobotLoadsAndResolvesCollisionMeshes) {
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/rbrobout.urdf"));

  // complete_chain = Ewellix prismatic lift (1) + UR10e revolute arm (6).
  EXPECT_EQ(model->dof(), 7u);

  // Exactly one movable prismatic joint (the lift), and it carries position limits so random-pose
  // sampling in the benchmark has a finite range.
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

  const RobotInstance robot(model);

  // Building the FCL scene resolves + loads EVERY robot collision mesh (it throws on any unresolved
  // or unloadable mesh — never silently skipped). This is the real Phase-A gate.
  std::unique_ptr<CollisionScene> scene;
  ASSERT_NO_THROW(scene = make_static_scene(model, SceneDescription{}, BackendHint::ForceCpuFcl,
                                            dtc_meshes()));
  const auto ws = scene->make_workspace();

  // FK across the full chain (including the prismatic lift) yields one transform per link.
  const JointPosition q = JointPosition::Zero(static_cast<int>(model->dof()));
  const std::vector<Transform> tf = fk_all(*model, q);
  EXPECT_EQ(tf.size(), model->num_links());

  QueryOptions opts;
  opts.check_self_collision = false; // ACM comes from the SRDF in a later phase; skip self here
  EXPECT_NO_THROW((void)scene->query(robot, q, opts, *ws));
}
