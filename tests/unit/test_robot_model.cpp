// Task 1.3 verify — URDF parsing into RobotModel (build plan Done-when: the toy URDF loads
// with correct joint count, parent/child wiring, and limit values).
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "quevedomp/robot/robot_model.hpp"

using quevedomp::JointType;
using quevedomp::RobotModel;

namespace {

std::string read_fixture(const std::string &rel) {
  const std::string path = std::string(QUEVEDOMP_FIXTURE_DIR) + "/" + rel;
  std::ifstream f(path);
  EXPECT_TRUE(f.good()) << "could not open fixture: " << path;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::shared_ptr<const RobotModel> load_two_link() {
  return RobotModel::from_urdf(read_fixture("robots/two_link.urdf"));
}

} // namespace

TEST(RobotModel, ParsesNameAndCounts) {
  const auto m = load_two_link();
  EXPECT_EQ(m->name(), "two_link_test");
  EXPECT_EQ(m->num_links(), 4u);  // base_link, link1, link2, ee_link
  EXPECT_EQ(m->num_joints(), 3u); // joint1, joint2, joint_ee
  EXPECT_EQ(m->dof(), 2u);        // joint_ee is fixed
  EXPECT_EQ(m->root_link(), "base_link");
}

TEST(RobotModel, RevoluteJointFieldsAndLimits) {
  const auto m = load_two_link();
  const auto *j = m->find_joint("joint1");
  ASSERT_NE(j, nullptr);
  EXPECT_EQ(j->type, JointType::Revolute);
  EXPECT_TRUE(j->is_movable());
  EXPECT_EQ(j->parent_link, "base_link");
  EXPECT_EQ(j->child_link, "link1");
  EXPECT_LT((j->axis - Eigen::Vector3d(0, 0, 1)).norm(), 1e-12);
  EXPECT_LT((j->origin.translation() - Eigen::Vector3d(0, 0, 0.1)).norm(), 1e-12);
  EXPECT_TRUE(j->limits.has_position_limit);
  EXPECT_DOUBLE_EQ(j->limits.lower, -1.57);
  EXPECT_DOUBLE_EQ(j->limits.upper, 1.57);
  EXPECT_DOUBLE_EQ(j->limits.velocity, 2.0);
  EXPECT_DOUBLE_EQ(j->limits.effort, 10.0);
}

TEST(RobotModel, PrismaticJointFieldsAndLimits) {
  const auto m = load_two_link();
  const auto *j = m->find_joint("joint2");
  ASSERT_NE(j, nullptr);
  EXPECT_EQ(j->type, JointType::Prismatic);
  EXPECT_EQ(j->parent_link, "link1");
  EXPECT_EQ(j->child_link, "link2");
  EXPECT_LT((j->axis - Eigen::Vector3d(1, 0, 0)).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(j->limits.lower, 0.0);
  EXPECT_DOUBLE_EQ(j->limits.upper, 0.5);
  EXPECT_DOUBLE_EQ(j->limits.velocity, 1.0);
  EXPECT_DOUBLE_EQ(j->limits.effort, 5.0);
}

TEST(RobotModel, FixedJointHasNoDofOrPositionLimit) {
  const auto m = load_two_link();
  const auto *j = m->find_joint("joint_ee");
  ASSERT_NE(j, nullptr);
  EXPECT_EQ(j->type, JointType::Fixed);
  EXPECT_FALSE(j->is_movable());
  EXPECT_FALSE(j->limits.has_position_limit);
  EXPECT_EQ(j->parent_link, "link2");
  EXPECT_EQ(j->child_link, "ee_link");
}

TEST(RobotModel, ParentChildWiring) {
  const auto m = load_two_link();

  const auto *base = m->find_link("base_link");
  ASSERT_NE(base, nullptr);
  EXPECT_EQ(base->parent_joint, -1); // root has no parent joint

  const auto *link1 = m->find_link("link1");
  ASSERT_NE(link1, nullptr);
  ASSERT_GE(link1->parent_joint, 0);
  EXPECT_EQ(m->joints()[link1->parent_joint].name, "joint1");

  // base_link's child joint is joint1.
  std::vector<std::string> base_children;
  for (int ji : base->child_joints)
    base_children.push_back(m->joints()[ji].name);
  EXPECT_EQ(base_children, (std::vector<std::string>{"joint1"}));
}

TEST(RobotModel, ChainToTipIsOrderedBaseToTip) {
  const auto m = load_two_link();
  const auto chain = m->chain_to("ee_link");
  EXPECT_EQ(chain.base_link, "base_link");
  EXPECT_EQ(chain.tip_link, "ee_link");

  std::vector<std::string> joint_names;
  for (int ji : chain.joints)
    joint_names.push_back(m->joints()[ji].name);
  EXPECT_EQ(joint_names, (std::vector<std::string>{"joint1", "joint2", "joint_ee"}));
}

TEST(RobotModel, UnknownLookupsReturnNullOrThrow) {
  const auto m = load_two_link();
  EXPECT_EQ(m->find_link("nope"), nullptr);
  EXPECT_EQ(m->find_joint("nope"), nullptr);
  EXPECT_THROW(m->chain_to("nope"), std::runtime_error);
}

TEST(RobotModel, MalformedUrdfThrows) {
  EXPECT_THROW(RobotModel::from_urdf("<robot><not-valid"), std::runtime_error);
}

TEST(RobotModel, YamlExtensionSuppliesAccelAndJerk) {
  const auto m = RobotModel::from_urdf(read_fixture("robots/two_link.urdf"),
                                       read_fixture("robots/two_link_limits.yaml"));
  const auto *j1 = m->find_joint("joint1");
  const auto *j2 = m->find_joint("joint2");
  ASSERT_NE(j1, nullptr);
  ASSERT_NE(j2, nullptr);
  EXPECT_DOUBLE_EQ(j1->limits.acceleration, 10.0);
  EXPECT_DOUBLE_EQ(j1->limits.jerk, 100.0);
  EXPECT_DOUBLE_EQ(j2->limits.acceleration, 8.0);
  EXPECT_DOUBLE_EQ(j2->limits.jerk, 80.0);

  // Without the extension, accel/jerk stay unspecified (0).
  const auto bare = load_two_link();
  EXPECT_DOUBLE_EQ(bare->find_joint("joint1")->limits.acceleration, 0.0);
}
