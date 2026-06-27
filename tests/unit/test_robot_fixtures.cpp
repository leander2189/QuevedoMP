// Task 1.4 verify — the five real robot URDFs load via RobotModel::from_urdf and their DOF
// matches the datasheet. Provenance/licenses: tests/fixtures/robots/PROVENANCE.md.
#include <gtest/gtest.h>

#include <cctype>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

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

std::size_t count_type(const RobotModel &m, JointType t) {
  std::size_t n = 0;
  for (const auto &j : m.joints()) {
    if (j.type == t)
      ++n;
  }
  return n;
}

struct RobotCase {
  const char *file;
  std::size_t arm_dof;       // datasheet DOF (revolute arm joints)
  std::size_t total_movable; // dof() incl. gripper/prismatic joints
  const char *label;
};

} // namespace

class RobotFixtures : public ::testing::TestWithParam<RobotCase> {};

TEST_P(RobotFixtures, LoadsAndDofMatchesDatasheet) {
  const RobotCase &tc = GetParam();
  std::shared_ptr<const RobotModel> m;
  ASSERT_NO_THROW(m = RobotModel::from_urdf(read_fixture(std::string("robots/") + tc.file)))
      << tc.label << " (" << tc.file << ") failed to parse";

  EXPECT_GT(m->num_links(), 0u) << tc.label;
  EXPECT_FALSE(m->root_link().empty()) << tc.label;

  // Datasheet DOF = number of revolute arm joints.
  EXPECT_EQ(count_type(*m, JointType::Revolute), tc.arm_dof) << tc.label;
  // Total movable (non-fixed) joints, including any gripper prismatic joints.
  EXPECT_EQ(m->dof(), tc.total_movable) << tc.label;
}

INSTANTIATE_TEST_SUITE_P(
    FiveRobots, RobotFixtures,
    ::testing::Values(RobotCase{"ur5.urdf", 6, 6, "UR5"}, RobotCase{"ur10.urdf", 6, 6, "UR10"},
                      RobotCase{"panda.urdf", 7, 9, "Franka Panda (7 arm + 2 finger)"},
                      RobotCase{"iiwa.urdf", 7, 7, "KUKA iiwa"},
                      RobotCase{"irb2400.urdf", 6, 6, "ABB IRB2400"}),
    [](const ::testing::TestParamInfo<RobotCase> &info) {
      // GoogleTest test-name suffix must be a valid C identifier.
      std::string s = info.param.file;
      for (char &c : s) {
        if (!std::isalnum(static_cast<unsigned char>(c)))
          c = '_';
      }
      return s;
    });
