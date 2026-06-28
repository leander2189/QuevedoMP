// Regression tests for malformed-input robustness, hardened after the Task 1.9 URDF fuzzer:
//  - deeply nested XML is rejected before urdfdom's recursive parser can stack-overflow;
//  - a cyclic joint structure must not hang or OOM (the fuzzer drove RobotModel::chain_to into
//    an unbounded walk -> a 2 GB allocation; fk_all/jacobian had the same exposure).
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/jacobian.hpp"
#include "quevedomp/robot/robot_model.hpp"

using quevedomp::JointPosition;
using quevedomp::RobotModel;

TEST(RobotRobustness, RejectsDeeplyNestedXml) {
  std::string xml = "<robot name=\"x\">";
  for (int i = 0; i < 500; ++i)
    xml += "<a>"; // 500 > the 256 nesting cap
  EXPECT_THROW(RobotModel::from_urdf(xml), std::runtime_error);
}

TEST(RobotRobustness, CyclicJointStructureTerminates) {
  // Two joints whose parent/child links form a cycle. urdfdom may accept or reject this; either
  // way the model's tree walks must TERMINATE rather than loop forever.
  const std::string xml =
      "<robot name=\"c\">"
      "<link name=\"a\"/><link name=\"b\"/>"
      "<joint name=\"j1\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>"
      "<joint name=\"j2\" type=\"fixed\"><parent link=\"b\"/><child link=\"a\"/></joint>"
      "</robot>";
  try {
    const auto m = RobotModel::from_urdf(xml);
    // Parsed: exercise the walks — the cycle guards make them throw, not spin.
    try {
      (void)m->chain_to("a");
    } catch (const std::exception &) {
    }
    try {
      (void)quevedomp::fk_all(*m, JointPosition::Zero(static_cast<Eigen::Index>(m->dof())));
    } catch (const std::exception &) {
    }
  } catch (const std::exception &) {
    // urdfdom rejected the cyclic model outright — also fine.
  }
  SUCCEED(); // reaching here at all means no hang and no crash
}
