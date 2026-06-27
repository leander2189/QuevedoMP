// Task 1.5 verify — forward kinematics. Three independent checks (no ROS/KDL needed):
//  1. Hand-derived SE(3) poses for the 2-link fixture at known configs (pins down conventions:
//     rotation sign, prismatic direction, origin chaining, fixed-joint handling).
//  2. fk()/fk_all() agree with an independent chain-walk reference (raw Eigen 4×4) at many
//     random configs for the 2-link fixture and all five real robots, to < 1e-12 m.
//  3. fk(link) == fk_all()[link]; q-size mismatch throws.
#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "quevedomp/core/rng.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/robot_model.hpp"

using quevedomp::fk;
using quevedomp::fk_all;
using quevedomp::JointPosition;
using quevedomp::JointType;
using quevedomp::Rng;
using quevedomp::RobotModel;

namespace {

std::string read_fixture(const std::string &rel) {
  std::ifstream f(std::string(QUEVEDOMP_FIXTURE_DIR) + "/" + rel);
  EXPECT_TRUE(f.good()) << "missing fixture: " << rel;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Independent reference: pose of a link as a raw 4×4 product along the root→link joint path
// (a different traversal from fk_all's pre-order tree walk).
Eigen::Matrix4d ref_pose(const RobotModel &m, const Eigen::VectorXd &q, int link_idx) {
  std::vector<int> path;
  const quevedomp::Link *l = &m.links()[link_idx];
  while (l->parent_joint >= 0) {
    path.push_back(l->parent_joint);
    l = m.find_link(m.joints()[l->parent_joint].parent_link);
  }
  std::reverse(path.begin(), path.end());

  Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
  for (const int ji : path) {
    const quevedomp::Joint &j = m.joints()[ji];
    const double qi = (j.dof_index >= 0) ? q[j.dof_index] : 0.0;
    Eigen::Matrix4d motion = Eigen::Matrix4d::Identity();
    if (j.type == JointType::Revolute || j.type == JointType::Continuous) {
      motion.block<3, 3>(0, 0) = Eigen::AngleAxisd(qi, j.axis.normalized()).toRotationMatrix();
    } else if (j.type == JointType::Prismatic) {
      motion.block<3, 1>(0, 3) = j.axis.normalized() * qi;
    }
    t = t * j.origin.matrix() * motion;
  }
  return t;
}

} // namespace

TEST(Fk, TwoLinkHandDerivedPoses) {
  const auto m = RobotModel::from_urdf(read_fixture("robots/two_link.urdf"));

  // q = 0: only the (translation-only) joint origins chain: (0,0,.1)+(0.2,0,0)+(0,0,.05).
  JointPosition q0(2);
  q0 << 0.0, 0.0;
  const auto ee0 = fk(*m, q0, "ee_link");
  EXPECT_LT((ee0.translation() - Eigen::Vector3d(0.2, 0.0, 0.15)).norm(), 1e-12);
  EXPECT_LT((ee0.rotation() - Eigen::Matrix3d::Identity()).norm(), 1e-12);

  // q = [pi/2 about +z, 0.3 along +x]. Worked out by hand: position (0, 0.5, 0.15), R = Rz(pi/2).
  JointPosition q1(2);
  q1 << M_PI / 2.0, 0.3;
  const auto ee1 = fk(*m, q1, "ee_link");
  const Eigen::Matrix3d rz = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).matrix();
  EXPECT_LT((ee1.translation() - Eigen::Vector3d(0.0, 0.5, 0.15)).norm(), 1e-12);
  EXPECT_LT((ee1.rotation() - rz).norm(), 1e-12);
}

TEST(Fk, QSizeMismatchThrows) {
  const auto m = RobotModel::from_urdf(read_fixture("robots/two_link.urdf"));
  EXPECT_THROW(fk_all(*m, JointPosition::Zero(1)), std::runtime_error);
  EXPECT_THROW(fk(*m, JointPosition::Zero(5), "ee_link"), std::runtime_error);
  EXPECT_THROW(fk(*m, JointPosition::Zero(2), "nope"), std::runtime_error);
}

class FkAgainstReference : public ::testing::TestWithParam<const char *> {};

TEST_P(FkAgainstReference, MatchesChainWalkReferenceAtRandomConfigs) {
  const auto m = RobotModel::from_urdf(read_fixture(std::string("robots/") + GetParam()));
  const int dof = static_cast<int>(m->dof());

  Rng rng(0xF00DULL);
  for (int trial = 0; trial < 50; ++trial) {
    JointPosition q(dof);
    for (int i = 0; i < dof; ++i)
      q[i] = (trial == 0) ? 0.0 : rng.uniform(-2.0, 2.0);

    const std::vector<quevedomp::Transform> all = fk_all(*m, q);
    ASSERT_EQ(all.size(), m->links().size());

    for (std::size_t li = 0; li < m->links().size(); ++li) {
      const Eigen::Matrix4d ref = ref_pose(*m, q, static_cast<int>(li));
      const double err = (all[li].matrix() - ref).norm();
      ASSERT_LT(err, 1e-12) << GetParam() << " link[" << li << "]=" << m->links()[li].name
                            << " trial " << trial << " err=" << err;
    }

    // fk(single) must equal fk_all() for a sampled tip link.
    const std::string &tip = m->links().back().name;
    const auto single = fk(*m, q, tip);
    const auto *lp = m->find_link(tip);
    const std::size_t idx = static_cast<std::size_t>(lp - m->links().data());
    EXPECT_LT((single.matrix() - all[idx].matrix()).norm(), 1e-12);
  }
}

INSTANTIATE_TEST_SUITE_P(Robots, FkAgainstReference,
                         ::testing::Values("two_link.urdf", "ur5.urdf", "ur10.urdf", "panda.urdf",
                                           "iiwa.urdf", "irb2400.urdf"),
                         [](const ::testing::TestParamInfo<const char *> &info) {
                           std::string s = info.param;
                           for (char &c : s) {
                             if (!std::isalnum(static_cast<unsigned char>(c)))
                               c = '_';
                           }
                           return s;
                         });
