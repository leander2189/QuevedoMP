// Task 1.6 verify — geometric Jacobian. The independent oracle is the finite difference of fk
// (itself validated in Task 1.5): analytic vs central-difference Jacobian < 1e-6 across random
// configs for all 5 robots, plus a hand-derived anchor for the 2-link fixture.
#include <gtest/gtest.h>

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "quevedomp/core/rng.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/jacobian.hpp"
#include "quevedomp/robot/robot_model.hpp"

using quevedomp::fk;
using quevedomp::jacobian;
using quevedomp::JointPosition;
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

// Central-difference Jacobian from fk: linear rows from the position delta, angular rows from
// the relative-rotation vector. This is the reference the analytic Jacobian must match.
Eigen::MatrixXd fd_jacobian(const RobotModel &m, const JointPosition &q, const std::string &link,
                            double h) {
  const int dof = static_cast<int>(m.dof());
  Eigen::MatrixXd j(6, dof);
  for (int i = 0; i < dof; ++i) {
    JointPosition qp = q;
    JointPosition qm = q;
    qp[i] += h;
    qm[i] -= h;
    const auto tp = fk(m, qp, link);
    const auto tm = fk(m, qm, link);
    j.block<3, 1>(0, i) = (tp.translation() - tm.translation()) / (2.0 * h);
    const Eigen::AngleAxisd aa(tp.rotation() * tm.rotation().transpose());
    j.block<3, 1>(3, i) = (aa.axis() * aa.angle()) / (2.0 * h);
  }
  return j;
}

} // namespace

TEST(Jacobian, TwoLinkHandDerivedAtZero) {
  const auto m = RobotModel::from_urdf(read_fixture("robots/two_link.urdf"));
  JointPosition q(2);
  q << 0.0, 0.0;
  const Eigen::MatrixXd j = jacobian(*m, q, "ee_link");
  ASSERT_EQ(j.rows(), 6);
  ASSERT_EQ(j.cols(), 2);

  // joint1 (revolute +z at (0,0,0.1)), ee at (0.2,0,0.15): Jv = z×(p_e-p) = (0,0.2,0), Jw = z.
  Eigen::Matrix<double, 6, 1> col0;
  col0 << 0, 0.2, 0, 0, 0, 1;
  // joint2 (prismatic +x): Jv = x, Jw = 0.
  Eigen::Matrix<double, 6, 1> col1;
  col1 << 1, 0, 0, 0, 0, 0;
  EXPECT_LT((j.col(0) - col0).norm(), 1e-12);
  EXPECT_LT((j.col(1) - col1).norm(), 1e-12);
}

TEST(Jacobian, ErrorsOnBadInput) {
  const auto m = RobotModel::from_urdf(read_fixture("robots/two_link.urdf"));
  EXPECT_THROW(jacobian(*m, JointPosition::Zero(2), "nope"), std::runtime_error);
  EXPECT_THROW(jacobian(*m, JointPosition::Zero(1), "ee_link"), std::runtime_error);
}

class JacobianFd : public ::testing::TestWithParam<const char *> {};

TEST_P(JacobianFd, AnalyticMatchesFiniteDifference) {
  const auto m = RobotModel::from_urdf(read_fixture(std::string("robots/") + GetParam()));
  const int dof = static_cast<int>(m->dof());
  const std::string tip = m->links().back().name;

  Rng rng(0xBEEFULL);
  for (int trial = 0; trial < 40; ++trial) {
    JointPosition q(dof);
    for (int i = 0; i < dof; ++i)
      q[i] = rng.uniform(-2.0, 2.0);

    const Eigen::MatrixXd ja = jacobian(*m, q, tip);
    const Eigen::MatrixXd jf = fd_jacobian(*m, q, tip, 1e-6);
    const double err = (ja - jf).cwiseAbs().maxCoeff();
    ASSERT_LT(err, 1e-6) << GetParam() << " trial " << trial << " max|Δ|=" << err;
  }
}

INSTANTIATE_TEST_SUITE_P(Robots, JacobianFd,
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
