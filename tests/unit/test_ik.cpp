// Task 1.7 verify — numerical IK (spec §6 DoD). Two facets:
//  * Global robustness: from a *cold* (far random) seed, IK recovers a configuration whose fk
//    matches a random reachable target (FK∘IK ≈ identity) for ~all of many seeds.
//  * Performance: from a *warm* seed (the typical IK call), convergence is fast (< 10 ms).
#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "quevedomp/core/rng.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/robot/robot_model.hpp"

using quevedomp::fk;
using quevedomp::JointPosition;
using quevedomp::make_numerical_ik;
using quevedomp::Rng;
using quevedomp::RobotModel;
using quevedomp::Transform;

namespace {

std::string read_fixture(const std::string &rel) {
  std::ifstream f(std::string(QUEVEDOMP_FIXTURE_DIR) + "/" + rel);
  EXPECT_TRUE(f.good()) << "missing fixture: " << rel;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// A real end-effector. links().back() is alphabetical, which for panda is a finger; use the
// arm flange there.
std::string tip_of(const RobotModel &m, const std::string &urdf) {
  return (urdf == "panda.urdf") ? std::string("panda_link8") : m.links().back().name;
}

double joint_lo(const quevedomp::Joint &j) {
  return j.limits.has_position_limit ? j.limits.lower : -M_PI;
}
double joint_hi(const quevedomp::Joint &j) {
  return j.limits.has_position_limit ? j.limits.upper : M_PI;
}

JointPosition sample_q(const RobotModel &m, Rng &rng) {
  JointPosition q(static_cast<Eigen::Index>(m.dof()));
  for (const auto &j : m.joints()) {
    if (j.dof_index >= 0)
      q[j.dof_index] = rng.uniform(joint_lo(j), joint_hi(j));
  }
  return q;
}

// A warm seed: the true config nudged by small per-joint noise, kept within limits.
JointPosition near_seed(const RobotModel &m, const JointPosition &q_true, Rng &rng) {
  JointPosition q = q_true;
  for (const auto &j : m.joints()) {
    if (j.dof_index < 0)
      continue;
    const double v = q[j.dof_index] + rng.uniform(-0.1, 0.1);
    q[j.dof_index] = std::min(std::max(v, joint_lo(j)), joint_hi(j));
  }
  return q;
}

double rot_error(const Transform &a, const Transform &b) {
  return Eigen::AngleAxisd(a.rotation() * b.rotation().transpose()).angle();
}

double percentile(std::vector<double> v, double p) {
  if (v.empty())
    return 0.0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, static_cast<std::size_t>(p * v.size()))];
}

// The spec's <10 ms target is a release metric; tests run under ASan/UBSan (~4x slower), so the
// median-time budget is relaxed for sanitized builds.
#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
constexpr double kWarmMedianBudgetMs = 10.0; // warm seeds converge fast enough even sanitized
#else
constexpr double kWarmMedianBudgetMs = 10.0;
#endif

constexpr int kN = 100;
const char *kRobots[] = {"ur5.urdf", "ur10.urdf", "panda.urdf", "iiwa.urdf", "irb2400.urdf"};

std::string param_name(const ::testing::TestParamInfo<const char *> &info) {
  std::string s = info.param;
  for (char &c : s) {
    if (!std::isalnum(static_cast<unsigned char>(c)))
      c = '_';
  }
  return s;
}

} // namespace

// --- Global robustness: cold (far) seed -----------------------------------------------------
class IkGlobal : public ::testing::TestWithParam<const char *> {};

TEST_P(IkGlobal, FkOfIkRecoversTargetFromColdSeed) {
  const auto model = RobotModel::from_urdf(read_fixture(std::string("robots/") + GetParam()));
  const std::string tip = tip_of(*model, GetParam());
  const auto ik = make_numerical_ik(model);

  Rng rng(0x1C5EEDULL);
  int success = 0;
  double max_pos = 0.0;
  double max_rot = 0.0;
  for (int i = 0; i < kN; ++i) {
    const JointPosition q_true = sample_q(*model, rng);
    const Transform target = fk(*model, q_true, tip);
    const auto res = ik->solve(tip, target, sample_q(*model, rng)); // far seed
    if (!res.success)
      continue;
    ++success;
    const Transform got = fk(*model, res.q, tip);
    const double pe = (got.translation() - target.translation()).norm();
    const double re = rot_error(got, target);
    max_pos = std::max(max_pos, pe);
    max_rot = std::max(max_rot, re);
    EXPECT_LT(pe, 1e-3) << GetParam();
    EXPECT_LT(re, 1e-2) << GetParam();
  }
  std::cout << "[ ik:cold ] " << GetParam() << ": success " << success << "/" << kN
            << ", max_pos=" << max_pos << " m, max_rot=" << max_rot << " rad\n";
  EXPECT_GE(success, static_cast<int>(0.99 * kN)) << GetParam();
}

INSTANTIATE_TEST_SUITE_P(Robots, IkGlobal, ::testing::ValuesIn(kRobots), param_name);

// --- Performance: warm seed (typical IK call) -----------------------------------------------
class IkWarm : public ::testing::TestWithParam<const char *> {};

TEST_P(IkWarm, ConvergesQuicklyFromWarmSeed) {
  const auto model = RobotModel::from_urdf(read_fixture(std::string("robots/") + GetParam()));
  const std::string tip = tip_of(*model, GetParam());
  const auto ik = make_numerical_ik(model);

  Rng rng(0x7A47ULL);
  int success = 0;
  std::vector<double> times_ms;
  times_ms.reserve(kN);
  for (int i = 0; i < kN; ++i) {
    const JointPosition q_true = sample_q(*model, rng);
    const Transform target = fk(*model, q_true, tip);
    const JointPosition seed = near_seed(*model, q_true, rng);

    const auto t0 = std::chrono::steady_clock::now();
    const auto res = ik->solve(tip, target, seed);
    times_ms.push_back(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    if (res.success)
      ++success;
  }

  const double median_ms = percentile(times_ms, 0.50);
  const double p95_ms = percentile(times_ms, 0.95);
  std::cout << "[ ik:warm ] " << GetParam() << ": success " << success << "/" << kN
            << ", median=" << median_ms << " ms, p95=" << p95_ms << " ms\n";

  EXPECT_GE(success, static_cast<int>(0.99 * kN)) << GetParam();
  EXPECT_LT(median_ms, kWarmMedianBudgetMs) << GetParam();
}

INSTANTIATE_TEST_SUITE_P(Robots, IkWarm, ::testing::ValuesIn(kRobots), param_name);

// --- solve_all: distinct IK branches (multi-solution) ----------------------------------------
// A 6R arm has up to 8 IK branches per reachable pose; solve_all must find several DISTINCT ones
// (all matching the target under fk), be deterministic per options.seed, and put the branch
// nearest a given seed first (the tracking ordering).

namespace {

quevedomp::IkOptions branch_options() {
  quevedomp::IkOptions o;
  o.max_restarts = 80; // exploration budget: one attempt per restart
  return o;
}

} // namespace

TEST(IkSolveAll, FindsMultipleDistinctBranchesMatchingTarget) {
  const auto model = RobotModel::from_urdf(read_fixture("robots/ur5.urdf"));
  const std::string tip = tip_of(*model, "ur5.urdf");
  const auto ik = make_numerical_ik(model, branch_options());

  Rng rng(0xB4A2C7ULL);
  const JointPosition q_true = sample_q(*model, rng);
  const Transform target = fk(*model, q_true, tip);

  const auto sols = ik->solve_all(tip, target, 8);
  ASSERT_GE(sols.size(), 2u) << "a 6R pose should expose several branches";
  for (std::size_t i = 0; i < sols.size(); ++i) {
    EXPECT_TRUE(sols[i].success);
    const Transform reached = fk(*model, sols[i].q, tip);
    EXPECT_LT((reached.translation() - target.translation()).norm(), 1e-3);
    EXPECT_LT(rot_error(reached, target), 1e-2);
    for (std::size_t j = i + 1; j < sols.size(); ++j) {
      EXPECT_GE((sols[i].q - sols[j].q).cwiseAbs().maxCoeff(), branch_options().branch_tol)
          << "solutions " << i << " and " << j << " are the same branch";
    }
  }
}

TEST(IkSolveAll, DeterministicPerSeed) {
  const auto model = RobotModel::from_urdf(read_fixture("robots/ur5.urdf"));
  const std::string tip = tip_of(*model, "ur5.urdf");
  const auto ik = make_numerical_ik(model, branch_options());

  Rng rng(0x5EEDULL);
  const Transform target = fk(*model, sample_q(*model, rng), tip);

  const auto a = ik->solve_all(tip, target, 6);
  const auto b = ik->solve_all(tip, target, 6);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i)
    EXPECT_TRUE(a[i].q.isApprox(b[i].q));
}

TEST(IkSolveAll, SeedOrdersNearestBranchFirst) {
  const auto model = RobotModel::from_urdf(read_fixture("robots/ur5.urdf"));
  const std::string tip = tip_of(*model, "ur5.urdf");
  const auto ik = make_numerical_ik(model, branch_options());

  Rng rng(0x0DE4ULL);
  const Transform target = fk(*model, sample_q(*model, rng), tip);
  const auto sols = ik->solve_all(tip, target, 8);
  ASSERT_GE(sols.size(), 2u);

  // Seed with the LAST discovered branch: it must come first, and distances must ascend.
  const JointPosition seed = sols.back().q;
  const auto ordered = ik->solve_all(tip, target, 8, seed);
  ASSERT_GE(ordered.size(), 2u);
  EXPECT_LT((ordered.front().q - seed).cwiseAbs().maxCoeff(), branch_options().branch_tol);
  for (std::size_t i = 0; i + 1 < ordered.size(); ++i)
    EXPECT_LE((ordered[i].q - seed).norm(), (ordered[i + 1].q - seed).norm());
}

TEST(IkSolveAll, UnreachableOrZeroBudgetIsEmpty) {
  const auto model = RobotModel::from_urdf(read_fixture("robots/ur5.urdf"));
  const std::string tip = tip_of(*model, "ur5.urdf");
  const auto ik = make_numerical_ik(model, branch_options());

  const Transform far = Transform::from_translation(Eigen::Vector3d(5, 0, 0));
  EXPECT_TRUE(ik->solve_all(tip, far, 4).empty()); // out of reach: no best-effort entries
  Rng rng(0x0FFULL);
  const Transform target = fk(*model, sample_q(*model, rng), tip);
  EXPECT_TRUE(ik->solve_all(tip, target, 0).empty());
}
