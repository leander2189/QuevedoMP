// Task 3.1 verify — planning types: construction of every type, JointGoal satisfaction, stats
// accounting, status naming, Goal clone, and (the build-plan Done-when) invalid-problem detection.
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::planning;

namespace {

// two_link.urdf: dof()==2 — joint1 revolute [-1.57, 1.57], joint2 prismatic [0, 0.5].
std::shared_ptr<const RobotModel> load_two_link() {
  const std::string path = std::string(QUEVEDOMP_FIXTURE_DIR) + "/robots/two_link.urdf";
  std::ifstream f(path);
  EXPECT_TRUE(f.good()) << "could not open fixture: " << path;
  std::ostringstream ss;
  ss << f.rdbuf();
  return RobotModel::from_urdf(ss.str());
}

JointPosition q2(double a, double b) {
  JointPosition q(2);
  q << a, b;
  return q;
}

// A minimal well-formed problem against the two_link model (start & goal inside limits).
PlanningProblem good_problem() {
  PlanningProblem p;
  p.start = q2(0.0, 0.1);
  p.goal = std::make_shared<JointGoal>(q2(0.5, 0.3), 1e-3);
  p.timeout = 1.0;
  return p;
}

} // namespace

// ---- Construction ------------------------------------------------------------------------

TEST(PlanningTypes, DefaultConstruction) {
  TaskLimits tl;
  EXPECT_EQ(tl.max_linear_velocity, 0.0);
  EXPECT_TRUE(tl.frame.empty());

  Constraints c;
  EXPECT_TRUE(c.joint_bounds.empty());

  PlanningResult r;
  EXPECT_EQ(r.status, PlanningStatus::NoSolution);
  EXPECT_FALSE(r.ok());
  EXPECT_TRUE(r.path.empty());
  EXPECT_EQ(r.used_seed, 0u);

  PlanningProblem p;
  EXPECT_EQ(p.goal, nullptr);
  EXPECT_FALSE(p.seed.has_value());
  EXPECT_DOUBLE_EQ(p.timeout, 1.0);
}

TEST(PlanningTypes, GoalTypesAndClone) {
  JointGoal jg(q2(1.0, 2.0), 0.01);
  EXPECT_EQ(jg.type(), GoalType::Joint);

  PoseGoal pg{Pose{}, "tool0"};
  EXPECT_EQ(pg.type(), GoalType::Pose);
  EXPECT_EQ(pg.tip_link, "tool0");

  auto a = std::make_shared<JointGoal>(q2(0, 0), 0.1);
  auto b = std::make_shared<PoseGoal>(Pose{}, "");
  MultiGoal mg({a, b});
  EXPECT_EQ(mg.type(), GoalType::Multi);
  ASSERT_EQ(mg.goals.size(), 2u);

  // clone() preserves the concrete type and is an independent object.
  auto clone = jg.clone();
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->type(), GoalType::Joint);
  EXPECT_NE(clone.get(), &jg);
}

// ---- JointGoal::satisfies ----------------------------------------------------------------

TEST(PlanningTypes, JointGoalSatisfies) {
  JointGoal jg(q2(1.0, 2.0), 0.05);
  EXPECT_TRUE(jg.satisfies(q2(1.0, 2.0)));      // exact
  EXPECT_TRUE(jg.satisfies(q2(1.04, 1.96)));    // within tol on both joints
  EXPECT_FALSE(jg.satisfies(q2(1.06, 2.0)));    // joint 0 outside tol
  EXPECT_FALSE(jg.satisfies(q2(1.0, 2.06)));    // joint 1 outside tol
  EXPECT_FALSE(jg.satisfies(JointPosition(3))); // size mismatch → never satisfied
}

// ---- PlanningStats -----------------------------------------------------------------------

TEST(PlanningTypes, StatsRecordBatchKeepsCountsConsistent) {
  PlanningStats s;
  s.record_batch(256);
  s.record_batch(256);
  s.record_batch(10);
  EXPECT_EQ(s.collision_queries, 3u);
  EXPECT_EQ(s.collision_configs, 256u + 256u + 10u);
  EXPECT_EQ(s.batch_size_histogram[256], 2u);
  EXPECT_EQ(s.batch_size_histogram[10], 1u);
}

TEST(PlanningTypes, StatusToString) {
  EXPECT_STREQ(to_string(PlanningStatus::Success), "Success");
  EXPECT_STREQ(to_string(PlanningStatus::Timeout), "Timeout");
  EXPECT_STREQ(to_string(PlanningStatus::NoSolution), "NoSolution");
  EXPECT_STREQ(to_string(PlanningStatus::InvalidProblem), "InvalidProblem");
}

// ---- validate: the well-formed case ------------------------------------------------------

TEST(PlanningValidate, AcceptsWellFormedProblem) {
  const auto model = load_two_link();
  EXPECT_FALSE(validate(good_problem(), *model).has_value());
}

TEST(PlanningValidate, AcceptsConfigExactlyOnLimit) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.start = q2(1.57, 0.5); // upper limits of both joints
  EXPECT_FALSE(validate(p, *model).has_value());
}

TEST(PlanningValidate, AcceptsPoseGoalWithKnownTipAndEmptyTip) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.goal = std::make_shared<PoseGoal>(Pose{}, "link2");
  EXPECT_FALSE(validate(p, *model).has_value());
  p.goal = std::make_shared<PoseGoal>(Pose{}, ""); // empty tip ⇒ model tip
  EXPECT_FALSE(validate(p, *model).has_value());
}

TEST(PlanningValidate, AcceptsValidJointBounds) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.constraints.joint_bounds = {{-1.0, 1.0}, {0.0, 0.4}};
  EXPECT_FALSE(validate(p, *model).has_value());
}

// ---- validate: rejection cases (invalid-problem detection) -------------------------------

TEST(PlanningValidate, RejectsWrongStartSize) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.start = JointPosition(3);
  p.start.setZero();
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsStartOutOfLimits) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.start = q2(2.0, 0.1); // joint0 beyond 1.57
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsNullGoal) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.goal = nullptr;
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsJointGoalWrongSize) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.goal = std::make_shared<JointGoal>(JointPosition(1), 1e-3);
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsJointGoalOutOfLimits) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.goal = std::make_shared<JointGoal>(q2(0.0, 0.9), 1e-3); // joint1 beyond 0.5
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsNegativeGoalTolerance) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.goal = std::make_shared<JointGoal>(q2(0.0, 0.1), -0.01);
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsUnknownPoseGoalTip) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.goal = std::make_shared<PoseGoal>(Pose{}, "no_such_link");
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsEmptyMultiGoal) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.goal = std::make_shared<MultiGoal>();
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsMultiGoalWithNullSubGoal) {
  const auto model = load_two_link();
  auto p = good_problem();
  std::vector<std::shared_ptr<const Goal>> subs{std::make_shared<JointGoal>(q2(0, 0.1), 1e-3),
                                                nullptr};
  p.goal = std::make_shared<MultiGoal>(std::move(subs));
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsMultiGoalWithInvalidSubGoal) {
  const auto model = load_two_link();
  auto p = good_problem();
  std::vector<std::shared_ptr<const Goal>> subs{
      std::make_shared<JointGoal>(q2(0, 0.1), 1e-3),
      std::make_shared<JointGoal>(q2(0, 5.0), 1e-3)}; // second out of limits
  p.goal = std::make_shared<MultiGoal>(std::move(subs));
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, AcceptsWellFormedMultiGoal) {
  const auto model = load_two_link();
  auto p = good_problem();
  std::vector<std::shared_ptr<const Goal>> subs{std::make_shared<JointGoal>(q2(0.2, 0.1), 1e-3),
                                                std::make_shared<PoseGoal>(Pose{}, "link1")};
  p.goal = std::make_shared<MultiGoal>(std::move(subs));
  EXPECT_FALSE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsWrongSizedJointBounds) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.constraints.joint_bounds = {{-1.0, 1.0}}; // only 1 of 2 DOF
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsEmptyJointBoundInterval) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.constraints.joint_bounds = {{1.0, -1.0}, {0.0, 0.4}}; // lower > upper
  EXPECT_TRUE(validate(p, *model).has_value());
}

TEST(PlanningValidate, RejectsNonPositiveTimeout) {
  const auto model = load_two_link();
  auto p = good_problem();
  p.timeout = 0.0;
  EXPECT_TRUE(validate(p, *model).has_value());
}
