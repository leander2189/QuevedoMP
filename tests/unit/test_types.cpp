// Task 1.1 verify — core/types value types. Headline check (build plan Task 1.1 Done-when):
// Transform compose/inverse round-trips to identity (< 1e-12), plus field access on the
// geometric POD types.
#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Geometry>

#include "quevedomp/core/types.hpp"

using quevedomp::Box;
using quevedomp::JointState;
using quevedomp::Mesh;
using quevedomp::Pose;
using quevedomp::Sphere;
using quevedomp::Trajectory;
using quevedomp::Transform;
using quevedomp::Waypoint;

namespace {

// A non-trivial rigid transform: a rotation about a tilted axis plus a translation.
Transform make_sample() {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
  return Transform::from_parts(Eigen::Vector3d(1.5, -3.0, 2.25), q);
}

} // namespace

TEST(Transform, DefaultIsIdentity) {
  EXPECT_TRUE(Transform{}.is_approx(Transform::Identity(), 1e-15));
  EXPECT_TRUE(Transform::Identity().matrix().isIdentity(0.0));
}

TEST(Transform, ComposeWithInverseIsIdentity) {
  const Transform t = make_sample();

  // The Task 1.1 Done-when bar: T ∘ T⁻¹ and T⁻¹ ∘ T both round-trip to identity < 1e-12.
  EXPECT_TRUE((t * t.inverse()).is_approx(Transform::Identity(), 1e-12));
  EXPECT_TRUE((t.inverse() * t).is_approx(Transform::Identity(), 1e-12));
}

TEST(Transform, InverseUndoesPointMapping) {
  const Transform t = make_sample();
  const Eigen::Vector3d p(0.3, 1.1, -0.7);
  const Eigen::Vector3d mapped = t * p;
  const Eigen::Vector3d back = t.inverse() * mapped;
  EXPECT_LT((back - p).norm(), 1e-12);
}

TEST(Transform, CompositionMatchesSequentialPointMapping) {
  const Transform a = Transform::from_translation(Eigen::Vector3d(1.0, 0.0, 0.0));
  const Transform b = Transform::from_rotation(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())));
  const Eigen::Vector3d p(2.0, 0.0, 0.0);

  // (a ∘ b) applied to p must equal a applied to (b applied to p).
  EXPECT_LT(((a * b) * p - a * (b * p)).norm(), 1e-12);
}

TEST(Transform, FromPartsRecoversTranslationAndRotation) {
  const Eigen::Vector3d t(4.0, -1.0, 0.5);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(1.2, Eigen::Vector3d(0.0, 1.0, 0.0)));
  const Transform tf = Transform::from_parts(t, q);

  EXPECT_LT((tf.translation() - t).norm(), 1e-12);
  EXPECT_LT((tf.rotation() - q.normalized().toRotationMatrix()).norm(), 1e-12);
}

TEST(Sphere, FieldAccess) {
  Sphere s{Eigen::Vector3d(1.0, 2.0, 3.0), 0.5};
  EXPECT_DOUBLE_EQ(s.center.y(), 2.0);
  EXPECT_DOUBLE_EQ(s.radius, 0.5);

  const Sphere def; // default-constructed
  EXPECT_TRUE(def.center.isZero(0.0));
  EXPECT_DOUBLE_EQ(def.radius, 0.0);
}

TEST(Box, FieldAccess) {
  Box b;
  b.half_extents = Eigen::Vector3d(0.1, 0.2, 0.3);
  b.tf = Transform::from_translation(Eigen::Vector3d(0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(b.half_extents.z(), 0.3);
  EXPECT_DOUBLE_EQ(b.tf.translation().z(), 1.0);
}

TEST(Pose, HasDocumentedDefaultTolerances) {
  const Pose p;
  EXPECT_DOUBLE_EQ(p.pos_tol, 1e-3);
  EXPECT_DOUBLE_EQ(p.rot_tol, 1e-2);
}

TEST(JointState, HoldsPosAndVel) {
  JointState s;
  s.pos = Eigen::VectorXd::Constant(6, 0.25);
  s.vel = Eigen::VectorXd::Zero(6);
  EXPECT_EQ(s.pos.size(), 6);
  EXPECT_DOUBLE_EQ(s.pos[3], 0.25);
  EXPECT_TRUE(s.vel.isZero(0.0));
}

TEST(Trajectory, IsAWaypointSequence) {
  Trajectory traj;
  Waypoint w0;
  w0.time = 0.0;
  w0.state.pos = Eigen::VectorXd::Zero(2);
  Waypoint w1;
  w1.time = 1.5;
  w1.state.pos = Eigen::VectorXd::Constant(2, 1.0);
  traj.push_back(w0);
  traj.push_back(w1);

  ASSERT_EQ(traj.size(), 2u);
  EXPECT_DOUBLE_EQ(traj.back().time, 1.5);
  EXPECT_DOUBLE_EQ(traj.front().state.pos[0], 0.0);
}

TEST(Mesh, StoresVerticesAndTriangles) {
  Mesh m;
  m.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  m.triangles = {{0, 1, 2}};
  EXPECT_EQ(m.vertices.size(), 3u);
  ASSERT_EQ(m.triangles.size(), 1u);
  EXPECT_EQ(m.triangles[0].x(), 0);
  EXPECT_EQ(m.triangles[0].z(), 2);
}
