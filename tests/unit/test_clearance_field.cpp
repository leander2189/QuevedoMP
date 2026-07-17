// Roadmap R3 verify — ClearanceField. Covers: distance vs ANALYTIC SDFs (box, sphere) within
// grid tolerance, negative sign inside a watertight solid, gradients pointing away from the
// surface, batched query consistency, the conservative robot sphere cover, clearance_batch vs
// analytic clearance on the gantry+wall fixture, and CPU/GPU build agreement (meaningful when a
// device is present; trivially equal otherwise).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "../../src/collision/tessellate.hpp"
#include "quevedomp/clearance/clearance_field.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::clearance;

namespace {

collision::SceneDescription box_env(const Eigen::Vector3d &he) {
  collision::SceneDescription env;
  env.objects.push_back({"box", collision::BoxShape{he}, Transform::Identity()});
  return env;
}

// Analytic SDF of an axis-aligned box centred at the origin.
double box_sdf(const Eigen::Vector3d &p, const Eigen::Vector3d &he) {
  const Eigen::Vector3d q = p.cwiseAbs() - he;
  const Eigen::Vector3d outside = q.cwiseMax(0.0);
  const double inside = std::min(std::max({q.x(), q.y(), q.z()}), 0.0);
  return outside.norm() + inside;
}

ClearanceFieldOptions opts(double res, bool gpu = true) {
  ClearanceFieldOptions o;
  o.resolution = res;
  o.margin = 0.3;
  o.use_gpu = gpu;
  return o;
}

} // namespace

TEST(ClearanceField, BoxMatchesAnalyticSdf) {
  const Eigen::Vector3d he(0.2, 0.3, 0.4);
  const auto field = ClearanceField::build(box_env(he), opts(0.02));
  const double tol = 2.5 * field.resolution();

  for (double x = -0.4; x <= 0.4; x += 0.13) {
    for (double y = -0.5; y <= 0.5; y += 0.17) {
      for (double z = -0.6; z <= 0.6; z += 0.19) {
        const Eigen::Vector3d p(x, y, z);
        EXPECT_NEAR(field.distance(p), box_sdf(p, he), tol) << "at " << p.transpose();
      }
    }
  }
  // Deep inside: clearly negative.
  EXPECT_LT(field.distance(Eigen::Vector3d::Zero()), -0.15);
}

TEST(ClearanceField, SphereMatchesAnalyticSdf) {
  collision::SceneDescription env;
  env.objects.push_back({"ball", collision::SphereShape{0.3}, Transform::Identity()});
  const auto field = ClearanceField::build(env, opts(0.02));
  // Tessellated sphere is inscribed (sub-mm at default segments) — allow that on top of grid.
  const double tol = 2.5 * field.resolution() + 2e-3;

  for (const double r : {0.05, 0.2, 0.45, 0.55}) {
    const Eigen::Vector3d p = Eigen::Vector3d(1, 0.7, 0.4).normalized() * r;
    EXPECT_NEAR(field.distance(p), r - 0.3, tol) << "radius " << r;
  }
}

TEST(ClearanceField, GradientPointsAwayFromSurface) {
  const Eigen::Vector3d he(0.2, 0.2, 0.2);
  const auto field = ClearanceField::build(box_env(he), opts(0.02));
  for (const auto &dir : {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0),
                          Eigen::Vector3d(0.6, 0.5, 0.4).normalized()}) {
    const Eigen::Vector3d p = dir * 0.45;
    const Eigen::Vector3d g = field.gradient(p);
    EXPECT_GT(g.normalized().dot(dir), 0.85) << "at " << p.transpose();
  }
}

TEST(ClearanceField, BatchedQueryMatchesScalar) {
  const auto field = ClearanceField::build(box_env({0.2, 0.2, 0.2}), opts(0.02));
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i < 40; ++i) {
    pts.emplace_back(0.03 * i - 0.6, 0.02 * i - 0.4, 0.015 * i - 0.3);
  }
  std::vector<double> d(pts.size());
  std::vector<Eigen::Vector3d> g(pts.size());
  field.query(pts, d, g);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_DOUBLE_EQ(d[i], field.distance(pts[i]));
    EXPECT_TRUE(g[i].isApprox(field.gradient(pts[i])));
  }
}

TEST(ClearanceField, GridCapFailsLoudlyWithSuggestion) {
  ClearanceFieldOptions o = opts(1e-4);
  o.max_voxels = 1000;
  EXPECT_THROW((void)ClearanceField::build(box_env({0.2, 0.2, 0.2}), o), std::runtime_error);
}

TEST(ClearanceField, GpuAndCpuBuildsAgree) {
  const auto gpu = ClearanceField::build(box_env({0.25, 0.2, 0.15}), opts(0.02, true));
  const auto cpu = ClearanceField::build(box_env({0.25, 0.2, 0.15}), opts(0.02, false));
  ASSERT_FALSE(cpu.built_on_gpu());
  ASSERT_EQ(gpu.data().size(), cpu.data().size());
  // JFA tie-breaks may pick different-but-equidistant seeds; distances must agree tightly.
  float worst = 0.0f;
  for (std::size_t i = 0; i < gpu.data().size(); ++i) {
    worst = std::max(worst, std::abs(gpu.data()[i] - cpu.data()[i]));
  }
  EXPECT_LT(worst, 1e-3f) << (gpu.built_on_gpu() ? "GPU vs CPU" : "CPU vs CPU (no device)");
}

// ---- Robot sphere cover + clearance ----------------------------------------------------------

namespace {

const char *kGantry2D = R"(<robot name="gantry2d">
  <link name="base"/>
  <joint name="jx" type="prismatic"><parent link="base"/><child link="cx"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
  <link name="cx"/>
  <joint name="jy" type="prismatic"><parent link="cx"/><child link="ee"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
  <link name="ee">
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
</robot>)";

} // namespace

TEST(RobotSpheres, CoverIsConservative) {
  const auto model = RobotModel::from_urdf(kGantry2D);
  const RobotSpheres rs = decompose_robot(*model, {}, /*target_radius=*/0.06);
  ASSERT_FALSE(rs.spheres.empty());
  // Every tessellated vertex of the ee sphere must lie inside some cover sphere.
  const Mesh m = collision::tessellate_sphere(0.1);
  for (const Eigen::Vector3d &v : m.vertices) {
    double best = 1e9;
    for (const auto &s : rs.spheres) {
      best = std::min(best, (v - s.center).norm() - s.radius);
    }
    EXPECT_LE(best, 1e-9);
  }
}

TEST(RobotSpheres, ClearanceBatchMatchesAnalytic) {
  const auto model = RobotModel::from_urdf(kGantry2D);
  const RobotSpheres rs = decompose_robot(*model);

  // Wall like the planner fixture: box half (0.1, 1.25, 0.5) at (0, -0.75, 0).
  collision::SceneDescription env;
  env.objects.push_back({"wall", collision::BoxShape{Eigen::Vector3d(0.1, 1.25, 0.5)},
                         Transform::from_translation(Eigen::Vector3d(0, -0.75, 0))});
  const auto field = ClearanceField::build(env, opts(0.02));

  std::vector<JointPosition> configs;
  for (const double x : {-1.0, -0.5, -0.25, 0.6}) {
    JointPosition q(2);
    q << x, -1.0;
    configs.push_back(q);
  }
  const auto clearances = clearance_batch(field, *model, rs, configs);
  ASSERT_EQ(clearances.size(), configs.size());
  const double tol = 3.0 * field.resolution() + 1e-3; // grid + sphere-cover conservatism
  for (std::size_t i = 0; i < configs.size(); ++i) {
    // Analytic: EE sphere centre (x, -1, 0) vs the wall box, minus the 0.1 sphere radius.
    const Eigen::Vector3d c(configs[i][0], -1.0, 0.0);
    const double expected = box_sdf(c - Eigen::Vector3d(0, -0.75, 0), {0.1, 1.25, 0.5}) - 0.1;
    EXPECT_NEAR(clearances[i], expected, tol) << "config " << i;
    // Conservative: never OVERestimates true clearance beyond grid noise.
    EXPECT_LE(clearances[i], expected + tol);
  }
}
