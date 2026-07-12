// Task 3.3d P2 verify — primitive tessellations are closed 2-manifolds with vertices on the
// exact surface, and they behave like their exact FCL counterparts in a collision scene
// (overlap/clear on the closed-form sphere-sphere cases from test_collision_fcl).
#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <utility>

#include "../../src/collision/tessellate.hpp"
#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

// Closed orientable 2-manifold: every edge is shared by exactly two triangles (opposite
// orientation), and Euler characteristic V - E + F == 2 (genus 0).
void expect_closed_manifold(const Mesh &m, const char *label) {
  std::map<std::pair<int, int>, int> edges;
  for (const Eigen::Vector3i &t : m.triangles) {
    const int idx[3] = {t.x(), t.y(), t.z()};
    for (int k = 0; k < 3; ++k) {
      int a = idx[k], b = idx[(k + 1) % 3];
      ++edges[{std::min(a, b), std::max(a, b)}];
    }
  }
  for (const auto &[edge, count] : edges)
    EXPECT_EQ(count, 2) << label << ": edge " << edge.first << "-" << edge.second
                        << " shared by " << count << " triangles";
  const auto v = static_cast<long>(m.vertices.size());
  const auto e = static_cast<long>(edges.size());
  const auto f = static_cast<long>(m.triangles.size());
  EXPECT_EQ(v - e + f, 2) << label << ": V-E+F";
}

} // namespace

TEST(Tessellate, BoxIsClosedWithVerticesOnCorners) {
  const Mesh m = tessellate_box(Eigen::Vector3d(0.1, 0.2, 0.3));
  ASSERT_EQ(m.vertices.size(), 8u);
  ASSERT_EQ(m.triangles.size(), 12u);
  expect_closed_manifold(m, "box");
  for (const Eigen::Vector3d &v : m.vertices) {
    EXPECT_NEAR(std::abs(v.x()), 0.1, 1e-15);
    EXPECT_NEAR(std::abs(v.y()), 0.2, 1e-15);
    EXPECT_NEAR(std::abs(v.z()), 0.3, 1e-15);
  }
}

TEST(Tessellate, SphereIsClosedWithVerticesOnSurface) {
  const double r = 0.042; // the dresskit sphere radius
  const Mesh m = tessellate_sphere(r);
  expect_closed_manifold(m, "sphere");
  for (const Eigen::Vector3d &v : m.vertices)
    EXPECT_NEAR(v.norm(), r, 1e-12);
  // Documented inscribed error bound at the default segments: sub-millimetre at this size.
  const double sagitta = r * (1.0 - std::cos(M_PI / 24.0) * std::cos(M_PI / 24.0));
  EXPECT_LT(sagitta, 1e-3);
}

TEST(Tessellate, CylinderIsClosedWithVerticesOnSurface) {
  const double r = 0.027, l = 0.472; // the dresskit cylinder
  const Mesh m = tessellate_cylinder(r, l);
  expect_closed_manifold(m, "cylinder");
  for (std::size_t i = 0; i + 2 < m.vertices.size(); ++i) { // rings (last two are cap centers)
    EXPECT_NEAR(m.vertices[i].head<2>().norm(), r, 1e-12);
    EXPECT_NEAR(std::abs(m.vertices[i].z()), l / 2.0, 1e-12);
  }
}

// The closed-form sphere-robot cases from test_collision_fcl, but with the ENVIRONMENT sphere
// replaced by its tessellation: same overlap/clear answers away from the boundary band.
TEST(Tessellate, TessellatedSphereMatchesExactAnswers) {
  const char *sphere_robot = R"(<robot name="r">
    <link name="base"><collision><geometry><sphere radius="0.5"/></geometry></collision></link>
  </robot>)";
  const auto model = RobotModel::from_urdf(sphere_robot);
  const RobotInstance robot(model);
  QueryOptions opts;
  opts.check_self_collision = false;

  for (const auto &[x, expected] : {std::pair{0.8, true}, std::pair{1.2, false}}) {
    SceneDescription env;
    env.objects.push_back({"obj", tessellate_sphere(0.5),
                           Transform::from_translation(Eigen::Vector3d(x, 0.0, 0.0))});
    const auto scene = make_static_scene(model, env, BackendHint::ForceCpuFcl);
    const auto ws = scene->make_workspace();
    EXPECT_EQ(scene->query(robot, JointPosition(), opts, *ws).in_collision, expected) << x;
  }
}
