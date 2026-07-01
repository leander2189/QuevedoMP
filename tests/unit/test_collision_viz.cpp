// Task 2a.7 verify — a collision scene driven together with the Visualizer. This runs in the
// normal (WITH_RERUN=OFF) suite where every viz call is a no-op: it asserts the collision + witness
// results and that the collision-visualization code path compiles and runs without crashing. Under
// the dev-viz preset (WITH_RERUN=ON) the same test additionally streams to an .rrd.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"
#include "quevedomp/viz/visualizer.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

// Prismatic robot: two r=0.3 spheres; child slides toward the base along +x (nominal x=1.0).
const char *kArm = R"(<robot name="arm">
  <link name="base"><collision><geometry><sphere radius="0.3"/></geometry></collision></link>
  <link name="link1"><collision><geometry><sphere radius="0.3"/></geometry></collision></link>
  <joint name="j1" type="prismatic">
    <parent link="base"/><child link="link1"/>
    <origin xyz="1 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
</robot>)";

JointPosition qv(double x) {
  JointPosition q(1);
  q << x;
  return q;
}

} // namespace

// Sweep the arm into a self-collision while driving the Visualizer exactly as the debugging harness
// does (log_robot, colored log_mesh, witness points + segments). Asserts the collision transition
// and that all viz calls are safe.
TEST(CollisionViz, SweepDrivesVizAndReportsCollision) {
  const auto model = RobotModel::from_urdf(kArm);
  const RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  Visualizer viz("quevedomp/test");
  viz.save(::testing::TempDir() + "/collision_viz_test.rrd"); // no-op unless WITH_RERUN

  QueryOptions opts;
  opts.distance = true;
  opts.max_distance = 2.0f;

  bool saw_free = false;
  bool saw_collision = false;
  const int frames = 20;
  for (int k = 0; k <= frames; ++k) {
    const double x = -1.0 * k / frames; // 0 -> -1: centers 1.0 -> 0.0 apart
    const JointPosition q = qv(x);
    const CollisionResult r = scene->query(robot, q, opts, *ws);

    viz.set_frame(k);
    viz.log_robot("world/arm", *model, q);
    const Visualizer::Color color =
        r.in_collision ? Visualizer::Color{220, 40, 40} : Visualizer::Color{180, 180, 190};
    Mesh dummy;
    dummy.vertices = {{0, 0, 0}, {0.1, 0, 0}, {0, 0.1, 0}};
    dummy.triangles = {{0, 1, 2}};
    viz.log_mesh("world/arm/flag", dummy, Transform{}, color);
    if (r.witness) {
      viz.log_points("world/witness", {r.witness->point_a, r.witness->point_b}, {240, 220, 40},
                     0.02f);
      viz.log_segments("world/witness/seg", {{r.witness->point_a, r.witness->point_b}},
                       {240, 220, 40});
    }

    if (r.in_collision) {
      saw_collision = true;
      EXPECT_LT(r.min_distance, opts.safety_margin + 1e-6f);
      ASSERT_TRUE(r.witness.has_value());
      EXPECT_EQ(r.witness->a, "base");
      EXPECT_EQ(r.witness->b, "link1");
    } else {
      saw_free = true;
    }
  }

  EXPECT_TRUE(saw_free);      // starts clear
  EXPECT_TRUE(saw_collision); // ends overlapping
}

// The distance witness points are in the world frame and lie on the +x axis between the two link
// centers — a sanity check that what the harness draws is geometrically meaningful.
TEST(CollisionViz, WitnessPointsAreWorldFrameOnAxis) {
  const auto model = RobotModel::from_urdf(kArm);
  const RobotInstance robot(model);
  const auto scene = make_static_scene(model, SceneDescription{});
  const auto ws = scene->make_workspace();

  QueryOptions opts;
  opts.distance = true;
  opts.max_distance = 2.0f;

  const CollisionResult r = scene->query(robot, qv(0.0), opts, *ws); // clear: centers 0 and 1.0
  ASSERT_TRUE(r.witness.has_value());
  // Nearest points on the two spheres: base surface at x=0.3, link1 surface at x=0.7.
  EXPECT_NEAR(r.witness->point_a.x(), 0.3, 1e-3);
  EXPECT_NEAR(r.witness->point_b.x(), 0.7, 1e-3);
  EXPECT_NEAR(r.witness->point_a.y(), 0.0, 1e-6);
  EXPECT_NEAR(r.witness->point_b.z(), 0.0, 1e-6);
}
