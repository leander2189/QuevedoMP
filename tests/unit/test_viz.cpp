// Task 1.8 verify — viz API. Builds and runs in both modes: WITH_RERUN=OFF (every call is a
// no-op, enabled()==false, no file written) and WITH_RERUN=ON (enabled()==true, save() writes a
// non-empty .rrd). The logging calls must never crash regardless of build.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_model.hpp"
#include "quevedomp/viz/visualizer.hpp"

using quevedomp::JointPosition;
using quevedomp::RobotModel;
using quevedomp::Trajectory;
using quevedomp::Transform;
using quevedomp::Visualizer;
using quevedomp::Waypoint;

namespace {
std::string read_fixture(const std::string &rel) {
  std::ifstream f(std::string(QUEVEDOMP_FIXTURE_DIR) + "/" + rel);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
} // namespace

TEST(Viz, NoOpWhenDisabledAndWritesWhenEnabled) {
  const auto model = RobotModel::from_urdf(read_fixture("robots/two_link.urdf"));
  const JointPosition q = JointPosition::Zero(static_cast<Eigen::Index>(model->dof()));
  const std::string path = std::string(::testing::TempDir()) + "/qmp_viz_test.rrd";
  std::remove(path.c_str());

  bool enabled = false;
  {
    Visualizer viz("quevedomp_test");
    enabled = viz.enabled();
    if (enabled)
      viz.save(path);

    // None of these may crash, in either build mode.
    viz.log_pose("world/pose", Transform::from_translation(Eigen::Vector3d(1, 2, 3)));
    viz.log_robot("world/robot", *model, q);
    Trajectory traj;
    Waypoint w;
    w.state.pos = q;
    traj.push_back(w);
    viz.log_trajectory("world/path", *model, traj, "ee_link");
  } // flush on destruction

  if (enabled) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    EXPECT_TRUE(f.good()) << "enabled build should have written " << path;
    EXPECT_GT(static_cast<long>(f.tellg()), 0) << "rrd should be non-empty";
  } else {
    std::ifstream f(path);
    EXPECT_FALSE(f.good()) << "disabled build must not write any file";
  }
}
