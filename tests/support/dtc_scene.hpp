// tests/support/dtc_scene — shared builder for the DTC benchmark cell (Phase B). One place that the
// DTC test, the benchmark, and the visualizer all use to construct the same scene: the rbrobout
// robot (UR10e on an Ewellix lift, with the ee_hilok end-effector baked onto the tip link), the
// work-object environment (mesh.stl + fiducial markers at the app's world poses), the SRDF-derived
// allowed-collision matrix, and a random-pose sampler. Header-only; every entry point takes the
// fixtures dir so it does not depend on any one target's compile definitions.
#pragma once

#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/core/rng.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::dtc {

inline std::string read_text(const std::string &path) {
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// The two mesh packages the flattened rbrobout URDF references.
inline collision::MeshSources meshes(const std::string &fixtures) {
  const std::string m = fixtures + "/robots/meshes/";
  return collision::MeshSources{
      {{"ur_description", m + "ur_description"}, {"dtc_test", m + "dtc_test"}}, ""};
}

inline std::shared_ptr<const RobotModel> load_robot(const std::string &fixtures) {
  return RobotModel::from_urdf(read_text(fixtures + "/robots/rbrobout.urdf"));
}

// Pose from a translation and a (w,x,y,z) quaternion — the layout used in the DTC source poses.
inline Transform pose_wxyz(double x, double y, double z, double qw, double qx, double qy,
                           double qz) {
  return Transform::from_parts(Eigen::Vector3d(x, y, z),
                               Eigen::Quaterniond(qw, qx, qy, qz).normalized());
}

// The work-object environment exactly as the DTC app places it: a single global pose applied to the
// work-object mesh (at the object origin) plus three fiducial markers at their object-frame poses.
inline collision::SceneDescription make_env(const std::string &fixtures) {
  const std::string md = fixtures + "/robots/meshes/dtc_test/meshes/";
  // Global work-object pose in the world frame (setup_scene: pos + quat (x,y,z,w)=(0,0,-0.7071,0.7071)).
  const Transform g = pose_wxyz(1.5258, 0.55341, -0.12717, 0.7071, 0.0, 0.0, -0.7071);

  collision::SceneDescription env;
  auto add = [&](const char *id, const char *file, const Transform &local) {
    env.objects.push_back({id, load_mesh(md + file), g * local});
  };
  add("work_object", "mesh.stl", Transform::Identity());
  add("marker_41", "marker_41.dae", pose_wxyz(0.231127, -0.0231547, 1.86345, 0, 0, -0.7071, 0.7071));
  add("marker_42", "marker_42.dae", pose_wxyz(0.681127, -0.0231547, 1.86345, 0, 0, -0.7071, 0.7071));
  add("marker_43", "marker_43.dae", pose_wxyz(1.03113, -0.0231547, 1.86345, 0, 0, -0.7071, 0.7071));
  return env;
}

// Populate an ACM from an SRDF's <disable_collisions link1=.. link2=..> entries. Attribute order is
// not guaranteed, so match link1/link2 independently within each element.
inline int load_srdf_acm(const std::string &srdf_xml, AllowedCollisionMatrix &acm) {
  const std::regex elem("<disable_collisions\\b[^>]*>");
  const std::regex l1("link1=\"([^\"]+)\"");
  const std::regex l2("link2=\"([^\"]+)\"");
  int n = 0;
  for (auto it = std::sregex_iterator(srdf_xml.begin(), srdf_xml.end(), elem);
       it != std::sregex_iterator(); ++it) {
    const std::string tag = it->str();
    std::smatch a, b;
    if (std::regex_search(tag, a, l1) && std::regex_search(tag, b, l2)) {
      acm.allow(a[1].str(), b[1].str());
      ++n;
    }
  }
  return n;
}

inline void load_acm(const std::string &fixtures, AllowedCollisionMatrix &acm) {
  load_srdf_acm(read_text(fixtures + "/robots/rbrobout.srdf"), acm);
}

// Random configs uniformly inside the joint limits (continuous joints fall back to [-pi, pi]).
inline std::vector<JointPosition> sample_configs(const RobotModel &m, Rng &rng, int n) {
  const int dof = static_cast<int>(m.dof());
  Eigen::VectorXd lo(dof), hi(dof);
  for (const Joint &j : m.joints()) {
    if (!j.is_movable())
      continue;
    const int i = j.dof_index;
    if (j.limits.has_position_limit) {
      lo[i] = j.limits.lower;
      hi[i] = j.limits.upper;
    } else {
      lo[i] = -M_PI;
      hi[i] = M_PI;
    }
  }
  std::vector<JointPosition> qs;
  qs.reserve(n);
  for (int k = 0; k < n; ++k)
    qs.push_back(rng.sample_in_box(lo, hi));
  return qs;
}

} // namespace quevedomp::dtc
