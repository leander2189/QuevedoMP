#include "quevedomp/kinematics/fk.hpp"

#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

namespace quevedomp {
namespace {

// Motion transform contributed by a joint at value qi (identity for fixed joints).
Transform joint_motion(const Joint &j, double qi) {
  switch (j.type) {
  case JointType::Revolute:
  case JointType::Continuous: {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::AngleAxisd(qi, j.axis.normalized()).toRotationMatrix();
    return Transform(iso);
  }
  case JointType::Prismatic:
    return Transform::from_translation(j.axis.normalized() * qi);
  case JointType::Fixed:
  default:
    return Transform::Identity();
  }
}

} // namespace

std::vector<Transform> fk_all(const RobotModel &model, const JointPosition &q) {
  if (static_cast<std::size_t>(q.size()) != model.dof()) {
    throw std::runtime_error("fk_all: q.size() does not match model.dof()");
  }

  const std::vector<Link> &links = model.links();
  const std::vector<Joint> &joints = model.joints();
  std::vector<Transform> tf(links.size());

  const Link *root = model.find_link(model.root_link());
  if (root == nullptr)
    throw std::runtime_error("fk_all: model has no root link");
  const int root_idx = static_cast<int>(root - links.data());

  // Pre-order tree walk from the root, accumulating base-frame poses:
  //   tf[child] = tf[parent] · joint_origin · joint_motion(q).
  // `visited` guards against a cyclic child graph in a malformed model (found by the Task 1.9
  // fuzzer): without it the walk would revisit links forever and grow the stack until OOM.
  std::vector<char> visited(links.size(), 0);
  tf[root_idx] = Transform::Identity();
  visited[root_idx] = 1;
  std::vector<int> stack{root_idx};
  while (!stack.empty()) {
    const int li = stack.back();
    stack.pop_back();
    for (const int jc : links[li].child_joints) {
      const Joint &j = joints[jc];
      const double qi = (j.dof_index >= 0) ? q[j.dof_index] : 0.0;
      const Link *child = model.find_link(j.child_link);
      if (child == nullptr)
        continue; // dangling joint child; skip defensively
      const int ci = static_cast<int>(child - links.data());
      if (visited[ci])
        continue; // cycle — already placed this link
      visited[ci] = 1;
      tf[ci] = tf[li] * j.origin * joint_motion(j, qi);
      stack.push_back(ci);
    }
  }
  return tf;
}

Transform fk(const RobotModel &model, const JointPosition &q, const std::string &link) {
  const Link *l = model.find_link(link);
  if (l == nullptr)
    throw std::runtime_error("fk: unknown link '" + link + "'");
  const std::vector<Transform> all = fk_all(model, q);
  return all[static_cast<std::size_t>(l - model.links().data())];
}

} // namespace quevedomp
