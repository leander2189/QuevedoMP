#include "quevedomp/kinematics/jacobian.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "quevedomp/kinematics/fk.hpp"

namespace quevedomp {

Eigen::MatrixXd jacobian(const RobotModel &model, const JointPosition &q, const std::string &link) {
  const Link *target = model.find_link(link);
  if (target == nullptr)
    throw std::runtime_error("jacobian: unknown link '" + link + "'");

  const std::vector<Transform> tf = fk_all(model, q); // validates q.size() == model.dof()
  const std::vector<Link> &links = model.links();
  const std::vector<Joint> &joints = model.joints();

  const Eigen::Vector3d p_e = tf[static_cast<std::size_t>(target - links.data())].translation();
  Eigen::MatrixXd j = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(model.dof()));

  // Each movable joint on the base→link path contributes one column. The step bound guards
  // against a cyclic parent relationship in a malformed model (see RobotModel::chain_to).
  std::size_t steps = 0;
  for (const Link *l = target; l->parent_joint >= 0;
       l = model.find_link(joints[l->parent_joint].parent_link)) {
    if (++steps > joints.size())
      throw std::runtime_error("jacobian: cyclic joint structure");
    const Joint &joint = joints[l->parent_joint];
    if (joint.dof_index < 0)
      continue; // fixed joint

    const Link *parent = model.find_link(joint.parent_link);
    const Transform frame = tf[static_cast<std::size_t>(parent - links.data())] * joint.origin;
    const Eigen::Vector3d axis = frame.rotation() * joint.axis.normalized(); // joint axis in base
    const Eigen::Vector3d origin = frame.translation();                      // point on the axis

    if (joint.type == JointType::Revolute || joint.type == JointType::Continuous) {
      j.block<3, 1>(0, joint.dof_index) = axis.cross(p_e - origin);
      j.block<3, 1>(3, joint.dof_index) = axis;
    } else if (joint.type == JointType::Prismatic) {
      j.block<3, 1>(0, joint.dof_index) = axis;
      // angular part stays zero
    }
  }
  return j;
}

} // namespace quevedomp
