#include "quevedomp/robot/robot_model.hpp"

#include <stdexcept>
#include <utility>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

namespace quevedomp {
namespace {

JointType to_joint_type(int urdf_type) {
  switch (urdf_type) {
  case urdf::Joint::REVOLUTE:
    return JointType::Revolute;
  case urdf::Joint::CONTINUOUS:
    return JointType::Continuous;
  case urdf::Joint::PRISMATIC:
    return JointType::Prismatic;
  case urdf::Joint::FIXED:
    return JointType::Fixed;
  default:
    throw std::runtime_error(
        "RobotModel::from_urdf: unsupported joint type (floating/planar not supported in v0)");
  }
}

Transform to_transform(const urdf::Pose &p) {
  double x, y, z, w;
  p.rotation.getQuaternion(x, y, z, w);
  const Eigen::Quaterniond q(w, x, y, z);
  const Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
  return Transform::from_parts(t, q);
}

void apply_yaml_extension(const std::string &yaml_text, std::vector<Joint> &joints,
                          const std::unordered_map<std::string, int> &joint_index) {
  const YAML::Node root = YAML::Load(yaml_text);
  const YAML::Node limits = root["joint_limits"];
  if (!limits)
    return;
  for (const auto &entry : limits) {
    const auto it = joint_index.find(entry.first.as<std::string>());
    if (it == joint_index.end())
      continue; // limits for an unknown joint are ignored
    JointLimits &lim = joints[it->second].limits;
    const YAML::Node spec = entry.second;
    if (spec["max_acceleration"])
      lim.acceleration = spec["max_acceleration"].as<double>();
    if (spec["max_jerk"])
      lim.jerk = spec["max_jerk"].as<double>();
  }
}

} // namespace

std::shared_ptr<const RobotModel> RobotModel::from_urdf(const std::string &urdf_xml,
                                                        std::optional<std::string> yaml_extension) {
  const urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_xml);
  if (!model)
    throw std::runtime_error("RobotModel::from_urdf: failed to parse URDF");

  std::shared_ptr<RobotModel> out(new RobotModel());
  out->name_ = model->getName();
  if (model->getRoot())
    out->root_link_ = model->getRoot()->name;

  // Links first so joints can resolve link indices. Root is inserted first for a sensible
  // ordering; the rest follow. Topology is captured via parent/child indices, so the exact
  // storage order is not load-bearing.
  auto add_link = [&](const std::string &lname) {
    if (out->link_index_.count(lname))
      return;
    out->link_index_.emplace(lname, static_cast<int>(out->links_.size()));
    Link link;
    link.name = lname;
    out->links_.push_back(std::move(link));
  };
  if (!out->root_link_.empty())
    add_link(out->root_link_);
  for (const auto &kv : model->links_)
    add_link(kv.first);

  for (const auto &kv : model->joints_) {
    const urdf::JointSharedPtr &uj = kv.second;
    Joint j;
    j.name = uj->name;
    j.type = to_joint_type(uj->type);
    j.parent_link = uj->parent_link_name;
    j.child_link = uj->child_link_name;
    j.axis = Eigen::Vector3d(uj->axis.x, uj->axis.y, uj->axis.z);
    j.origin = to_transform(uj->parent_to_joint_origin_transform);
    if (uj->limits) {
      j.limits.lower = uj->limits->lower;
      j.limits.upper = uj->limits->upper;
      j.limits.velocity = uj->limits->velocity;
      j.limits.effort = uj->limits->effort;
    }
    j.limits.has_position_limit = (j.type == JointType::Revolute || j.type == JointType::Prismatic);
    out->joint_index_.emplace(j.name, static_cast<int>(out->joints_.size()));
    out->joints_.push_back(std::move(j));
  }

  // Wire link adjacency (parent_joint / child_joints) from the parsed joints.
  for (int jidx = 0; jidx < static_cast<int>(out->joints_.size()); ++jidx) {
    const Joint &j = out->joints_[jidx];
    const auto pit = out->link_index_.find(j.parent_link);
    const auto cit = out->link_index_.find(j.child_link);
    if (pit != out->link_index_.end())
      out->links_[pit->second].child_joints.push_back(jidx);
    if (cit != out->link_index_.end())
      out->links_[cit->second].parent_joint = jidx;
  }

  if (yaml_extension)
    apply_yaml_extension(*yaml_extension, out->joints_, out->joint_index_);

  return out;
}

std::size_t RobotModel::dof() const noexcept {
  std::size_t n = 0;
  for (const Joint &j : joints_) {
    if (j.is_movable())
      ++n;
  }
  return n;
}

const Link *RobotModel::find_link(const std::string &name) const {
  const auto it = link_index_.find(name);
  return it == link_index_.end() ? nullptr : &links_[it->second];
}

const Joint *RobotModel::find_joint(const std::string &name) const {
  const auto it = joint_index_.find(name);
  return it == joint_index_.end() ? nullptr : &joints_[it->second];
}

KinematicChain RobotModel::chain_to(const std::string &tip_link) const {
  if (link_index_.find(tip_link) == link_index_.end()) {
    throw std::runtime_error("RobotModel::chain_to: unknown link '" + tip_link + "'");
  }
  KinematicChain chain;
  chain.base_link = root_link_;
  chain.tip_link = tip_link;

  // Walk parent joints from tip up to the root, then reverse into base→tip order.
  std::vector<int> rev;
  const Link *link = find_link(tip_link);
  while (link != nullptr && link->parent_joint >= 0) {
    rev.push_back(link->parent_joint);
    link = find_link(joints_[link->parent_joint].parent_link);
  }
  chain.joints.assign(rev.rbegin(), rev.rend());
  return chain;
}

} // namespace quevedomp
