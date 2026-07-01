// robot/RobotModel — immutable robot description parsed from URDF (Task 1.3, spec §6).
//
// Data only: topology (links/joints), joint kinematics (type, axis, origin), and limits. No FK
// or Jacobian here — those land in kinematics/ (Tasks 1.5/1.6). Parsed with urdfdom (see
// README "Dependencies"); the public interface deliberately does not leak any urdfdom type.
#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/core/types.hpp"

namespace quevedomp {

enum class JointType { Fixed, Revolute, Continuous, Prismatic };

struct JointLimits {
  double lower = 0.0;    // position min (rad for revolute, m for prismatic)
  double upper = 0.0;    // position max
  double velocity = 0.0; // |q̇| max (from URDF <limit>)
  double effort = 0.0;   // |τ| max (from URDF <limit>)
  // Acceleration/jerk are absent from standard URDF; supplied by the optional yaml extension.
  // 0 ⇒ unspecified.
  double acceleration = 0.0;
  double jerk = 0.0;
  bool has_position_limit = true; // false for Continuous/Fixed
};

struct Joint {
  std::string name;
  JointType type = JointType::Fixed;
  std::string parent_link;
  std::string child_link;
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ(); // unit axis in the joint frame
  Transform origin;                                // parent-link frame → joint frame
  JointLimits limits;
  // Index of this joint's value in a configuration vector q (size == RobotModel::dof()).
  // Assigned in joints() order over movable joints; -1 for fixed joints.
  int dof_index = -1;

  bool is_movable() const noexcept { return type != JointType::Fixed; }
};

enum class GeometryType { Mesh, Box, Sphere, Cylinder };

// One collision shape attached to a link. Mesh filenames are kept as the raw URDF URI
// (e.g. "package://pkg/.../foo.stl"); resolve them with resolve_mesh_uri() + load_mesh()
// (Task 1.4b). Primitive dimensions are filled for Box/Sphere/Cylinder.
struct CollisionGeometry {
  GeometryType type = GeometryType::Mesh;
  Transform origin; // link frame → geometry frame

  std::string mesh_filename;                            // URDF URI; empty for primitives
  Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones(); // URDF <mesh scale="...">

  Eigen::Vector3d box_half_extents = Eigen::Vector3d::Zero(); // Box
  double sphere_radius = 0.0;                                 // Sphere
  double cylinder_radius = 0.0;                               // Cylinder
  double cylinder_length = 0.0;                               // Cylinder
};

struct Link {
  std::string name;
  int parent_joint = -1;         // index into RobotModel::joints(); -1 at the root link
  std::vector<int> child_joints; // indices into RobotModel::joints()
  std::vector<CollisionGeometry> collisions;
};

// An ordered serial chain, base → tip, as joint indices into RobotModel::joints(). Includes
// fixed joints along the path (FK skips them via Joint::is_movable()).
struct KinematicChain {
  std::string base_link;
  std::string tip_link;
  std::vector<int> joints; // base→tip order
};

class RobotModel {
public:
  // Parse a URDF XML string into an immutable model. `yaml_extension`, if given, supplies
  // per-joint acceleration/jerk limits (which standard URDF lacks). Throws std::runtime_error
  // on parse failure or an unsupported joint type (floating/planar are out of scope for v0).
  static std::shared_ptr<const RobotModel>
  from_urdf(const std::string &urdf_xml, std::optional<std::string> yaml_extension = std::nullopt);

  const std::string &name() const noexcept { return name_; }
  const std::vector<Link> &links() const noexcept { return links_; }
  const std::vector<Joint> &joints() const noexcept { return joints_; }
  const std::string &root_link() const noexcept { return root_link_; }

  // The exact source this model was parsed from — retained so a capture can re-inline and re-parse
  // it losslessly (spec §5.3; used by capture/serialize). Empty yaml ⇒ none was supplied.
  const std::string &source_urdf() const noexcept { return source_urdf_; }
  const std::optional<std::string> &source_yaml() const noexcept { return source_yaml_; }

  std::size_t num_links() const noexcept { return links_.size(); }
  std::size_t num_joints() const noexcept { return joints_.size(); }
  std::size_t dof() const noexcept; // count of movable (non-fixed) joints

  const Link *find_link(const std::string &name) const;
  const Joint *find_joint(const std::string &name) const;

  // base→tip serial chain ending at `tip_link`. Throws std::runtime_error if unknown.
  KinematicChain chain_to(const std::string &tip_link) const;

private:
  RobotModel() = default; // construct only via from_urdf

  std::string name_;
  std::string root_link_;
  std::vector<Link> links_;
  std::vector<Joint> joints_;
  std::unordered_map<std::string, int> link_index_;
  std::unordered_map<std::string, int> joint_index_;
  std::string source_urdf_;
  std::optional<std::string> source_yaml_;
};

} // namespace quevedomp
