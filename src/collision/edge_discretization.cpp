// collision/edge_discretization — Cartesian-bounded edge stepping (Task 3.3d P3).
//
// The weight recursion is a chain of bounding balls, tip→base. For a joint j define R_j = the max
// distance from j's frame origin to any collision-geometry point distal of j, over ALL
// configurations. R_j is computable at q = 0 because every distal motion preserves it: a revolute
// child rotates its ball about an axis through its own origin (radius invariant), and a prismatic
// child's travel is added to its offset explicitly. Then moving joint i by |Δq_i| displaces any
// distal point by at most w_i·|Δq_i| (arc chord ≤ radius × angle for revolute, exactly the travel
// for prismatic), and summing over joints bounds the total sweep of an edge step — conservative,
// never under.
#include "quevedomp/collision/edge_discretization.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"

namespace quevedomp::collision {
namespace {

// Max distance from the link-frame origin to any point of one collision shape. Primitives use
// their circumradius about the geometry frame plus the origin offset; meshes are exact per vertex
// (scale applied before origin, matching how the collision backends pose them). The cache keys on
// the resolved path so a mesh shared across links loads once (scale is applied per call).
double geometry_radius(const CollisionGeometry &cg, const MeshSources &meshes,
                       std::unordered_map<std::string, Mesh> &mesh_cache) {
  const double offset = cg.origin.translation().norm();
  switch (cg.type) {
  case GeometryType::Box:
    return offset + cg.box_half_extents.norm();
  case GeometryType::Sphere:
    return offset + cg.sphere_radius;
  case GeometryType::Cylinder:
    return offset + std::hypot(cg.cylinder_radius, 0.5 * cg.cylinder_length);
  case GeometryType::Mesh: {
    const std::string path =
        resolve_mesh_uri(cg.mesh_filename, meshes.package_dirs, meshes.base_dir);
    auto it = mesh_cache.find(path);
    if (it == mesh_cache.end()) {
      it = mesh_cache.emplace(path, load_mesh(path)).first;
    }
    double r = 0.0;
    for (const Eigen::Vector3d &v : it->second.vertices) {
      r = std::max(r, (cg.origin * v.cwiseProduct(cg.mesh_scale).eval()).norm());
    }
    return r;
  }
  }
  return 0.0;
}

} // namespace

JointPosition cartesian_lever_weights(const RobotModel &model, const MeshSources &meshes) {
  const auto &links = model.links();
  const auto &joints = model.joints();

  // Per-link geometry extent about the link-frame origin (== the parent joint's frame in URDF).
  std::unordered_map<std::string, Mesh> mesh_cache;
  std::vector<double> link_radius(links.size(), 0.0);
  for (std::size_t li = 0; li < links.size(); ++li) {
    for (const CollisionGeometry &cg : links[li].collisions) {
      link_radius[li] = std::max(link_radius[li], geometry_radius(cg, meshes, mesh_cache));
    }
  }

  // R_j per joint, memoized tip→base. -1 marks "not yet computed"; the URDF tree has no cycles,
  // so plain recursion terminates.
  std::vector<double> reach(joints.size(), -1.0);
  const auto compute_reach = [&](auto &&self, int ji) -> double {
    if (reach[static_cast<std::size_t>(ji)] >= 0.0) {
      return reach[static_cast<std::size_t>(ji)];
    }
    const Joint &j = joints[static_cast<std::size_t>(ji)];
    const Link *child = model.find_link(j.child_link);
    if (child == nullptr) {
      throw std::runtime_error("cartesian_lever_weights: joint '" + j.name +
                               "' has unknown child link '" + j.child_link + "'");
    }
    const auto ci = static_cast<std::size_t>(child - links.data());
    double r = link_radius[ci];
    for (const int k : child->child_joints) {
      const Joint &jk = joints[static_cast<std::size_t>(k)];
      double travel = 0.0;
      if (jk.type == JointType::Prismatic) {
        if (!jk.limits.has_position_limit) {
          throw std::runtime_error("cartesian_lever_weights: prismatic joint '" + jk.name +
                                   "' has no position limits (unbounded travel)");
        }
        travel = std::max(std::abs(jk.limits.lower), std::abs(jk.limits.upper));
      }
      r = std::max(r, jk.origin.translation().norm() + travel + self(self, k));
    }
    reach[static_cast<std::size_t>(ji)] = r;
    return r;
  };

  JointPosition w = JointPosition::Zero(static_cast<Eigen::Index>(model.dof()));
  for (std::size_t ji = 0; ji < joints.size(); ++ji) {
    const Joint &j = joints[ji];
    if (!j.is_movable() || j.dof_index < 0) {
      continue;
    }
    w[j.dof_index] =
        (j.type == JointType::Prismatic) ? 1.0 : compute_reach(compute_reach, static_cast<int>(ji));
  }
  return w;
}

int EdgeDiscretization::steps(const JointPosition &delta) const {
  if (max_link_sweep > 0.0) {
    if (lever_weights.size() != delta.size()) {
      throw std::runtime_error("EdgeDiscretization: lever_weights size " +
                               std::to_string(lever_weights.size()) + " != dof " +
                               std::to_string(delta.size()));
    }
    const double sweep = lever_weights.cwiseAbs().dot(delta.cwiseAbs());
    return std::max(1, static_cast<int>(std::ceil(sweep / max_link_sweep)));
  }
  const double max_step = delta.size() > 0 ? delta.cwiseAbs().maxCoeff() : 0.0;
  return std::max(1, static_cast<int>(std::ceil(max_step / joint_resolution)));
}

EdgeDiscretization make_edge_discretization(double joint_resolution, double max_link_sweep,
                                            const JointPosition &lever_weights,
                                            const RobotModel &model) {
  if (!(joint_resolution > 0.0)) {
    throw std::runtime_error("make_edge_discretization: joint resolution must be > 0");
  }
  EdgeDiscretization d;
  d.joint_resolution = joint_resolution;
  d.max_link_sweep = max_link_sweep;
  if (max_link_sweep > 0.0) {
    if (lever_weights.size() == 0) {
      d.lever_weights = cartesian_lever_weights(model); // primitive/absolute-URI robots only
    } else if (lever_weights.size() == static_cast<Eigen::Index>(model.dof())) {
      d.lever_weights = lever_weights;
    } else {
      throw std::runtime_error("make_edge_discretization: lever_weights size " +
                               std::to_string(lever_weights.size()) + " != dof " +
                               std::to_string(model.dof()));
    }
  }
  return d;
}

} // namespace quevedomp::collision
