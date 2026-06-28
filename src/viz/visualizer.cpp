#include "quevedomp/viz/visualizer.hpp"

#include <vector>

#ifdef QUEVEDOMP_WITH_RERUN
#include <Eigen/Geometry>
#include <rerun.hpp>

#include "quevedomp/kinematics/fk.hpp"
#endif

namespace quevedomp {

#ifdef QUEVEDOMP_WITH_RERUN

namespace {

rerun::Transform3D to_rr(const Transform &tf) {
  const Eigen::Vector3d t = tf.translation();
  const Eigen::Quaterniond q(tf.rotation());
  return rerun::Transform3D(
      rerun::Vec3D{static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z())},
      rerun::Quaternion::from_xyzw(static_cast<float>(q.x()), static_cast<float>(q.y()),
                                   static_cast<float>(q.z()), static_cast<float>(q.w())));
}

rerun::Vec3D vec(const Eigen::Vector3d &v) {
  return rerun::Vec3D{static_cast<float>(v.x()), static_cast<float>(v.y()),
                      static_cast<float>(v.z())};
}

} // namespace

struct Visualizer::Impl {
  rerun::RecordingStream rec;
  explicit Impl(const std::string &app_id) : rec(app_id) {}
};

Visualizer::Visualizer(const std::string &app_id) : impl_(std::make_unique<Impl>(app_id)) {}

bool Visualizer::enabled() const noexcept { return true; }

void Visualizer::save(const std::string &path) { impl_->rec.save(path).handle(); }
void Visualizer::spawn() { impl_->rec.spawn().handle(); }
void Visualizer::set_frame(int64_t index) { impl_->rec.set_time_sequence("frame", index); }

void Visualizer::log_pose(const std::string &entity, const Transform &tf) {
  impl_->rec.log(entity, to_rr(tf));
}

void Visualizer::log_mesh(const std::string &entity, const Mesh &mesh, const Transform &tf) {
  impl_->rec.log(entity, to_rr(tf));
  std::vector<rerun::Position3D> positions;
  positions.reserve(mesh.vertices.size());
  for (const auto &v : mesh.vertices) {
    positions.emplace_back(static_cast<float>(v.x()), static_cast<float>(v.y()),
                           static_cast<float>(v.z()));
  }
  std::vector<rerun::TriangleIndices> tris;
  tris.reserve(mesh.triangles.size());
  for (const auto &t : mesh.triangles) {
    tris.emplace_back(static_cast<uint32_t>(t.x()), static_cast<uint32_t>(t.y()),
                      static_cast<uint32_t>(t.z()));
  }

  // Per-vertex normals (area-weighted sum of incident face normals). rerun's mesh renderer
  // needs normals to shade; without them surfaces render unlit/black and look invisible.
  std::vector<Eigen::Vector3d> accum(mesh.vertices.size(), Eigen::Vector3d::Zero());
  for (const auto &t : mesh.triangles) {
    const Eigen::Vector3d &a = mesh.vertices[static_cast<std::size_t>(t.x())];
    const Eigen::Vector3d &b = mesh.vertices[static_cast<std::size_t>(t.y())];
    const Eigen::Vector3d &c = mesh.vertices[static_cast<std::size_t>(t.z())];
    const Eigen::Vector3d face = (b - a).cross(c - a); // length ∝ triangle area
    accum[static_cast<std::size_t>(t.x())] += face;
    accum[static_cast<std::size_t>(t.y())] += face;
    accum[static_cast<std::size_t>(t.z())] += face;
  }
  std::vector<rerun::Vector3D> normals;
  normals.reserve(accum.size());
  for (const auto &n : accum) {
    const Eigen::Vector3d u = n.squaredNorm() > 0.0 ? n.normalized() : Eigen::Vector3d::UnitZ();
    normals.emplace_back(static_cast<float>(u.x()), static_cast<float>(u.y()),
                         static_cast<float>(u.z()));
  }

  impl_->rec.log(entity,
                 rerun::Mesh3D(positions).with_triangle_indices(tris).with_vertex_normals(normals));
}

void Visualizer::log_robot(const std::string &entity, const RobotModel &model,
                           const JointPosition &q) {
  const std::vector<Transform> tf = fk_all(model, q);
  const auto &links = model.links();

  // Per-link coordinate frames.
  for (std::size_t i = 0; i < links.size(); ++i) {
    impl_->rec.log(entity + "/" + links[i].name, to_rr(tf[i]));
  }
  // Skeleton: a segment from each link's origin to each child link's origin.
  std::vector<rerun::Vec3D> a;
  std::vector<rerun::Vec3D> b;
  for (std::size_t i = 0; i < links.size(); ++i) {
    for (const int jc : links[i].child_joints) {
      const Link *child = model.find_link(model.joints()[jc].child_link);
      if (child == nullptr)
        continue;
      const std::size_t ci = static_cast<std::size_t>(child - links.data());
      a.push_back(vec(tf[i].translation()));
      b.push_back(vec(tf[ci].translation()));
    }
  }
  std::vector<rerun::components::LineStrip3D> strips;
  strips.reserve(a.size());
  for (std::size_t k = 0; k < a.size(); ++k) {
    strips.emplace_back(std::vector<rerun::Vec3D>{a[k], b[k]});
  }
  impl_->rec.log(entity + "/skeleton", rerun::LineStrips3D(strips));
}

void Visualizer::log_trajectory(const std::string &entity, const RobotModel &model,
                                const Trajectory &traj, const std::string &tip) {
  std::vector<rerun::Vec3D> path;
  path.reserve(traj.size());
  for (const Waypoint &wp : traj) {
    path.push_back(vec(fk(model, wp.state.pos, tip).translation()));
  }
  impl_->rec.log(entity, rerun::LineStrips3D(rerun::components::LineStrip3D(path)));
}

#else // ---- WITH_RERUN=OFF : every method is a no-op ----------------------------------------

struct Visualizer::Impl {};

Visualizer::Visualizer(const std::string &) : impl_(std::make_unique<Impl>()) {}
bool Visualizer::enabled() const noexcept { return false; }
void Visualizer::save(const std::string &) {}
void Visualizer::spawn() {}
void Visualizer::set_frame(int64_t) {}
void Visualizer::log_pose(const std::string &, const Transform &) {}
void Visualizer::log_mesh(const std::string &, const Mesh &, const Transform &) {}
void Visualizer::log_robot(const std::string &, const RobotModel &, const JointPosition &) {}
void Visualizer::log_trajectory(const std::string &, const RobotModel &, const Trajectory &,
                                const std::string &) {}

#endif

// Out-of-line special members (Impl is complete here in both builds).
Visualizer::~Visualizer() = default;
Visualizer::Visualizer(Visualizer &&) noexcept = default;
Visualizer &Visualizer::operator=(Visualizer &&) noexcept = default;

} // namespace quevedomp
