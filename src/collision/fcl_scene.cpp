// collision/fcl_scene — the FCL CPU backend for CollisionScene (Task 2a.2, spec §4.2/§4.3).
//
// Scope of THIS task: boolean robot-vs-environment and robot-vs-self collision. Signed distance
// + witnesses (§4.3) land in Task 2a.3, edge checking in 2a.4 — both behind the same interface.
//
// Design (matches the contract headers and ADR-015 scene-internal FK):
//   * Environment geometry is built once into a broad-phase manager owned by the scene and read
//     concurrently (const) by every query.
//   * Robot collision geometry is immutable (shared across threads); only its per-config world
//     transforms move. Those posed CollisionObjects + their broad-phase manager live in a
//     per-thread Workspace, so concurrent const queries are lock-free.
//   * Per config: FK the robot (scene-internal FK), push transforms onto the workspace objects,
//     test robot-vs-env, then robot-vs-self honoring the ACM.
//
// Mesh geometry (Task 2a.2b): environment meshes come fully-populated in SceneDescription; robot
// mesh collision links carry only an unresolved URDF URI (package://…), so make_static_scene takes
// a MeshSources {package→dir} map and the scene resolves (resolve_mesh_uri) + loads (load_mesh) +
// scales them at build time, caching by URI+scale so a shared mesh loads once. A robot mesh that
// cannot be resolved/loaded throws — it is never silently skipped. All geometry kinds (primitive
// and mesh, robot and environment) then funnel through the identical FCL collide path.
#include "quevedomp/collision/collision_scene.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <variant>

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"

namespace quevedomp::collision {
namespace {

using S = double;
using FclGeom = fcl::CollisionGeometry<S>;
using FclObject = fcl::CollisionObject<S>;
using FclManager = fcl::DynamicAABBTreeCollisionManager<S>;

fcl::Transform3<S> to_fcl(const Transform &t) {
  fcl::Transform3<S> f;
  f.matrix() = t.isometry().matrix();
  return f;
}

std::shared_ptr<FclGeom> make_bvh(const Mesh &mesh) {
  auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSS<S>>>();
  model->beginModel(static_cast<int>(mesh.triangles.size()),
                    static_cast<int>(mesh.vertices.size()));
  for (const Eigen::Vector3i &tri : mesh.triangles) {
    model->addTriangle(mesh.vertices.at(tri[0]), mesh.vertices.at(tri[1]),
                       mesh.vertices.at(tri[2]));
  }
  model->endModel();
  return model;
}

// Environment shape (collision::Geometry variant) -> FCL geometry.
std::shared_ptr<FclGeom> make_env_geom(const Geometry &g) {
  return std::visit(
      [](const auto &shape) -> std::shared_ptr<FclGeom> {
        using T = std::decay_t<decltype(shape)>;
        if constexpr (std::is_same_v<T, BoxShape>) {
          return std::make_shared<fcl::Box<S>>(2.0 * shape.half_extents.x(),
                                               2.0 * shape.half_extents.y(),
                                               2.0 * shape.half_extents.z());
        } else if constexpr (std::is_same_v<T, SphereShape>) {
          return std::make_shared<fcl::Sphere<S>>(shape.radius);
        } else if constexpr (std::is_same_v<T, CylinderShape>) {
          return std::make_shared<fcl::Cylinder<S>>(shape.radius, shape.length);
        } else { // Mesh
          return make_bvh(shape);
        }
      },
      g);
}

// Robot-link URDF collision shape -> FCL geometry. Returns null for an (unsupported here) mesh.
std::shared_ptr<FclGeom> make_link_geom(const CollisionGeometry &cg) {
  switch (cg.type) {
  case GeometryType::Box:
    return std::make_shared<fcl::Box<S>>(2.0 * cg.box_half_extents.x(),
                                         2.0 * cg.box_half_extents.y(),
                                         2.0 * cg.box_half_extents.z());
  case GeometryType::Sphere:
    return std::make_shared<fcl::Sphere<S>>(cg.sphere_radius);
  case GeometryType::Cylinder:
    return std::make_shared<fcl::Cylinder<S>>(cg.cylinder_radius, cg.cylinder_length);
  case GeometryType::Mesh:
    return nullptr; // resolved+loaded by FclScene (needs MeshSources); see robot_mesh_geom
  }
  return nullptr;
}

// Scale a loaded mesh's vertices in place by the URDF <mesh scale>. Non-uniform scale cannot be
// baked into the rigid object transform, so it is applied to the geometry here.
void apply_scale(Mesh &m, const Eigen::Vector3d &scale) {
  if (scale == Eigen::Vector3d::Ones())
    return;
  for (Eigen::Vector3d &v : m.vertices)
    v = v.cwiseProduct(scale);
}

// One posed robot collision shape: which link drives it, its shared geometry, and the fixed
// link-frame -> geometry-frame offset (CollisionGeometry::origin).
struct LinkShape {
  int link_index = -1;
  std::shared_ptr<FclGeom> geom;
  Transform origin;
};

// Self-collision broad-phase callback context. Robot CollisionObjects carry their link index as
// user data; we skip same-link geometry pairs and ACM-allowed pairs, then narrow-phase the rest.
struct SelfData {
  const RobotModel *model = nullptr;
  const AllowedCollisionMatrix *acm = nullptr;
  bool collision = false;
};

bool self_callback(FclObject *o1, FclObject *o2, void *cdata) {
  auto *d = static_cast<SelfData *>(cdata);
  if (d->collision)
    return true;
  const auto li = static_cast<int>(reinterpret_cast<std::intptr_t>(o1->getUserData()));
  const auto lj = static_cast<int>(reinterpret_cast<std::intptr_t>(o2->getUserData()));
  if (li == lj)
    return false; // two shapes on the same link never collide with each other
  const std::string &na = d->model->links()[li].name;
  const std::string &nb = d->model->links()[lj].name;
  if (d->acm->is_allowed(na, nb))
    return false;
  fcl::CollisionRequest<S> req;
  req.num_max_contacts = 1;
  fcl::CollisionResult<S> res;
  fcl::collide(o1, o2, req, res);
  if (res.isCollision()) {
    d->collision = true;
    return true;
  }
  return false;
}

class FclWorkspace; // fwd

class FclScene final : public CollisionScene {
public:
  FclScene(std::shared_ptr<const RobotModel> model, MeshSources sources)
      : model_(std::move(model)), sources_(std::move(sources)) {
    // Build the immutable per-link collision shape templates once. Primitives are built inline;
    // mesh links are resolved+loaded here (throws on failure — never silently skipped).
    const auto &links = model_->links();
    for (int li = 0; li < static_cast<int>(links.size()); ++li) {
      for (const CollisionGeometry &cg : links[li].collisions) {
        std::shared_ptr<FclGeom> geom =
            cg.type == GeometryType::Mesh ? robot_mesh_geom(cg) : make_link_geom(cg);
        if (geom)
          link_shapes_.push_back({li, std::move(geom), cg.origin});
      }
    }
  }

  SceneHandle add_object(std::string /*id*/, const Geometry &geom, const Transform &pose) override {
    const SceneHandle handle = next_handle_++;
    auto obj = std::make_unique<FclObject>(make_env_geom(geom), to_fcl(pose));
    obj->computeAABB();
    env_manager_.registerObject(obj.get());
    env_objects_.emplace(handle, std::move(obj));
    env_manager_.update();
    return handle;
  }

  void remove_object(SceneHandle handle) override {
    const auto it = env_objects_.find(handle);
    if (it == env_objects_.end())
      return;
    env_manager_.unregisterObject(it->second.get());
    env_objects_.erase(it);
    env_manager_.update();
  }

  void move_object(SceneHandle handle, const Transform &pose) override {
    const auto it = env_objects_.find(handle);
    if (it == env_objects_.end())
      return;
    it->second->setTransform(to_fcl(pose));
    it->second->computeAABB();
    env_manager_.update();
  }

  [[nodiscard]] std::unique_ptr<Workspace> make_workspace() const override;

  [[nodiscard]] BatchResult query_batch(const RobotInstance &robot,
                                        std::span<const JointPosition> qs, const QueryOptions &opts,
                                        Workspace &ws) const override;

private:
  friend class FclWorkspace;

  // Resolve + load + scale a robot mesh collision link into FCL geometry, caching by URI+scale so
  // a mesh shared across links loads once. Throws (via resolve_mesh_uri/load_mesh) on failure.
  std::shared_ptr<FclGeom> robot_mesh_geom(const CollisionGeometry &cg) {
    const std::string path =
        resolve_mesh_uri(cg.mesh_filename, sources_.package_dirs, sources_.base_dir);
    const std::string key = path + '|' + std::to_string(cg.mesh_scale.x()) + ',' +
                            std::to_string(cg.mesh_scale.y()) + ',' +
                            std::to_string(cg.mesh_scale.z());
    if (const auto it = mesh_cache_.find(key); it != mesh_cache_.end())
      return it->second;
    Mesh m = load_mesh(path);
    apply_scale(m, cg.mesh_scale);
    auto geom = make_bvh(m);
    mesh_cache_.emplace(key, geom);
    return geom;
  }

  std::shared_ptr<const RobotModel> model_;
  MeshSources sources_;
  std::unordered_map<std::string, std::shared_ptr<FclGeom>> mesh_cache_;
  std::vector<LinkShape> link_shapes_;
  std::unordered_map<SceneHandle, std::unique_ptr<FclObject>> env_objects_;
  mutable FclManager env_manager_; // broad-phase queries are logically const
  SceneHandle next_handle_ = 0;
};

// Per-thread scratch: one posed CollisionObject per robot LinkShape (sharing the scene's
// immutable geometry) plus their own broad-phase manager.
class FclWorkspace final : public Workspace {
public:
  explicit FclWorkspace(const FclScene &scene) {
    objects_.reserve(scene.link_shapes_.size());
    for (const LinkShape &ls : scene.link_shapes_) {
      auto obj = std::make_unique<FclObject>(ls.geom);
      obj->setUserData(reinterpret_cast<void *>(static_cast<std::intptr_t>(ls.link_index)));
      manager_.registerObject(obj.get());
      objects_.push_back(std::move(obj));
    }
    manager_.setup();
  }

  std::vector<std::unique_ptr<FclObject>> objects_;
  FclManager manager_;
};

std::unique_ptr<Workspace> FclScene::make_workspace() const {
  return std::make_unique<FclWorkspace>(*this);
}

BatchResult FclScene::query_batch(const RobotInstance &robot, std::span<const JointPosition> qs,
                                  const QueryOptions &opts, Workspace &ws) const {
  auto &fws = dynamic_cast<FclWorkspace &>(ws);
  const RobotModel &model = robot.model();

  BatchResult out;
  out.in_collision.assign(qs.size(), 0);

  for (std::size_t i = 0; i < qs.size(); ++i) {
    const std::vector<Transform> link_poses = fk_all(model, qs[i]);

    for (std::size_t s = 0; s < link_shapes_.size(); ++s) {
      const LinkShape &ls = link_shapes_[s];
      fws.objects_[s]->setTransform(to_fcl(link_poses[ls.link_index] * ls.origin));
      fws.objects_[s]->computeAABB();
    }
    fws.manager_.update();

    bool hit = false;
    if (!env_objects_.empty()) {
      fcl::DefaultCollisionData<S> data;
      fws.manager_.collide(&env_manager_, &data, fcl::DefaultCollisionFunction<S>);
      hit = data.result.isCollision();
    }
    if (!hit && opts.check_self_collision) {
      SelfData sd{&model, &robot.acm(), false};
      fws.manager_.collide(&sd, self_callback);
      hit = sd.collision;
    }
    out.in_collision[i] = hit ? 1 : 0;
  }
  return out;
}

} // namespace

std::unique_ptr<CollisionScene> make_static_scene(std::shared_ptr<const RobotModel> robot,
                                                  const SceneDescription &environment,
                                                  BackendHint hint, const MeshSources &meshes) {
  if (hint == BackendHint::ForceOptix)
    throw std::runtime_error(
        "make_static_scene: OptiX backend not built (lands in Phase 2b); use Auto or ForceCpuFcl");

  auto scene = std::make_unique<FclScene>(std::move(robot), meshes);
  for (const SceneObject &o : environment.objects)
    scene->add_object(o.id, o.geometry, o.pose);
  return scene;
}

} // namespace quevedomp::collision
