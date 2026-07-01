// collision/containment — see containment.hpp.
#include "containment.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <utility>
#include <variant>

namespace quevedomp::collision {
namespace {

// Möller–Trumbore ray/triangle intersection; reports a forward hit (t > eps).
bool ray_triangle(const Eigen::Vector3d &o, const Eigen::Vector3d &d, const Eigen::Vector3d &a,
                  const Eigen::Vector3d &b, const Eigen::Vector3d &c) {
  constexpr double kEps = 1e-12;
  const Eigen::Vector3d e1 = b - a, e2 = c - a;
  const Eigen::Vector3d pv = d.cross(e2);
  const double det = e1.dot(pv);
  if (std::abs(det) < kEps)
    return false; // ray parallel to triangle
  const double inv = 1.0 / det;
  const Eigen::Vector3d tv = o - a;
  const double u = tv.dot(pv) * inv;
  if (u < 0.0 || u > 1.0)
    return false;
  const Eigen::Vector3d qv = tv.cross(e1);
  const double v = d.dot(qv) * inv;
  if (v < 0.0 || u + v > 1.0)
    return false;
  const double t = e2.dot(qv) * inv;
  return t > 1e-9;
}

// A mesh is watertight if every edge is shared by exactly two triangles.
bool is_watertight(const Mesh &m) {
  if (m.triangles.empty())
    return false;
  std::map<std::pair<int, int>, int> edges;
  auto bump = [&](int a, int b) { edges[a < b ? std::pair{a, b} : std::pair{b, a}]++; };
  for (const Eigen::Vector3i &t : m.triangles) {
    bump(t.x(), t.y());
    bump(t.y(), t.z());
    bump(t.z(), t.x());
  }
  for (const auto &[edge, count] : edges)
    if (count != 2)
      return false;
  return true;
}

} // namespace

EnvContainment::EnvContainment(const SceneDescription &env) {
  int mesh_id = 0;
  for (const SceneObject &o : env.objects) {
    if (const auto *b = std::get_if<BoxShape>(&o.geometry)) {
      boxes_.push_back({o.pose.inverse(), b->half_extents});
      has_solids_ = true;
    } else if (const auto *s = std::get_if<SphereShape>(&o.geometry)) {
      spheres_.push_back({o.pose.translation(), s->radius});
      has_solids_ = true;
    } else if (const auto *c = std::get_if<CylinderShape>(&o.geometry)) {
      cyls_.push_back({o.pose.inverse(), c->radius, 0.5 * c->length});
      has_solids_ = true;
    } else if (const auto *m = std::get_if<Mesh>(&o.geometry)) {
      const int id = mesh_id++;
      if (!is_watertight(*m)) {
        std::cerr << "[quevedomp] containment: environment mesh '"
                  << (o.id.empty() ? std::to_string(id) : o.id)
                  << "' is not watertight; containment inside it is undetectable (ADR-012)\n";
        continue;
      }
      for (const Eigen::Vector3i &t : m->triangles)
        mesh_tris_.push_back({o.pose * m->vertices[t.x()], o.pose * m->vertices[t.y()],
                              o.pose * m->vertices[t.z()]});
      has_solids_ = true;
    }
  }
}

bool EnvContainment::inside(const Eigen::Vector3d &p) const {
  for (const BoxSolid &b : boxes_) {
    const Eigen::Vector3d q = b.inv_pose * p;
    if (std::abs(q.x()) <= b.he.x() && std::abs(q.y()) <= b.he.y() && std::abs(q.z()) <= b.he.z())
      return true;
  }
  for (const SphereSolid &s : spheres_)
    if ((p - s.c).norm() <= s.r)
      return true;
  for (const CylSolid &c : cyls_) {
    const Eigen::Vector3d q = c.inv_pose * p;
    if (std::hypot(q.x(), q.y()) <= c.r && std::abs(q.z()) <= c.half_len)
      return true;
  }
  if (!mesh_tris_.empty()) {
    // Parity ray in a generic direction (avoid axis alignment with typical meshes).
    const Eigen::Vector3d d = Eigen::Vector3d(0.3, 0.6, 0.74).normalized();
    int hits = 0;
    for (const auto &tri : mesh_tris_)
      hits += ray_triangle(p, d, tri[0], tri[1], tri[2]) ? 1 : 0;
    if (hits % 2 == 1)
      return true;
  }
  return false;
}

Eigen::Vector3d mesh_centroid(const std::vector<Eigen::Vector3d> &verts) {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d &v : verts)
    sum += v;
  return verts.empty() ? sum : (sum / static_cast<double>(verts.size())).eval();
}

} // namespace quevedomp::collision
