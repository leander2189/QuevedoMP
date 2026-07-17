// clearance/ClearanceField — see clearance_field.hpp. CPU reference implementation of every
// stage (seed / JFA / sign / queries); the CUDA JFA lives in jfa.cu behind
// QUEVEDOMP_CLEARANCE_CUDA and is numerically equivalent (same seeds, same propagation rule).
#include "quevedomp/clearance/clearance_field.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include <Eigen/Eigenvalues>

#include "../collision/tessellate.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace quevedomp::clearance {

#ifdef QUEVEDOMP_CLEARANCE_CUDA
// Defined in jfa.cu. Returns false when no usable CUDA device (caller falls back to CPU).
bool jfa_gpu(const std::vector<Eigen::Vector3f> &seed_points, std::vector<std::int32_t> &ids,
             int nx, int ny, int nz, float res);
#endif

namespace {

using Clock = std::chrono::steady_clock;
constexpr std::int32_t kNoSeed = -1;

// Closest point on triangle (a, b, c) to p — Ericson, Real-Time Collision Detection §5.1.5.
Eigen::Vector3d closest_on_triangle(const Eigen::Vector3d &p, const Eigen::Vector3d &a,
                                    const Eigen::Vector3d &b, const Eigen::Vector3d &c) {
  const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
  const double d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return a;
  }
  const Eigen::Vector3d bp = p - b;
  const double d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return b;
  }
  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return a + (d1 / (d1 - d3)) * ab;
  }
  const Eigen::Vector3d cp = p - c;
  const double d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return c;
  }
  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return a + (d2 / (d2 - d6)) * ac;
  }
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
  }
  const double denom = 1.0 / (va + vb + vc);
  return a + ab * (vb * denom) + ac * (vc * denom);
}

// A mesh is watertight if every edge is shared by exactly two triangles (ADR-012's test).
bool is_watertight(const Mesh &m) {
  if (m.triangles.empty()) {
    return false;
  }
  std::map<std::pair<int, int>, int> edges;
  const auto bump = [&](int a, int b) { edges[a < b ? std::pair{a, b} : std::pair{b, a}]++; };
  for (const Eigen::Vector3i &t : m.triangles) {
    bump(t.x(), t.y());
    bump(t.y(), t.z());
    bump(t.z(), t.x());
  }
  for (const auto &[edge, count] : edges) {
    if (count != 2) {
      return false;
    }
  }
  return true;
}

struct WorldTris {
  std::vector<Eigen::Vector3d> v;     // 3 per triangle
  std::vector<std::uint8_t> signable; // per triangle: part of a watertight object
};

// Gather all environment geometry as world-space triangles (primitives via the P2 closed
// tessellations — the same geometry story the GPU backend uses).
WorldTris gather_triangles(const collision::SceneDescription &env) {
  WorldTris out;
  for (const collision::SceneObject &o : env.objects) {
    Mesh m;
    if (const auto *b = std::get_if<collision::BoxShape>(&o.geometry)) {
      m = collision::tessellate_box(b->half_extents);
    } else if (const auto *s = std::get_if<collision::SphereShape>(&o.geometry)) {
      m = collision::tessellate_sphere(s->radius);
    } else if (const auto *c = std::get_if<collision::CylinderShape>(&o.geometry)) {
      m = collision::tessellate_cylinder(c->radius, c->length);
    } else {
      m = std::get<Mesh>(o.geometry);
    }
    const bool signable = is_watertight(m);
    if (!signable) {
      std::cerr << "[quevedomp] clearance: environment mesh '" << o.id
                << "' is not watertight; it contributes UNSIGNED distance only (ADR-012)\n";
    }
    for (const Eigen::Vector3i &t : m.triangles) {
      out.v.push_back(o.pose * m.vertices[t.x()]);
      out.v.push_back(o.pose * m.vertices[t.y()]);
      out.v.push_back(o.pose * m.vertices[t.z()]);
      out.signable.push_back(signable ? 1 : 0);
    }
  }
  return out;
}

} // namespace

struct ClearanceField::Impl {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  double res = 0.01;
  int nx = 0, ny = 0, nz = 0;
  std::vector<float> dist; // signed, row-major x fastest
  bool on_gpu = false;
  double build_s = 0.0;

  [[nodiscard]] std::size_t idx(int x, int y, int z) const {
    return (static_cast<std::size_t>(z) * static_cast<std::size_t>(ny) +
            static_cast<std::size_t>(y)) *
               static_cast<std::size_t>(nx) +
           static_cast<std::size_t>(x);
  }

  // Trilinear sample at a world point, clamped to the border; adds the Euclidean gap when the
  // point lies outside the grid (documented approximation).
  [[nodiscard]] double sample(const Eigen::Vector3d &p) const {
    const Eigen::Vector3d local = (p - origin) / res;
    const Eigen::Vector3d clamped(std::clamp(local.x(), 0.0, static_cast<double>(nx - 1) - 1e-9),
                                  std::clamp(local.y(), 0.0, static_cast<double>(ny - 1) - 1e-9),
                                  std::clamp(local.z(), 0.0, static_cast<double>(nz - 1) - 1e-9));
    const double outside = (local - clamped).norm() * res;

    const int x0 = static_cast<int>(clamped.x());
    const int y0 = static_cast<int>(clamped.y());
    const int z0 = static_cast<int>(clamped.z());
    const double fx = clamped.x() - x0, fy = clamped.y() - y0, fz = clamped.z() - z0;
    const auto at = [&](int dx, int dy, int dz) {
      return static_cast<double>(dist[idx(std::min(x0 + dx, nx - 1), std::min(y0 + dy, ny - 1),
                                          std::min(z0 + dz, nz - 1))]);
    };
    const double c00 = at(0, 0, 0) * (1 - fx) + at(1, 0, 0) * fx;
    const double c10 = at(0, 1, 0) * (1 - fx) + at(1, 1, 0) * fx;
    const double c01 = at(0, 0, 1) * (1 - fx) + at(1, 0, 1) * fx;
    const double c11 = at(0, 1, 1) * (1 - fx) + at(1, 1, 1) * fx;
    const double c0 = c00 * (1 - fy) + c10 * fy;
    const double c1 = c01 * (1 - fy) + c11 * fy;
    return c0 * (1 - fz) + c1 * fz + outside;
  }
};

ClearanceField::ClearanceField() : impl_(std::make_unique<Impl>()) {}
ClearanceField::ClearanceField(ClearanceField &&) noexcept = default;
ClearanceField &ClearanceField::operator=(ClearanceField &&) noexcept = default;
ClearanceField::~ClearanceField() = default;

ClearanceField ClearanceField::build(const collision::SceneDescription &env,
                                     const ClearanceFieldOptions &options) {
  const auto t0 = Clock::now();
  if (!(options.resolution > 0.0)) {
    throw std::runtime_error("ClearanceField: resolution must be > 0");
  }
  const WorldTris tris = gather_triangles(env);
  const std::size_t n_tris = tris.signable.size();
  if (n_tris == 0) {
    throw std::runtime_error("ClearanceField: empty environment");
  }

  // Grid bounds: environment AABB padded by the margin.
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d hi = -lo;
  for (const Eigen::Vector3d &v : tris.v) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  lo.array() -= options.margin;
  hi.array() += options.margin;

  ClearanceField out;
  Impl &f = *out.impl_;
  f.origin = lo;
  f.res = options.resolution;
  f.nx = static_cast<int>(std::ceil((hi.x() - lo.x()) / f.res)) + 1;
  f.ny = static_cast<int>(std::ceil((hi.y() - lo.y()) / f.res)) + 1;
  f.nz = static_cast<int>(std::ceil((hi.z() - lo.z()) / f.res)) + 1;
  const std::size_t voxels = static_cast<std::size_t>(f.nx) * f.ny * f.nz;
  if (voxels > options.max_voxels) {
    const double factor = std::cbrt(static_cast<double>(voxels) / options.max_voxels);
    throw std::runtime_error("ClearanceField: grid would need " + std::to_string(voxels) +
                             " voxels (cap " + std::to_string(options.max_voxels) +
                             "); raise resolution to >= " + std::to_string(f.res * factor));
  }

  // ---- SEED: exact point-to-triangle distance for voxels near each triangle. 64-bit encode
  // (float distance bits << 32 | triangle index) makes the per-voxel min a single atomic; the
  // monotone float→uint map keeps the comparison correct for non-negative distances.
  const auto encode = [](float d, std::int32_t tri) {
    std::uint32_t bits;
    static_assert(sizeof(bits) == sizeof(d));
    std::memcpy(&bits, &d, sizeof(bits));
    return (static_cast<std::uint64_t>(bits) << 32) | static_cast<std::uint32_t>(tri);
  };
  const std::uint64_t kEmpty = ~std::uint64_t{0};
  std::unique_ptr<std::atomic<std::uint64_t>[]> best(new std::atomic<std::uint64_t>[voxels]);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(voxels); ++i) {
    best[static_cast<std::size_t>(i)].store(kEmpty, std::memory_order_relaxed);
  }

  const double inv_res = 1.0 / f.res;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 256)
#endif
  for (std::ptrdiff_t t = 0; t < static_cast<std::ptrdiff_t>(n_tris); ++t) {
    const Eigen::Vector3d &a = tris.v[3 * static_cast<std::size_t>(t)];
    const Eigen::Vector3d &b = tris.v[3 * static_cast<std::size_t>(t) + 1];
    const Eigen::Vector3d &c = tris.v[3 * static_cast<std::size_t>(t) + 2];
    Eigen::Vector3d tlo = a.cwiseMin(b).cwiseMin(c);
    Eigen::Vector3d thi = a.cwiseMax(b).cwiseMax(c);
    // Dilate by ~1.8 voxels so every voxel whose center is within a voxel of the surface seeds.
    const int x0 = std::max(0, static_cast<int>((tlo.x() - f.origin.x()) * inv_res - 1.8));
    const int y0 = std::max(0, static_cast<int>((tlo.y() - f.origin.y()) * inv_res - 1.8));
    const int z0 = std::max(0, static_cast<int>((tlo.z() - f.origin.z()) * inv_res - 1.8));
    const int x1 = std::min(f.nx - 1, static_cast<int>((thi.x() - f.origin.x()) * inv_res + 1.8));
    const int y1 = std::min(f.ny - 1, static_cast<int>((thi.y() - f.origin.y()) * inv_res + 1.8));
    const int z1 = std::min(f.nz - 1, static_cast<int>((thi.z() - f.origin.z()) * inv_res + 1.8));
    for (int z = z0; z <= z1; ++z) {
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          const Eigen::Vector3d p = f.origin + Eigen::Vector3d(x, y, z) * f.res;
          const auto d = static_cast<float>((p - closest_on_triangle(p, a, b, c)).norm());
          const std::uint64_t candidate = encode(d, static_cast<std::int32_t>(t));
          std::uint64_t current = best[f.idx(x, y, z)].load(std::memory_order_relaxed);
          while (candidate < current && !best[f.idx(x, y, z)].compare_exchange_weak(
                                            current, candidate, std::memory_order_relaxed)) {
          }
        }
      }
    }
  }

  // Compact seeds: per seeded voxel, the exact nearest surface point (recomputed from the
  // winning triangle — no races on 3-float payloads).
  std::vector<Eigen::Vector3f> seed_points;
  std::vector<std::int32_t> ids(voxels, kNoSeed);
  for (int z = 0; z < f.nz; ++z) {
    for (int y = 0; y < f.ny; ++y) {
      for (int x = 0; x < f.nx; ++x) {
        const std::uint64_t e = best[f.idx(x, y, z)].load(std::memory_order_relaxed);
        if (e == kEmpty) {
          continue;
        }
        const auto tri = static_cast<std::size_t>(e & 0xFFFFFFFFu);
        const Eigen::Vector3d p = f.origin + Eigen::Vector3d(x, y, z) * f.res;
        const Eigen::Vector3d q =
            closest_on_triangle(p, tris.v[3 * tri], tris.v[3 * tri + 1], tris.v[3 * tri + 2]);
        ids[f.idx(x, y, z)] = static_cast<std::int32_t>(seed_points.size());
        // Origin-relative: identical arithmetic on the CPU and CUDA JFA paths, better float
        // precision than world coordinates.
        seed_points.push_back((q - f.origin).cast<float>());
      }
    }
  }
  best.reset();
  if (seed_points.empty()) {
    throw std::runtime_error("ClearanceField: seeding produced no surface voxels");
  }

  // ---- JFA: propagate nearest-seed ids (GPU when possible, OpenMP otherwise).
  bool gpu_ran = false;
#ifdef QUEVEDOMP_CLEARANCE_CUDA
  if (options.use_gpu) {
    gpu_ran = jfa_gpu(seed_points, ids, f.nx, f.ny, f.nz, static_cast<float>(f.res));
  }
#endif
  if (!gpu_ran) {
    std::vector<std::int32_t> next(voxels);
    const auto seed_at = [&](std::int32_t id) { return seed_points[static_cast<std::size_t>(id)]; };
    for (int step = std::max({f.nx, f.ny, f.nz}) / 2; step >= 1; step /= 2) {
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
      for (std::ptrdiff_t zi = 0; zi < f.nz; ++zi) {
        const int z = static_cast<int>(zi);
        for (int y = 0; y < f.ny; ++y) {
          for (int x = 0; x < f.nx; ++x) {
            const Eigen::Vector3f p = Eigen::Vector3f(x, y, z) * static_cast<float>(f.res);
            std::int32_t bid = ids[f.idx(x, y, z)];
            float bd = bid == kNoSeed ? std::numeric_limits<float>::infinity()
                                      : (p - seed_at(bid)).squaredNorm();
            for (int dz = -1; dz <= 1; ++dz) {
              for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                  if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                  }
                  const int qx = x + dx * step, qy = y + dy * step, qz = z + dz * step;
                  if (qx < 0 || qy < 0 || qz < 0 || qx >= f.nx || qy >= f.ny || qz >= f.nz) {
                    continue;
                  }
                  const std::int32_t cid = ids[f.idx(qx, qy, qz)];
                  if (cid == kNoSeed) {
                    continue;
                  }
                  const float cd = (p - seed_at(cid)).squaredNorm();
                  if (cd < bd) {
                    bd = cd;
                    bid = cid;
                  }
                }
              }
            }
            next[f.idx(x, y, z)] = bid;
          }
        }
      }
      ids.swap(next);
    }
  }

  // ---- Distance grid from final ids.
  f.dist.resize(voxels);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (std::ptrdiff_t zi = 0; zi < f.nz; ++zi) {
    const int z = static_cast<int>(zi);
    for (int y = 0; y < f.ny; ++y) {
      for (int x = 0; x < f.nx; ++x) {
        const std::int32_t id = ids[f.idx(x, y, z)];
        const Eigen::Vector3f p = Eigen::Vector3f(x, y, z) * static_cast<float>(f.res);
        f.dist[f.idx(x, y, z)] = id == kNoSeed
                                     ? std::numeric_limits<float>::infinity()
                                     : (p - seed_points[static_cast<std::size_t>(id)]).norm();
      }
    }
  }

  // ---- SIGN: even-odd column parity over the watertight triangles. Crossing z-values are
  // accumulated per (x, y) column, then each column's voxels flip sign between odd crossings.
  {
    // Sub-voxel column shift: a grid column crossing EXACTLY through a shared triangle edge (the
    // tessellated box's face diagonals hit dozens of columns dead-on) is counted by BOTH
    // triangles, flipping the whole column's parity. Shifting the sample point by an irrational-
    // ish fraction of a voxel makes exact edge/vertex hits measure-zero; the induced crossing-z
    // error is slope · shift ≈ nothing.
    const double ox = 7.31e-4 * f.res, oy = 4.57e-4 * f.res;
    std::vector<std::vector<float>> crossings(static_cast<std::size_t>(f.nx) * f.ny);
    for (std::size_t t = 0; t < n_tris; ++t) {
      if (!tris.signable[t]) {
        continue;
      }
      const Eigen::Vector3d &a = tris.v[3 * t], &b = tris.v[3 * t + 1], &c = tris.v[3 * t + 2];
      const double x_lo = std::min({a.x(), b.x(), c.x()}), x_hi = std::max({a.x(), b.x(), c.x()});
      const double y_lo = std::min({a.y(), b.y(), c.y()}), y_hi = std::max({a.y(), b.y(), c.y()});
      const int cx0 =
          std::max(0, static_cast<int>(std::ceil((x_lo - f.origin.x() - ox) * inv_res)) - 1);
      const int cx1 = std::min(f.nx - 1, static_cast<int>((x_hi - f.origin.x()) * inv_res) + 1);
      const int cy0 =
          std::max(0, static_cast<int>(std::ceil((y_lo - f.origin.y() - oy) * inv_res)) - 1);
      const int cy1 = std::min(f.ny - 1, static_cast<int>((y_hi - f.origin.y()) * inv_res) + 1);
      for (int y = cy0; y <= cy1; ++y) {
        for (int x = cx0; x <= cx1; ++x) {
          const double px = f.origin.x() + x * f.res + ox, py = f.origin.y() + y * f.res + oy;
          // Ray (px, py, -inf)→+z vs triangle: 2D barycentric point-in-triangle in the xy plane.
          const double d = (b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y());
          if (std::abs(d) < 1e-18) {
            continue; // degenerate in projection
          }
          const double w0 = ((b.y() - c.y()) * (px - c.x()) + (c.x() - b.x()) * (py - c.y())) / d;
          const double w1 = ((c.y() - a.y()) * (px - c.x()) + (a.x() - c.x()) * (py - c.y())) / d;
          const double w2 = 1.0 - w0 - w1;
          if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0) {
            continue;
          }
          const double zc = w0 * a.z() + w1 * b.z() + w2 * c.z();
          crossings[static_cast<std::size_t>(y) * f.nx + x].push_back(static_cast<float>(zc));
        }
      }
    }
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
    for (std::ptrdiff_t col = 0; col < static_cast<std::ptrdiff_t>(crossings.size()); ++col) {
      auto &cs = crossings[static_cast<std::size_t>(col)];
      if (cs.empty()) {
        continue;
      }
      std::sort(cs.begin(), cs.end());
      const int x = static_cast<int>(col % f.nx), y = static_cast<int>(col / f.nx);
      std::size_t k = 0;
      for (int z = 0; z < f.nz; ++z) {
        const auto zw = static_cast<float>(f.origin.z() + z * f.res);
        while (k < cs.size() && cs[k] < zw) {
          ++k;
        }
        if (k % 2 == 1) { // odd crossings below ⇒ inside
          f.dist[f.idx(x, y, z)] = -std::abs(f.dist[f.idx(x, y, z)]);
        }
      }
    }
  }

  f.on_gpu = gpu_ran;
  f.build_s = std::chrono::duration<double>(Clock::now() - t0).count();
  return out;
}

double ClearanceField::distance(const Eigen::Vector3d &p) const { return impl_->sample(p); }

Eigen::Vector3d ClearanceField::gradient(const Eigen::Vector3d &p) const {
  const double h = impl_->res;
  Eigen::Vector3d g;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    dp[axis] = h;
    g[axis] = (impl_->sample(p + dp) - impl_->sample(p - dp)) / (2.0 * h);
  }
  return g;
}

void ClearanceField::query(std::span<const Eigen::Vector3d> points, std::span<double> distances,
                           std::span<Eigen::Vector3d> gradients) const {
  if (distances.size() != points.size() ||
      (!gradients.empty() && gradients.size() != points.size())) {
    throw std::runtime_error("ClearanceField::query: output spans must match points");
  }
#ifdef _OPENMP
#pragma omp parallel for schedule(static) if (points.size() >= 256)
#endif
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(points.size()); ++i) {
    const auto k = static_cast<std::size_t>(i);
    distances[k] = distance(points[k]);
    if (!gradients.empty()) {
      gradients[k] = gradient(points[k]);
    }
  }
}

Eigen::Vector3d ClearanceField::origin() const noexcept { return impl_->origin; }
double ClearanceField::resolution() const noexcept { return impl_->res; }
Eigen::Vector3i ClearanceField::dims() const noexcept { return {impl_->nx, impl_->ny, impl_->nz}; }
const std::vector<float> &ClearanceField::data() const noexcept { return impl_->dist; }
bool ClearanceField::built_on_gpu() const noexcept { return impl_->on_gpu; }
double ClearanceField::build_seconds() const noexcept { return impl_->build_s; }

// ---- Robot sphere cover + batched clearance --------------------------------------------------

RobotSpheres decompose_robot(const RobotModel &model, const collision::MeshSources &meshes,
                             double target_radius, int max_spheres_per_geometry) {
  RobotSpheres out;
  const auto &links = model.links();
  for (int li = 0; li < static_cast<int>(links.size()); ++li) {
    for (const CollisionGeometry &cg : links[static_cast<std::size_t>(li)].collisions) {
      Mesh m;
      Eigen::Vector3d scale = Eigen::Vector3d::Ones();
      switch (cg.type) {
      case GeometryType::Mesh:
        m = load_mesh(resolve_mesh_uri(cg.mesh_filename, meshes.package_dirs, meshes.base_dir));
        scale = cg.mesh_scale;
        break;
      case GeometryType::Box:
        m = collision::tessellate_box(cg.box_half_extents);
        break;
      case GeometryType::Sphere:
        m = collision::tessellate_sphere(cg.sphere_radius);
        break;
      case GeometryType::Cylinder:
        m = collision::tessellate_cylinder(cg.cylinder_radius, cg.cylinder_length);
        break;
      }
      std::vector<Eigen::Vector3d> pts;
      pts.reserve(m.vertices.size());
      for (const Eigen::Vector3d &v : m.vertices) {
        pts.push_back(cg.origin * v.cwiseProduct(scale));
      }
      if (pts.empty()) {
        continue;
      }
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto &p : pts) {
        mean += p;
      }
      mean /= static_cast<double>(pts.size());
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (const auto &p : pts) {
        cov += (p - mean) * (p - mean).transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
      const Eigen::Vector3d axis = eig.eigenvectors().col(2); // principal direction
      double t_lo = std::numeric_limits<double>::infinity(), t_hi = -t_lo;
      for (const auto &p : pts) {
        const double t = axis.dot(p - mean);
        t_lo = std::min(t_lo, t);
        t_hi = std::max(t_hi, t);
      }

      // Grow the center count along the principal axis until the cover radius reaches the
      // target (or the cap) — conservative: radius = max vertex distance to nearest center.
      std::vector<Eigen::Vector3d> centers;
      double radius = 0.0;
      for (int n = 1; n <= max_spheres_per_geometry; ++n) {
        centers.clear();
        for (int k = 0; k < n; ++k) {
          const double t = t_lo + (t_hi - t_lo) * (n == 1 ? 0.5 : (k + 0.5) / n);
          centers.push_back(mean + axis * t);
        }
        radius = 0.0;
        for (const auto &p : pts) {
          double best = std::numeric_limits<double>::infinity();
          for (const auto &c : centers) {
            best = std::min(best, (p - c).norm());
          }
          radius = std::max(radius, best);
        }
        if (radius <= target_radius) {
          break;
        }
      }
      for (const auto &c : centers) {
        out.spheres.push_back({li, c, radius});
      }
    }
  }
  return out;
}

std::vector<double> clearance_batch(const ClearanceField &field, const RobotModel &model,
                                    const RobotSpheres &spheres,
                                    std::span<const JointPosition> configs) {
  std::vector<double> out(configs.size(), std::numeric_limits<double>::infinity());
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 8) if (configs.size() >= 8)
#endif
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(configs.size()); ++i) {
    const auto k = static_cast<std::size_t>(i);
    const std::vector<Transform> poses = fk_all(model, configs[k]);
    double c = std::numeric_limits<double>::infinity();
    for (const RobotSpheres::Sphere &s : spheres.spheres) {
      const Eigen::Vector3d center = poses[static_cast<std::size_t>(s.link)] * s.center;
      c = std::min(c, field.distance(center) - s.radius);
    }
    out[k] = c;
  }
  return out;
}

} // namespace quevedomp::clearance
