#include "tessellate.hpp"

#include <cmath>

namespace quevedomp::collision {

Mesh tessellate_box(const Eigen::Vector3d &half_extents) {
  Mesh m;
  for (int i = 0; i < 8; ++i)
    m.vertices.emplace_back((i & 1 ? 1 : -1) * half_extents.x(),
                            (i & 2 ? 1 : -1) * half_extents.y(),
                            (i & 4 ? 1 : -1) * half_extents.z());
  // Outward winding; same closed cube the FCL tests validate against.
  const int faces[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                            {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  for (const auto &f : faces)
    m.triangles.emplace_back(f[0], f[1], f[2]);
  return m;
}

Mesh tessellate_sphere(double radius, int n_lat, int n_lon) {
  Mesh m;
  m.vertices.emplace_back(0.0, 0.0, radius); // top pole
  for (int i = 1; i < n_lat; ++i) {
    const double t = M_PI * i / n_lat;
    for (int j = 0; j < n_lon; ++j) {
      const double p = 2.0 * M_PI * j / n_lon;
      m.vertices.emplace_back(radius * std::sin(t) * std::cos(p),
                              radius * std::sin(t) * std::sin(p), radius * std::cos(t));
    }
  }
  m.vertices.emplace_back(0.0, 0.0, -radius);                 // bottom pole
  const auto ring = [n_lon](int i) { return 1 + i * n_lon; }; // first index of ring i
  const int bottom = static_cast<int>(m.vertices.size()) - 1;

  for (int j = 0; j < n_lon; ++j) // top cap
    m.triangles.emplace_back(0, ring(0) + j, ring(0) + (j + 1) % n_lon);
  for (int i = 0; i + 1 < n_lat - 1; ++i) // quads between rings
    for (int j = 0; j < n_lon; ++j) {
      const int a = ring(i) + j, b = ring(i) + (j + 1) % n_lon;
      const int c = ring(i + 1) + j, d = ring(i + 1) + (j + 1) % n_lon;
      m.triangles.emplace_back(a, c, d);
      m.triangles.emplace_back(a, d, b);
    }
  for (int j = 0; j < n_lon; ++j) // bottom cap
    m.triangles.emplace_back(bottom, ring(n_lat - 2) + (j + 1) % n_lon, ring(n_lat - 2) + j);
  return m;
}

Mesh tessellate_cylinder(double radius, double length, int n) {
  Mesh m;
  const double half = length / 2.0;
  for (int j = 0; j < n; ++j) { // top ring
    const double a = 2.0 * M_PI * j / n;
    m.vertices.emplace_back(radius * std::cos(a), radius * std::sin(a), half);
  }
  for (int j = 0; j < n; ++j) { // bottom ring
    const double a = 2.0 * M_PI * j / n;
    m.vertices.emplace_back(radius * std::cos(a), radius * std::sin(a), -half);
  }
  m.vertices.emplace_back(0.0, 0.0, half); // cap centers
  m.vertices.emplace_back(0.0, 0.0, -half);
  const int c_top = 2 * n, c_bot = 2 * n + 1;

  for (int j = 0; j < n; ++j) {
    const int k = (j + 1) % n;
    m.triangles.emplace_back(j, n + j, n + k); // side quad
    m.triangles.emplace_back(j, n + k, k);
    m.triangles.emplace_back(c_top, j, k);         // top cap
    m.triangles.emplace_back(c_bot, n + k, n + j); // bottom cap
  }
  return m;
}

} // namespace quevedomp::collision
