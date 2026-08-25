// Task 1.4b verify — load_mesh: formats (OBJ/STL/DAE), unit normalization to metres,
// degenerate-triangle removal, and error handling on bad input.
#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/core/mesh_io.hpp"

using quevedomp::load_mesh;
using quevedomp::Mesh;

namespace {

std::string fixture(const std::string &rel) {
  return std::string(QUEVEDOMP_FIXTURE_DIR) + "/meshes/" + rel;
}

struct Aabb {
  Eigen::Vector3d lo;
  Eigen::Vector3d hi;
};

Aabb aabb(const Mesh &m) {
  Aabb b{Eigen::Vector3d::Constant(std::numeric_limits<double>::max()),
         Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest())};
  for (const auto &v : m.vertices) {
    b.lo = b.lo.cwiseMin(v);
    b.hi = b.hi.cwiseMax(v);
  }
  return b;
}

} // namespace

TEST(MeshIo, LoadsObjCube) {
  const Mesh m = load_mesh(fixture("cube.obj"));
  EXPECT_EQ(m.triangles.size(), 12u);
  EXPECT_GT(m.vertices.size(), 0u);
  const Aabb b = aabb(m);
  EXPECT_LT((b.lo - Eigen::Vector3d(0, 0, 0)).norm(), 1e-9);
  EXPECT_LT((b.hi - Eigen::Vector3d(1, 1, 1)).norm(), 1e-9);
  // All triangle indices must be in range.
  for (const auto &t : m.triangles) {
    for (int k = 0; k < 3; ++k) {
      EXPECT_GE(t[k], 0);
      EXPECT_LT(t[k], static_cast<int>(m.vertices.size()));
    }
  }
}

TEST(MeshIo, LoadsStlTriangle) {
  const Mesh m = load_mesh(fixture("tri.stl"));
  EXPECT_EQ(m.triangles.size(), 1u);
  const Aabb b = aabb(m);
  EXPECT_NEAR(b.hi.x(), 1.0, 1e-6);
  EXPECT_NEAR(b.hi.y(), 1.0, 1e-6);
  EXPECT_NEAR(b.hi.z(), 0.0, 1e-6);
}

TEST(MeshIo, DropsDegenerateTriangles) {
  const Mesh m = load_mesh(fixture("degenerate.obj"));
  EXPECT_EQ(m.triangles.size(), 1u); // the repeated-index face is removed
}

// Degeneracy must be a property of the GEOMETRY, not of the unit it happens to be written in.
// assimp's own area test compares against an absolute 1e-6 square units, so before this was
// disabled the millimetre copy below kept every triangle while the identical metre copy lost the
// ones under 1 mm^2 — on a real 4.4M-triangle part that was 53% of the mesh, silently.
TEST(MeshIo, DegeneracyFilterIsScaleInvariant) {
  // Half the triangles are comfortably large (1 cm edges, area 5e-5 m^2) and half are merely SMALL
  // (1 mm edges, area 5e-7 m^2) — small, not thin: an absolute area epsilon kills a triangle for
  // its size regardless of aspect ratio. At metre scale the small half sits below assimp's 1e-6
  // cutoff and the large half above it; multiplying every coordinate by 1000 lifts both above it.
  // So an absolute test keeps 64 triangles in millimetres but only 32 in metres, while a
  // scale-invariant one keeps all 64 either way. Edges stay >= 1 mm against a ~0.7 m bounding box
  // so that assimp's (relative) vertex welding never merges a triangle out of existence.
  constexpr int kLarge = 32, kSmall = 32, kTriangles = kLarge + kSmall;
  const auto write_stl = [](const std::string &path, float scale) {
    std::ofstream f(path, std::ios::binary);
    const std::vector<char> header(80, 0);
    f.write(header.data(), 80);
    const std::uint32_t n = kTriangles;
    f.write(reinterpret_cast<const char *>(&n), 4);
    const auto emit = [&f, scale](double x0, double edge) {
      const float v[12] = {0, 0, 1, // normal (unused by the loader)
                           static_cast<float>(scale * x0),          0.0f, 0.0f,
                           static_cast<float>(scale * (x0 + edge)), 0.0f, 0.0f,
                           static_cast<float>(scale * x0),          static_cast<float>(scale * edge),
                           0.0f};
      f.write(reinterpret_cast<const char *>(v), sizeof(v));
      const std::uint16_t attr = 0;
      f.write(reinterpret_cast<const char *>(&attr), 2);
    };
    double x = 0.0;
    for (int i = 0; i < kLarge; ++i, x += 2e-2)
      emit(x, 1e-2); // area 5e-5 m^2 — above an absolute 1e-6 cutoff
    for (int i = 0; i < kSmall; ++i, x += 2e-3)
      emit(x, 1e-3); // area 5e-7 m^2 — below it
  };

  const std::string metres = (std::filesystem::temp_directory_path() / "qmp_scale_m.stl").string();
  const std::string millis = (std::filesystem::temp_directory_path() / "qmp_scale_mm.stl").string();
  write_stl(metres, 1.0f);
  write_stl(millis, 1000.0f);

  const Mesh m = load_mesh(metres);
  const Mesh mm = load_mesh(millis);
  std::filesystem::remove(metres);
  std::filesystem::remove(millis);

  EXPECT_EQ(m.triangles.size(), mm.triangles.size())
      << "the same geometry kept a different triangle count at metre vs millimetre scale — the "
         "degeneracy filter is unit-dependent again";
  EXPECT_EQ(m.triangles.size(), static_cast<std::size_t>(kTriangles))
      << "no triangle here has zero area, so none should have been dropped";
}

TEST(MeshIo, HonorsColladaMillimetreUnit) {
  // The triangle is authored at 1000 mm; with <unit meter="0.001"> it must come back at 1 m.
  const Mesh m = load_mesh(fixture("mm_triangle.dae"));
  ASSERT_EQ(m.triangles.size(), 1u);
  const Aabb b = aabb(m);
  const double extent = (b.hi - b.lo).maxCoeff();
  EXPECT_NEAR(extent, 1.0, 1e-3) << "COLLADA mm→m unit conversion not applied (got " << extent
                                 << ")";
}

TEST(MeshIo, MissingFileThrows) {
  EXPECT_THROW(load_mesh(fixture("does_not_exist.stl")), std::runtime_error);
}

TEST(MeshIo, EmptyMeshThrows) { EXPECT_THROW(load_mesh(fixture("empty.obj")), std::runtime_error); }
