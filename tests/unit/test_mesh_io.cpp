// Task 1.4b verify — load_mesh: formats (OBJ/STL/DAE), unit normalization to metres,
// degenerate-triangle removal, and error handling on bad input.
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>

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
