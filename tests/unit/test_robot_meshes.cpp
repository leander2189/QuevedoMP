// Task 1.4b verify (integration) — every collision mesh of the 5 real robots resolves
// (resolve_mesh_uri) and loads (load_mesh) with a plausible, metre-scale bounding box.
#include <gtest/gtest.h>

#include <cctype>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_model.hpp"

using quevedomp::GeometryType;
using quevedomp::load_mesh;
using quevedomp::resolve_mesh_uri;
using quevedomp::RobotModel;

namespace {

std::string fixtures() { return std::string(QUEVEDOMP_FIXTURE_DIR); }
std::string mesh_root() { return fixtures() + "/robots/meshes"; }

std::string read_text(const std::string &path) {
  std::ifstream f(path);
  EXPECT_TRUE(f.good()) << "missing: " << path;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Shared package map — package names are unique across the five robots, so one map serves all.
std::unordered_map<std::string, std::string> packages() {
  return {{"example-robot-data", mesh_root() + "/example-robot-data"},
          {"collision", mesh_root() + "/abb_irb2400/collision"}};
}

struct RobotMeshCase {
  const char *urdf;
  const char *base_subdir;     // mesh_root()/<base_subdir> for relative mesh paths; "" if none
  std::size_t expected_meshes; // number of collision mesh geometries
  const char *label;
};

} // namespace

class RobotMeshes : public ::testing::TestWithParam<RobotMeshCase> {};

TEST_P(RobotMeshes, AllCollisionMeshesResolveAndLoadInMetres) {
  const RobotMeshCase &tc = GetParam();
  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/" + tc.urdf));
  const std::string base_dir =
      std::string(tc.base_subdir).empty() ? std::string() : mesh_root() + "/" + tc.base_subdir;

  std::size_t mesh_count = 0;
  for (const auto &link : model->links()) {
    for (const auto &col : link.collisions) {
      if (col.type != GeometryType::Mesh)
        continue;
      ++mesh_count;

      const std::string path = resolve_mesh_uri(col.mesh_filename, packages(), base_dir);
      const quevedomp::Mesh m = load_mesh(path); // throws on failure

      EXPECT_GT(m.vertices.size(), 0u) << tc.label << " : " << col.mesh_filename;
      EXPECT_GT(m.triangles.size(), 0u) << tc.label << " : " << col.mesh_filename;

      // Bounding-box extent must be metre-scale: strictly positive and well under a few metres
      // (a missed mm→m conversion would blow this up into the hundreds/thousands).
      Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
      Eigen::Vector3d hi = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
      for (const auto &v : m.vertices) {
        lo = lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
      }
      const double extent = (hi - lo).maxCoeff();
      EXPECT_GT(extent, 1e-4) << tc.label << " : " << col.mesh_filename;
      EXPECT_LT(extent, 5.0) << tc.label << " : " << col.mesh_filename << " (extent=" << extent
                             << " — mm not converted to m?)";
    }
  }
  EXPECT_EQ(mesh_count, tc.expected_meshes) << tc.label;
}

INSTANTIATE_TEST_SUITE_P(FiveRobots, RobotMeshes,
                         ::testing::Values(RobotMeshCase{"ur5.urdf", "", 7, "UR5"},
                                           RobotMeshCase{"ur10.urdf", "", 7, "UR10"},
                                           RobotMeshCase{"panda.urdf", "", 9, "Franka Panda"},
                                           RobotMeshCase{"iiwa.urdf", "kuka_iiwa", 8, "KUKA iiwa"},
                                           RobotMeshCase{"irb2400.urdf", "", 7, "ABB IRB2400"}),
                         [](const ::testing::TestParamInfo<RobotMeshCase> &info) {
                           std::string s = info.param.urdf;
                           for (char &c : s) {
                             if (!std::isalnum(static_cast<unsigned char>(c)))
                               c = '_';
                           }
                           return s;
                         });
