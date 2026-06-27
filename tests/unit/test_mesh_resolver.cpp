// Task 1.4b — resolve_mesh_uri: package://, file://, absolute, and relative URI handling.
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <unordered_map>

#include "quevedomp/robot/mesh_resolver.hpp"

using quevedomp::resolve_mesh_uri;

namespace {
std::unordered_map<std::string, std::string> pkgs() {
  return {{"ur_description", "/data/ur"}, {"abb", "/data/abb"}};
}
} // namespace

TEST(MeshResolver, ResolvesPackageUri) {
  EXPECT_EQ(resolve_mesh_uri("package://ur_description/meshes/base.stl", pkgs()),
            "/data/ur/meshes/base.stl");
  EXPECT_EQ(resolve_mesh_uri("package://abb/collision/link_1.stl", pkgs()),
            "/data/abb/collision/link_1.stl");
}

TEST(MeshResolver, UnknownPackageThrows) {
  EXPECT_THROW(resolve_mesh_uri("package://nope/x.stl", pkgs()), std::runtime_error);
}

TEST(MeshResolver, FileSchemeStripped) {
  EXPECT_EQ(resolve_mesh_uri("file:///abs/path/m.dae", pkgs()), "/abs/path/m.dae");
}

TEST(MeshResolver, AbsolutePathUnchanged) {
  EXPECT_EQ(resolve_mesh_uri("/already/abs.obj", pkgs()), "/already/abs.obj");
}

TEST(MeshResolver, RelativePathAnchoredToBaseDir) {
  EXPECT_EQ(resolve_mesh_uri("meshes/link_0.stl", pkgs(), "/data/iiwa"),
            "/data/iiwa/meshes/link_0.stl");
  // No base dir -> returned as-is.
  EXPECT_EQ(resolve_mesh_uri("meshes/link_0.stl", pkgs()), "meshes/link_0.stl");
}
