#include "quevedomp/core/mesh_io.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Geometry>

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace quevedomp {

Mesh load_mesh(const std::string &path) {
  Assimp::Importer importer;
  // Actually delete degenerate faces (default only flags them) and drop point/line primitives so
  // only real triangles survive.
  importer.SetPropertyInteger(AI_CONFIG_PP_FD_REMOVE, 1);
  importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                              aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  // ...but NOT via assimp's area test, which compares against an ABSOLUTE 1e-6 square units and is
  // therefore unit-dependent: the same part authored in metres loses every triangle under 1 mm^2,
  // while in millimetres it loses essentially nothing. Measured on a 4,380,564-triangle fixture,
  // that silently discarded 2,301,272 triangles (53%) purely because the file was in metres.
  // Turning the area test off leaves assimp's scale-invariant check — a face whose vertex
  // POSITIONS coincide — and we handle true slivers ourselves below.
  importer.SetPropertyInteger(AI_CONFIG_PP_FD_CHECKAREA, 0);

  const unsigned int flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                             aiProcess_PreTransformVertices | // bake node hierarchy transforms
                             aiProcess_GlobalScale |          // honor file units → metres
                             aiProcess_FindDegenerates | aiProcess_FindInvalidData |
                             aiProcess_SortByPType;

  const aiScene *scene = importer.ReadFile(path, flags);
  if (scene == nullptr || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) != 0 ||
      scene->mNumMeshes == 0) {
    throw std::runtime_error("load_mesh: failed to load '" + path +
                             "': " + importer.GetErrorString());
  }

  Mesh out;
  for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi) {
    const aiMesh *m = scene->mMeshes[mi];
    if ((m->mPrimitiveTypes & aiPrimitiveType_TRIANGLE) == 0)
      continue; // skip non-triangle meshes

    const int base = static_cast<int>(out.vertices.size());
    out.vertices.reserve(out.vertices.size() + m->mNumVertices);
    for (unsigned int v = 0; v < m->mNumVertices; ++v) {
      const aiVector3D &p = m->mVertices[v];
      out.vertices.emplace_back(p.x, p.y, p.z);
    }
    out.triangles.reserve(out.triangles.size() + m->mNumFaces);
    for (unsigned int f = 0; f < m->mNumFaces; ++f) {
      const aiFace &face = m->mFaces[f];
      if (face.mNumIndices != 3)
        continue; // post-triangulation should guarantee 3; be defensive
      out.triangles.emplace_back(base + static_cast<int>(face.mIndices[0]),
                                 base + static_cast<int>(face.mIndices[1]),
                                 base + static_cast<int>(face.mIndices[2]));
    }
  }

  // Exactly-zero-area faces survive assimp's position check when their three vertices are distinct
  // but collinear. They carry no surface, and their cross product is the zero vector, so any
  // consumer that normalizes a face normal gets a NaN. Drop them — and ONLY them. A merely small
  // triangle is real surface: dropping it would punch a hole in an otherwise watertight mesh and
  // silently cost us ADR-012 containment, which is exactly the failure mode the absolute area test
  // above produced. Zero-area faces cover nothing, so removing them cannot open a hole.
  const auto area2 = [&out](const Eigen::Vector3i &t) {
    const Eigen::Vector3d &a = out.vertices[static_cast<std::size_t>(t[0])];
    const Eigen::Vector3d &b = out.vertices[static_cast<std::size_t>(t[1])];
    const Eigen::Vector3d &c = out.vertices[static_cast<std::size_t>(t[2])];
    return (b - a).cross(c - a).squaredNorm();
  };
  out.triangles.erase(
      std::remove_if(out.triangles.begin(), out.triangles.end(),
                     [&area2](const Eigen::Vector3i &t) { return !(area2(t) > 0.0); }),
      out.triangles.end());

  if (out.triangles.empty()) {
    throw std::runtime_error("load_mesh: '" + path + "' contains no triangles");
  }
  return out;
}

} // namespace quevedomp
