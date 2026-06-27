#include "quevedomp/core/mesh_io.hpp"

#include <stdexcept>

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

  if (out.triangles.empty()) {
    throw std::runtime_error("load_mesh: '" + path + "' contains no triangles");
  }
  return out;
}

} // namespace quevedomp
