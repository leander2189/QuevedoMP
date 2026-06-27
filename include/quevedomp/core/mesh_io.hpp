// core/mesh_io — load triangle meshes from disk into core Mesh (Task 1.4b).
//
// Backed by assimp (STL/DAE/OBJ and more). Kept out of core/types.hpp so the dependency-free
// value types stay assimp-free; this header exposes only the core Mesh type.
#pragma once

#include <string>

#include "quevedomp/core/types.hpp"

namespace quevedomp {

// Load a triangle mesh from `path` (STL/DAE/OBJ/…) into a core Mesh, normalized to METRES.
// COLLADA unit metadata is honored (the classic DAE millimetre-vs-metre bug), node transforms
// are baked, and degenerate triangles are dropped. All submeshes are merged into one Mesh.
// Throws std::runtime_error if the file cannot be read or yields no triangles.
Mesh load_mesh(const std::string &path);

} // namespace quevedomp
