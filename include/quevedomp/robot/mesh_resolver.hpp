// robot/mesh_resolver — turn a URDF mesh URI into a filesystem path (Task 1.4b).
//
// URDF references geometry by URI; the most common is the ROS `package://` scheme, which needs
// a {package name → directory} map to resolve (there is no global package index here — callers
// provide one). This is the bridge between RobotModel's stored mesh filenames and load_mesh().
#pragma once

#include <string>
#include <unordered_map>

namespace quevedomp {

// Resolve `uri` to a filesystem path:
//   "package://<pkg>/<rest>" -> package_dirs.at(<pkg>) + "/" + <rest>
//   "file://<path>"          -> <path>
//   absolute path ("/...")   -> returned unchanged
//   relative path            -> base_dir + "/" + uri   (or unchanged if base_dir is empty)
// Throws std::runtime_error if a `package://` package has no entry in `package_dirs`.
std::string resolve_mesh_uri(const std::string &uri,
                             const std::unordered_map<std::string, std::string> &package_dirs,
                             const std::string &base_dir = "");

} // namespace quevedomp
