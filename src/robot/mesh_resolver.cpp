#include "quevedomp/robot/mesh_resolver.hpp"

#include <stdexcept>

namespace quevedomp {

std::string resolve_mesh_uri(const std::string &uri,
                             const std::unordered_map<std::string, std::string> &package_dirs,
                             const std::string &base_dir) {
  static const std::string kPackage = "package://";
  static const std::string kFile = "file://";

  if (uri.rfind(kPackage, 0) == 0) {
    const std::string rest = uri.substr(kPackage.size());
    const std::string::size_type slash = rest.find('/');
    const std::string pkg = rest.substr(0, slash);
    const std::string tail = (slash == std::string::npos) ? std::string() : rest.substr(slash + 1);
    const auto it = package_dirs.find(pkg);
    if (it == package_dirs.end()) {
      throw std::runtime_error("resolve_mesh_uri: no directory mapped for package '" + pkg +
                               "' (uri: " + uri + ")");
    }
    return it->second + "/" + tail;
  }

  if (uri.rfind(kFile, 0) == 0)
    return uri.substr(kFile.size());
  if (!uri.empty() && uri.front() == '/')
    return uri; // already absolute
  if (base_dir.empty())
    return uri; // relative, nothing to anchor to
  return base_dir + "/" + uri;
}

} // namespace quevedomp
