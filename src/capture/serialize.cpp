// capture/serialize — see include/quevedomp/capture/serialize.hpp.
#include "quevedomp/capture/serialize.hpp"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <variant>

namespace quevedomp::capture {
namespace {

constexpr std::uint32_t kVersion = 1;

// ---- little binary reader/writer over std::string (same-endianness) ----------------------------
struct Writer {
  std::string buf;

  void raw(const void *p, std::size_t n) { buf.append(static_cast<const char *>(p), n); }
  void magic(const char m[4]) { raw(m, 4); }
  void u8(std::uint8_t v) { raw(&v, 1); }
  void u32(std::uint32_t v) { raw(&v, 4); }
  void u64(std::uint64_t v) { raw(&v, 8); }
  void i32(std::int32_t v) { raw(&v, 4); }
  void f64(double v) { raw(&v, 8); }
  void str(const std::string &s) {
    u64(s.size());
    raw(s.data(), s.size());
  }
  void vec3d(const Eigen::Vector3d &v) {
    f64(v.x());
    f64(v.y());
    f64(v.z());
  }
  void vec3i(const Eigen::Vector3i &v) {
    i32(v.x());
    i32(v.y());
    i32(v.z());
  }
  void tf(const Transform &t) {
    const Eigen::Matrix4d m = t.matrix();
    for (int i = 0; i < 16; ++i)
      f64(m.data()[i]);
  }
};

struct Reader {
  const char *p;
  const char *end;
  explicit Reader(const std::string &s) : p(s.data()), end(s.data() + s.size()) {}

  void need(std::size_t n) const {
    if (static_cast<std::size_t>(end - p) < n)
      throw std::runtime_error("capture::deserialize: truncated blob");
  }
  void raw(void *out, std::size_t n) {
    need(n);
    std::memcpy(out, p, n);
    p += n;
  }
  void magic(const char m[4]) {
    char g[4];
    raw(g, 4);
    if (std::memcmp(g, m, 4) != 0)
      throw std::runtime_error("capture::deserialize: bad magic");
  }
  std::uint8_t u8() {
    std::uint8_t v;
    raw(&v, 1);
    return v;
  }
  std::uint32_t u32() {
    std::uint32_t v;
    raw(&v, 4);
    return v;
  }
  std::uint64_t u64() {
    std::uint64_t v;
    raw(&v, 8);
    return v;
  }
  std::int32_t i32() {
    std::int32_t v;
    raw(&v, 4);
    return v;
  }
  double f64() {
    double v;
    raw(&v, 8);
    return v;
  }
  std::string str() {
    const std::uint64_t n = u64();
    need(n);
    std::string s(p, n);
    p += n;
    return s;
  }
  Eigen::Vector3d vec3d() {
    Eigen::Vector3d v;
    v.x() = f64();
    v.y() = f64();
    v.z() = f64();
    return v;
  }
  Eigen::Vector3i vec3i() {
    Eigen::Vector3i v;
    v.x() = i32();
    v.y() = i32();
    v.z() = i32();
    return v;
  }
  Transform tf() {
    Eigen::Matrix4d m;
    for (int i = 0; i < 16; ++i)
      m.data()[i] = f64();
    Eigen::Isometry3d iso;
    iso.matrix() = m;
    return Transform(iso);
  }
};

void expect_version(Reader &r) {
  if (r.u32() != kVersion)
    throw std::runtime_error("capture::deserialize: unsupported format version");
}

// Geometry variant tags (order is load-bearing on disk; append only).
enum GeomTag : std::uint8_t { kBox = 0, kSphere = 1, kCylinder = 2, kMesh = 3 };

} // namespace

std::string serialize_robot_model(const RobotModel &model) {
  Writer w;
  w.magic("QMRM");
  w.u32(kVersion);
  w.str(model.source_urdf());
  w.u8(model.source_yaml().has_value() ? 1 : 0);
  if (model.source_yaml())
    w.str(*model.source_yaml());
  return std::move(w.buf);
}

std::shared_ptr<const RobotModel> deserialize_robot_model(const std::string &blob) {
  Reader r(blob);
  r.magic("QMRM");
  expect_version(r);
  const std::string urdf = r.str();
  std::optional<std::string> yaml;
  if (r.u8() != 0)
    yaml = r.str();
  return RobotModel::from_urdf(urdf, yaml);
}

std::string serialize_robot_instance(const RobotInstance &robot) {
  Writer w;
  w.magic("QMRI");
  w.u32(kVersion);
  w.str(serialize_robot_model(robot.model()));
  const auto &pairs = robot.acm().pairs();
  w.u64(pairs.size());
  for (const auto &[a, b] : pairs) {
    w.str(a);
    w.str(b);
  }
  return std::move(w.buf);
}

RobotInstance deserialize_robot_instance(const std::string &blob) {
  Reader r(blob);
  r.magic("QMRI");
  expect_version(r);
  auto model = deserialize_robot_model(r.str());
  RobotInstance robot(std::move(model));
  const std::uint64_t n = r.u64();
  for (std::uint64_t i = 0; i < n; ++i) {
    const std::string a = r.str();
    const std::string b = r.str();
    robot.acm().allow(a, b);
  }
  return robot;
}

std::string serialize_scene(const collision::SceneDescription &scene) {
  using namespace collision;
  Writer w;
  w.magic("QMSC");
  w.u32(kVersion);
  w.u64(scene.objects.size());
  for (const SceneObject &o : scene.objects) {
    w.str(o.id);
    w.tf(o.pose);
    std::visit(
        [&](const auto &shape) {
          using T = std::decay_t<decltype(shape)>;
          if constexpr (std::is_same_v<T, BoxShape>) {
            w.u8(kBox);
            w.vec3d(shape.half_extents);
          } else if constexpr (std::is_same_v<T, SphereShape>) {
            w.u8(kSphere);
            w.f64(shape.radius);
          } else if constexpr (std::is_same_v<T, CylinderShape>) {
            w.u8(kCylinder);
            w.f64(shape.radius);
            w.f64(shape.length);
          } else { // Mesh
            w.u8(kMesh);
            w.u64(shape.vertices.size());
            for (const Eigen::Vector3d &v : shape.vertices)
              w.vec3d(v);
            w.u64(shape.triangles.size());
            for (const Eigen::Vector3i &t : shape.triangles)
              w.vec3i(t);
          }
        },
        o.geometry);
  }
  return std::move(w.buf);
}

collision::SceneDescription deserialize_scene(const std::string &blob) {
  using namespace collision;
  Reader r(blob);
  r.magic("QMSC");
  expect_version(r);
  SceneDescription scene;
  const std::uint64_t n = r.u64();
  scene.objects.reserve(n);
  for (std::uint64_t i = 0; i < n; ++i) {
    SceneObject o;
    o.id = r.str();
    o.pose = r.tf();
    switch (r.u8()) {
    case kBox:
      o.geometry = BoxShape{r.vec3d()};
      break;
    case kSphere:
      o.geometry = SphereShape{r.f64()};
      break;
    case kCylinder: {
      CylinderShape c;
      c.radius = r.f64();
      c.length = r.f64();
      o.geometry = c;
      break;
    }
    case kMesh: {
      Mesh m;
      const std::uint64_t nv = r.u64();
      m.vertices.reserve(nv);
      for (std::uint64_t k = 0; k < nv; ++k)
        m.vertices.push_back(r.vec3d());
      const std::uint64_t nt = r.u64();
      m.triangles.reserve(nt);
      for (std::uint64_t k = 0; k < nt; ++k)
        m.triangles.push_back(r.vec3i());
      o.geometry = std::move(m);
      break;
    }
    default:
      throw std::runtime_error("capture::deserialize_scene: unknown geometry tag");
    }
    scene.objects.push_back(std::move(o));
  }
  return scene;
}

} // namespace quevedomp::capture
