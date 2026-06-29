// collision/types — value types crossing the collision boundary (spec §4.2).
// CPU vocabulary only: NO CUDA type ever appears in any collision/ header, so the FCL and OptiX
// backends are interchangeable and the no-GPU build is honest (spec §4.1).
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace quevedomp::collision {

// Per link/object-pair collision padding override, in metres (ADR-013). Order-independent key.
class PaddingMap {
public:
  void set(const std::string &a, const std::string &b, float padding) {
    values_[key(a, b)] = padding;
  }
  // Padding for pair (a,b), or `fallback` if no override is present.
  float get(const std::string &a, const std::string &b, float fallback = 0.0f) const {
    const auto it = values_.find(key(a, b));
    return it == values_.end() ? fallback : it->second;
  }
  bool empty() const noexcept { return values_.empty(); }

private:
  static std::pair<std::string, std::string> key(const std::string &a, const std::string &b) {
    return (a <= b) ? std::pair{a, b} : std::pair{b, a};
  }
  std::map<std::pair<std::string, std::string>, float> values_;
};

struct QueryOptions {
  bool distance = false;            // also compute signed min-distance + witness
  float safety_margin = 0.0f;       // operational threshold: collision if dist < margin
  float robot_padding = 0.0f;       // physical inflation of robot collision geometry
  float max_distance = 0.10f;       // distances clamp beyond this (perf)
  bool check_self_collision = true; // robot-vs-robot, honoring the ACM
  std::optional<PaddingMap> per_pair_padding;
};

struct CollisionPair { // witness, for debug/visualization
  std::string a, b;    // link or object ids
  Eigen::Vector3d point_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d point_b = Eigen::Vector3d::Zero();
};

struct CollisionResult {
  bool in_collision = false;
  float min_distance = 0.0f; // signed; valid only if distance was requested
  std::optional<CollisionPair> witness;
};

struct BatchResult {
  std::vector<std::uint8_t> in_collision; // one per config (NOT vector<bool>)
  std::vector<float> min_distance;        // empty unless distance requested
  std::vector<CollisionPair> witnesses;   // empty unless requested (expensive)
};

} // namespace quevedomp::collision
