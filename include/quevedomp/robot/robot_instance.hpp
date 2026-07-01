// robot/RobotInstance — a mutable instance of a (shared, immutable) RobotModel: it owns the
// Allowed Collision Matrix and (later) attached objects. Spec §6: shared_ptr<const RobotModel>
// is the immutable description; RobotInstance is the per-owner mutable state a CollisionScene
// queries against.
#pragma once

#include <memory>
#include <set>
#include <string>
#include <utility>

#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp {

// Pairs of link/object ids whose mutual collisions are IGNORED (adjacent links, permanent
// touches, etc.). Order-independent. Header-only — it's just a normalized set.
class AllowedCollisionMatrix {
public:
  void allow(const std::string &a, const std::string &b) { allowed_.insert(key(a, b)); }
  void disallow(const std::string &a, const std::string &b) { allowed_.erase(key(a, b)); }
  bool is_allowed(const std::string &a, const std::string &b) const {
    return allowed_.count(key(a, b)) != 0;
  }
  std::size_t size() const noexcept { return allowed_.size(); }

  // The normalized allowed pairs (a <= b), for serialization + comparison.
  const std::set<std::pair<std::string, std::string>> &pairs() const noexcept { return allowed_; }

private:
  static std::pair<std::string, std::string> key(const std::string &a, const std::string &b) {
    return (a <= b) ? std::pair{a, b} : std::pair{b, a};
  }
  std::set<std::pair<std::string, std::string>> allowed_;
};

// A queryable robot: the shared immutable model plus this owner's mutable ACM. Attached objects
// (tools, grasped parts) are a future addition behind this type.
class RobotInstance {
public:
  explicit RobotInstance(std::shared_ptr<const RobotModel> model) : model_(std::move(model)) {}

  const RobotModel &model() const noexcept { return *model_; }
  const std::shared_ptr<const RobotModel> &model_ptr() const noexcept { return model_; }

  AllowedCollisionMatrix &acm() noexcept { return acm_; }
  const AllowedCollisionMatrix &acm() const noexcept { return acm_; }

private:
  std::shared_ptr<const RobotModel> model_;
  AllowedCollisionMatrix acm_;
};

} // namespace quevedomp
