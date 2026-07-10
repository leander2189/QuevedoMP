// planning/Planner — the make_planner registry (Task 3.2). Concrete algorithms are registered
// under a string id at build time; make_planner() dispatches on params.algorithm and throws a
// clear error for an unknown id (no silent fallback — performance contract).
#include "quevedomp/planning/planner.hpp"

#include <functional>
#include <stdexcept>
#include <unordered_map>

#include "planners_internal.hpp"

namespace quevedomp::planning {
namespace {

using Factory = std::function<std::unique_ptr<Planner>(
    const PlannerParams &, std::shared_ptr<const RobotInstance>,
    std::shared_ptr<const collision::CollisionScene>)>;

// The registry. A function-local static (constructed on first use) avoids static-init-order
// issues; adding a planner is one line here plus its factory in planners_internal.hpp.
const std::unordered_map<std::string, Factory> &registry() {
  static const std::unordered_map<std::string, Factory> r = {
      {"rrt_connect", &detail::make_rrt_connect},
  };
  return r;
}

} // namespace

std::unique_ptr<Planner> make_planner(const PlannerParams &params,
                                      std::shared_ptr<const RobotInstance> robot,
                                      std::shared_ptr<const collision::CollisionScene> scene) {
  if (robot == nullptr) {
    throw std::runtime_error("make_planner: robot instance is null");
  }
  if (scene == nullptr) {
    throw std::runtime_error("make_planner: collision scene is null");
  }
  const auto &r = registry();
  const auto it = r.find(params.algorithm);
  if (it == r.end()) {
    throw std::runtime_error("make_planner: no planner registered under id '" + params.algorithm +
                             "'");
  }
  return it->second(params, std::move(robot), std::move(scene));
}

std::vector<std::string> registered_planners() {
  std::vector<std::string> ids;
  for (const auto &[id, _] : registry()) {
    ids.push_back(id);
  }
  return ids;
}

} // namespace quevedomp::planning
