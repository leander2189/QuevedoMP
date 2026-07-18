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
//
// "chomp" (the R4 optimization refiner) is listed so it is discoverable via
// registered_planners() and selecting it fails LOUDLY, never silently — but it cannot be built
// through this (params, robot, scene) entry point: it needs a ClearanceField + robot sphere cover.
// The stub throws a directive error pointing at make_refiner() (see refiner.hpp / ADR-019).
const std::unordered_map<std::string, Factory> &registry() {
  static const std::unordered_map<std::string, Factory> r = {
      {"rrt_connect", &detail::make_rrt_connect},
      {"chomp",
       [](const PlannerParams &, std::shared_ptr<const RobotInstance>,
          std::shared_ptr<const collision::CollisionScene>) -> std::unique_ptr<Planner> {
         throw std::runtime_error(
             "make_planner: the 'chomp' refiner needs a ClearanceField and a robot sphere cover "
             "that this (params, robot, scene) entry point cannot carry; build it with "
             "make_refiner(RefinerParams, robot, scene, field, spheres) instead (see "
             "refiner.hpp).");
       }},
      {"prm",
       [](const PlannerParams &, std::shared_ptr<const RobotInstance>,
          std::shared_ptr<const collision::CollisionScene>) -> std::unique_ptr<Planner> {
         throw std::runtime_error(
             "make_planner: the 'prm' roadmap planner carries construction config (num_nodes, "
             "connectivity) that this flat-PlannerParams entry point cannot; build it with "
             "make_prm_planner(PrmParams, robot, scene) instead (see roadmap.hpp).");
       }},
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
