// parameterization/PathSpline — C⁴ geometric path over planner waypoints (Task 3.4 Stage 0).
//
// The time parametrizer needs q(s), q'(s), q''(s), q'''(s): jerk limits are meaningless on a
// polyline (q''' is a train of Dirac spikes at the waypoints — no time profile can bound them;
// see the Task 3.4 spec §7.2). PathSpline interpolates the waypoints with a degree-5 B-spline
// (C⁴ continuous, one derivative beyond what jerk needs), chord-length parameterized on s∈[0,1].
//
// The spline deviates from the collision-validated polyline, so it MUST be re-validated:
// fit_collision_free() fits, samples the curve at the edge-discretization fidelity (P3 policy),
// checks the whole sample set as ONE query_batch, and on collision densifies the interpolation
// points along the offending polyline segments and refits — pulling the spline toward the
// (free) polyline until it clears or the round budget is spent. No silent fallback: failure is
// reported, and the caller decides (e.g. parameterize the polyline without jerk rows).
#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace quevedomp::parameterization {

class PathSpline {
public:
  // Interpolating degree-5 B-spline through `waypoints` (each size dof; ≥ 2 required; consecutive
  // duplicates dropped). Waypoint lists shorter than 6 are densified along the polyline first so
  // degree 5 (C⁴) is always available. Throws std::runtime_error on bad input.
  static PathSpline fit(const planning::Path &waypoints);

  [[nodiscard]] JointPosition eval(double s) const; // q(s), s clamped to [0, 1]
  [[nodiscard]] JointPosition d1(double s) const;   // dq/ds
  [[nodiscard]] JointPosition d2(double s) const;   // d²q/ds²
  [[nodiscard]] JointPosition d3(double s) const;   // d³q/ds³

  [[nodiscard]] std::size_t dof() const noexcept;
  // Parameter values of the interpolated waypoints (chord-length, s∈[0,1]) — diagnostics.
  [[nodiscard]] const std::vector<double> &waypoint_params() const noexcept;

  PathSpline(PathSpline &&) noexcept;
  PathSpline &operator=(PathSpline &&) noexcept;
  ~PathSpline();

private:
  PathSpline();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct SplineFitResult {
  bool success = false;
  std::string message;
  std::optional<PathSpline> spline; // engaged only when success
  int rounds = 0;                   // fit rounds used (1 = first fit was already free)
  std::size_t checked_samples = 0;
};

// Fit + collision re-validate (see file comment). The sample density follows `disc` exactly like
// planner edges: consecutive samples are refined until they are within one discretization step,
// then checked as one batch. `max_rounds` densify-and-refit rounds before reporting failure.
[[nodiscard]] SplineFitResult fit_collision_free(const planning::Path &waypoints,
                                                 const collision::CollisionScene &scene,
                                                 const RobotInstance &robot,
                                                 const collision::EdgeDiscretization &disc,
                                                 const collision::QueryOptions &opts,
                                                 collision::Workspace &ws, int max_rounds = 4);

} // namespace quevedomp::parameterization
