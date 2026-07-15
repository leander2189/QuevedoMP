// parameterization/parametrize — time-optimal path parameterization with joint velocity /
// acceleration / jerk limits and Cartesian tip velocity / acceleration limits (Task 3.4,
// docs/topp_jerk_tip_spec.md; ADR-011 revision).
//
// Two-phase design per the spec:
//   Phase A (this header's ConvexOnly mode): the jerk-free problem — joint vel/acc, tip
//   vel/acc — is solved to GLOBAL optimality by a TOPP-RA-style backward/forward pass over the
//   squared-velocity profile β(s). Every constraint is linear in (β, u = s̈): the backward pass
//   computes the controllable interval K_k per node with tiny exact 2-variable LPs, the forward
//   pass rides the pointwise-maximal profile (the classic TOPP-RA optimality result). No
//   external solver, microsecond-fast, deterministic.
//   Phase B (JerkLimited mode, Stage 2; ADR-017 as amended): per-joint jerk rows are non-convex
//   (√β·(...)). Instead of an NLP, a VELOCITY-REDUCTION KERNEL dips the Phase A profile where
//   the exactly-evaluated node jerk exceeds its limit (smooth min-filtered envelopes, widened
//   until the ramps respect the acceleration rows), with a certified terminal fallback: uniform
//   β scaling reduces node jerk by EXACTLY α³ while shrinking every other constraint. Suboptimal
//   by construction (it slows the Phase A shape rather than reshaping it) but microsecond-fast,
//   deterministic, and always certified to jerk_tolerance. Jerk needs a C³ path — feed a
//   PathSpline (see path_spline.hpp), not a polyline.
//
// Tip (TCP) limits: velocity uses the Euclidean norm (‖J_t·q'‖²·β ≤ v²  — one more β bound in
// the MVC); acceleration is PER-AXIS (box) form — ẍ = (J·q')·u + (J·q'' + J'·q')·β is linear in
// (β, u), so each axis is one more acceleration-type row. A single scalar limit is applied per
// axis divided by √3 (conservative: the box inscribes the sphere). Time-optimal profiles ride
// the binding constraint, so a tip-speed cap yields near-constant tool speed along the stroke
// (the paint-pass use case).
#pragma once

#include <string>
#include <vector>

#include "quevedomp/core/types.hpp"
#include "quevedomp/parameterization/path_spline.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::parameterization {

// Limits gathered for one parameterization call. Joint vectors must be size dof and positive;
// zeros/empty disable that constraint family. Tip limits are 0 = off.
struct Limits {
  JointPosition max_velocity;     // rad/s (m/s prismatic); required (URDF <limit velocity>)
  JointPosition max_acceleration; // rad/s²; required for a meaningful profile
  JointPosition max_jerk;         // rad/s³; empty or 0 ⇒ no jerk rows (Phase A behavior)

  double tip_linear_velocity = 0.0;      // m/s, Euclidean norm of the TCP linear velocity
  double tip_angular_velocity = 0.0;     // rad/s, Euclidean norm of the TCP angular velocity
  double tip_linear_acceleration = 0.0;  // m/s², applied per axis as limit/√3 (box ⊂ sphere)
  double tip_angular_acceleration = 0.0; // rad/s², same convention
  std::string tip_link;                  // TCP link; empty ⇒ the model's last link
};

// Build Limits from the model's URDF limits (+ yaml accel/jerk extension) and the problem's
// TaskLimits. Joints without an acceleration limit get `default_acceleration` (rad/s²) — URDF
// has no <acceleration>, so a fallback keeps the parameterizer usable on plain URDFs.
[[nodiscard]] Limits limits_from_model(const RobotModel &model,
                                       const planning::TaskLimits &task = {},
                                       double default_acceleration = 10.0);

struct ParameterizationOptions {
  int nodes = 200;   // grid intervals N (N+1 nodes); more near-curvature ⇒ raise
  double eps = 1e-9; // √β floor for the time integral at interior near-rest nodes
  // JerkLimited mode (Task 3.4 Stage 2, ADR-017 as amended): pass budget for the velocity-
  // reduction kernel and the accepted relative jerk violation (max |q⃛|/j_max − 1 ≤
  // jerk_tolerance at the interior nodes ⇒ certified). The kernel always terminates certified:
  // its limit case is a uniform β scaling, which reduces node jerk by EXACTLY α³.
  int max_jerk_passes = 12;
  double jerk_tolerance = 1e-2;
  enum class Mode { ConvexOnly, JerkLimited } mode = Mode::ConvexOnly;
};

struct ParameterizationResult {
  bool success = false;
  std::string message;
  Trajectory trajectory; // per node: q, q̇, q̈ (JointState) + time; empty on failure
  double duration = 0.0; // trajectory.back().time

  // Diagnostics: the grid and the squared-velocity profile (β_k = ṡ²), for tests/plots.
  std::vector<double> s;
  std::vector<double> beta;
  int jerk_passes = 0;           // JerkLimited: kernel passes actually run
  double max_jerk_violation = 0; // JerkLimited: worst |q⃛|/j_max − 1 at the returned profile
};

// Parameterize `path` under `limits`. Rest-to-rest (β_0 = β_N = 0). ConvexOnly ignores
// max_jerk; JerkLimited applies the kernel when any max_jerk entry is > 0 (and reduces to
// ConvexOnly otherwise). Deterministic; no collision checks (the path must already be
// validated — see fit_collision_free).
[[nodiscard]] ParameterizationResult parametrize(const RobotModel &model, const PathSpline &path,
                                                 const Limits &limits,
                                                 const ParameterizationOptions &options = {});

} // namespace quevedomp::parameterization
