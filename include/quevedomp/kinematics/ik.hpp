// kinematics/ik — inverse kinematics interface + numerical (DLS) solver (Task 1.7, spec §6).
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp {

struct IkOptions {
  double pos_tol = 1e-4;           // success: position error below this (metres)
  double rot_tol = 1e-3;           // success: rotation error below this (radians)
  int max_iters = 200;             // damped-least-squares iterations per attempt
  int max_restarts = 50;           // random re-seeds if an attempt stalls
  int stall_iters = 20;            // re-seed after this many no-progress iterations
  double stall_eps = 1e-7;         // minimum error decrease that counts as progress
  double damping = 1e-2;           // damped-least-squares λ
  double max_step = 0.5;           // clamp ‖Δq‖ per iteration (rad/m) for stability
  std::uint64_t seed = 0xA11CEULL; // RNG seed for restart configurations (deterministic)
  // solve_all: two solutions closer than this (max per-joint |Δq|, rad/m) are the SAME branch —
  // the duplicate is dropped.
  double branch_tol = 1e-2;
};

struct IkResult {
  bool success = false;
  JointPosition q;        // solution (size model.dof()); best-effort config if !success
  int iterations = 0;     // total DLS iterations across attempts
  int restarts = 0;       // attempts beyond the first
  double pos_error = 0.0; // final position error (m)
  double rot_error = 0.0; // final rotation error (rad)
};

class InverseKinematics {
public:
  virtual ~InverseKinematics() = default;

  // Solve for a configuration placing `link` at `target` (base frame). If `seed` has size
  // model.dof() it is the first attempt's initial guess; otherwise the first attempt starts
  // from a random config. Further attempts re-seed randomly (multi-seed restart).
  virtual IkResult solve(const std::string &link, const Transform &target,
                         const JointPosition &seed = JointPosition()) const = 0;

  // Collect up to `max_solutions` DISTINCT solutions (IK branches) for `link` at `target`,
  // running the same multi-seed restarts as solve() but continuing past the first success until
  // the branch count or the restart budget (options.max_restarts attempts) is exhausted.
  // Distinctness is per options.branch_tol. Deterministic per options.seed.
  //
  // Ordering: ascending joint-space L2 distance to `seed` when one of size model.dof() is given
  // (tracking: the branch nearest the previous position comes first); discovery order otherwise.
  // Any other preference is a caller-side sort — the vector is the cost-function hook (e.g. in
  // Python: `sorted(solver.solve_all(...), key=my_cost)`).
  //
  // Every returned result has success == true; an unreachable target yields an empty vector
  // (solve() is the API that reports best-effort errors).
  virtual std::vector<IkResult> solve_all(const std::string &link, const Transform &target,
                                          int max_solutions,
                                          const JointPosition &seed = JointPosition()) const = 0;
};

// Damped-least-squares numerical IK with multi-seed restart, using fk()/jacobian().
std::unique_ptr<InverseKinematics> make_numerical_ik(std::shared_ptr<const RobotModel> model,
                                                     IkOptions options = {});

} // namespace quevedomp
