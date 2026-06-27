#include "quevedomp/kinematics/ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "quevedomp/core/rng.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/jacobian.hpp"

namespace quevedomp {
namespace {

using Vector6 = Eigen::Matrix<double, 6, 1>;

// 6-vector pose error [target - current]: position delta and base-frame rotation vector.
// Matches jacobian()'s [linear; angular] row layout, so J·Δq drives `current` toward `target`.
Vector6 pose_error(const Transform &target, const Transform &current) {
  Vector6 e;
  e.head<3>() = target.translation() - current.translation();
  const Eigen::AngleAxisd aa(target.rotation() * current.rotation().transpose());
  e.tail<3>() = aa.axis() * aa.angle();
  return e;
}

class NumericalIk final : public InverseKinematics {
public:
  NumericalIk(std::shared_ptr<const RobotModel> model, IkOptions opt)
      : model_(std::move(model)), opt_(opt) {
    const int dof = static_cast<int>(model_->dof());
    const double inf = std::numeric_limits<double>::infinity();
    sample_lo_.assign(dof, -M_PI);
    sample_hi_.assign(dof, M_PI);
    clamp_lo_.assign(dof, -inf);
    clamp_hi_.assign(dof, inf);
    for (const Joint &j : model_->joints()) {
      if (j.dof_index < 0 || !j.limits.has_position_limit)
        continue; // continuous: leave unbounded
      sample_lo_[j.dof_index] = j.limits.lower;
      sample_hi_[j.dof_index] = j.limits.upper;
      clamp_lo_[j.dof_index] = j.limits.lower;
      clamp_hi_[j.dof_index] = j.limits.upper;
    }
  }

  IkResult solve(const std::string &link, const Transform &target,
                 const JointPosition &seed) const override {
    const int dof = static_cast<int>(model_->dof());
    Rng rng(opt_.seed);

    IkResult best;
    best.q = JointPosition::Zero(dof);
    best.pos_error = std::numeric_limits<double>::infinity();
    best.rot_error = std::numeric_limits<double>::infinity();
    int total_iters = 0;

    for (int restart = 0; restart <= opt_.max_restarts; ++restart) {
      JointPosition q = (restart == 0 && seed.size() == dof) ? seed : random_config(rng);
      double attempt_best = std::numeric_limits<double>::infinity();
      int stall = 0;

      for (int it = 0; it < opt_.max_iters; ++it) {
        ++total_iters;
        const Transform current = fk(*model_, q, link);
        const Vector6 e = pose_error(target, current);
        const double pe = e.head<3>().norm();
        const double re = e.tail<3>().norm();

        if (pe < opt_.pos_tol && re < opt_.rot_tol) {
          IkResult r;
          r.success = true;
          r.q = q;
          r.iterations = total_iters;
          r.restarts = restart;
          r.pos_error = pe;
          r.rot_error = re;
          return r;
        }

        const double score = pe + re;
        if (score < best.pos_error + best.rot_error) {
          best.q = q;
          best.pos_error = pe;
          best.rot_error = re;
        }
        // Stall detection: abandon a non-improving attempt and re-seed instead of grinding the
        // full iteration budget near a singularity / local minimum.
        if (score + opt_.stall_eps < attempt_best) {
          attempt_best = score;
          stall = 0;
        } else if (++stall >= opt_.stall_iters) {
          break;
        }

        // Damped least squares: Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e.
        const Eigen::MatrixXd jac = jacobian(*model_, q, link);
        const Eigen::Matrix<double, 6, 6> jjt =
            jac * jac.transpose() +
            (opt_.damping * opt_.damping) * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::VectorXd dq = jac.transpose() * jjt.ldlt().solve(e);

        const double n = dq.norm();
        if (n > opt_.max_step)
          dq *= opt_.max_step / n;
        q += dq;
        clamp(q);
      }
    }

    best.iterations = total_iters;
    best.restarts = opt_.max_restarts;
    return best;
  }

private:
  JointPosition random_config(Rng &rng) const {
    JointPosition q(static_cast<Eigen::Index>(sample_lo_.size()));
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      q[i] = rng.uniform(sample_lo_[i], sample_hi_[i]);
    }
    return q;
  }

  void clamp(JointPosition &q) const {
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      q[i] = std::min(std::max(q[i], clamp_lo_[i]), clamp_hi_[i]);
    }
  }

  std::shared_ptr<const RobotModel> model_;
  IkOptions opt_;
  std::vector<double> sample_lo_, sample_hi_, clamp_lo_, clamp_hi_;
};

} // namespace

std::unique_ptr<InverseKinematics> make_numerical_ik(std::shared_ptr<const RobotModel> model,
                                                     IkOptions options) {
  return std::make_unique<NumericalIk>(std::move(model), options);
}

} // namespace quevedomp
