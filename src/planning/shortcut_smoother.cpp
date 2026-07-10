// planning/ShortcutSmoother — iterative shortcut path smoothing (Task 3.3, spec §6 Phase 3).
//
// Each iteration picks two path indices i < j (with at least one node between them), validates the
// direct chord path[i]→path[j] as ONE batch via collision::check_edge, and if it is free deletes
// the intervening nodes — replacing a sub-polyline with its chord. That is collision-safe (the
// chord was checked) and length-safe (a chord is ≤ the polyline it replaces by the triangle
// inequality), so the output is collision-free and never longer than the input. Endpoints are
// never removed (i ≥ 0, j ≤ n-1). A single Rng(seed) drives the index choice ⇒ deterministic.
//
// Follow-up (not needed yet): batched shortcut (validate k non-overlapping candidate chords per
// batch) and continuous/partial shortcut (endpoints interpolated along segments, not just at
// nodes) — both fatten the batches and shorten harder. v0 keeps it one chord per iteration.
#include "quevedomp/planning/smoother.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/core/rng.hpp"

namespace quevedomp::planning {
namespace {

class ShortcutSmoother final : public Smoother {
public:
  ShortcutSmoother(SmootherParams params, std::shared_ptr<const RobotInstance> robot,
                   std::shared_ptr<const collision::CollisionScene> scene)
      : params_(std::move(params)), robot_(std::move(robot)), scene_(std::move(scene)) {}

  Path smooth(const Path &path) const override {
    // Nothing to shorten with fewer than one interior node.
    if (path.size() <= 2) {
      return path;
    }

    Path p = path;
    Rng rng(params_.seed);
    const auto ws = scene_->make_workspace();
    const float res = static_cast<float>(params_.edge_resolution);

    for (std::size_t iter = 0; iter < params_.max_iterations; ++iter) {
      const std::size_t n = p.size();
      if (n <= 2) {
        break;
      }
      // i ∈ [0, n-3], j ∈ [i+2, n-1] — guarantees ≥ 1 interior node between i and j.
      const auto i = static_cast<std::size_t>(rng.uniform(0.0, static_cast<double>(n - 2)));
      const auto j =
          static_cast<std::size_t>(rng.uniform(static_cast<double>(i + 2), static_cast<double>(n)));

      const auto edge =
          collision::check_edge(*scene_, *robot_, p[i], p[j], res, params_.collision, *ws);
      if (edge.valid) {
        // Drop the interior nodes (i, j) become adjacent, joined by the validated chord.
        p.erase(p.begin() + static_cast<std::ptrdiff_t>(i + 1),
                p.begin() + static_cast<std::ptrdiff_t>(j));
      }
    }
    return p;
  }

private:
  SmootherParams params_;
  std::shared_ptr<const RobotInstance> robot_;
  std::shared_ptr<const collision::CollisionScene> scene_;
};

} // namespace

std::unique_ptr<Smoother>
make_shortcut_smoother(SmootherParams params, std::shared_ptr<const RobotInstance> robot,
                       std::shared_ptr<const collision::CollisionScene> scene) {
  if (robot == nullptr) {
    throw std::runtime_error("make_shortcut_smoother: robot instance is null");
  }
  if (scene == nullptr) {
    throw std::runtime_error("make_shortcut_smoother: collision scene is null");
  }
  return std::make_unique<ShortcutSmoother>(std::move(params), std::move(robot), std::move(scene));
}

} // namespace quevedomp::planning
