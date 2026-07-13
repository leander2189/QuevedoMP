// planning/ShortcutSmoother — batched iterative shortcut path smoothing (Task 3.3, spec §6
// Phase 3; batched + time-budgeted in Task 3.3d P6).
//
// Each ROUND samples up to `batch_size` candidate chords path[i]→path[j] whose interiors are
// DISJOINT (they may share endpoints), validates them all as ONE CollisionScene::query_batch —
// the planner's fat-batch trick applied to smoothing, so the parallel-CPU/GPU backends engage —
// and deletes the interior nodes of every free chord (right-to-left, so earlier erasures cannot
// shift later ones). Each replacement is collision-safe (the chord was checked) and length-safe
// (a chord is ≤ the polyline it replaces by the triangle inequality), so the output is
// collision-free and never longer than the input, exactly as in the serial smoother. Endpoints
// are never removed (i ≥ 0, j ≤ n-1). A single Rng(seed) drives all chord choices ⇒
// deterministic; batch_size = 1 reproduces the pre-P6 smoother draw-for-draw.
//
// `time_budget` (checked before each round) makes smoothing anytime: the pipeline can spend
// exactly its polish budget and stop with a valid, partially-shortened path.
#include "quevedomp/planning/smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_discretization.hpp"
#include "quevedomp/core/rng.hpp"

namespace quevedomp::planning {
namespace {

using Clock = std::chrono::steady_clock;

class ShortcutSmoother final : public Smoother {
public:
  ShortcutSmoother(SmootherParams params, std::shared_ptr<const RobotInstance> robot,
                   std::shared_ptr<const collision::CollisionScene> scene)
      : params_(std::move(params)), robot_(std::move(robot)), scene_(std::move(scene)),
        disc_(collision::make_edge_discretization(params_.edge_resolution, params_.max_link_sweep,
                                                  params_.lever_weights, robot_->model())) {}

  Path smooth(const Path &path) const override {
    // Nothing to shorten with fewer than one interior node.
    if (path.size() <= 2) {
      return path;
    }

    const auto t_begin = Clock::now();
    Path p = path;
    Rng rng(params_.seed);
    const auto ws = scene_->make_workspace();
    const std::size_t batch = std::max<std::size_t>(1, params_.batch_size);

    std::size_t attempts = 0; // chords validated; max_iterations bounds this in both modes
    while (attempts < params_.max_iterations && p.size() > 2) {
      if (params_.time_budget > 0.0 &&
          std::chrono::duration<double>(Clock::now() - t_begin).count() >= params_.time_budget) {
        break;
      }
      const std::size_t n = p.size();

      // Sample candidate chords with pairwise-disjoint interiors. i ∈ [0, n-3], j ∈ [i+2, n-1]
      // guarantees ≥ 1 interior node. Overlapping draws are discarded (bounded redraws, not
      // counted as attempts); batch_size = 1 never redraws, matching the pre-P6 Rng sequence.
      std::vector<std::pair<std::size_t, std::size_t>> chords;
      for (std::size_t draw = 0;
           draw < 4 * batch && chords.size() < batch &&
           attempts + chords.size() < params_.max_iterations && chords.size() + 2 < n;
           ++draw) {
        const auto i = static_cast<std::size_t>(rng.uniform(0.0, static_cast<double>(n - 2)));
        const auto j = static_cast<std::size_t>(
            rng.uniform(static_cast<double>(i + 2), static_cast<double>(n)));
        const bool overlaps = std::any_of(
            chords.begin(), chords.end(), [&](const std::pair<std::size_t, std::size_t> &c) {
              return i < c.second && c.first < j; // open intervals intersect
            });
        if (!overlaps) {
          chords.emplace_back(i, j);
        }
      }
      if (chords.empty()) {
        break; // path too short for another disjoint chord this round (n == 3 handled above)
      }
      attempts += chords.size();

      // ONE batch for the whole round: every chord discretized (P3 policy) and concatenated.
      std::vector<JointPosition> samples;
      std::vector<std::pair<std::size_t, int>> slices; // (offset, steps) per chord
      slices.reserve(chords.size());
      for (const auto &[i, j] : chords) {
        const JointPosition delta = p[j] - p[i];
        const int steps = disc_.steps(delta);
        slices.emplace_back(samples.size(), steps);
        for (int k = 0; k <= steps; ++k) {
          samples.push_back(p[i] + (static_cast<double>(k) / steps) * delta);
        }
      }
      const collision::BatchResult br =
          scene_->query_batch(*robot_, samples, params_.collision, *ws);

      // Accept every free chord, highest i first: disjoint interiors mean earlier (right-side)
      // erasures never move a later (left-side) chord's nodes.
      std::vector<std::size_t> order(chords.size());
      for (std::size_t k = 0; k < order.size(); ++k) {
        order[k] = k;
      }
      std::sort(order.begin(), order.end(),
                [&](std::size_t a, std::size_t b) { return chords[a].first > chords[b].first; });
      for (const std::size_t c : order) {
        const auto [off, steps] = slices[c];
        bool free = true;
        for (int k = 0; k <= steps; ++k) {
          if (br.in_collision[off + static_cast<std::size_t>(k)] != 0) {
            free = false;
            break;
          }
        }
        if (free) {
          const auto [i, j] = chords[c];
          p.erase(p.begin() + static_cast<std::ptrdiff_t>(i + 1),
                  p.begin() + static_cast<std::ptrdiff_t>(j));
        }
      }
    }
    return p;
  }

private:
  SmootherParams params_;
  std::shared_ptr<const RobotInstance> robot_;
  std::shared_ptr<const collision::CollisionScene> scene_;
  collision::EdgeDiscretization disc_;
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
