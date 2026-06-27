// core/rng — deterministic RNG with substream spawning (spec §5.2, ADR-006).
//
// No global RNG: every randomized component receives an `Rng` explicitly. `spawn(stream_id)`
// returns an independent substream that is a *pure function of (this Rng's seed, stream_id)* —
// so CPU-side sampling is reproducible regardless of thread count or call order. Determinism
// is per-build (same compiler/stdlib); it is best-effort, NOT bit-exact, across platforms
// (ADR-006).
#pragma once

#include <cstdint>
#include <random>

#include <Eigen/Core>

namespace quevedomp {

class Rng {
public:
  explicit Rng(std::uint64_t seed);

  // Independent deterministic substream. A pure function of (seed(), stream_id): it does NOT
  // depend on how many values this Rng has already drawn. `const` and side-effect-free, so it
  // is safe to call concurrently from multiple threads on a shared Rng.
  Rng spawn(std::uint64_t stream_id) const;

  // Uniform double in [lo, hi). Mutates the engine; not thread-safe on a shared Rng (give each
  // thread its own spawned substream).
  double uniform(double lo, double hi);

  // Per-component uniform sample in the axis-aligned box [lo, hi). Requires lo.size()==hi.size().
  Eigen::VectorXd sample_in_box(const Eigen::VectorXd &lo, const Eigen::VectorXd &hi);

  // The seed this Rng was constructed with — always recorded into PlanningResult::used_seed.
  std::uint64_t seed() const noexcept { return seed_; }

private:
  double canonical(); // uniform in [0, 1) from 53 fresh engine bits

  std::uint64_t seed_;
  std::mt19937_64 gen_;
};

} // namespace quevedomp
