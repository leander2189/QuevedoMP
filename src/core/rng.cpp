#include "quevedomp/core/rng.hpp"

#include <array>
#include <cassert>
#include <cstddef>

namespace quevedomp {
namespace {

// SplitMix64 finalizer — a well-distributed 64→64-bit mix. Used both to derive substream
// seeds and to expand a single 64-bit seed into the Mersenne-Twister state.
std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

// Seed mt19937_64 from one 64-bit value via a SplitMix64-expanded seed_seq, rather than the
// engine's weak single-value seeding (which leaves much of the 19937-bit state correlated).
std::mt19937_64 make_engine(std::uint64_t seed) {
  std::array<std::uint32_t, 16> words{};
  std::uint64_t s = seed;
  for (std::size_t i = 0; i < words.size(); i += 2) {
    s = splitmix64(s);
    words[i] = static_cast<std::uint32_t>(s & 0xFFFFFFFFULL);
    words[i + 1] = static_cast<std::uint32_t>(s >> 32);
  }
  std::seed_seq seq(words.begin(), words.end());
  return std::mt19937_64(seq);
}

} // namespace

Rng::Rng(std::uint64_t seed) : seed_(seed), gen_(make_engine(seed)) {}

Rng Rng::spawn(std::uint64_t stream_id) const {
  // Child seed depends only on (seed_, stream_id) — never on the engine's draw count — which
  // is what makes substreams independent of call order and thread count.
  const std::uint64_t child_seed = splitmix64(splitmix64(seed_) ^ splitmix64(stream_id));
  return Rng(child_seed);
}

double Rng::canonical() {
  // 53-bit uniform in [0, 1). Computed by hand rather than via std::uniform_real_distribution
  // (whose mapping is implementation-defined) so the bits→double step is itself portable; only
  // the engine's output stream governs determinism.
  return static_cast<double>(gen_() >> 11) * (1.0 / 9007199254740992.0); // 2^-53
}

double Rng::uniform(double lo, double hi) { return lo + (hi - lo) * canonical(); }

Eigen::VectorXd Rng::sample_in_box(const Eigen::VectorXd &lo, const Eigen::VectorXd &hi) {
  assert(lo.size() == hi.size() && "sample_in_box: lo/hi dimension mismatch");
  Eigen::VectorXd out(lo.size());
  for (Eigen::Index i = 0; i < lo.size(); ++i) {
    out[i] = uniform(lo[i], hi[i]);
  }
  return out;
}

} // namespace quevedomp
