// Task 1.2 verify — core/rng determinism (build plan Task 1.2 Done-when + spec §5.2, ADR-006):
//  - same seed -> identical sequence
//  - spawn(k) independent of parent draw count and sibling spawn order
//  - spawn(k) identical across 1- vs 4-thread execution
//  - statistical sanity: mean of uniform ≈ midpoint over 1e6 draws
//  - sample_in_box bounds + dimensioning
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/core/rng.hpp"

using quevedomp::Rng;

namespace {

std::vector<double> draw(Rng &rng, int n) {
  std::vector<double> v;
  v.reserve(n);
  for (int i = 0; i < n; ++i)
    v.push_back(rng.uniform(0.0, 1.0));
  return v;
}

} // namespace

TEST(Rng, SameSeedSameSequence) {
  Rng a(123456789ULL);
  Rng b(123456789ULL);
  EXPECT_EQ(draw(a, 256), draw(b, 256));
}

TEST(Rng, DifferentSeedDiffersAndSeedIsRecorded) {
  Rng a(1ULL);
  Rng b(2ULL);
  EXPECT_EQ(a.seed(), 1ULL);
  EXPECT_EQ(b.seed(), 2ULL);
  EXPECT_NE(draw(a, 64), draw(b, 64));
}

TEST(Rng, SpawnIndependentOfParentDrawCount) {
  // Drawing from the parent before spawning must not change the spawned substream.
  Rng p1(42ULL);
  Rng s1 = p1.spawn(7ULL);

  Rng p2(42ULL);
  for (int i = 0; i < 1000; ++i)
    (void)p2.uniform(-1.0, 1.0); // advance the parent engine
  Rng s2 = p2.spawn(7ULL);

  EXPECT_EQ(draw(s1, 128), draw(s2, 128));
}

TEST(Rng, SpawnIndependentOfSiblingOrder) {
  // spawn(5) yields the same substream whether or not spawn(2) happened first.
  Rng pa(99ULL);
  Rng a5 = pa.spawn(5ULL);

  Rng pb(99ULL);
  Rng b2 = pb.spawn(2ULL);
  (void)b2;
  Rng b5 = pb.spawn(5ULL);

  EXPECT_EQ(draw(a5, 128), draw(b5, 128));
}

TEST(Rng, SpawnDistinctStreamsDiffer) {
  Rng p(2024ULL);
  Rng s0 = p.spawn(0ULL);
  Rng s1 = p.spawn(1ULL);
  EXPECT_NE(draw(s0, 64), draw(s1, 64));
}

TEST(Rng, SpawnIdenticalAcrossThreadCounts) {
  constexpr int kStreams = 64;
  constexpr int kDraws = 64;
  const Rng parent(0xC0FFEEULL);

  auto collect_single = [&]() {
    std::map<int, std::vector<double>> out;
    for (int k = 0; k < kStreams; ++k) {
      Rng s = parent.spawn(static_cast<std::uint64_t>(k));
      out[k] = draw(s, kDraws);
    }
    return out;
  };

  std::map<int, std::vector<double>> single = collect_single();

  // Same work spread over 4 threads; spawn() is const + pure, so the result must be identical.
  std::map<int, std::vector<double>> multi;
  std::mutex mtx;
  std::vector<std::thread> pool;
  constexpr int kThreads = 4;
  for (int t = 0; t < kThreads; ++t) {
    pool.emplace_back([&, t]() {
      for (int k = t; k < kStreams; k += kThreads) {
        Rng s = parent.spawn(static_cast<std::uint64_t>(k));
        std::vector<double> seq = draw(s, kDraws);
        std::lock_guard<std::mutex> lock(mtx);
        multi[k] = std::move(seq);
      }
    });
  }
  for (auto &th : pool)
    th.join();

  EXPECT_EQ(single, multi);
}

TEST(Rng, UniformInRangeAndMeanNearMidpoint) {
  Rng rng(7ULL);
  constexpr int kN = 1'000'000;
  double sum = 0.0;
  for (int i = 0; i < kN; ++i) {
    const double x = rng.uniform(0.0, 1.0);
    ASSERT_GE(x, 0.0);
    ASSERT_LT(x, 1.0);
    sum += x;
  }
  const double mean = sum / kN;
  // Std error of the mean ≈ (1/√12)/√1e6 ≈ 2.9e-4, so 5e-3 is ~17σ — robust, not flaky.
  EXPECT_NEAR(mean, 0.5, 5e-3);
}

TEST(Rng, SampleInBoxRespectsBoundsAndDimension) {
  Rng rng(555ULL);
  Eigen::VectorXd lo(3);
  Eigen::VectorXd hi(3);
  lo << -1.0, 0.0, 10.0;
  hi << 1.0, 0.5, 20.0;

  for (int trial = 0; trial < 10000; ++trial) {
    const Eigen::VectorXd s = rng.sample_in_box(lo, hi);
    ASSERT_EQ(s.size(), 3);
    for (int i = 0; i < 3; ++i) {
      EXPECT_GE(s[i], lo[i]);
      EXPECT_LT(s[i], hi[i]);
    }
  }
}
