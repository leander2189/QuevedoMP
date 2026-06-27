# ADR-006 — RNG substream architecture, CPU determinism, best-effort replay

**Status:** Accepted — implemented in Task 1.2 (`core/rng`).

## Context

The planner is randomized (RRT sampling, multi-seed IK restarts, future MPPI). For the
reproducibility story (spec §5: *best-effort capture + replay*, "CPU/RNG determinism") we need
sampling that is **reproducible regardless of thread count or call order**, without paying for
GPU bit-exactness (deferred with the MPPI/optimization epic, spec §5.2 `[DEFERRED]`).

Forces:

- **No global RNG.** A process-global generator makes results depend on unrelated call
  interleavings and is a data race under threading. Every randomized component must receive an
  `Rng` explicitly (spec §5.2).
- **Order/thread independence.** If a parallel RRT spawns a per-worker (or per-tree-node)
  substream, the numbers that worker draws must not depend on how many workers there are or in
  what order substreams were created — otherwise replay of a multi-threaded run is impossible.
- **The seed is always recorded.** `PlanningResult::used_seed` is always populated, whether the
  caller passed a seed or one was auto-generated, so any run can be re-driven.
- **Cross-platform bit-exactness is out of scope for v0.** It is expensive (sorted reductions,
  fixed launch configs) and concentrated in deferred GPU work.

## Decision

1. **`Rng` wraps `std::mt19937_64`** and stores the `uint64_t seed_` it was constructed with.
   `mt19937_64` is seeded not by its weak single-value constructor but via a **SplitMix64**-
   expanded `std::seed_seq` (8 SplitMix64 outputs → 16×32-bit words), so the full engine state
   is well-distributed from a single 64-bit seed.

2. **`spawn(stream_id)` is a pure function of `(seed_, stream_id)`.** The child seed is
   `splitmix64( splitmix64(seed_) ^ splitmix64(stream_id) )`; the child `Rng` is constructed
   from it. Crucially it reads **only `seed_`**, never the engine's current draw position — so
   `spawn(k)` returns the identical substream no matter how many values the parent has drawn,
   in what order siblings were spawned, or how many threads are running. `spawn` is therefore
   `const` and safe to call concurrently on a shared `Rng`.

3. **`uniform` maps engine bits to `[0,1)` by hand** — `(gen_() >> 11) · 2⁻⁵³` — instead of
   `std::uniform_real_distribution`, whose algorithm is implementation-defined. This makes the
   bits→double step itself portable, so determinism depends only on the (standard) `mt19937_64`
   output stream. `uniform(lo,hi)` and `sample_in_box(lo,hi)` (per-component, in index order)
   build on it.

4. **Determinism is per-build, best-effort across platforms — not bit-exact.** Same
   binary + same seed ⇒ identical sequences (the property we test). Across compilers/stdlibs the
   `mt19937_64` stream is standardized, and our hand-rolled `uniform` is portable, so results
   are stable in practice; we still do **not** contractually promise cross-platform bit-exact
   replay in v0 (spec §5, ADR list line "best-effort (not bit-exact) replay").

## Consequences

- Parallel planners get reproducibility "for free": give node/worker `k` the substream
  `rng.spawn(k)` and the run replays identically single- or multi-threaded.
- `spawn` being `const`/side-effect-free removes a class of threading bugs (no shared mutable
  generator across workers) — each worker mutates only its own substream.
- `uniform`/`sample_in_box` mutate the engine and are **not** thread-safe on a shared `Rng`;
  the contract is "one `Rng` (or spawned substream) per thread."
- Substream collisions are possible in principle (two `stream_id`s hashing close) but at
  ~2⁻⁶⁴ probability they are irrelevant; SplitMix64 mixing keeps sibling streams decorrelated.
- Upgrading to GPU bit-exact determinism later (the deferred `DeterminismOptions`) does not
  change this CPU contract; it adds guarantees on top.

## Tested by

`tests/unit/test_rng.cpp`: same seed → identical sequence; `spawn` independent of parent draw
count and sibling order; `spawn` results identical across 1- vs 4-thread execution; `uniform`
range + mean ≈ midpoint over 1e6 draws; `sample_in_box` bounds and dimensioning.
