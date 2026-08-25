# QuevedoMP Benchmark Protocol

> Single source of truth for every performance claim this project makes. Any number quoted in a
> README, roadmap record, design doc, ADR or sales material must be reproducible by following this
> document. Changes to this protocol go through review like code.

> **Revised 2026-08-25.** The original version specified a MoveIt 2 baseline container, three
> synthetic scene fixtures and a "5× faster than MoveIt" exit gate. None of that was ever built —
> all four paths it referenced were dangling — and the comparison is no longer the plan. This
> revision documents the instruments that actually exist and are actually used. The v0 build plan
> (`docs/QuevedoMP-BUILD-PLAN.md`) keeps the original text as a historical record; do not treat its
> gate language as live.

## 1. The claim under test

**QuevedoMP plans quickly in complex, high-polygon, quasi-static robot cells**, deterministically,
with an exact collision certificate.

Honest positioning, and the measurements that force it:

- **We do not claim superiority over cuRobo**, and we do not claim wins on low-poly or
  primitive-only scenes. On those, well-tuned CPU checking wins and `BackendHint::Auto` may
  correctly select FCL.
- **Environment polygon count is close to free, and must not be sold as the differentiator on its
  own.** Per-config cost is 7.3 µs on the low-poly inlet and 7.6 µs on the 7.3M-triangle hires
  variant — 4% for 1800× the triangles. What high-poly geometry actually buys us is that it does
  *not* fall over; that is a robustness claim, not a speed claim.
- **A GPU-versus-CPU speedup number is meaningless without its core budget.** On 16 idle cores FCL
  wins; at one thread OptiX wins 3.09× on the same scene. Always state the thread count.
- No claim about **success rate** or **narrow-passage solvability** is a performance claim. Those
  belong to the planner (R8), not to the backends, and must be reported separately.

## 2. The instruments

Three, in decreasing order of how much a number from them means.

### 2.1 End-to-end plan attribution — `session_profile.py` (the number that matters)

A versioned `.qmps` session planned headless, reporting `PlanningStats`. This is the only
instrument that measures what a user experiences.

```bash
PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
  python3 examples/python/session_profile.py sessions/benchmark.qmps \
    --seeds 8 [--backend keep|fcl|optix|auto] [--timeout S] [--sweep M]
```

Reference problem: **`sessions/benchmark.qmps`** (versioned; `rbrobout_inlet`, joint goal, seed 8).
It reports total / planner / collision ms, config count, and the batch-size histogram. Report all
of them — a plan that got faster by checking fewer configs is a different result from one that got
faster per config.

### 2.2 Collision microbenchmarks — `bench_collision`

FCL vs OptiX `query_batch` latency and throughput across batch sizes, on a UR5.

```bash
./build/bench-optix/bench_collision                 # synthetic triangle-count sweep
./build/bench-optix/bench_collision <mesh> [scale]  # one real mesh from disk as the environment
```

The synthetic sweep is the acceleration-structure lever (48 tris → 500k). The file-mesh form is the
reality check on it. `scale` exists because STL records no unit metadata.

### 2.3 Cost decomposition — `bench_dtc`

The instrument that answers *where the time goes*, on a real industrial cell rather than a
synthetic wall. Times the same batch under four combinations — floor / env-only / self-only / full
— so robot-vs-self and robot-vs-env can be read off directly.

```bash
./build/bench-optix/bench_dtc [mt_part|inlet]
```

**Read `D` against `B` and `C`, never against the isolated deltas.** The passes early-out into one
another, so the components are not additive (measured `D/sum = 0.84`). This is the mistake that
made "GPU collision checking" look like the top priority for a year; see
[`../QuevedoMP-R10-DESIGN.md`](../QuevedoMP-R10-DESIGN.md) §1.

## 3. Metrics

### 3.1 End-to-end (per §2.1)

- Total plan wall time (plan + smooth); time parameterization reported separately.
- **Collision ms and config count**, always together.
- Batch-size histogram — a planner that issues small batches is a planner that cannot use a GPU.
- Success within the timeout, and path length (joint-space L2) as the guard against fast garbage.
- Scene build time and `move_object` update latency, reported as their own numbers, never
  amortized into query time.

### 3.2 Collision (per §2.2, §2.3)

- Per-batch latency at 1 / 10 / 100 / 1000 / 10000, both backends.
- Bulk throughput (configs/sec) at batch 10000.
- **Thread scaling**: the same table at `OMP_NUM_THREADS=1`. Without it a backend comparison is
  unfalsifiable — see §1.
- The four-way decomposition with its collision fractions and the anatomy line (collision shapes,
  checked self-pairs after the ACM, triangles per side).

## 4. Methodology rules

1. **Hardware is recorded** with every result: CPU core count, GPU model, driver version. Laptops:
   plugged in, performance mode.
2. **Warm up, then take the minimum of several trials.** The minimum rejects one-off scheduling
   jitter that inflates a mean. Acceleration-structure and scene build time is its own metric,
   never hidden inside query time.
3. **Release builds only** (`bench-optix` or `release-py`). Sanitizer and Debug builds are never
   benchmarked — Debug is ~9× slower.
4. **Same machine, same session** for any A-vs-B comparison. No cross-machine comparisons, and no
   comparing against a number recorded on another day.
5. **A cold machine is not a benchmark.** Contention from anything starting up in the background
   will silently produce a plausible-looking wrong number; re-run until consecutive runs agree.
6. **Seeds recorded** for every stochastic component (problems, planner, sampling).
7. **Edge-validation resolution is part of the protocol, not a tuning knob.** Two runs at different
   `edge_resolution` or `max_link_sweep` are not comparable. When comparing, state the
   equal-guarantee setting; changing it is a protocol change.
8. **State the thread count** on every backend comparison (§1).

## 5. Where numbers are recorded

There is no results database; numbers live next to the decision they informed.

| kind | home |
|---|---|
| A feature's measured outcome | its roadmap row in [`../QuevedoMP-ROADMAP-v1.md`](../QuevedoMP-ROADMAP-v1.md) |
| The reasoning behind a design | the R-design doc's "numbers to record" section |
| A ratified decision's evidence | the ADR in [`../architecture/`](../architecture/) |
| Fixture properties and baselines | that fixture's `PROVENANCE.md` |

Every recorded number carries: date, git SHA, machine, and the exact command line.

Regression threshold: **>10% on any §3 metric** is a finding, not noise, and needs an explanation
before it is accepted.
