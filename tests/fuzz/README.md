# Fuzzing & coverage gates (Task 1.9)

## URDF fuzzer

`fuzz_urdf` feeds arbitrary bytes to `RobotModel::from_urdf` (and a few accessors). Built with
clang + libFuzzer + ASan/UBSan via the `fuzz` preset.

```bash
cmake --preset fuzz && cmake --build --preset fuzz

mkdir -p build/fuzz/corpus
cp tests/fixtures/robots/*.urdf build/fuzz/corpus/      # seed with the real robot URDFs

# Bounded gate run: leak detection ON, urdfdom's third-party leaks suppressed.
LSAN_OPTIONS=suppressions=tests/fuzz/lsan_suppressions.txt \
  ./build/fuzz/tests/fuzz/fuzz_urdf -runs=100000 -max_total_time=200 \
    -artifact_prefix=build/fuzz/art/ build/fuzz/corpus
# Expect: "Done … runs", exit 0, no artifacts written.
```

### Notes on the gate

- **Run it bounded** (`-runs`/`-max_total_time`) with the **default** RSS limit. urdfdom leaks a
  little memory on each malformed input; with `-rss_limit_mb=0` those suppressed leaks accumulate
  over a long run and eventually exhaust memory (a misleading "out-of-memory"/SIGSEGV). A bounded
  run keeps RSS flat (~450 MB).
- `lsan_suppressions.txt` suppresses **urdfdom's** leaks (third-party, `model.cpp:252`) so the
  fuzzer surfaces only issues in *our* code. In normal use a URDF is parsed once from trusted
  disk, so the urdfdom leak is immaterial.

### Bugs this found (and fixes)

- **Unbounded cycle walk → 2 GB allocation.** A malformed model with a cyclic parent/child
  relationship drove `RobotModel::chain_to` (and `fk_all`/`jacobian`) into an infinite walk.
  Fixed with cycle guards (bounded by joint count; `fk_all` uses a visited set). Regression:
  `tests/unit/test_robot_robustness.cpp`.
- **Deeply nested XML.** Guarded in `from_urdf` (`reject_if_pathological`, depth cap 256) before
  urdfdom's recursive parser can stack-overflow.
- **Unchecked `dynamic_pointer_cast`** in collision parsing — now null-checked.

## Coverage gate (≥ 80% in core / robot / kinematics)

g++ `--coverage` via the `coverage` preset, reported with `gcovr`.

```bash
cmake --preset coverage && cmake --build --preset coverage
ctest --test-dir build/coverage
gcovr --root . --filter "src/(core|robot|kinematics)/" build/coverage \
      --print-summary --fail-under-line 80
# Latest: 91% line coverage (all modules ≥ 84%).
```
