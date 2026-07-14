# Time-Optimal Path Parametrization with Tip-Velocity and Jerk Limits

Implementation spec for a TOPP-style time parametrizer that extends the classic
(velocity + acceleration) formulation with **Cartesian tip-velocity limits** and
**per-joint jerk limits**, solved as a warm-started nonlinear program.

(Leandro's spec, adopted 2026-07-14 for Task 3.4 — see ADR-017 for the ratified decisions:
Phase A is implemented as a dependency-free TOPP-RA-style recursion rather than an SOCP, tip
ACCELERATION is in scope in per-axis form, and Phase B uses SCP over OSQP.)

The problem is convex in everything *except* the jerk constraints. The strategy is
therefore: solve the convex jerk-free problem to global optimality, then use that
solution to warm-start a sequential-convex-programming (SCP) loop that adds jerk.
Result: a fast, robust, locally-optimal trajectory.

---

## 0. Inputs and outputs

**Inputs**
- Geometric path `q(s)`, `s ∈ [0, 1]`, in joint space, `n` joints. Must be at least
  `C³` for jerk limits to be meaningful (see §7). Provide a way to evaluate
  `q(s), q'(s), q''(s), q'''(s)` — e.g. a quintic spline.
- Forward kinematics + geometric Jacobian `J(q) ∈ ℝ^{6×n}` (rows 0–2 translational,
  3–5 angular).
- Limits: `v_max ∈ ℝ^n` (joint vel), `a_max ∈ ℝ^n` (joint acc), `j_max ∈ ℝ^n`
  (joint jerk), `vtip_max` (scalar translational, and/or per-axis, and/or angular).
- Boundary conditions: start/end path speed and acceleration (usually zero).

**Output**
- Time-stamped trajectory: nodes `s_k` with times `t_k`, and hence
  `q(t_k), q̇(t_k), q̈(t_k)`. Resample to the controller rate afterward.

---

## 1. Notation and the change of variable

Let `s(t)` be the time parametrization. With `ṡ = ds/dt`:

```
q̇   = q'·ṡ
q̈   = q'·s̈ + q''·ṡ²
q⃛   = q'·s⃛ + 3·q''·ṡ·s̈ + q'''·ṡ³
```

Use the standard TOPP substitution

```
β(s) = ṡ²        (β ≥ 0, "squared path velocity")
```

Then, writing `β' = dβ/ds`, `β'' = d²β/ds²`:

```
ṡ  = √β
s̈  = ½·β'
s⃛  = ½·β''·√β
```

Substituting gives the key identities (per joint `i`):

```
q̇_i  = q'_i · √β                                    (1st order, in √β)
q̈_i  = q'_i · (½β')  + q''_i · β                    (LINEAR in β, β')
q⃛_i  = √β · ( ½·q'_i·β'' + (3/2)·q''_i·β' + q'''_i·β )   (NON-CONVEX)
```

- Velocity is a bound on `β` → **linear**.
- Acceleration is **linear** in the decision variables.
- Jerk carries a `√β` multiplying terms that themselves contain `β` (so a `β^{3/2}`
  appears) → **non-convex**. This is the only hard part.

---

## 2. Discretization (TOPP-RA-style control variable)

Grid: `s_0 = 0 < s_1 < … < s_N = 1`, interval widths `Δ_k = s_{k+1} − s_k`
(uniform `Δ` recommended). Precompute at every node the path derivatives and,
for the tip constraint, the Jacobian.

**Decision variables**
- `β_k ≥ 0`, `k = 0..N` — squared path velocity at nodes (`N+1` vars).
- `u_k`, `k = 0..N−1` — path acceleration `s̈`, piecewise-constant on interval `k`
  (`N` vars).

**Kinematic consistency** (exact under piecewise-constant `u`; this is the TOPP-RA
relation `β' = 2s̈`):

```
β_{k+1} = β_k + 2·u_k·Δ_k          k = 0..N−1      (C1)
```

This links `β` and `u` and means `s̈ = u_k` on interval `k`, with no finite-
difference noise on acceleration.

---

## 3. Objective (total time)

```
T = ∫₀¹ ds/√β  ≈  Σ_{k=0}^{N−1}  2·Δ_k / (√β_k + √β_{k+1})        (OBJ)
```

Derivation: `Δt_k = Δs_k / mean(ṡ)` with `mean(ṡ) = (√β_k + √β_{k+1})/2`.
Each term is **convex** in `β` (`√·` concave → positive sum concave → reciprocal
convex). Minimize `T`.

Recover times by cumulative sum: `t_0 = 0`, `t_{k+1} = t_k + 2Δ_k/(√β_k+√β_{k+1})`.

---

## 4. Constraint rows

Evaluate per node `k` and joint `i`. Everything in §4.1–4.3 is linear in the
decision variables; §4.4 is the nonconvex jerk row handled by SCP.

### 4.1 Joint velocity → upper bound on β (linear)
```
|q̇_i| ≤ v_i^max   →   β_k ≤ ( v_i^max / |q'_i(s_k)| )²      ∀ i, k     (V-JNT)
```
(If `q'_i(s_k)=0`, that joint imposes no bound at that node — skip it.)

### 4.2 Tip / Cartesian velocity → upper bound on β (linear)
Let `g_k = J(q(s_k)) · q'(s_k) ∈ ℝ⁶`. Then `v_tip = g_k · √β_k`. Split into
translational rows `g_k^t = g_k[0:3]` and angular rows `g_k^ω = g_k[3:6]`.

**Norm form (translational speed):**
```
‖g_k^t‖² · β_k ≤ (vtip_max)²   →   β_k ≤ (vtip_max)² / ‖g_k^t‖²           (V-TIP)
```
**Per-axis form (if you have axis-wise limits, e.g. feed rate along one axis):**
```
β_k ≤ ( vaxis_max / |g_k^t[axis]| )²
```
Angular tip-speed limits use `g_k^ω` identically. All of these are just more upper
bounds on `β_k`.

> **Maximum Velocity Curve (MVC).** V-JNT, V-TIP and any axis limits all collapse
> to `β_k ≤ β_k^max`, where `β_k^max = min` over every velocity-type bound at node
> `k`. Precompute the single array `β_max[k]`.

### 4.3 Joint acceleration (linear, two-sided)
Using `s̈ = u_k` and `ṡ² = β_k` on interval `k`:
```
−a_i^max ≤ q'_i(s_k)·u_k + q''_i(s_k)·β_k ≤ a_i^max     ∀ i,  k=0..N−1   (A-JNT)
```
(You may additionally enforce the same row at the interval end using `β_{k+1}`;
enforcing at both endpoints is the conservative choice.)

**Cartesian tip acceleration (ratified IN scope, per-axis form — ADR-017):**
`ẍ_tip = (J·q')·u + (J·q'' + J'·q')·β` — the `√β·√β` products collapse, so each
axis is one more A-JNT-shaped row. A scalar limit is applied per axis divided by
√3 (the box inscribed in the sphere; conservative, never permissive). `J'` by
finite difference of `J` over `s`.

### 4.4 Joint jerk (NON-CONVEX — handled in §5 Phase B)
Define the acceleration-rate `u'` by finite difference of the control:
```
u'_k = (u_k − u_{k−1}) / Δ̃_k ,   Δ̃_k = ½(Δ_{k−1}+Δ_k)   (interior k = 1..N−1)
```
Then, from identity (q⃛) with `β'=2u`, `β''=2u'`:
```
q⃛_i(s_k) = √β_k · ( q'_i(s_k)·u'_k + 3·q''_i(s_k)·u_k + q'''_i(s_k)·β_k )     (J-JNT)
|q⃛_i(s_k)| ≤ j_i^max
```
Each jerk row couples three consecutive control/state values: `β_k, u_{k−1}, u_k`.

> Formal caveat: with piecewise-constant `u`, the true acceleration jumps at nodes, so
> J-JNT bounds jerk in the finite-difference sense, not pointwise — the standard practical
> treatment; controller-rate resampling smooths the residual.

---

## 5. Two-phase solve

### Phase A — convex, global (drop jerk)
Solve:
```
minimize    T(β)                                   (OBJ)
subject to  β_{k+1} = β_k + 2 u_k Δ_k              (C1)
            0 ≤ β_k ≤ β_max[k]                     (MVC: V-JNT, V-TIP)
            A-JNT (+ per-axis tip acceleration)     (linear)
            boundary conditions (§6)
```
Convex objective + linear constraints. **Ratified implementation (ADR-017): the
TOPP-RA backward/forward recursion** — controllable sets via exact 2-variable LPs,
then the greedy pointwise-maximal profile, which is provably time-optimal for
velocity/acceleration-type constraints. Zero dependencies, microseconds, and exactly
the classic velocity+acc+tip profile. **Use it as the warm start.**

### Phase B — SCP for jerk
Linearize each jerk row (J-JNT) around the current iterate `(β̂, û)`.
Let `h_{i,k}(β,u,u') = √β_k·( q'_i u'_k + 3 q''_i u_k + q'''_i β_k )` and define
`p_{i,k} = q'_i u'_k + 3 q''_i u_k + q'''_i β_k`. Gradients at the iterate:
```
∂h/∂β_k  = p_{i,k}/(2√β̂_k)  +  √β̂_k · q'''_i
∂h/∂u_k  = √β̂_k · 3 q''_i        (plus the u'_k dependence on u_k)
∂h/∂u_{k−1} = √β̂_k · q'_i · (∂u'_k/∂u_{k−1})
with  ∂u'_k/∂u_k = 1/Δ̃_k ,  ∂u'_k/∂u_{k−1} = −1/Δ̃_k .
```
Linearized constraint: `|ĥ_{i,k} + ∇h·(Δβ,Δu)| ≤ j_i^max`. Add a trust region
`‖(β,u) − (β̂,û)‖_∞ ≤ ρ` and re-solve the (still convex) subproblem. Iterate:

```
(β̂, û) ← Phase A solution
repeat:
    build linearized jerk rows about (β̂, û)
    solve convex subproblem  (OBJ, C1, MVC, A-JNT, linearized J-JNT, trust region)
    update trust radius ρ (shrink if objective/constraint worsened, else grow)
    (β̂, û) ← solution
until  ‖Δ(β,u)‖ < tol  and  max jerk violation < tol
```

**Ratified implementation (ADR-017): the subproblem objective is the second-order
Taylor model of OBJ at the iterate (convex ⇒ PSD tridiagonal Hessian), making every
subproblem a QP solved by OSQP** (Apache-2, C, FetchContent — the D2 escape hatch,
recorded as a §12 decision). A monolithic warm-started IPOPT solve is the documented
fallback if SCP stalls.

---

## 6. Boundary conditions

Set as equality constraints. Typical rest-to-rest:
```
β_0 = ṡ_start²            β_N = ṡ_end²           (usually 0)
u_0 = 0                    u_{N−1} = 0            (zero path acceleration at ends)
```
For zero jerk at the ends, additionally constrain the first/last `u'` (i.e.
`u_1 = u_0`, `u_{N−1} = u_{N−2}`). When `β_0 = 0` or `β_N = 0`, see §7 numerics.
(Note: `u_0 = 0` **with** `β_0 = 0` never starts moving in Phase A — zero-acc/jerk
ends only make sense once the jerk phase can build acceleration up gradually; Phase A
fixes β at the ends and leaves u free.)

---

## 7. Numerical caveats (read before debugging)

1. **`√β` at rest.** At start/stop `β → 0`, so the objective term and the jerk
   gradient `p/(2√β)` blow up. Mitigate by flooring `β_k ≥ ε` (e.g. `1e−6`) for
   interior nodes, handling the exact endpoints analytically, or adding a small
   Tikhonov term. Do **not** evaluate the jerk gradient at `β=0`.

2. **Path smoothness dominates jerk feasibility.** The `q'''` term means a path
   whose third derivative is discontinuous (linear segments, or a `C²` spline with
   knot-jumps in `q'''`) injects unbounded joint jerk at *any* nonzero speed — no
   choice of `β` removes it. Feed a `C³`/quintic or corner-blended path, or accept
   that jerk limits only bind in the smooth interior. Time-parametrization limits
   *temporal* jerk; it cannot fix *geometric* jerk.

3. **Grid resolution.** `N` too small under-samples the MVC near curvature spikes
   and can make jerk look feasible when it isn't. Start `N ≈ 200–500` for a typical
   path; refine near high-curvature regions.

4. **Constraint qualification.** Near the time-optimal bang-bang regime many box
   constraints are simultaneously active and LICQ can degrade, slowing the solver.
   The trust region and warm start are what keep this well-behaved.

5. **Feasibility of jerk.** Aggressive `j_max` with tight `v_max/a_max` may be
   infeasible on a given grid. Detect via SCP non-convergence; report the max jerk
   violation node so the caller can relax limits or resmooth the path.

---

## 8. Module interface (as landed — C++ core, Python mirror)

```cpp
parameterization::PathSpline::fit(waypoints)            // C⁴ quintic B-spline
parameterization::fit_collision_free(waypoints, scene, robot, disc, opts, ws)
parameterization::limits_from_model(model, task_limits, default_acceleration)
parameterization::parametrize(model, path, limits, options) -> ParameterizationResult
// options.mode: ConvexOnly (Phase A) | Scp (Phase A warm start + jerk, Stage 2)
```

Build order (executed):
1. ✅ Per-node `q', q'', (q''')`, `g_k = J·q'`, tip-acc rows, and `β_max[k]`.
2. ✅ **Phase A** (ConvexOnly) — validated against pip-installed toppra on the same
   dense path (velocity+acceleration; durations within 2%) and against analytic
   1-DOF trapezoid/triangle profiles.
3. ✅ **V-TIP + per-axis tip acceleration** in the MVC/rows; tip speed saturation
   verified (near-constant tool speed on the capped stretch).
4. ⏳ Jerk rows via **SCP over OSQP** (Stage 2); validate `max |q⃛| ≤ j_max` in the
   smooth interior and that runtime stays within the polished budget.
```
