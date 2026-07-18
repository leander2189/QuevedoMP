# ADR-021 — Studio working modes: modal UI over a shared StudioContext (roadmap R6)

**Status:** Accepted 2026-07-18 (roadmap R6, ratified with Leandro; prerequisite for the R7
`quevedomp_tasks` inspector). Restructures the ADR-016 studio UI; `StudioSession` and the `.qmps`
format are unchanged.

## Context

After R2–R5 the studio stacked **nine** always-visible gui folders built by one 961-line
`StudioApp`: Session, Robot, IK, Obstacles, Planning, Trajectory (+Plots), Clearance, Refine
(CHOMP), Roadmap (PRM). Two problems:

1. **Panels were named after algorithms, not activities.** Clearance/Refine/Roadmap are
   implementation vocabulary; the user's actual intents are "test IK", "debug a planner",
   "shape a trajectory". The upcoming R7 tasks UI had no coherent place to land.
2. **Panels read each other's widgets freely.** Session-save read Planning's
   `timeout/do_smooth/edge_res/link_sweep`; Refine read Planning's params *and* Clearance's
   `sdf_res`; Roadmap read Planning's params and poked `session._prm`; nearly everything read
   IK's link dropdown through `_ee()`; `_show_attempt` wrote Trajectory's status. The Clearance
   panel even kept its own `ClearanceField`, separate from the session cache the refiner uses.

## Decision

**Five working modes, grouped by role, over one shared `StudioContext`.** Only the Session panel
and the mode switcher stay globally visible.

- **Scene** — joint sliders + obstacle editing (the world every mode operates on; robot/obstacle
  meshes and drag gizmos stay visible in all modes — only panels are modal).
- **IK** — gizmo, tracked/global solve, branch picker. Sole writer of `ctx.ee_link`.
- **Plan** — *planners as peers*: one dropdown (RRT-Connect / PRM roadmap / CHOMP standalone)
  behind one Plan button, with per-planner sub-folders (PRM build, CHOMP knobs). Debug views live
  here: exploration trees (R2) and the clearance heatmap slice (R3). The heatmap builds through
  `session.clearance_field()` — the panel-local duplicate is deleted; heatmap and refiner share
  one cache and its invalidation.
- **Trajectory** — parameterize (Task 3.4), CHOMP *polish* (R4 refiner mode), scrub/play/timed
  playback, vel/acc/tip plots. CHOMP deliberately appears twice — as a Plan-mode planner
  (standalone) and here as a polisher — because that is how it is used; the old
  `refine_standalone` checkbox is gone.
- **Tasks** — placeholder until R7; will become the MTC-lite *inspector/runner* (stage tree,
  per-stage status/solutions, preview, hand-off to Trajectory) for tasks defined in Python.

### StudioContext — shared state, explicitly *not* an event bus

`context.py` holds what more than one mode needs: session, server, the ADR-005 UI lock,
`ee_link`, the IK gizmo (created here because Plan-goal capture and pose-goal restore touch it
while IK is inactive), `last_attempt`, and `AttemptView` (path/tree/marker scene nodes with
mode-controlled visibility). Cross-mode *reactions* use three plain listener lists —
`config_listeners` (slider sync), `scene_changed_listeners` (staleness), `attempt_listeners`
(timing/scrub invalidation). At this scale a pub/sub framework would be ceremony; direct widget
reads across modes are what we just removed.

Replacing the old couplings: every Plan dispatch pushes its widget values onto the session
(`_push_params`), and Trajectory's refine certifies at those *session* values — "the fidelity the
last plan used" — instead of re-reading another panel's live widgets. Obstacle edits fire
`ctx.scene_changed()`; Plan marks built roadmap/field status lines **STALE** using the new
`session.has_roadmap` / `has_clearance_field` (no more `session._prm` pokes).

### Mode switcher — button group toggling `.visible`, isolated in one method

viser 1.0.30 (verified in-container) has no server-side tab-change event, so tabs can't drive
scene-node visibility. The switcher is a `gui.add_button_group` whose handler calls
`StudioApp._set_mode` — THE one place with the visibility policy (folders; gizmo only in IK;
trees + heatmap slice only in Plan; path in Plan/Trajectory). If a later viser adds a reactive
tab group, the swap is local to that method. The active mode survives `load_session` remounts;
`Mode.shutdown()` stops playback threads before the gui/scene reset.

### Session-layer touch-ups

`build_roadmap_async` joins `plan_async`/`refine_async` on the single worker thread — the old
inline app thread bypassed `is_planning` and could race a plan; the PRM query gained the same
guard. Playback stays deliberately unguarded (it only reads the path and writes configs).

### Headless surface

`StudioApp` keeps the documented synchronous entry points (`plan_now`, `parametrize_now`,
`refine_now`, `build_clearance_now`, `build_roadmap_now`, `query_roadmap_now`, `play`,
`play_timed`, `set_config`, `add_obstacle`, `load_session`) plus `app.ctx` and
`app.scene/ik/plan/trajectory/tasks`. The transitional monolith aliases were dropped; tests
address modes directly.

## Verify (2026-07-18)

Studio smoke suite 34 passed in-container after every migration step (the split landed as seven
compiling, green commits), including new tests: mode switching toggles folder/gizmo/tree/path
visibility; obstacle edit marks the roadmap status STALE; the Plan dispatch runs CHOMP-standalone
end to end (`refiner_mode == "standalone"`); `build_roadmap_async` respects `is_planning`;
`load_session` preserves the active mode. Manual click-through of `sessions/benchmark.qmps`
across all five modes.

## Consequences

- `app.py` 961 → ~190 lines; new `context.py` + `modes/{base,scene,ik,plan,trajectory,tasks,
  chomp_params}.py`. `StudioSession` untouched; `.qmps` stays `quevedomp-studio/2`.
- Per-mode widget values (CHOMP knobs, PRM sizes, SDF resolution, caps) still do **not** persist
  in `.qmps` — unchanged from before, now documented here.
- The CHOMP knob set exists twice on screen (Plan standalone vs Trajectory polish) by design —
  separate widgets, separate values, zero shared reads (`modes/chomp_params.py` builds both).
- The R7 tasks work lands as a sixth file in `modes/` against a stable contract
  (`build/set_active/shutdown` + ctx listeners).

## Alternatives rejected

- **viser tab group as the primary switcher** — no server-side selected-tab event at 1.0.30, so
  modes couldn't drive scene-node visibility. Revisit if viser adds one; the swap is one method.
- **Keep per-algorithm panels under a Plan umbrella** — less rework, but preserves exactly the
  algorithm-first naming and crowding this ADR exists to remove.
- **An event bus** — three listener lists cover every cross-mode reaction in a ~2k-line UI.
- **A `__getattr__` compatibility facade over the old attribute namespace** — would freeze the
  monolith's names as API and hide ownership; the test rename was one mechanical pass.
