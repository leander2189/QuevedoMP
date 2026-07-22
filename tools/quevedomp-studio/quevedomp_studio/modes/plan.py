"""Plan mode: pick a planner, set the problem, plan, and debug the result (ADR-021).

The three planners are peers behind one Plan button — RRT-Connect (single-query sampling),
the PRM roadmap (build once, query in ms — R5), and standalone CHOMP (optimize a straight
line — R4). Debug views live here too: the exploration-tree drawing and the clearance-field
heatmap slice. The heatmap builds through the SESSION's cached field (shared with the R4
refiner and invalidated on obstacle edits) — not a panel-local copy.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

import quevedomp as q

from ..session import Attempt
from .base import Mode
from .chomp_params import ChompParamsWidgets

PLANNER_LABELS = {"rrt": "RRT-Connect", "prm": "PRM roadmap", "chomp": "CHOMP (standalone)"}
STALE_SUFFIX = " · STALE (obstacles changed)"

# Distinct colors cycled per roadmap connected component (component id % len). A bottleneck no
# edge crosses splits the roadmap into ≥2 components → start-side and goal-side draw in different
# colors, so the gap is visible at a glance.
COMPONENT_COLORS = np.array([
    [ 80, 160, 255], [255, 140,  60], [ 90, 200, 110], [220,  90, 200],
    [230, 200,  60], [120, 120, 235], [ 60, 200, 200], [235,  90,  90],
], dtype=float)


class PlanMode(Mode):
    name = "plan"
    title = "Plan"

    def build(self) -> None:
        ctx = self.ctx
        gui = ctx.server.gui
        session = ctx.session
        self.folder = gui.add_folder(self.title)
        with self.folder:
            self.planner_pick = gui.add_dropdown(
                "planner", options=tuple(PLANNER_LABELS.values()),
                initial_value=PLANNER_LABELS["rrt"],
            )
            set_start = gui.add_button("Set start = current")
            set_goal = gui.add_button("Set goal = current")
            self.use_ik_goal = gui.add_checkbox("goal = IK gizmo pose", initial_value=False)
            self.timeout = gui.add_number("timeout (s)", initial_value=float(session.timeout),
                                          min=0.1, max=60.0)
            self.seed = gui.add_number("seed (0 = auto)", initial_value=0, min=0, max=2**31)
            self.edge_res = gui.add_number(
                "edge check step (rad|m)", initial_value=session.planner_params.edge_resolution,
                min=0.001, max=0.2, step=0.001,
            )
            self.link_sweep = gui.add_number(
                "max link sweep (mm, 0 = off)",  # P3: workspace-bounded edge steps; overrides ↑
                initial_value=session.planner_params.max_link_sweep * 1e3,
                min=0.0, max=100.0, step=0.5,
            )
            self.do_smooth = gui.add_checkbox("shortcut smoothing",
                                              initial_value=bool(session.smooth))
            self.show_tree = gui.add_checkbox(
                "record exploration tree", initial_value=False
            )  # R2: one snapshot copy at plan exit; drawn as line clouds per tree
            self.plan_button = gui.add_button("Plan")
            self.status = gui.add_text("result", initial_value="—", disabled=True)

            self.rrt_folder = gui.add_folder("RRT-Connect")
            with self.rrt_folder:
                # In a narrow pocket the goal tree can't take a single 0.5-rad step (growth is
                # all-or-nothing), so exposing the extension step is the key knob for those motions.
                self.rrt_max_ext = gui.add_number(
                    "extension step (rad)",
                    initial_value=float(session.planner_params.max_extension),
                    min=0.01, max=2.0, step=0.01,
                )
                self.rrt_goal_bias = gui.add_number(
                    "goal bias", initial_value=float(session.planner_params.goal_bias),
                    min=0.0, max=0.5, step=0.01,
                )

            self.prm_folder = gui.add_folder("PRM roadmap")
            with self.prm_folder:
                self.prm_nodes = gui.add_number("nodes", initial_value=1000, min=50, max=20000,
                                                step=50)
                self.prm_k = gui.add_number("k neighbours", initial_value=10, min=2, max=50,
                                            step=1)
                self.prm_seed = gui.add_number("build seed", initial_value=0, min=0, max=2**31)
                self.prm_smooth = gui.add_checkbox("smooth query path", initial_value=True)
                self.prm_build_button = gui.add_button("Build roadmap")
                self.prm_status = gui.add_text("roadmap", initial_value="—", disabled=True)
                self.draw_roadmap_cb = gui.add_checkbox(
                    "draw roadmap (by component)", initial_value=False
                )  # debug view: nodes+edges at EE, colored per connected component

            self.chomp_folder = gui.add_folder("CHOMP (standalone)")
            with self.chomp_folder:
                self.chomp = ChompParamsWidgets(gui)

            with gui.add_folder("Debug: goal escapability"):
                # "Is the goal wedged?" — measure the largest collision-free step away from the goal
                # per joint. A goal that's free but can't move in ANY direction is an isolated pocket
                # (why the goal tree / roadmap has no nodes there); its max reach is a good ceiling
                # for the RRT extension step.
                self.escape_step = gui.add_number("probe span (rad)", initial_value=0.5, min=0.05,
                                                  max=2.0, step=0.05)
                self.escape_button = gui.add_button("Probe goal escapability")
                self.escape_status = gui.add_text("escape", initial_value="—", disabled=True)

            with gui.add_folder("Debug: clearance heatmap"):
                self.sdf_res = gui.add_number("SDF resolution (mm)", initial_value=10.0, min=2.0,
                                              max=100.0, step=1.0)
                self.sdf_build_button = gui.add_button("Build clearance field")
                self.sdf_status = gui.add_text("field", initial_value="—", disabled=True)
                self.sdf_slice = gui.add_slider("slice height (z)", min=0.0, max=1.0, step=0.01,
                                                initial_value=0.5)
                self.sdf_range = gui.add_number("color range ± (m)", initial_value=0.3, min=0.05,
                                                max=2.0, step=0.05)

        self._field = None  # the session-cached field this panel last drew
        self._slice_node = None
        self._roadmap_nodes: list = []  # viser handles for the roadmap debug drawing
        self._active = False

        set_start.on_click(lambda _e: self.set_start())
        set_goal.on_click(lambda _e: self.set_goal())
        self.plan_button.on_click(lambda _e: self._on_plan_clicked())
        self.planner_pick.on_update(lambda _e: self._sync_planner_sections())
        self.prm_build_button.on_click(lambda _e: self._on_build_roadmap())
        self.draw_roadmap_cb.on_update(lambda _e: self._on_draw_roadmap_toggled())
        self.escape_button.on_click(lambda _e: self._on_probe_escapability())
        self.sdf_build_button.on_click(lambda _e: self._on_build_clearance())
        self.sdf_slice.on_update(lambda _e: self.draw_clearance_slice())
        self.sdf_range.on_update(lambda _e: self.draw_clearance_slice())
        ctx.scene_changed_listeners.append(self._on_scene_changed)
        self._sync_planner_sections()

    def set_active(self, active: bool) -> None:
        super().set_active(active)
        self._active = active
        self.ctx.attempt_view.set_trees_visible(active)  # exploration trees are a debug view
        if self._slice_node is not None:
            self._slice_node.visible = active
        for node in self._roadmap_nodes:
            node.visible = active and self.draw_roadmap_cb.value

    # ---- problem setup -----------------------------------------------------------------------

    def set_start(self) -> None:
        session = self.ctx.session
        session.set_start()
        self.ctx.attempt_view.set_start_marker(session.start)

    def set_goal(self) -> None:
        ctx = self.ctx
        if self.use_ik_goal.value:
            target = q.Transform.from_parts(
                np.asarray(ctx.ik_gizmo.position), np.asarray(ctx.ik_gizmo.wxyz)
            )
            ctx.session.set_goal_pose(ctx.ee_link, target)
        else:
            ctx.session.set_goal_joints()
        ctx.attempt_view.set_goal_marker(ctx.session.q)

    def _push_params(self) -> None:
        """Widget values → session, so every dispatch (and a later refine) runs and certifies
        at the fidelity shown on screen."""
        session = self.ctx.session
        session.timeout = float(self.timeout.value)
        session.smooth = bool(self.do_smooth.value)
        session.planner_params.edge_resolution = float(self.edge_res.value)
        session.planner_params.max_link_sweep = float(self.link_sweep.value) * 1e-3
        session.planner_params.record_tree = bool(self.show_tree.value)
        session.planner_params.max_extension = float(self.rrt_max_ext.value)  # RRT-only
        session.planner_params.goal_bias = float(self.rrt_goal_bias.value)    # RRT-only

    # ---- the Plan dispatch -------------------------------------------------------------------

    def _planner_key(self) -> str:
        label = self.planner_pick.value
        return next(k for k, v in PLANNER_LABELS.items() if v == label)

    def _sync_planner_sections(self) -> None:
        kind = self._planner_key()
        self.rrt_folder.visible = kind == "rrt"
        self.prm_folder.visible = kind == "prm"
        self.chomp_folder.visible = kind == "chomp"

    def _on_plan_clicked(self) -> None:
        session = self.ctx.session
        if session.is_planning:
            return
        if session.goal is None:
            self.status.value = "set a goal first"
            return
        self._push_params()
        kind = self._planner_key()
        if kind == "rrt":
            self.status.value = "planning…"
            self.plan_button.disabled = True
            session.plan_async(self._on_plan_done, seed=int(self.seed.value) or None)
        elif kind == "prm":
            if not session.has_roadmap:
                self.status.value = "build the roadmap first"
                return
            try:  # queries are ms once the roadmap exists (R5) — synchronous is fine
                attempt = session.plan_roadmap(seed=int(self.prm_seed.value) or None)
            except Exception as error:  # noqa: BLE001 — surface query failures to the UI
                self.status.value = f"FAILED: {error}"
                return
            self.show_attempt(attempt)
        else:  # chomp standalone: optimize a straight line to the resolved goal (R4)
            self.status.value = "optimizing…"
            self.plan_button.disabled = True
            self.ctx.session.refine_async(self._on_plan_done,
                                          **self.chomp.kwargs(standalone=True))

    def _on_plan_done(self, attempt: Optional[Attempt]) -> None:
        try:
            if attempt is None:
                self.status.value = "ERROR — see server console"
            else:
                self.show_attempt(attempt)
        finally:
            self.plan_button.disabled = False

    def show_attempt(self, attempt: Attempt) -> None:
        """Format the result line and hand the attempt to the context for drawing."""
        r = attempt.result
        stats = r.stats
        if r.ok():
            mode_note = f" · {stats.refiner_mode}" if getattr(stats, "refiner_mode", "") else ""
            self.status.value = (
                f"{r.status} · {len(attempt.path)} wp · {stats.time_total * 1e3:.0f} ms · "
                f"{stats.collision_configs} configs · seed {r.used_seed}{mode_note}"
            )
        else:
            self.status.value = f"{r.status} · {r.message} · seed {r.used_seed}"
        self.ctx.show_attempt(attempt)

    def plan_now(self, seed: Optional[int] = None) -> Attempt:
        """Synchronous RRT plan + display — the headless smoke-test entry point."""
        session = self.ctx.session
        session.planner_params.record_tree = bool(self.show_tree.value)
        attempt = session.plan(seed)
        self.show_attempt(attempt)
        return attempt

    # ---- PRM roadmap build (R5) --------------------------------------------------------------

    def _roadmap_build_kwargs(self) -> dict:
        return dict(
            num_nodes=int(self.prm_nodes.value),
            k_neighbors=int(self.prm_k.value),
            seed=int(self.prm_seed.value),
            smooth=bool(self.prm_smooth.value),
            export_roadmap=True,  # keep geometry so the debug view can draw without a rebuild
            force=True,
        )

    def _on_build_roadmap(self) -> None:
        session = self.ctx.session
        if session.is_planning:
            return
        session.planner_params.edge_resolution = float(self.edge_res.value)
        session.planner_params.max_link_sweep = float(self.link_sweep.value) * 1e-3
        self.prm_status.value = "building…"
        self.prm_build_button.disabled = True
        session.build_roadmap_async(self._on_roadmap_built, **self._roadmap_build_kwargs())

    def _on_roadmap_built(self, stats) -> None:
        try:
            if stats is None:
                self.prm_status.value = "FAILED — see server console"
            else:
                comp = (f" · {stats.num_components} comp (largest {stats.largest_component})"
                        + (" ⚠ SPLIT" if stats.num_components > 1 else ""))
                self.prm_status.value = (
                    f"{stats.nodes} nodes · {stats.edges} edges{comp} · "
                    f"{stats.collision_configs} configs · {stats.build_seconds * 1e3:.0f} ms"
                )
                self.draw_roadmap()  # refresh the debug view against the new roadmap
        finally:
            self.prm_build_button.disabled = False

    # ---- roadmap debug view (nodes + edges at the EE, colored by connected component) ----------

    def _on_draw_roadmap_toggled(self) -> None:
        if self.draw_roadmap_cb.value and not self._roadmap_nodes:
            self.draw_roadmap()
        else:
            for node in self._roadmap_nodes:
                node.visible = self._active and self.draw_roadmap_cb.value

    def _clear_roadmap(self) -> None:
        for node in self._roadmap_nodes:
            node.remove()
        self._roadmap_nodes = []

    def draw_roadmap(self) -> None:
        """Draw the roadmap's nodes and edges at their end-effector positions, colored by connected
        component. Reveals a bottleneck directly: a roadmap split by a narrow passage shows two (or
        more) differently-colored clusters with no edges bridging them."""
        self._clear_roadmap()
        stats = self.ctx.session.roadmap_stats
        if stats is None or not self.draw_roadmap_cb.value:
            return
        nodes_q = getattr(stats, "roadmap_nodes", None)
        if not nodes_q:  # roadmap built without geometry export (e.g. a headless build)
            self.prm_status.value += " · (rebuild to draw)"
            return

        model, ee = self.ctx.session.model, self.ctx.ee_link
        pos = np.array([q.fk(model, np.asarray(w), ee).translation() for w in nodes_q])
        comp = np.asarray(stats.roadmap_component, dtype=int)
        node_colors = COMPONENT_COLORS[comp % len(COMPONENT_COLORS)]
        server = self.ctx.server
        self._roadmap_nodes.append(server.scene.add_point_cloud(
            "/plan/roadmap/nodes", points=pos, colors=node_colors, point_size=0.006,
        ))

        edges = list(getattr(stats, "roadmap_edges", []) or [])
        if edges:
            e = np.asarray(edges, dtype=int)  # (E, 2) index pairs
            if len(e) > 20000:  # cap the line count on huge roadmaps (nodes still all drawn)
                e = e[np.linspace(0, len(e) - 1, 20000).astype(int)]
            segments = np.stack([pos[e[:, 0]], pos[e[:, 1]]], axis=1)  # (E, 2, 3)
            edge_colors = np.repeat(
                (COMPONENT_COLORS[comp[e[:, 0]] % len(COMPONENT_COLORS)])[:, None, :], 2, axis=1
            )
            self._roadmap_nodes.append(server.scene.add_line_segments(
                "/plan/roadmap/edges", points=segments, colors=edge_colors,
            ))
        for node in self._roadmap_nodes:
            node.visible = self._active and self.draw_roadmap_cb.value

    def build_roadmap_now(self):
        """Synchronous build — the headless smoke-test entry point."""
        stats = self.ctx.session.build_roadmap(**self._roadmap_build_kwargs())
        self.prm_status.value = f"{stats.nodes} nodes · {stats.edges} edges"
        return stats

    def query_roadmap_now(self) -> Attempt:
        """Synchronous build-if-needed + query — the headless smoke-test entry point."""
        session = self.ctx.session
        if not session.has_roadmap:
            session.build_roadmap(**self._roadmap_build_kwargs())
        attempt = session.plan_roadmap(seed=int(self.prm_seed.value) or None)
        self.show_attempt(attempt)
        return attempt

    # ---- goal escapability probe (is the goal a wedged pocket?) -------------------------------

    def _goal_config(self) -> Optional[np.ndarray]:
        """Resolve the current goal to a concrete config to probe (joint goal → its target; pose
        goal → one IK solution). None if there is no goal / IK fails."""
        session = self.ctx.session
        g = session.goal
        if g is None:
            return None
        if g.kind == "joint":
            return None if g.q is None else np.asarray(g.q, dtype=float)
        r = session.solve_ik(g.link, g.pose)  # pose goal → a config
        return np.asarray(r.q, dtype=float) if getattr(r, "success", False) else None

    def _on_probe_escapability(self) -> None:
        session = self.ctx.session
        q_goal = self._goal_config()
        if q_goal is None:
            self.escape_status.value = "set a (reachable) goal first"
            return
        with self.ctx.ui_lock:
            if session.collision_state(q_goal).in_collision:
                self.escape_status.value = "goal is IN COLLISION — not a valid pocket to probe"
                return
            esc = session.escapability(q_goal, max_step=float(self.escape_step.value))
        self._report_escapability(esc)

    def _report_escapability(self, esc) -> None:
        step0 = float(esc.ladder[0])
        mx = esc.max_reach
        if mx <= step0 * 1.5:
            verdict = "WEDGED — goal barely escapable (pose/mesh calibration?)"
        elif mx < 0.15:
            verdict = f"TIGHT — try RRT extension step ≈ {mx:.2f}"
        else:
            verdict = "escapable — goal is not the bottleneck"
        thr = min(0.05, float(self.escape_step.value))
        self.escape_status.value = (
            f"max free {mx:.3f} rad ({esc.best_label}) · {esc.n_free_at(thr)} dirs ≥ {thr:.2f} · "
            f"{verdict}"
        )
        # Full per-joint table to the server console (the widget is one line).
        print("goal escapability — largest collision-free step per joint (rad):")
        for i, plus, minus in esc.per_joint():
            print(f"  j{i}: +{plus:.3f} / -{minus:.3f}")
        rand = [r for r, l in zip(esc.reach, esc.labels) if l == "rand"]
        if rand:
            print(f"  best of {len(rand)} random directions: {max(rand):.3f}")

    def probe_escapability_now(self):
        """Synchronous probe — the headless smoke-test entry point."""
        self._on_probe_escapability()
        return self.escape_status.value

    # ---- staleness (obstacle edits invalidate the session caches) ----------------------------

    def _on_scene_changed(self) -> None:
        session = self.ctx.session
        for status, valid in ((self.prm_status, session.has_roadmap),
                              (self.sdf_status, session.has_clearance_field)):
            if status.value != "—" and not valid and not status.value.endswith(STALE_SUFFIX):
                status.value += STALE_SUFFIX
        if not session.has_roadmap:  # the drawn roadmap describes the old environment
            self._clear_roadmap()

    # ---- clearance heatmap (R3 debug view) ---------------------------------------------------

    def build_clearance_now(self) -> None:
        """Synchronous build + slice — the headless smoke-test entry point."""
        self._on_build_clearance()

    def _on_build_clearance(self) -> None:
        ctx = self.ctx
        if not ctx.session.obstacles:
            self.sdf_status.value = "no obstacles — add environment first"
            return
        try:
            with ctx.ui_lock:
                # force=True: an explicit click always rebuilds at the requested resolution;
                # the field lands in the session cache the R4 refiner shares.
                self._field = ctx.session.clearance_field(
                    float(self.sdf_res.value) * 1e-3, force=True
                )
        except RuntimeError as error:
            self.sdf_status.value = f"FAILED: {error}"
            return
        f = self._field
        nx, ny, nz = int(f.dims[0]), int(f.dims[1]), int(f.dims[2])
        self.sdf_status.value = (
            f"{nx}×{ny}×{nz} vox · {f.build_seconds * 1e3:.0f} ms · "
            f"{'GPU' if f.built_on_gpu else 'CPU'} JFA"
        )
        self.draw_clearance_slice()

    def draw_clearance_slice(self) -> None:
        """One z-layer of the SDF as a colored point cloud: red = penetration/near, white = at
        the color-range edge, blue = far. The slider maps [0, 1] onto the grid height."""
        f = self._field
        if f is None:
            return
        if self._slice_node is not None:
            self._slice_node.remove()
            self._slice_node = None
        data = np.asarray(f.data)  # (nz, ny, nx) float32 view
        nz = data.shape[0]
        zi = min(int(float(self.sdf_slice.value) * (nz - 1)), nz - 1)
        layer = data[zi]  # (ny, nx)
        res = float(f.resolution)
        origin = np.asarray(f.origin)

        stride = max(1, int(np.ceil(np.sqrt(layer.size / 60000.0))))  # ≤ ~60k points
        ys, xs = np.mgrid[0 : layer.shape[0] : stride, 0 : layer.shape[1] : stride]
        d = layer[ys, xs].astype(np.float64).ravel()
        points = np.stack(
            [
                origin[0] + xs.ravel() * res,
                origin[1] + ys.ravel() * res,
                np.full(d.shape, origin[2] + zi * res),
            ],
            axis=1,
        )
        rng = max(float(self.sdf_range.value), 1e-6)
        t = np.clip(d / rng, -1.0, 1.0)
        colors = np.empty((len(d), 3))
        near = t < 0  # penetration → solid red fading to white at the surface
        colors[near] = np.stack([np.ones(near.sum()), 1 + t[near], 1 + t[near]], axis=1)
        colors[~near] = np.stack([1 - t[~near], 1 - t[~near], np.ones((~near).sum())], axis=1)
        self._slice_node = self.ctx.server.scene.add_point_cloud(
            "/clearance/slice", points=points, colors=colors, point_size=res * stride * 0.9,
        )
