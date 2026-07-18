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

            self.chomp_folder = gui.add_folder("CHOMP (standalone)")
            with self.chomp_folder:
                self.chomp = ChompParamsWidgets(gui)

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

        set_start.on_click(lambda _e: self.set_start())
        set_goal.on_click(lambda _e: self.set_goal())
        self.plan_button.on_click(lambda _e: self._on_plan_clicked())
        self.planner_pick.on_update(lambda _e: self._sync_planner_sections())
        self.prm_build_button.on_click(lambda _e: self._on_build_roadmap())
        self.sdf_build_button.on_click(lambda _e: self._on_build_clearance())
        self.sdf_slice.on_update(lambda _e: self.draw_clearance_slice())
        self.sdf_range.on_update(lambda _e: self.draw_clearance_slice())
        ctx.scene_changed_listeners.append(self._on_scene_changed)
        self._sync_planner_sections()

    def set_active(self, active: bool) -> None:
        super().set_active(active)
        self.ctx.attempt_view.set_trees_visible(active)  # exploration trees are a debug view
        if self._slice_node is not None:
            self._slice_node.visible = active

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

    # ---- the Plan dispatch -------------------------------------------------------------------

    def _planner_key(self) -> str:
        label = self.planner_pick.value
        return next(k for k, v in PLANNER_LABELS.items() if v == label)

    def _sync_planner_sections(self) -> None:
        kind = self._planner_key()
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
                self.prm_status.value = (
                    f"{stats.nodes} nodes · {stats.edges} edges · "
                    f"{stats.collision_configs} configs · {stats.build_seconds * 1e3:.0f} ms"
                )
        finally:
            self.prm_build_button.disabled = False

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

    # ---- staleness (obstacle edits invalidate the session caches) ----------------------------

    def _on_scene_changed(self) -> None:
        session = self.ctx.session
        for status, valid in ((self.prm_status, session.has_roadmap),
                              (self.sdf_status, session.has_clearance_field)):
            if status.value != "—" and not valid and not status.value.endswith(STALE_SUFFIX):
                status.value += STALE_SUFFIX

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
