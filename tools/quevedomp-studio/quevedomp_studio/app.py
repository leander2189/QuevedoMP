"""The studio UI: viser widgets over a StudioSession.

Flow: joint sliders / IK gizmo edit the config (FK + collision tint update live) -> Set start /
Set goal capture configurations -> Plan runs on a worker thread (the bindings drop the GIL) ->
the result draws as an end-effector trace plus a scrub slider that animates the robot along
the (smoothed) path.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import viser

import quevedomp as q

from .robot_view import ObstacleView, RobotView
from .session import Attempt, StudioSession

PATH_COLOR = (80, 140, 240)
START_COLOR = (80, 200, 80)
GOAL_COLOR = (80, 120, 240)


class StudioApp:
    def __init__(self, session: StudioSession, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.session = session
        self.server = viser.ViserServer(host=host, port=port, label="quevedomp-studio")
        self.robot_view = RobotView(self.server, session)
        self.obstacle_view = ObstacleView(self.server, session)
        self._path_nodes: list = []
        self._obstacle_counter = 0

        self._build_robot_panel()
        self._build_ik_panel()
        self._build_obstacle_panel()
        self._build_planning_panel()
        self.refresh()

    # ---- Robot panel -----------------------------------------------------------------------

    def _build_robot_panel(self) -> None:
        self.sliders = []
        with self.server.gui.add_folder("Robot"):
            self.status = self.server.gui.add_text("status", initial_value="ready", disabled=True)
            for joint in self.session.movable_joints():
                limits = joint.limits
                lo, hi = (limits.lower, limits.upper) if limits.has_position_limit else (-np.pi, np.pi)
                slider = self.server.gui.add_slider(
                    joint.name, min=float(lo), max=float(hi), step=(hi - lo) / 200.0,
                    initial_value=float(np.clip(0.0, lo, hi)),
                )
                slider.on_update(lambda _e: self._on_sliders())
                self.sliders.append(slider)

    def _on_sliders(self) -> None:
        self.session.set_config(np.array([s.value for s in self.sliders]))
        self.refresh()

    def set_config(self, q_new: np.ndarray) -> None:
        """Set config from code (IK / scrub): syncs sliders without feedback loops."""
        self.session.set_config(q_new)
        for slider, value in zip(self.sliders, q_new):
            slider.value = float(value)
        self.refresh()

    def refresh(self) -> None:
        state = self.robot_view.update_config(self.session.q)
        if state.in_collision:
            witness = f" ({state.witness.a} ↔ {state.witness.b})" if state.witness else ""
            self.status.value = f"COLLIDING{witness}"
        else:
            self.status.value = f"free · clearance {state.min_distance:.3f} m"

    # ---- IK panel --------------------------------------------------------------------------

    def _build_ik_panel(self) -> None:
        links = self.session.ik_links() or self.session.leaf_links() or [self.session.model.root_link]
        with self.server.gui.add_folder("IK"):
            self.ik_link = self.server.gui.add_dropdown("link", options=tuple(links),
                                                        initial_value=links[0])
            self.ik_status = self.server.gui.add_text("ik", initial_value="—", disabled=True)
            snap = self.server.gui.add_button("Snap gizmo to link")

        self.ik_gizmo = self.server.scene.add_transform_controls("/ik_target", scale=0.25)
        self._snap_gizmo()

        snap.on_click(lambda _e: self._snap_gizmo())
        self.ik_gizmo.on_update(lambda _e: self._on_ik_gizmo())
        self.ik_link.on_update(lambda _e: self._snap_gizmo())

    def _snap_gizmo(self) -> None:
        pose = q.fk(self.session.model, self.session.q, self.ik_link.value)
        self.ik_gizmo.position = pose.translation()
        self.ik_gizmo.wxyz = pose.quaternion()

    def _on_ik_gizmo(self) -> None:
        target = q.Transform.from_parts(
            np.asarray(self.ik_gizmo.position), np.asarray(self.ik_gizmo.wxyz)
        )
        result = self.session.solve_ik(self.ik_link.value, target)
        if result.success:
            self.ik_status.value = (
                f"ok · {result.iterations} iters · pos {result.pos_error * 1e3:.2f} mm"
            )
            self.set_config(result.q)
        else:
            self.ik_status.value = f"FAILED · pos {result.pos_error * 1e3:.1f} mm"

    # ---- Obstacles -------------------------------------------------------------------------

    def _build_obstacle_panel(self) -> None:
        with self.server.gui.add_folder("Obstacles"):
            kind = self.server.gui.add_dropdown("kind", options=("box", "sphere", "cylinder"),
                                                initial_value="box")
            size = self.server.gui.add_number("size (m)", initial_value=0.2, min=0.01, max=2.0)
            add = self.server.gui.add_button("Add obstacle")
            mesh_path = self.server.gui.add_text("mesh file", initial_value="")
            add_mesh = self.server.gui.add_button("Add mesh obstacle")
            self.obstacle_status = self.server.gui.add_text("obstacles", initial_value="—",
                                                            disabled=True)
            self.obstacle_pick = self.server.gui.add_dropdown("selected", options=("—",),
                                                              initial_value="—")
            remove = self.server.gui.add_button("Remove selected")

        def on_add(_e) -> None:
            self._obstacle_counter += 1
            oid = f"{kind.value}_{self._obstacle_counter}"
            s = float(size.value)
            geometry = {
                "box": lambda: q.BoxShape(np.full(3, s / 2.0)),
                "sphere": lambda: q.SphereShape(s / 2.0),
                "cylinder": lambda: q.CylinderShape(s / 4.0, s),
            }[kind.value]()
            self.add_obstacle(oid, geometry, q.Transform.from_translation(np.array([0.5, 0.0, s / 2.0])))

        def on_add_mesh(_e) -> None:
            path = mesh_path.value.strip()
            if not path:
                self.obstacle_status.value = "enter a mesh path (STL/DAE/OBJ)"
                return
            try:
                mesh = q.load_mesh(path)  # normalized to metres, GIL released
            except RuntimeError as error:
                self.obstacle_status.value = f"load failed: {error}"
                return
            self._obstacle_counter += 1
            oid = f"mesh_{self._obstacle_counter}"
            self.add_obstacle(oid, mesh, q.Transform.from_translation(np.array([0.5, 0.0, 0.0])))
            self.obstacle_status.value = f"{oid}: {mesh.vertices.shape[0]} verts"

        def on_remove(_e) -> None:
            oid = self.obstacle_pick.value
            if oid in self.session.obstacles:
                self.session.remove_obstacle(oid)
                self.obstacle_view.remove(oid)
                self._sync_obstacle_pick()
                self.refresh()

        add.on_click(on_add)
        add_mesh.on_click(on_add_mesh)
        remove.on_click(on_remove)

    def add_obstacle(self, oid: str, geometry, pose) -> None:
        obstacle = self.session.add_obstacle(oid, geometry, pose)
        self.obstacle_view.add(obstacle, on_moved=lambda _id: self.refresh())
        self._sync_obstacle_pick()
        self.refresh()

    def _sync_obstacle_pick(self) -> None:
        ids = tuple(self.session.obstacles) or ("—",)
        self.obstacle_pick.options = ids
        self.obstacle_pick.value = ids[-1]

    # ---- Planning --------------------------------------------------------------------------

    def _build_planning_panel(self) -> None:
        with self.server.gui.add_folder("Planning"):
            set_start = self.server.gui.add_button("Set start = current")
            set_goal = self.server.gui.add_button("Set goal = current")
            self.use_ik_goal = self.server.gui.add_checkbox("goal = IK gizmo pose", initial_value=False)
            self.timeout = self.server.gui.add_number("timeout (s)", initial_value=2.0, min=0.1, max=60.0)
            self.seed = self.server.gui.add_number("seed (0 = auto)", initial_value=0, min=0, max=2**31)
            self.do_smooth = self.server.gui.add_checkbox("shortcut smoothing", initial_value=True)
            self.plan_button = self.server.gui.add_button("Plan")
            self.plan_status = self.server.gui.add_text("result", initial_value="—", disabled=True)
            self.scrub = self.server.gui.add_slider("scrub", min=0.0, max=1.0, step=0.005,
                                                    initial_value=0.0)

        set_start.on_click(lambda _e: self._set_start())
        set_goal.on_click(lambda _e: self._set_goal())
        self.plan_button.on_click(lambda _e: self._on_plan_clicked())
        self.scrub.on_update(lambda _e: self._on_scrub())
        self._start_marker: Optional[object] = None
        self._goal_marker: Optional[object] = None
        self._last_attempt: Optional[Attempt] = None

    def _ee(self) -> str:
        return self.ik_link.value

    def _marker(self, name: str, q_at: np.ndarray, color) -> object:
        p = q.fk(self.session.model, q_at, self._ee()).translation()
        return self.server.scene.add_point_cloud(
            name, points=p[None, :], colors=np.array([color]), point_size=0.025
        )

    def _set_start(self) -> None:
        self.session.set_start()
        if self._start_marker is not None:
            self._start_marker.remove()
        self._start_marker = self._marker("/plan/start", self.session.start, START_COLOR)

    def _set_goal(self) -> None:
        if self.use_ik_goal.value:
            target = q.Transform.from_parts(
                np.asarray(self.ik_gizmo.position), np.asarray(self.ik_gizmo.wxyz)
            )
            self.session.set_goal_pose(self._ee(), target)
            goal_preview = self.session.q
        else:
            self.session.set_goal_joints()
            goal_preview = self.session.q
        if self._goal_marker is not None:
            self._goal_marker.remove()
        self._goal_marker = self._marker("/plan/goal", goal_preview, GOAL_COLOR)

    def _on_plan_clicked(self) -> None:
        if self.session.is_planning:
            return
        self.session.timeout = float(self.timeout.value)
        self.session.smooth = bool(self.do_smooth.value)
        seed = int(self.seed.value) or None
        self.plan_status.value = "planning…"
        self.plan_button.disabled = True
        self.session.plan_async(self._on_plan_done, seed=seed)

    def plan_now(self, seed: Optional[int] = None) -> Attempt:
        """Synchronous plan + display — the headless smoke-test entry point."""
        attempt = self.session.plan(seed)
        self._show_attempt(attempt)
        return attempt

    def _on_plan_done(self, attempt: Optional[Attempt]) -> None:
        try:
            if attempt is None:
                self.plan_status.value = "ERROR — see server console"
            else:
                self._show_attempt(attempt)
        finally:
            self.plan_button.disabled = False

    def _show_attempt(self, attempt: Attempt) -> None:
        self._last_attempt = attempt
        r = attempt.result
        stats = r.stats
        if r.ok():
            self.plan_status.value = (
                f"{r.status} · {len(attempt.path)} wp · {stats.time_total * 1e3:.0f} ms · "
                f"{stats.collision_configs} configs · seed {r.used_seed}"
            )
            self._draw_path(attempt.path)
            self.scrub.value = 0.0
        else:
            self.plan_status.value = f"{r.status} · {r.message} · seed {r.used_seed}"
            self._clear_path()

    def _draw_path(self, path) -> None:
        self._clear_path()
        model, ee = self.session.model, self._ee()
        points = np.array([q.fk(model, w, ee).translation() for w in path])
        self._path_nodes.append(
            self.server.scene.add_point_cloud(
                "/plan/path_points", points=points,
                colors=np.tile(np.array([PATH_COLOR]), (len(points), 1)), point_size=0.008,
            )
        )
        segments = np.stack([points[:-1], points[1:]], axis=1)
        self._path_nodes.append(
            self.server.scene.add_line_segments(
                "/plan/path", points=segments,
                colors=np.tile(np.array([PATH_COLOR]), (len(segments), 2, 1)),
            )
        )

    def _clear_path(self) -> None:
        for node in self._path_nodes:
            node.remove()
        self._path_nodes = []

    def _on_scrub(self) -> None:
        attempt = self._last_attempt
        if attempt is None or len(attempt.path) < 2:
            return
        t = float(self.scrub.value) * (len(attempt.path) - 1)
        i = min(int(t), len(attempt.path) - 2)
        frac = t - i
        q_at = (1.0 - frac) * attempt.path[i] + frac * attempt.path[i + 1]
        self.set_config(q_at)

    # ---- Lifecycle -------------------------------------------------------------------------

    def serve_forever(self) -> None:
        print(f"quevedomp-studio running — open http://localhost:{self.server.get_port()}")
        while True:
            time.sleep(3600)
