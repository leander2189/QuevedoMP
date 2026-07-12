"""The studio UI: viser widgets over a StudioSession.

Flow: joint sliders / IK gizmo edit the config (FK + collision tint update live) -> Set start /
Set goal capture configurations -> Plan runs on a worker thread (the bindings drop the GIL) ->
the result draws as an end-effector trace plus a scrub slider that animates the robot along
the (smoothed) path.
"""

from __future__ import annotations

import threading
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
        # viser callbacks can run concurrently; the session's UI workspace is one-per-thread
        # (ADR-005), so every query/IK from a callback goes through this lock.
        self._ui_lock = threading.Lock()
        self._mount()

    def _mount(self) -> None:
        """Build (or rebuild, after load_session) the whole scene + GUI over self.session."""
        self.robot_view = RobotView(self.server, self.session)
        self.obstacle_view = ObstacleView(self.server, self.session)
        self._path_nodes: list = []
        self._obstacle_counter = len(self.session.obstacles)
        self._playing = False
        self._syncing_sliders = False

        self._build_session_panel()
        self._build_robot_panel()
        self._build_ik_panel()
        self._build_obstacle_panel()
        self._build_planning_panel()

        # A loaded session already carries obstacles + a configured problem: render them.
        for obstacle in self.session.obstacles.values():
            self.obstacle_view.add(obstacle, on_moved=lambda _id: self.refresh())
        if self.session.obstacles:
            self._sync_obstacle_pick()
        self.set_config(self.session.q)  # syncs sliders + collision tint
        self._restore_plan_markers()

    def load_session(self, path: str) -> None:
        """Swap in a saved session: tear down every scene node + GUI panel and remount."""
        if self.session.is_planning:
            raise RuntimeError("cannot load while a plan is running")
        new_session = StudioSession.load(path)
        self._playing = False
        with self._ui_lock:
            self.session = new_session
            self.server.scene.reset()
            self.server.gui.reset()
        self._mount()

    def _restore_plan_markers(self) -> None:
        goal = self.session.goal
        if goal is None:
            return
        self._start_marker = self._marker("/plan/start", self.session.start, START_COLOR)
        if goal.kind == "joint" and goal.q is not None:
            self._goal_marker = self._marker("/plan/goal", goal.q, GOAL_COLOR)
        elif goal.pose is not None:
            self.ik_gizmo.position = goal.pose.translation()
            self.ik_gizmo.wxyz = goal.pose.quaternion()
            self._goal_marker = self.server.scene.add_point_cloud(
                "/plan/goal", points=goal.pose.translation()[None, :],
                colors=np.array([GOAL_COLOR]), point_size=0.025,
            )

    # ---- Session panel -----------------------------------------------------------------------

    def _build_session_panel(self) -> None:
        with self.server.gui.add_folder("Session"):
            self.session_path = self.server.gui.add_text("file", initial_value="sessions/scene.qmps")
            save_btn = self.server.gui.add_button("Save session")
            load_btn = self.server.gui.add_button("Load session")
            self.session_status = self.server.gui.add_text("io", initial_value="—", disabled=True)

        save_btn.on_click(lambda _e: self._on_save_session())
        load_btn.on_click(lambda _e: self._on_load_session())

    def _on_save_session(self) -> None:
        # Pull the live widget values into the session so the file captures what's on screen.
        self.session.timeout = float(self.timeout.value)
        self.session.smooth = bool(self.do_smooth.value)
        self.session.planner_params.edge_resolution = float(self.edge_res.value)
        self.session.planner_params.max_link_sweep = float(self.link_sweep.value) * 1e-3
        try:
            self.session.save(self.session_path.value.strip())
            self.session_status.value = f"saved {self.session_path.value.strip()}"
        except OSError as error:
            self.session_status.value = f"save failed: {error}"

    def _on_load_session(self) -> None:
        path = self.session_path.value.strip()
        try:
            self.load_session(path)
        except (OSError, ValueError, RuntimeError) as error:
            self.session_status.value = f"load failed: {error}"
            return
        # _mount rebuilt every widget, including session_status — set the fresh one.
        self.session_path.value = path
        self.session_status.value = f"loaded {path}"

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
        # Programmatic slider writes (set_config) fire this too — mid-sync the slider set is
        # half new / half stale, and rebuilding the config from it walks the robot through
        # mixed configurations (the "scrub looks like IK" bug). The guard drops those events.
        if self._syncing_sliders:
            return
        self.session.set_config(np.array([s.value for s in self.sliders]))
        self.refresh()

    def set_config(self, q_new: np.ndarray) -> None:
        """Set config from code (IK / scrub / play): syncs sliders without feedback loops."""
        self.session.set_config(q_new)
        self._syncing_sliders = True
        try:
            for slider, value in zip(self.sliders, q_new):
                slider.value = float(value)
        finally:
            self._syncing_sliders = False
        self.refresh()

    def refresh(self) -> None:
        with self._ui_lock:
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
            solve_global = self.server.gui.add_button("Solve (global, multi-restart)")

        self.ik_gizmo = self.server.scene.add_transform_controls("/ik_target", scale=0.25)
        self._ik_busy = False
        self._snap_gizmo()

        snap.on_click(lambda _e: self._snap_gizmo())
        solve_global.on_click(lambda _e: self._on_ik_solve(interactive=False))
        self.ik_gizmo.on_update(lambda _e: self._on_ik_gizmo())
        self.ik_link.on_update(lambda _e: self._snap_gizmo())

    def _snap_gizmo(self) -> None:
        pose = q.fk(self.session.model, self.session.q, self.ik_link.value)
        self.ik_gizmo.position = pose.translation()
        self.ik_gizmo.wxyz = pose.quaternion()

    def _on_ik_gizmo(self) -> None:
        # Drop events that arrive while a solve is running (a drag emits dozens); the next
        # event re-solves from the latest gizmo pose, so tracking never falls behind.
        if self._ik_busy:
            return
        self._on_ik_solve(interactive=True)

    def _on_ik_solve(self, interactive: bool) -> None:
        self._ik_busy = True
        try:
            target = q.Transform.from_parts(
                np.asarray(self.ik_gizmo.position), np.asarray(self.ik_gizmo.wxyz)
            )
            with self._ui_lock:
                result = self.session.solve_ik(self.ik_link.value, target,
                                               interactive=interactive)
            if result.success:
                mode = "track" if interactive else "global"
                self.ik_status.value = (
                    f"{mode} ok · {result.iterations} iters · pos {result.pos_error * 1e3:.2f} mm"
                )
                self.set_config(result.q)
            elif interactive:
                # Tracking failure = out of reach from the current branch: hold the pose
                # instead of teleporting; "Solve (global)" is the explicit branch switch.
                self.ik_status.value = f"out of reach · pos {result.pos_error * 1e3:.1f} mm"
            else:
                self.ik_status.value = f"global FAILED · pos {result.pos_error * 1e3:.1f} mm"
        finally:
            self._ik_busy = False

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
            self.timeout = self.server.gui.add_number("timeout (s)", initial_value=float(self.session.timeout),
                                                      min=0.1, max=60.0)
            self.seed = self.server.gui.add_number("seed (0 = auto)", initial_value=0, min=0, max=2**31)
            self.edge_res = self.server.gui.add_number(
                "edge check step (rad|m)", initial_value=self.session.planner_params.edge_resolution,
                min=0.001, max=0.2, step=0.001,
            )
            self.link_sweep = self.server.gui.add_number(
                "max link sweep (mm, 0 = off)",  # P3: workspace-bounded edge steps; overrides ↑
                initial_value=self.session.planner_params.max_link_sweep * 1e3,
                min=0.0, max=100.0, step=0.5,
            )
            self.do_smooth = self.server.gui.add_checkbox("shortcut smoothing",
                                                          initial_value=bool(self.session.smooth))
            self.plan_button = self.server.gui.add_button("Plan")
            self.plan_status = self.server.gui.add_text("result", initial_value="—", disabled=True)
            self.scrub = self.server.gui.add_slider("scrub", min=0.0, max=1.0, step=0.002,
                                                    initial_value=0.0)
            self.play_button = self.server.gui.add_button("▶ Play")
            self.play_speed = self.server.gui.add_number("play speed (rad/s)", initial_value=0.5,
                                                         min=0.05, max=5.0)

        set_start.on_click(lambda _e: self._set_start())
        set_goal.on_click(lambda _e: self._set_goal())
        self.plan_button.on_click(lambda _e: self._on_plan_clicked())
        self.play_button.on_click(lambda _e: self._on_play_clicked())
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
        self.session.planner_params.edge_resolution = float(self.edge_res.value)
        self.session.planner_params.max_link_sweep = float(self.link_sweep.value) * 1e-3
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
        # Waypoint markers show the plan's structure; the curve is the TRUE end-effector path —
        # FK sampled along the joint-space interpolation, not a straight polyline between
        # waypoints (in Cartesian space the EE arcs between joint-space waypoints).
        waypoints = np.array([q.fk(model, w, ee).translation() for w in path])
        dense = self.session.sample_path(path, samples_per_segment=12)
        curve = np.array([q.fk(model, w, ee).translation() for w in dense])
        self._path_nodes.append(
            self.server.scene.add_point_cloud(
                "/plan/waypoints", points=waypoints,
                colors=np.tile(np.array([PATH_COLOR]), (len(waypoints), 1)), point_size=0.008,
            )
        )
        segments = np.stack([curve[:-1], curve[1:]], axis=1)
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

    # ---- Play ------------------------------------------------------------------------------

    def _on_play_clicked(self) -> None:
        if self._playing:
            self._playing = False  # acts as a Stop button while running
            return
        self.play(blocking=False)

    def play(self, blocking: bool = False, duration: Optional[float] = None) -> None:
        """Animate the robot along the last path at constant joint speed via the scrub slider."""
        attempt = self._last_attempt
        if attempt is None or len(attempt.path) < 2 or self._playing:
            return
        if duration is None:
            length = sum(
                float(np.max(np.abs(b - a))) for a, b in zip(attempt.path, attempt.path[1:])
            )
            duration = float(np.clip(length / float(self.play_speed.value), 0.5, 60.0))

        def set_label(text: str) -> None:
            try:
                self.play_button.label = text
            except AttributeError:  # older viser: label immutable — Play just re-runs
                pass

        def run() -> None:
            self._playing = True
            start = time.monotonic()
            try:
                set_label("■ Stop")
                while self._playing:
                    t = (time.monotonic() - start) / duration
                    self.scrub.value = min(t, 1.0)
                    self._on_scrub()
                    if t >= 1.0:
                        break
                    time.sleep(1.0 / 30.0)
            finally:
                self._playing = False
                set_label("▶ Play")

        if blocking:
            run()
        else:
            threading.Thread(target=run, name="quevedomp-play", daemon=True).start()

    # ---- Lifecycle -------------------------------------------------------------------------

    def serve_forever(self) -> None:
        print(f"quevedomp-studio running — open http://localhost:{self.server.get_port()}")
        while True:
            time.sleep(3600)
