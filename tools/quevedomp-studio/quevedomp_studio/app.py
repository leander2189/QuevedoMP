"""The studio UI: a viser server + working modes over a StudioSession (ADR-021).

Five modes — Scene (pose robot, edit obstacles), IK (gizmo/solve/branches), Plan (pick a
planner, plan, debug with trees + clearance heatmap), Trajectory (parameterize, polish,
play back), Tasks (MTC-lite inspector, R7 placeholder). Only the Session panel and the
mode switcher are always visible; the switcher toggles each mode's folder and scene nodes.

Shared state lives on StudioContext (context.py); the modes are in modes/. This class
keeps the server lifecycle, the Session panel, the switcher, and the headless synchronous
entry points the smoke tests drive (plan_now, refine_now, ...).
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import viser

from .context import StudioContext
from .modes import IkMode, PlanMode, SceneMode, TasksMode, TrajectoryMode
from .robot_view import ObstacleView, RobotView
from .session import Attempt, StudioSession

MODE_LABELS = {"scene": "Scene", "ik": "IK", "plan": "Plan",
               "trajectory": "Trajectory", "tasks": "Tasks"}


class StudioApp:
    def __init__(self, session: StudioSession, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.session = session
        self.server = viser.ViserServer(host=host, port=port, label="quevedomp-studio")
        self._current_mode = "scene"
        self._mount()

    def _mount(self) -> None:
        """Build (or rebuild, after load_session) the whole scene + GUI over self.session."""
        self.robot_view = RobotView(self.server, self.session)
        self.obstacle_view = ObstacleView(self.server, self.session)
        self.ctx = StudioContext(self.session, self.server, self.robot_view, self.obstacle_view)

        self._build_session_panel()
        # The switcher widget is created before the mode folders so it sits above them; its
        # handler is wired after the modes exist.
        self._mode_switcher = self.server.gui.add_button_group(
            "mode", options=tuple(MODE_LABELS.values())
        )

        self.scene = SceneMode(self.ctx)
        self.scene.build()  # renders any loaded obstacles too
        self.ik = IkMode(self.ctx)
        self.ik.build()
        self.plan = PlanMode(self.ctx)
        self.plan.build()
        self.trajectory = TrajectoryMode(self.ctx)
        self.trajectory.build()
        self.tasks = TasksMode(self.ctx)
        self.tasks.build()
        self._modes = [self.scene, self.ik, self.plan, self.trajectory, self.tasks]

        self._mode_switcher.on_click(lambda _e: self._on_mode_clicked())
        self._set_mode(self._current_mode)  # survives load_session remounts

        self.set_config(self.session.q)  # syncs sliders + collision tint
        self._restore_plan_markers()

    # ---- mode switching ----------------------------------------------------------------------

    def _on_mode_clicked(self) -> None:
        label = self._mode_switcher.value
        self._set_mode(next(k for k, v in MODE_LABELS.items() if v == label))

    def _set_mode(self, key: str) -> None:
        """THE visibility policy, kept in one place so a later swap to a viser tab group (if a
        tab-change event ever lands) stays a local change (ADR-021)."""
        self._current_mode = key
        for mode in self._modes:
            mode.set_active(mode.name == key)
        # The planned path is a result view: shown while planning or working on trajectories.
        self.ctx.attempt_view.set_path_visible(key in ("plan", "trajectory"))

    # ---- session panel + lifecycle -----------------------------------------------------------

    def load_session(self, path: str) -> None:
        """Swap in a saved session: tear down every scene node + GUI panel and remount."""
        if self.session.is_planning:
            raise RuntimeError("cannot load while a plan is running")
        new_session = StudioSession.load(path)
        for mode in getattr(self, "_modes", []):
            mode.shutdown()  # stop playback threads before the gui/scene reset
        with self.ctx.ui_lock:
            self.session = new_session
            self.server.scene.reset()
            self.server.gui.reset()
        self._mount()

    def _restore_plan_markers(self) -> None:
        goal = self.session.goal
        if goal is None:
            return
        view = self.ctx.attempt_view
        view.set_start_marker(self.session.start)
        if goal.kind == "joint" and goal.q is not None:
            view.set_goal_marker(goal.q)
        elif goal.pose is not None:
            self.ctx.ik_gizmo.position = goal.pose.translation()
            self.ctx.ik_gizmo.wxyz = goal.pose.quaternion()
            view.set_goal_marker_point(goal.pose.translation())

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
        self.session.timeout = float(self.plan.timeout.value)
        self.session.smooth = bool(self.plan.do_smooth.value)
        self.session.planner_params.edge_resolution = float(self.plan.edge_res.value)
        self.session.planner_params.max_link_sweep = float(self.plan.link_sweep.value) * 1e-3
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

    # ---- headless entry points (the documented scripting/test surface) -----------------------

    def set_config(self, q_new: np.ndarray) -> None:
        """Set config from code (IK / scrub / play): session + slider sync + robot refresh."""
        self.ctx.set_config(q_new)

    def refresh(self) -> None:
        self.ctx.refresh()

    def add_obstacle(self, oid: str, geometry, pose) -> None:
        self.scene.add_obstacle(oid, geometry, pose)

    def plan_now(self, seed: Optional[int] = None) -> Attempt:
        """Synchronous RRT plan + display."""
        return self.plan.plan_now(seed)

    def parametrize_now(self) -> None:
        """Synchronous parameterize + plots."""
        self.trajectory.parametrize()

    def refine_now(self) -> Attempt:
        """Synchronous CHOMP polish of the last plan."""
        return self.trajectory.refine_now()

    def build_clearance_now(self) -> None:
        """Synchronous clearance-field build + heatmap slice."""
        self.plan.build_clearance_now()

    def build_roadmap_now(self):
        """Synchronous PRM roadmap build."""
        return self.plan.build_roadmap_now()

    def query_roadmap_now(self) -> Attempt:
        """Synchronous PRM build-if-needed + query."""
        return self.plan.query_roadmap_now()

    def probe_escapability_now(self):
        """Synchronous goal-escapability probe."""
        return self.plan.probe_escapability_now()

    def play(self, blocking: bool = False, duration: Optional[float] = None) -> None:
        self.trajectory.play(blocking=blocking, duration=duration)

    def play_timed(self, blocking: bool = False) -> None:
        self.trajectory.play_timed(blocking=blocking)

    def serve_forever(self) -> None:
        print(f"quevedomp-studio running — open http://localhost:{self.server.get_port()}")
        while True:
            time.sleep(3600)
