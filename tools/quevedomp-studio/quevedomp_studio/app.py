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

from .context import StudioContext
from .modes import IkMode, PlanMode, SceneMode, TrajectoryMode
from .robot_view import ObstacleView, RobotView
from .session import Attempt, StudioSession


class StudioApp:
    def __init__(self, session: StudioSession, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.session = session
        self.server = viser.ViserServer(host=host, port=port, label="quevedomp-studio")
        self._mount()

    def _mount(self) -> None:
        """Build (or rebuild, after load_session) the whole scene + GUI over self.session."""
        self.robot_view = RobotView(self.server, self.session)
        self.obstacle_view = ObstacleView(self.server, self.session)
        self.ctx = StudioContext(self.session, self.server, self.robot_view, self.obstacle_view)

        self._build_session_panel()
        self.scene = SceneMode(self.ctx)
        self.scene.build()  # renders any loaded obstacles too
        self.ik = IkMode(self.ctx)
        self.ik.build()
        self.plan = PlanMode(self.ctx)
        self.plan.build()
        self.trajectory = TrajectoryMode(self.ctx)
        self.trajectory.build()
        self._modes = [self.scene, self.ik, self.plan, self.trajectory]

        self.set_config(self.session.q)  # syncs sliders + collision tint
        self._restore_plan_markers()

    def load_session(self, path: str) -> None:
        """Swap in a saved session: tear down every scene node + GUI panel and remount."""
        if self.session.is_planning:
            raise RuntimeError("cannot load while a plan is running")
        new_session = StudioSession.load(path)
        for mode in getattr(self, "_modes", []):
            mode.shutdown()  # stop playback threads before the gui/scene reset
        with self._ui_lock:
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

    # ---- Shared-state aliases (ADR-021 transition: the modes read ctx directly) --------------

    @property
    def _ui_lock(self) -> threading.Lock:
        return self.ctx.ui_lock

    @property
    def ik_gizmo(self):
        return self.ctx.ik_gizmo

    @property
    def _last_attempt(self) -> Optional[Attempt]:
        return self.ctx.last_attempt

    @_last_attempt.setter
    def _last_attempt(self, attempt: Optional[Attempt]) -> None:
        self.ctx.last_attempt = attempt

    @property
    def _path_nodes(self) -> list:
        return self.ctx.attempt_view._path_nodes

    @property
    def _tree_nodes(self) -> list:
        return self.ctx.attempt_view._tree_nodes

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

    # ---- Scene-mode aliases (ADR-021 transition) ---------------------------------------------

    @property
    def sliders(self) -> list:
        return self.scene.sliders

    @property
    def status(self):
        return self.scene.status

    @property
    def obstacle_status(self):
        return self.scene.obstacle_status

    @property
    def obstacle_pick(self):
        return self.scene.obstacle_pick

    def add_obstacle(self, oid: str, geometry, pose) -> None:
        self.scene.add_obstacle(oid, geometry, pose)

    def _sync_obstacle_pick(self) -> None:
        self.scene._sync_obstacle_pick()

    def set_config(self, q_new: np.ndarray) -> None:
        """Set config from code (IK / scrub / play): session + slider sync + robot refresh."""
        self.ctx.set_config(q_new)

    def refresh(self) -> None:
        self.ctx.refresh()

    # ---- IK-mode aliases (ADR-021 transition) ------------------------------------------------

    @property
    def ik_link(self):
        return self.ik.link

    @property
    def ik_status(self):
        return self.ik.status

    @property
    def ik_branch_pick(self):
        return self.ik.branch_pick

    @property
    def _ik_branches(self) -> list:
        return self.ik.branches

    def _snap_gizmo(self) -> None:
        self.ik.snap_gizmo()

    def _on_ik_branches(self) -> None:
        self.ik.solve_branches()

    def _on_ik_branch_pick(self) -> None:
        self.ik.on_branch_pick()

    # ---- Plan-mode aliases (ADR-021 transition) ----------------------------------------------

    @property
    def use_ik_goal(self):
        return self.plan.use_ik_goal

    @property
    def timeout(self):
        return self.plan.timeout

    @property
    def seed(self):
        return self.plan.seed

    @property
    def edge_res(self):
        return self.plan.edge_res

    @property
    def link_sweep(self):
        return self.plan.link_sweep

    @property
    def do_smooth(self):
        return self.plan.do_smooth

    @property
    def show_tree(self):
        return self.plan.show_tree

    @property
    def plan_status(self):
        return self.plan.status

    @property
    def prm_nodes(self):
        return self.plan.prm_nodes

    @property
    def prm_k(self):
        return self.plan.prm_k

    @property
    def prm_seed(self):
        return self.plan.prm_seed

    @property
    def prm_status(self):
        return self.plan.prm_status

    @property
    def sdf_res(self):
        return self.plan.sdf_res

    @property
    def sdf_status(self):
        return self.plan.sdf_status

    @property
    def sdf_slice(self):
        return self.plan.sdf_slice

    @property
    def _slice_node(self):
        return self.plan._slice_node

    def _set_start(self) -> None:
        self.plan.set_start()

    def _set_goal(self) -> None:
        self.plan.set_goal()

    def plan_now(self, seed: Optional[int] = None) -> Attempt:
        """Synchronous plan + display — the headless smoke-test entry point."""
        return self.plan.plan_now(seed)

    def build_clearance_now(self) -> None:
        """Synchronous clearance-field build + slice — the headless smoke-test entry point."""
        self.plan.build_clearance_now()

    def _draw_clearance_slice(self) -> None:
        self.plan.draw_clearance_slice()

    def build_roadmap_now(self):
        """Synchronous roadmap build — the headless smoke-test entry point."""
        return self.plan.build_roadmap_now()

    def query_roadmap_now(self) -> Attempt:
        """Synchronous roadmap build-if-needed + query — the headless smoke-test entry point."""
        return self.plan.query_roadmap_now()

    # ---- Trajectory-mode aliases (ADR-021 transition) ----------------------------------------

    @property
    def traj_accel(self):
        return self.trajectory.accel

    @property
    def traj_tip_speed(self):
        return self.trajectory.tip_speed

    @property
    def traj_tip_accel(self):
        return self.trajectory.tip_accel

    @property
    def traj_jerk(self):
        return self.trajectory.jerk

    @property
    def traj_status(self):
        return self.trajectory.status

    @property
    def time_scale(self):
        return self.trajectory.time_scale

    @property
    def scrub(self):
        return self.trajectory.scrub

    @property
    def _plot_handles(self) -> list:
        return self.trajectory._plot_handles

    @property
    def _playing(self) -> bool:
        return self.trajectory._playing

    @property
    def refine_waypoints(self):
        return self.trajectory.chomp.waypoints

    @property
    def refine_iters(self):
        return self.trajectory.chomp.iterations

    @property
    def refine_status(self):
        return self.trajectory.refine_status

    def parametrize_now(self) -> None:
        """Synchronous parameterize + plots — the headless smoke-test entry point."""
        self.trajectory.parametrize()

    def refine_now(self) -> Attempt:
        """Synchronous CHOMP polish of the last plan — the headless smoke-test entry point."""
        return self.trajectory.refine_now()

    def _on_scrub(self) -> None:
        self.trajectory._on_scrub()

    def play(self, blocking: bool = False, duration: Optional[float] = None) -> None:
        self.trajectory.play(blocking=blocking, duration=duration)

    def play_timed(self, blocking: bool = False) -> None:
        self.trajectory.play_timed(blocking=blocking)

    # ---- Lifecycle -------------------------------------------------------------------------

    def serve_forever(self) -> None:
        print(f"quevedomp-studio running — open http://localhost:{self.server.get_port()}")
        while True:
            time.sleep(3600)
