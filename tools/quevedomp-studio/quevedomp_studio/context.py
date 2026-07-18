"""Shared studio state: StudioContext + AttemptView (ADR-021).

The context is the seam between the working modes: it owns everything more than one mode
needs — the session, the UI lock (ADR-005: one workspace per thread), the current
end-effector link, the IK gizmo, the last attempt and its drawing — plus three small
listener lists. It is deliberately a plain object, not an event bus: modes read it
directly and register a callback only where a *reaction* is needed (slider sync,
staleness display, timing invalidation).

Rebuilt from scratch on every StudioApp._mount, so load_session stays trivially correct.
"""

from __future__ import annotations

import threading
from typing import Callable, Optional

import numpy as np
import viser

import quevedomp as q

from .robot_view import ObstacleView, RobotView
from .session import Attempt, StudioSession

PATH_COLOR = (80, 140, 240)
START_COLOR = (80, 200, 80)
GOAL_COLOR = (80, 120, 240)
TREE_COLORS = ((110, 170, 255), (255, 170, 90))  # start tree, goal tree (R2 exploration view)


class AttemptView:
    """Scene-side drawing of the current attempt: start/goal markers, the true EE path curve,
    and the R2 exploration trees. Owns its nodes; visibility is mode-controlled."""

    def __init__(self, ctx: "StudioContext") -> None:
        self.ctx = ctx
        self._path_nodes: list = []
        self._tree_nodes: list = []
        self._start_marker: Optional[object] = None
        self._goal_marker: Optional[object] = None
        self._path_visible = True
        self._trees_visible = True

    # ---- markers ----------------------------------------------------------------------------

    def _marker(self, name: str, q_at: np.ndarray, color) -> object:
        p = q.fk(self.ctx.session.model, q_at, self.ctx.ee_link).translation()
        return self.ctx.server.scene.add_point_cloud(
            name, points=p[None, :], colors=np.array([color]), point_size=0.025
        )

    def set_start_marker(self, q_at: np.ndarray) -> None:
        if self._start_marker is not None:
            self._start_marker.remove()
        self._start_marker = self._marker("/plan/start", q_at, START_COLOR)

    def set_goal_marker(self, q_at: np.ndarray) -> None:
        if self._goal_marker is not None:
            self._goal_marker.remove()
        self._goal_marker = self._marker("/plan/goal", q_at, GOAL_COLOR)

    def set_goal_marker_point(self, point: np.ndarray) -> None:
        """Pose goals restored from a session draw at the target translation directly."""
        if self._goal_marker is not None:
            self._goal_marker.remove()
        self._goal_marker = self.ctx.server.scene.add_point_cloud(
            "/plan/goal", points=np.asarray(point)[None, :],
            colors=np.array([GOAL_COLOR]), point_size=0.025,
        )

    # ---- path -------------------------------------------------------------------------------

    def draw_path(self, path) -> None:
        self.clear_path()
        session = self.ctx.session
        model, ee = session.model, self.ctx.ee_link
        # Waypoint markers show the plan's structure; the curve is the TRUE end-effector path —
        # FK sampled along the joint-space interpolation, not a straight polyline between
        # waypoints (in Cartesian space the EE arcs between joint-space waypoints).
        waypoints = np.array([q.fk(model, w, ee).translation() for w in path])
        dense = session.sample_path(path, samples_per_segment=12)
        curve = np.array([q.fk(model, w, ee).translation() for w in dense])
        self._path_nodes.append(
            self.ctx.server.scene.add_point_cloud(
                "/plan/waypoints", points=waypoints,
                colors=np.tile(np.array([PATH_COLOR]), (len(waypoints), 1)), point_size=0.008,
            )
        )
        segments = np.stack([curve[:-1], curve[1:]], axis=1)
        self._path_nodes.append(
            self.ctx.server.scene.add_line_segments(
                "/plan/path", points=segments,
                colors=np.tile(np.array([PATH_COLOR]), (len(segments), 2, 1)),
            )
        )
        self.set_path_visible(self._path_visible)

    def clear_path(self) -> None:
        for node in self._path_nodes:
            node.remove()
        self._path_nodes = []

    # ---- exploration trees ------------------------------------------------------------------

    def draw_trees(self, result) -> None:
        """R2 exploration view: each snapshot tree as parent→child EE segments (start=blue,
        goal=orange). Empty result.trees just clears the previous drawing."""
        for node in self._tree_nodes:
            node.remove()
        self._tree_nodes = []
        if not getattr(result, "trees", None):
            return
        session = self.ctx.session
        model, ee = session.model, self.ctx.ee_link
        for t_idx, tree in enumerate(result.trees):
            points = np.array([q.fk(model, node, ee).translation() for node in tree.nodes])
            segments = np.array(
                [(points[parent], points[i]) for i, parent in enumerate(tree.parents) if parent >= 0]
            )
            if len(segments) == 0:
                continue
            color = TREE_COLORS[min(t_idx, len(TREE_COLORS) - 1)]
            self._tree_nodes.append(
                self.ctx.server.scene.add_line_segments(
                    f"/plan/tree_{t_idx}", points=segments,
                    colors=np.tile(np.array([color]), (len(segments), 2, 1)),
                )
            )
        self.set_trees_visible(self._trees_visible)

    # ---- mode-controlled visibility ---------------------------------------------------------

    def set_path_visible(self, visible: bool) -> None:
        self._path_visible = visible
        for node in self._path_nodes:
            node.visible = visible

    def set_trees_visible(self, visible: bool) -> None:
        self._trees_visible = visible
        for node in self._tree_nodes:
            node.visible = visible


class StudioContext:
    """Everything more than one mode needs. Constructed fresh on every mount."""

    def __init__(
        self,
        session: StudioSession,
        server: viser.ViserServer,
        robot_view: RobotView,
        obstacle_view: ObstacleView,
    ) -> None:
        self.session = session
        self.server = server
        self.robot_view = robot_view
        self.obstacle_view = obstacle_view
        # viser callbacks can run concurrently; the session's UI workspace is one-per-thread
        # (ADR-005), so every query/IK from a callback serializes through this lock.
        self.ui_lock = threading.Lock()
        links = session.ik_links() or session.leaf_links() or [session.model.root_link]
        self.ee_link: str = links[0]  # written only by the IK mode's dropdown
        # The gizmo lives here (not in the IK mode) because plan-goal capture and pose-goal
        # restore read/write it even while the IK mode is inactive and the gizmo hidden.
        self.ik_gizmo = server.scene.add_transform_controls("/ik_target", scale=0.25)
        self.last_attempt: Optional[Attempt] = None
        self.attempt_view = AttemptView(self)

        self.config_listeners: list[Callable[[np.ndarray], None]] = []
        self.scene_changed_listeners: list[Callable[[], None]] = []
        self.attempt_listeners: list[Callable[[Attempt], None]] = []
        self.status_sink: Optional[Callable[[str], None]] = None  # set by the Scene mode

    # ---- config -----------------------------------------------------------------------------

    def set_config(self, q_new: np.ndarray) -> None:
        """Set config from code (IK / scrub / play): updates the session, lets listeners sync
        their widgets (sliders), then refreshes the robot view."""
        self.session.set_config(q_new)
        for listener in self.config_listeners:
            listener(np.asarray(q_new, dtype=float))
        self.refresh()

    def refresh(self) -> None:
        with self.ui_lock:
            state = self.robot_view.update_config(self.session.q)
        if self.status_sink is not None:
            if state.in_collision:
                witness = f" ({state.witness.a} ↔ {state.witness.b})" if state.witness else ""
                self.status_sink(f"COLLIDING{witness}")
            else:
                self.status_sink(f"free · clearance {state.min_distance:.3f} m")

    # ---- events -----------------------------------------------------------------------------

    def scene_changed(self) -> None:
        """The static environment changed (obstacle add/move/remove) — caches went stale."""
        for listener in self.scene_changed_listeners:
            listener()

    def show_attempt(self, attempt: Attempt) -> None:
        """Record + draw a finished attempt. Status text is the caller's job — each dispatch
        formats its own; listeners handle cross-mode reactions (timing invalidation)."""
        self.last_attempt = attempt
        if attempt.result.ok():
            self.attempt_view.draw_path(attempt.path)
        else:
            self.attempt_view.clear_path()
        self.attempt_view.draw_trees(attempt.result)  # draws when captured, clears otherwise
        for listener in self.attempt_listeners:
            listener(attempt)
