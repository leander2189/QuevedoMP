"""Headless studio core: robot + scene state and the verbs the UI (or a script) drives.

No viser/rerun imports here — this layer is what the smoke test exercises, and what a future
capture-replay flow reuses. All heavy calls go straight to the quevedomp bindings, which
release the GIL (ADR-016), so `plan_async` keeps a UI thread responsive.
"""

from __future__ import annotations

import base64
import json
import threading
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Union

import numpy as np

import quevedomp as q

GeometryLike = Union["q.BoxShape", "q.SphereShape", "q.CylinderShape", "q.Mesh"]

SAVE_FORMAT = "quevedomp-studio/1"


@dataclass
class Obstacle:
    id: str
    geometry: GeometryLike
    pose: "q.Transform"
    handle: int = -1  # SceneHandle in the live CollisionScene


@dataclass
class GoalSpec:
    """Either a joint-space target or a Cartesian pose of `link`."""

    kind: str  # "joint" | "pose"
    q: Optional[np.ndarray] = None
    link: str = ""
    pose: Optional["q.Transform"] = None
    pos_tol: float = 1e-3
    rot_tol: float = 1e-2

    def to_goal(self) -> "q.Goal":
        if self.kind == "joint":
            assert self.q is not None
            return q.JointGoal(self.q, 1e-3)
        assert self.pose is not None
        return q.PoseGoal(q.Pose(self.pose, self.pos_tol, self.rot_tol), self.link)


@dataclass
class Attempt:
    """One planning attempt, kept for the attempt log / rerun export."""

    index: int
    start: np.ndarray
    goal: GoalSpec
    result: "q.PlanningResult"
    path: list  # final (possibly smoothed) path as list of (dof,) arrays
    smoothed: bool = False


class StudioSession:
    """Robot + environment + planning state behind the studio UI."""

    def __init__(
        self,
        urdf_text: str,
        package_dirs: Optional[dict[str, str]] = None,
        base_dir: str = "",
        yaml_extension: Optional[str] = None,
        allow_adjacent: bool = True,
    ) -> None:
        self.model = q.RobotModel.from_urdf(urdf_text, yaml_extension)
        self.robot = q.RobotInstance(self.model)
        self.package_dirs = dict(package_dirs or {})
        self.base_dir = base_dir

        # Adjacent links touch by construction; a URDF carries no SRDF-style ACM, so seed the
        # standard default (every parent/child pair allowed) or the rest pose self-collides.
        if allow_adjacent:
            for joint in self.model.joints:
                self.robot.acm.allow(joint.parent_link, joint.child_link)

        self.mesh_sources = q.MeshSources(self.package_dirs, self.base_dir)
        self.scene = q.make_static_scene(
            self.model, q.SceneDescription(), q.BackendHint.Auto, self.mesh_sources
        )
        self._ws = self.scene.make_workspace()  # UI-thread workspace (ADR-005: one per thread)

        self.q = np.zeros(self.model.dof)
        self.start = self.q.copy()
        self.goal: Optional[GoalSpec] = None
        self.obstacles: dict[str, Obstacle] = {}

        self.planner_params = q.PlannerParams()
        self.query_options = q.QueryOptions()
        self.timeout = 2.0
        self.smooth = True

        self.attempts: list[Attempt] = []
        self.attempt_listeners: list[Callable[[Attempt], None]] = []
        self._plan_thread: Optional[threading.Thread] = None
        self._ik = q.make_numerical_ik(self.model)

    # ---- Robot state ---------------------------------------------------------------------------

    @property
    def dof(self) -> int:
        return self.model.dof

    def movable_joints(self) -> list:
        return [j for j in self.model.joints if j.is_movable()]

    def leaf_links(self) -> list[str]:
        return [l.name for l in self.model.links if not l.child_joints]

    def set_config(self, q_new: np.ndarray) -> None:
        self.q = np.asarray(q_new, dtype=float).copy()

    def link_poses(self, q_at: Optional[np.ndarray] = None) -> list:
        return q.fk_all(self.model, self.q if q_at is None else np.asarray(q_at, dtype=float))

    def collision_state(self, q_at: Optional[np.ndarray] = None) -> "q.CollisionResult":
        """Boolean + witness (distance requested so the witness pair names come back)."""
        opts = q.QueryOptions()
        opts.distance = True
        opts.check_self_collision = self.query_options.check_self_collision
        return self.scene.query(
            self.robot, self.q if q_at is None else np.asarray(q_at, dtype=float), opts, self._ws
        )

    def solve_ik(self, link: str, target: "q.Transform", seed: Optional[np.ndarray] = None):
        return self._ik.solve(link, target, self.q if seed is None else seed)

    # ---- Environment ---------------------------------------------------------------------------

    def add_obstacle(self, id: str, geometry: GeometryLike, pose: "q.Transform") -> Obstacle:
        if id in self.obstacles:
            raise ValueError(f"obstacle id already exists: {id!r}")
        handle = self.scene.add_object(id, geometry, pose)
        obstacle = Obstacle(id, geometry, pose, handle)
        self.obstacles[id] = obstacle
        return obstacle

    def move_obstacle(self, id: str, pose: "q.Transform") -> None:
        obstacle = self.obstacles[id]
        obstacle.pose = pose
        self.scene.move_object(obstacle.handle, pose)

    def remove_obstacle(self, id: str) -> None:
        self.scene.remove_object(self.obstacles.pop(id).handle)

    def environment(self) -> "q.SceneDescription":
        env = q.SceneDescription()
        for o in self.obstacles.values():
            env.add(o.id, o.geometry, o.pose)
        return env

    # ---- Planning ------------------------------------------------------------------------------

    def set_start(self, q_start: Optional[np.ndarray] = None) -> None:
        self.start = (self.q if q_start is None else np.asarray(q_start, dtype=float)).copy()

    def set_goal_joints(self, q_goal: Optional[np.ndarray] = None) -> None:
        self.goal = GoalSpec(
            "joint", q=(self.q if q_goal is None else np.asarray(q_goal, dtype=float)).copy()
        )

    def set_goal_pose(
        self, link: str, pose: "q.Transform", pos_tol: float = 1e-3, rot_tol: float = 1e-2
    ) -> None:
        self.goal = GoalSpec("pose", link=link, pose=pose, pos_tol=pos_tol, rot_tol=rot_tol)

    @property
    def is_planning(self) -> bool:
        return self._plan_thread is not None and self._plan_thread.is_alive()

    def plan(self, seed: Optional[int] = None) -> Attempt:
        """Blocking plan (+ optional shortcut smoothing). GIL is released inside the C++ calls."""
        if self.goal is None:
            raise RuntimeError("no goal set — call set_goal_joints() or set_goal_pose() first")

        problem = q.PlanningProblem()
        problem.start = self.start
        problem.goal = self.goal.to_goal()
        problem.collision = self.query_options
        problem.timeout = self.timeout
        problem.seed = seed

        planner = q.make_planner(self.planner_params, self.robot, self.scene)
        result = planner.plan(problem)

        path = list(result.path)
        smoothed = False
        if result.ok() and self.smooth and len(path) > 2:
            sp = q.SmootherParams()
            sp.edge_resolution = self.planner_params.edge_resolution
            sp.collision = self.query_options
            sp.seed = result.used_seed
            path = list(q.make_shortcut_smoother(sp, self.robot, self.scene).smooth(path))
            smoothed = True

        attempt = Attempt(len(self.attempts), self.start.copy(), self.goal, result, path, smoothed)
        self.attempts.append(attempt)
        for listener in self.attempt_listeners:
            try:
                listener(attempt)
            except Exception:  # a broken logger must never eat a finished plan
                traceback.print_exc()
        return attempt

    def plan_async(
        self, on_done: Callable[[Optional[Attempt]], None], seed: Optional[int] = None
    ) -> None:
        """One plan at a time; `on_done` fires on the worker thread — with None if the plan
        itself raised (so the UI can recover instead of wedging)."""
        if self.is_planning:
            raise RuntimeError("a plan is already running")

        def run() -> None:
            attempt: Optional[Attempt] = None
            try:
                attempt = self.plan(seed)
            except Exception:
                traceback.print_exc()
            on_done(attempt)

        self._plan_thread = threading.Thread(target=run, name="quevedomp-plan", daemon=True)
        self._plan_thread.start()

    # ---- Save / load (Task 2a.5 serializers -> Phase 3b captures open here later) ---------------

    def save(self, path: str | Path) -> None:
        blob = {
            "format": SAVE_FORMAT,
            "robot_instance": base64.b64encode(q.serialize_robot_instance(self.robot)).decode(),
            "scene": base64.b64encode(q.serialize_scene(self.environment())).decode(),
            "package_dirs": self.package_dirs,
            "base_dir": self.base_dir,
            "q": self.q.tolist(),
            "start": self.start.tolist(),
        }
        Path(path).write_text(json.dumps(blob, indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "StudioSession":
        blob = json.loads(Path(path).read_text())
        if blob.get("format") != SAVE_FORMAT:
            raise ValueError(f"not a {SAVE_FORMAT} file: {path}")
        robot = q.deserialize_robot_instance(base64.b64decode(blob["robot_instance"]))
        session = cls(
            robot.model.source_urdf,
            package_dirs=blob.get("package_dirs") or {},
            base_dir=blob.get("base_dir", ""),
            yaml_extension=robot.model.source_yaml,
            allow_adjacent=False,  # the serialized ACM is authoritative
        )
        for a, b in robot.acm.pairs():
            session.robot.acm.allow(a, b)
        env = q.deserialize_scene(base64.b64decode(blob["scene"]))
        for obj in env.objects:
            session.add_obstacle(obj.id, obj.geometry, obj.pose)
        session.set_config(np.asarray(blob["q"], dtype=float))
        session.set_start(np.asarray(blob["start"], dtype=float))
        return session
