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

SAVE_FORMAT = "quevedomp-studio/2"  # v2 adds goal/planner/query state; v1 files still load


def _tf_to_json(tf: "q.Transform") -> dict:
    return {"t": tf.translation().tolist(), "q_wxyz": tf.quaternion().tolist()}


def _tf_from_json(blob: dict) -> "q.Transform":
    return q.Transform.from_parts(np.asarray(blob["t"]), np.asarray(blob["q_wxyz"]))


def load_srdf_acm(srdf_xml: str, acm) -> int:
    """Populate an ACM from an SRDF's <disable_collisions link1=.. link2=..> entries (the same
    subset the C++ DTC harness reads). Returns the number of pairs added."""
    import xml.etree.ElementTree as ET

    count = 0
    for elem in ET.fromstring(srdf_xml).iter("disable_collisions"):
        a, b = elem.get("link1"), elem.get("link2")
        if a and b:
            acm.allow(a, b)
            count += 1
    return count


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
class IkBranch:
    """One distinct IK solution (branch), collision-annotated against the current scene."""

    q: np.ndarray
    free: bool  # collision-free (self + environment, current query options)
    pos_error: float
    rot_error: float


@dataclass
class Attempt:
    """One planning attempt, kept for the attempt log / rerun export."""

    index: int
    start: np.ndarray
    goal: GoalSpec
    result: "q.PlanningResult"
    path: list  # final (possibly smoothed) path as list of (dof,) arrays
    smoothed: bool = False


@dataclass
class TimedTrajectory:
    """A time-parameterized trajectory (Task 3.4) ready for playback and plotting."""

    times: np.ndarray          # (n,) seconds from start, monotone
    positions: np.ndarray      # (n, dof)
    velocities: np.ndarray     # (n, dof)
    accelerations: np.ndarray  # (n, dof)
    duration: float
    message: str = ""
    jerk_passes: int = 0
    max_jerk_violation: float = 0.0

    def sample(self, t: float) -> np.ndarray:
        """Configuration at time t (clamped), linear between nodes — dense enough at N>=200."""
        t = float(np.clip(t, self.times[0], self.times[-1]))
        k = int(np.searchsorted(self.times, t, side="right") - 1)
        k = min(max(k, 0), len(self.times) - 2)
        dt = self.times[k + 1] - self.times[k]
        a = (t - self.times[k]) / dt if dt > 0 else 0.0
        return (1.0 - a) * self.positions[k] + a * self.positions[k + 1]


class StudioSession:
    """Robot + environment + planning state behind the studio UI."""

    def __init__(
        self,
        urdf_text: str,
        package_dirs: Optional[dict[str, str]] = None,
        base_dir: str = "",
        yaml_extension: Optional[str] = None,
        allow_adjacent: bool = True,
        srdf_text: Optional[str] = None,
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
        # An SRDF supplies the full curated matrix (permanent contacts, never-colliding pairs).
        if srdf_text:
            load_srdf_acm(srdf_text, self.robot.acm)

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
        self.trajectory: Optional[TimedTrajectory] = None  # last parametrize() output
        # R4 refiner scratch: a ClearanceField + robot sphere cover, both cached. The field is
        # rebuilt lazily and invalidated whenever the (quasi-static) environment changes.
        self._clearance_field = None
        self._clearance_resolution = 0.0
        self._robot_spheres = None
        self.attempt_listeners: list[Callable[[Attempt], None]] = []
        self._plan_thread: Optional[threading.Thread] = None
        self._ik = q.make_numerical_ik(self.model)
        # Interactive tracker: seeded-only (no random restarts). Restarts are what make gizmo
        # dragging flicker — each stall re-seeds randomly and lands on a different branch.
        track_options = q.IkOptions()
        track_options.max_restarts = 0
        track_options.max_iters = 60
        self._ik_track = q.make_numerical_ik(self.model, track_options)

        # Interactive queries default to a finer edge check than the library's 0.05 rad/m —
        # at studio scale the discretization gap is visible when scrubbing a planned path.
        self.planner_params.edge_resolution = 0.02

    # ---- Robot state ---------------------------------------------------------------------------

    @property
    def dof(self) -> int:
        return self.model.dof

    def movable_joints(self) -> list:
        return [j for j in self.model.joints if j.is_movable()]

    def leaf_links(self) -> list[str]:
        return [l.name for l in self.model.links if not l.child_joints]

    def ik_links(self) -> list[str]:
        """Leaf links IK can actually move: >= 2 movable joints in their chain, ordered most-
        articulated first, TCP/tool/EE-named links ahead of ties. (A robot cell URDF is full of
        static frames — cameras, fiducials, racks — that made a naive 'last leaf' default dead.)"""
        joints = self.model.joints
        scored = []
        for name in self.leaf_links():
            movable = sum(1 for ji in self.model.chain_to(name).joints if joints[ji].is_movable())
            if movable >= 2:
                preferred = any(tag in name.lower() for tag in ("tcp", "tool", "tip", "ee"))
                scored.append((-movable, not preferred, name))
        return [name for _, _, name in sorted(scored)]

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

    def solve_ik(
        self,
        link: str,
        target: "q.Transform",
        seed: Optional[np.ndarray] = None,
        interactive: bool = False,
    ):
        """interactive=True: seeded-only tracking (fast, branch-stable, may fail out of reach);
        False: the full multi-restart solver (global, may switch branches)."""
        solver = self._ik_track if interactive else self._ik
        return solver.solve(link, target, self.q if seed is None else seed)

    def solve_ik_branches(
        self,
        link: str,
        target: "q.Transform",
        n: int = 8,
        seed: Optional[np.ndarray] = None,
        cost: Optional[Callable[[np.ndarray], float]] = None,
    ) -> list[IkBranch]:
        """Up to `n` DISTINCT IK branches for `link` at `target`, each collision-checked against
        the current scene. Default order: nearest the current config first (tracking); pass a
        different `seed` config to order around it, or `cost` (q -> float, ascending) for a
        custom ranking — e.g. joint-limit margin, elbow-up preference, distance to a home pose."""
        io = q.IkOptions()
        io.max_restarts = max(60, 12 * n)  # exploration budget scales with the ask
        solver = q.make_numerical_ik(self.model, io)
        seed_q = self.q if seed is None else np.asarray(seed, dtype=float)
        branches = [
            IkBranch(
                np.asarray(r.q),
                not self.collision_state(r.q).in_collision,
                r.pos_error,
                r.rot_error,
            )
            for r in solver.solve_all(link, target, n, seed_q)
        ]
        if cost is not None:
            branches.sort(key=lambda b: cost(b.q))
        return branches

    @staticmethod
    def sample_path(path, samples_per_segment: int = 8) -> np.ndarray:
        """Densify a joint-space path by linear interpolation: the TRUE motion between
        waypoints (what the EE actually traces), not the waypoint polyline."""
        if len(path) < 2:
            return np.asarray(path, dtype=float).reshape(len(path), -1)
        out = []
        for a, b in zip(path, path[1:]):
            for t in np.linspace(0.0, 1.0, samples_per_segment, endpoint=False):
                out.append((1.0 - t) * a + t * b)
        out.append(np.asarray(path[-1], dtype=float))
        return np.array(out)

    # ---- Environment ---------------------------------------------------------------------------

    def add_obstacle(self, id: str, geometry: GeometryLike, pose: "q.Transform") -> Obstacle:
        if id in self.obstacles:
            raise ValueError(f"obstacle id already exists: {id!r}")
        handle = self.scene.add_object(id, geometry, pose)
        obstacle = Obstacle(id, geometry, pose, handle)
        self.obstacles[id] = obstacle
        self._clearance_field = None  # the SDF describes the old environment
        return obstacle

    def move_obstacle(self, id: str, pose: "q.Transform") -> None:
        obstacle = self.obstacles[id]
        obstacle.pose = pose
        self.scene.move_object(obstacle.handle, pose)
        self._clearance_field = None

    def remove_obstacle(self, id: str) -> None:
        self.scene.remove_object(self.obstacles.pop(id).handle)
        self._clearance_field = None

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

    def lever_weights(self) -> np.ndarray:
        """Per-dof Cartesian lever weights (m/rad) for max_link_sweep, resolved through this
        session's mesh sources; cached — they depend only on the (immutable) model."""
        if getattr(self, "_lever_weights", None) is None:
            self._lever_weights = q.cartesian_lever_weights(self.model, self.mesh_sources)
        return self._lever_weights

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

        # Cartesian-bounded edge stepping (P3) needs the lever weights, and this robot's mesh
        # URIs need our package dirs — so compute them here (once; they depend only on the model).
        if self.planner_params.max_link_sweep > 0 and self.planner_params.lever_weights.size == 0:
            self.planner_params.lever_weights = self.lever_weights()

        planner = q.make_planner(self.planner_params, self.robot, self.scene)
        result = planner.plan(problem)

        path = list(result.path)
        smoothed = False
        if result.ok() and self.smooth and len(path) > 2:
            sp = q.SmootherParams()
            sp.edge_resolution = self.planner_params.edge_resolution
            sp.max_link_sweep = self.planner_params.max_link_sweep
            sp.lever_weights = self.planner_params.lever_weights
            sp.collision = self.query_options
            sp.seed = result.used_seed
            path = list(q.make_shortcut_smoother(sp, self.robot, self.scene).smooth(path))
            smoothed = True

        attempt = Attempt(len(self.attempts), self.start.copy(), self.goal, result, path, smoothed)
        self.trajectory = None  # timing belongs to the previous plan
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

    def refine_async(
        self, on_done: Callable[[Optional[Attempt]], None], **kwargs
    ) -> None:
        """Run refine() on the plan worker thread (shares the one-at-a-time guard with plan_async);
        `on_done` fires with None if the refine raised. `kwargs` pass through to refine()."""
        if self.is_planning:
            raise RuntimeError("a plan or refine is already running")

        def run() -> None:
            attempt: Optional[Attempt] = None
            try:
                attempt = self.refine(**kwargs)
            except Exception:
                traceback.print_exc()
            on_done(attempt)

        self._plan_thread = threading.Thread(target=run, name="quevedomp-refine", daemon=True)
        self._plan_thread.start()

    # ---- Optimization refiner (roadmap R4) ------------------------------------------------------

    def clearance_field(self, resolution: float = 0.02, force: bool = False):
        """The environment's ClearanceField (R3), built lazily and cached. Invalidated whenever an
        obstacle is added/moved/removed (the field is a snapshot of the quasi-static scene)."""
        if (
            force
            or self._clearance_field is None
            or abs(self._clearance_resolution - resolution) > 1e-12
        ):
            opts = q.ClearanceFieldOptions()
            opts.resolution = resolution
            self._clearance_field = q.ClearanceField.build(self.environment(), opts)
            self._clearance_resolution = resolution
        return self._clearance_field

    def robot_spheres(self):
        """Conservative sphere cover of the robot's collision geometry, cached (model-immutable)."""
        if self._robot_spheres is None:
            self._robot_spheres = q.decompose_robot(self.model, self.mesh_sources)
        return self._robot_spheres

    def refine(
        self,
        attempt: Optional[Attempt] = None,
        *,
        standalone: bool = False,
        waypoints: int = 64,
        max_iterations: int = 100,
        smoothness_weight: float = 1.0,
        clearance_weight: float = 1.0,
        clearance_epsilon: float = 0.10,
        step_size: float = 0.1,
        resolution: float = 0.02,
    ) -> Attempt:
        """Run the R4 CHOMP/TrajOpt refiner and record the result as a new Attempt.

        Refiner mode (default) polishes an existing attempt's (smoothed) path; `standalone=True`
        seeds a straight line from start to a resolved goal instead. The output is CERTIFIED
        collision-free by the exact backend — the ClearanceField only supplies gradients."""
        if self.goal is None:
            raise RuntimeError("no goal set — call set_goal_joints() or set_goal_pose() first")

        params = q.RefinerParams()
        params.waypoints = waypoints
        params.max_iterations = max_iterations
        params.smoothness_weight = smoothness_weight
        params.clearance_weight = clearance_weight
        params.clearance_epsilon = clearance_epsilon
        params.step_size = step_size
        params.edge_resolution = self.planner_params.edge_resolution
        params.collision = self.query_options
        if self.planner_params.max_link_sweep > 0:
            params.max_link_sweep = self.planner_params.max_link_sweep
            params.lever_weights = self.lever_weights()

        if not standalone:
            src = attempt if attempt is not None else (self.attempts[-1] if self.attempts else None)
            if src is None or len(src.path) < 2:
                raise RuntimeError("refine: no attempt with a path to polish — plan first (or "
                                   "pass standalone=True)")
            params.seed = [np.asarray(w, dtype=float) for w in src.path]

        problem = q.PlanningProblem()
        problem.start = self.start
        problem.goal = self.goal.to_goal()
        problem.collision = self.query_options
        problem.timeout = self.timeout

        field = self.clearance_field(resolution)
        refiner = q.make_refiner(params, self.robot, self.scene, field, self.robot_spheres())
        result = refiner.plan(problem)

        attempt_out = Attempt(
            len(self.attempts), self.start.copy(), self.goal, result, list(result.path),
            smoothed=True,
        )
        self.trajectory = None
        self.attempts.append(attempt_out)
        for listener in self.attempt_listeners:
            try:
                listener(attempt_out)
            except Exception:
                traceback.print_exc()
        return attempt_out

    # ---- Time parameterization (Task 3.4 / roadmap R2) ------------------------------------------

    def parametrize(
        self,
        attempt: Optional[Attempt] = None,
        *,
        default_acceleration: float = 8.0,
        tip_linear_velocity: float = 0.0,
        tip_linear_acceleration: float = 0.0,
        max_jerk: float = 0.0,
        tip_link: str = "",
        nodes: int = 200,
    ) -> TimedTrajectory:
        """Time-parameterize an attempt's (smoothed) path: fit a C4 spline, RE-VALIDATE it
        against the current scene at the planner's edge fidelity, then run the Task 3.4
        parameterizer (JerkLimited when max_jerk > 0). Limits come from the URDF (+ yaml
        extension), with `default_acceleration` where the model has none and the scalar tip /
        jerk caps applied uniformly. Stores and returns the trajectory for playback/plots."""
        attempt = attempt if attempt is not None else (self.attempts[-1] if self.attempts else None)
        if attempt is None or len(attempt.path) < 2:
            raise RuntimeError("parametrize: no planned attempt with a path — plan first")

        disc = q.EdgeDiscretization()
        disc.joint_resolution = self.planner_params.edge_resolution
        if self.planner_params.max_link_sweep > 0:
            disc.max_link_sweep = self.planner_params.max_link_sweep
            disc.lever_weights = self.lever_weights()
        fit = q.fit_collision_free(
            attempt.path, self.scene, self.robot, disc, self.query_options, self._ws
        )
        if not fit.success:
            raise RuntimeError(fit.message)

        task = q.TaskLimits()
        task.max_linear_velocity = tip_linear_velocity
        task.max_linear_acceleration = tip_linear_acceleration
        task.frame = tip_link
        lim = q.limits_from_model(self.model, task, default_acceleration)
        opt = q.ParameterizationOptions()
        opt.nodes = nodes
        if max_jerk > 0:
            lim.max_jerk = np.full(self.dof, float(max_jerk))
            opt.mode = q.ParameterizationMode.JerkLimited
        r = q.parametrize(self.model, fit.spline, lim, opt)
        if not r.success:
            raise RuntimeError(r.message)

        wp = r.trajectory
        self.trajectory = TimedTrajectory(
            times=np.array([w.time for w in wp]),
            positions=np.array([w.state.pos for w in wp]),
            velocities=np.array([w.state.vel for w in wp]),
            accelerations=np.array([w.state.acc for w in wp]),
            duration=r.duration,
            message=r.message,
            jerk_passes=r.jerk_passes,
            max_jerk_violation=r.max_jerk_violation,
        )
        return self.trajectory

    # ---- Save / load (Task 2a.5 serializers -> Phase 3b captures open here later) ---------------

    def save(self, path: str | Path) -> None:
        """The full problem setup: robot+ACM and scene via the Task 2a.5 serializers, plus the
        studio-level state (config, start/goal, planner + query settings) as plain JSON."""
        goal = None
        if self.goal is not None:
            goal = {
                "kind": self.goal.kind,
                "q": None if self.goal.q is None else self.goal.q.tolist(),
                "link": self.goal.link,
                "pose": None if self.goal.pose is None else _tf_to_json(self.goal.pose),
                "pos_tol": self.goal.pos_tol,
                "rot_tol": self.goal.rot_tol,
            }
        p = self.planner_params
        blob = {
            "format": SAVE_FORMAT,
            "robot_instance": base64.b64encode(q.serialize_robot_instance(self.robot)).decode(),
            "scene": base64.b64encode(q.serialize_scene(self.environment())).decode(),
            "package_dirs": self.package_dirs,
            "base_dir": self.base_dir,
            "q": self.q.tolist(),
            "start": self.start.tolist(),
            "goal": goal,
            "timeout": self.timeout,
            "smooth": self.smooth,
            "planner": {
                "algorithm": p.algorithm,
                "edge_resolution": p.edge_resolution,
                "max_link_sweep": p.max_link_sweep,  # lever weights are derived, not saved
                "max_extension": p.max_extension,
                "goal_bias": p.goal_bias,
                "batch_size": p.batch_size,
                "max_iterations": p.max_iterations,
            },
            "query": {
                "check_self_collision": self.query_options.check_self_collision,
                "safety_margin": self.query_options.safety_margin,
                "robot_padding": self.query_options.robot_padding,
            },
        }
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(blob, indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "StudioSession":
        blob = json.loads(Path(path).read_text())
        if blob.get("format") not in ("quevedomp-studio/1", SAVE_FORMAT):
            raise ValueError(f"not a quevedomp-studio session file: {path}")
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

        goal = blob.get("goal")
        if goal is not None:
            session.goal = GoalSpec(
                goal["kind"],
                q=None if goal.get("q") is None else np.asarray(goal["q"], dtype=float),
                link=goal.get("link", ""),
                pose=None if goal.get("pose") is None else _tf_from_json(goal["pose"]),
                pos_tol=goal.get("pos_tol", 1e-3),
                rot_tol=goal.get("rot_tol", 1e-2),
            )
        session.timeout = blob.get("timeout", session.timeout)
        session.smooth = blob.get("smooth", session.smooth)
        planner = blob.get("planner") or {}
        p = session.planner_params
        p.algorithm = planner.get("algorithm", p.algorithm)
        p.edge_resolution = planner.get("edge_resolution", p.edge_resolution)
        p.max_link_sweep = planner.get("max_link_sweep", p.max_link_sweep)
        p.max_extension = planner.get("max_extension", p.max_extension)
        p.goal_bias = planner.get("goal_bias", p.goal_bias)
        p.batch_size = planner.get("batch_size", p.batch_size)
        p.max_iterations = planner.get("max_iterations", p.max_iterations)
        query = blob.get("query") or {}
        session.query_options.check_self_collision = query.get(
            "check_self_collision", session.query_options.check_self_collision
        )
        session.query_options.safety_margin = query.get(
            "safety_margin", session.query_options.safety_margin
        )
        session.query_options.robot_padding = query.get(
            "robot_padding", session.query_options.robot_padding
        )
        return session
