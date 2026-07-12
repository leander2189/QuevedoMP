"""Task 4a.5 — planner + smoother.

Mirrors tests/unit/test_planner.cpp and test_smoother.cpp: the same 2-DOF planar gantry
(q = (x, y) IS the end-effector position), the same wall fixture (straight line at y=-1
blocked, detour above the wall exists), determinism per seed, InvalidProblem detection.
"""

import threading
import time

import numpy as np
import pytest

import quevedomp as q

# Two prismatic joints translate a sphere end-effector in x/y — a clean 2D point robot.
GANTRY_2D = """<robot name="gantry2d">
  <link name="base"/>
  <joint name="jx" type="prismatic">
    <parent link="base"/><child link="cx"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
  <link name="cx"/>
  <joint name="jy" type="prismatic">
    <parent link="cx"/><child link="ee"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
  <link name="ee">
    <collision><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
</robot>"""


def make_fixture(with_wall: bool):
    """Robot + scene; the wall blocks x~0 for y <= 0.5, leaving a gap above it."""
    model = q.RobotModel.from_urdf(GANTRY_2D)
    robot = q.RobotInstance(model)
    env = q.SceneDescription()
    if with_wall:
        env.add(
            "wall",
            q.BoxShape(np.array([0.1, 1.25, 0.5])),
            q.Transform.from_translation(np.array([0.0, -0.75, 0.0])),
        )
    scene = q.make_static_scene(model, env)
    return model, robot, scene


def problem_to(start, goal, timeout: float = 2.0, seed: int = 1) -> q.PlanningProblem:
    p = q.PlanningProblem()
    p.start = np.asarray(start, dtype=float)
    p.goal = q.JointGoal(np.asarray(goal, dtype=float), 1e-3)
    p.timeout = timeout
    p.seed = seed
    return p


# ---- Registry ----------------------------------------------------------------------------------


def test_registry_and_unknown_algorithm() -> None:
    assert "rrt_connect" in q.registered_planners()
    model, robot, scene = make_fixture(False)
    params = q.PlannerParams()
    params.algorithm = "nope"
    with pytest.raises(RuntimeError):
        q.make_planner(params, robot, scene)


# ---- Planning ----------------------------------------------------------------------------------


def test_plan_free_space() -> None:
    model, robot, scene = make_fixture(False)
    planner = q.make_planner(q.PlannerParams(), robot, scene)
    r = planner.plan(problem_to([-1, -1], [1, -1], seed=1))
    assert r.ok()
    assert r.status == q.PlanningStatus.Success
    assert r.used_seed == 1
    assert np.linalg.norm(r.path[0] - [-1, -1]) < 1e-9
    assert np.linalg.norm(r.path[-1] - [1, -1]) < 1e-9
    arr = r.path_array()
    assert arr.shape == (len(r.path), 2)
    assert np.allclose(arr[0], [-1, -1])


def test_plan_around_wall_is_collision_free() -> None:
    model, robot, scene = make_fixture(True)
    params = q.PlannerParams()
    planner = q.make_planner(params, robot, scene)
    r = planner.plan(problem_to([-1, -1], [1, -1], seed=2))
    assert r.ok(), r.message
    # Independently re-validate every edge, exactly as the C++ test does.
    ws = scene.make_workspace()
    for a, b in zip(r.path, r.path[1:]):
        e = q.check_edge(scene, robot, a, b, params.edge_resolution, q.QueryOptions(), ws)
        assert e.valid, f"edge collides at t={e.first_contact_t}"
    # The detour must actually leave the blocked corridor (the wall gap is above y=0.5).
    assert r.path_array()[:, 1].max() > 0.4
    assert r.stats.collision_queries > 0
    assert r.stats.iterations > 0
    assert sum(r.stats.batch_size_histogram.values()) == r.stats.collision_queries


def test_determinism_per_seed() -> None:
    model, robot, scene = make_fixture(True)
    planner = q.make_planner(q.PlannerParams(), robot, scene)
    a = planner.plan(problem_to([-1, -1], [1, -1], seed=7))
    b = planner.plan(problem_to([-1, -1], [1, -1], seed=7))
    assert a.ok() and b.ok()
    assert len(a.path) == len(b.path)
    for wa, wb in zip(a.path, b.path):
        assert np.linalg.norm(wa - wb) < 1e-12


def test_pose_goal() -> None:
    model, robot, scene = make_fixture(False)
    planner = q.make_planner(q.PlannerParams(), robot, scene)
    p = q.PlanningProblem()
    p.start = np.array([-1.0, -1.0])
    p.goal = q.PoseGoal(
        q.Pose(q.Transform.from_translation(np.array([0.5, 0.5, 0.0])), pos_tol=1e-2), "ee"
    )
    p.timeout = 2.0
    p.seed = 3
    r = planner.plan(p)
    assert r.ok(), r.message
    assert np.linalg.norm(r.path[-1] - [0.5, 0.5]) < 1e-2  # q IS the ee position on the gantry


def test_multi_goal_reaches_one() -> None:
    model, robot, scene = make_fixture(False)
    planner = q.make_planner(q.PlannerParams(), robot, scene)
    g1, g2 = np.array([1.0, -1.0]), np.array([-1.0, 1.0])
    p = q.PlanningProblem()
    p.start = np.array([0.0, 0.0])
    p.goal = q.MultiGoal([q.JointGoal(g1, 1e-3), q.JointGoal(g2, 1e-3)])
    p.timeout = 2.0
    p.seed = 4
    r = planner.plan(p)
    assert r.ok(), r.message
    end = r.path[-1]
    assert min(np.linalg.norm(end - g1), np.linalg.norm(end - g2)) < 1e-2


def test_auto_seed_via_none() -> None:
    # The studio's "seed 0 = auto" path: optional<> fields must accept None.
    model, robot, scene = make_fixture(False)
    planner = q.make_planner(q.PlannerParams(), robot, scene)
    p = problem_to([-1, -1], [1, -1])
    p.seed = None  # regression: this raised TypeError before the .none() setter fix
    assert p.seed is None
    r = planner.plan(p)
    assert r.ok()

    opts = q.QueryOptions()
    opts.per_pair_padding = None  # the other optional<> setter
    assert opts.per_pair_padding is None


def test_invalid_problem_detected_without_search() -> None:
    model, robot, scene = make_fixture(False)
    planner = q.make_planner(q.PlannerParams(), robot, scene)
    p = problem_to([3.0, 0.0], [1.0, -1.0], seed=5)  # start outside the x-limit [-2, 2]
    assert isinstance(q.validate(p, model), str)  # validate() explains the rejection
    r = planner.plan(p)
    assert r.status == q.PlanningStatus.InvalidProblem
    assert r.message
    assert r.used_seed == 5  # populated even on rejection


def test_colliding_goal_is_no_solution() -> None:
    model, robot, scene = make_fixture(True)
    planner = q.make_planner(q.PlannerParams(), robot, scene)
    r = planner.plan(problem_to([-1, -1], [0.0, -1.0], seed=6))  # goal inside the wall
    assert r.status == q.PlanningStatus.NoSolution
    assert r.message


# ---- Cartesian-bounded edge stepping (Task 3.3d P3) ---------------------------------------------


def test_lever_weights_and_sweep_mode() -> None:
    model, robot, scene = make_fixture(True)
    w = q.cartesian_lever_weights(model)
    assert w.shape == (2,)
    assert np.allclose(w, [1.0, 1.0])  # prismatic joints: metres per metre, exactly

    params = q.PlannerParams()
    params.max_link_sweep = 0.01  # 1 cm sweep bound; weights auto-computed (primitive robot)
    planner = q.make_planner(params, robot, scene)
    r = planner.plan(problem_to([-1, -1], [1, -1], seed=2))
    assert r.ok(), r.message

    sp = q.SmootherParams()
    sp.max_link_sweep = params.max_link_sweep
    sp.lever_weights = w
    sp.seed = 2
    smoothed = q.make_shortcut_smoother(sp, robot, scene).smooth(r.path)
    assert path_length(smoothed) <= path_length(r.path) + 1e-12

    bad = q.PlannerParams()
    bad.max_link_sweep = 0.01
    bad.lever_weights = np.array([1.0])  # wrong size for a 2-dof robot: loud failure
    with pytest.raises(RuntimeError):
        q.make_planner(bad, robot, scene)


# ---- GIL release (the IDE's responsiveness contract, ADR-016) -----------------------------------


def test_plan_releases_the_gil() -> None:
    """A plan on a worker thread must leave the main thread running (Planner.plan drops the GIL)."""
    model = q.RobotModel.from_urdf(GANTRY_2D)
    robot = q.RobotInstance(model)
    env = q.SceneDescription()
    # A wall spanning the whole y-range splits the space: the free goal is unreachable, so the
    # planner burns its full timeout instead of returning early.
    env.add(
        "full_wall",
        q.BoxShape(np.array([0.1, 2.5, 0.5])),
        q.Transform.from_translation(np.array([0.0, 0.0, 0.0])),
    )
    scene = q.make_static_scene(model, env)
    planner = q.make_planner(q.PlannerParams(), robot, scene)

    result = None
    done = threading.Event()

    def run() -> None:
        nonlocal result
        result = planner.plan(problem_to([-1, 0], [1, 0], timeout=0.6, seed=9))
        done.set()

    ticks = 0
    worker = threading.Thread(target=run)
    worker.start()
    while not done.is_set():
        ticks += 1
        time.sleep(0.01)
    worker.join()

    assert result is not None
    assert result.status in (q.PlanningStatus.Timeout, q.PlanningStatus.NoSolution)
    assert ticks >= 10  # main thread got scheduled throughout the ~0.6 s plan


# ---- Smoother ----------------------------------------------------------------------------------


def path_length(path) -> float:
    return sum(float(np.linalg.norm(b - a)) for a, b in zip(path, path[1:]))


def test_shortcut_smoother_shortens_and_stays_free() -> None:
    model, robot, scene = make_fixture(True)
    params = q.PlannerParams()
    planner = q.make_planner(params, robot, scene)
    r = planner.plan(problem_to([-1, -1], [1, -1], seed=8))
    assert r.ok()

    sp = q.SmootherParams()
    sp.seed = 8
    smoother = q.make_shortcut_smoother(sp, robot, scene)
    smoothed = smoother.smooth(r.path)

    assert path_length(smoothed) <= path_length(r.path) + 1e-12
    assert np.linalg.norm(smoothed[0] - r.path[0]) < 1e-12
    assert np.linalg.norm(smoothed[-1] - r.path[-1]) < 1e-12
    ws = scene.make_workspace()
    for a, b in zip(smoothed, smoothed[1:]):
        assert q.check_edge(scene, robot, a, b, sp.edge_resolution, q.QueryOptions(), ws).valid
