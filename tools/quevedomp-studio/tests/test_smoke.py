"""Task 4a.6 verify — headless studio smoke test.

Drives the plan's scripted gate: load UR5 (real meshes), add an obstacle, plan, assert the
path is drawn — session layer first (no UI), then the full viser app running headless.
"""

from pathlib import Path

import numpy as np
import pytest

import quevedomp as q
from quevedomp_studio import StudioSession
from quevedomp_studio.app import StudioApp

FIXTURES = Path(__file__).resolve().parents[3] / "tests" / "fixtures"
GOAL = np.array([0.3, -0.3, 0.3, -0.3, 0.3, 0.3])


@pytest.fixture(scope="module")
def session() -> StudioSession:
    return StudioSession(
        (FIXTURES / "robots" / "ur5.urdf").read_text(),
        package_dirs={
            "example-robot-data": str(FIXTURES / "robots" / "meshes" / "example-robot-data")
        },
    )


# ---- Session layer (no UI) ----------------------------------------------------------------------


def test_session_loads_ur5(session: StudioSession) -> None:
    assert session.dof == 6
    assert not session.collision_state(np.zeros(6)).in_collision  # adjacent ACM seeded
    assert "wrist_3_link" in [j.child_link for j in session.model.joints]


def test_session_ik(session: StudioSession) -> None:
    target = q.fk(session.model, GOAL, "wrist_3_link")
    result = session.solve_ik("wrist_3_link", target, seed=GOAL + 0.05)
    assert result.success


def test_session_ik_branches(session: StudioSession) -> None:
    target = q.fk(session.model, GOAL, "wrist_3_link")
    branches = session.solve_ik_branches("wrist_3_link", target, n=6)
    assert len(branches) >= 2  # a 6R pose exposes several branches
    for b in branches:
        reached = q.fk(session.model, b.q, "wrist_3_link")
        assert np.linalg.norm(reached.translation() - target.translation()) < 1e-3
        assert isinstance(b.free, bool)
    # Default order: nearest the current config first.
    here = session.q
    dists = [np.linalg.norm(b.q - here) for b in branches]
    assert dists == sorted(dists)
    # Custom cost re-ranks (here: prefer the LAST branch by inverting the distance order).
    ranked = session.solve_ik_branches(
        "wrist_3_link", target, n=6, cost=lambda qq: -float(np.linalg.norm(qq - here))
    )
    assert np.allclose(ranked[0].q, branches[-1].q)


def test_session_obstacle_toggles_collision(session: StudioSession) -> None:
    ee = q.fk(session.model, np.zeros(6), "wrist_3_link").translation()
    session.add_obstacle("probe", q.SphereShape(0.15), q.Transform.from_translation(ee))
    assert session.collision_state(np.zeros(6)).in_collision
    session.move_obstacle("probe", q.Transform.from_translation(ee + [0.0, 0.0, 1.0]))
    assert not session.collision_state(np.zeros(6)).in_collision
    session.remove_obstacle("probe")


def test_session_plan_with_auto_seed(session: StudioSession) -> None:
    # The studio Plan button with "seed (0 = auto)" left at 0 passes seed=None.
    session.set_start(np.zeros(6))
    session.set_goal_joints(GOAL)
    attempt = session.plan(seed=None)
    assert attempt.result.ok(), attempt.result.message
    session.goal = None


def test_session_plan_and_listeners(session: StudioSession) -> None:
    seen = []
    session.attempt_listeners.append(seen.append)
    session.set_start(np.zeros(6))
    session.set_goal_joints(GOAL)
    attempt = session.plan(seed=1)
    assert attempt.result.ok(), attempt.result.message
    assert np.linalg.norm(attempt.path[-1] - GOAL) < 1e-2
    assert attempt.result.used_seed == 1
    assert seen == [attempt]
    session.attempt_listeners.clear()


def test_session_save_load_round_trip(tmp_path: Path, session: StudioSession) -> None:
    session.add_obstacle(
        "keeper", q.BoxShape(np.array([0.1, 0.1, 0.1])),
        q.Transform.from_translation(np.array([0.8, 0.0, 0.1])),
    )
    session.robot.acm.allow("probe_a", "probe_b")
    session.set_config(GOAL)
    file = tmp_path / "session.qmps"
    session.save(file)

    back = StudioSession.load(file)
    assert back.model.name == session.model.name
    assert "keeper" in back.obstacles
    assert back.robot.acm.is_allowed("probe_a", "probe_b")
    assert np.allclose(back.q, GOAL)
    session.remove_obstacle("keeper")


def test_rerun_logger_records_attempts(tmp_path: Path, session: StudioSession) -> None:
    from quevedomp_studio.rerun_log import RerunLogger

    rrd = tmp_path / "attempts.rrd"
    logger = RerunLogger(session, str(rrd), ee_link="wrist_3_link")
    try:
        session.set_start(np.zeros(6))
        session.set_goal_joints(GOAL)
        attempt = session.plan(seed=3)
        assert attempt.result.ok()
    finally:
        session.attempt_listeners.remove(logger.log_attempt)
    assert rrd.exists() and rrd.stat().st_size > 0


def test_broken_listener_does_not_eat_the_plan(session: StudioSession) -> None:
    def bad_listener(_attempt) -> None:
        raise RuntimeError("boom")

    session.attempt_listeners.append(bad_listener)
    try:
        session.set_start(np.zeros(6))
        session.set_goal_joints(GOAL)
        attempt = session.plan(seed=4)  # must not raise
        assert attempt.result.ok()
    finally:
        session.attempt_listeners.remove(bad_listener)


def test_fixture_shorthand_builds_sessions() -> None:
    from quevedomp_studio.__main__ import FIXTURE_ROBOTS, fixture_session

    for name in FIXTURE_ROBOTS:
        s = fixture_session(name)  # resolves URDF + all mesh dirs, builds the scene
        assert s.dof >= 2, name


def test_rbrobout_inlet_tcp_ik() -> None:
    # The restored jointA TCP frame must be the IK dropdown's default, and IK must reach a
    # pose generated by FK on it (the static camera/fiducial leaves must NOT be offered).
    from quevedomp_studio.__main__ import fixture_session

    s = fixture_session("rbrobout_inlet")
    links = s.ik_links()
    assert links[0] == "ee_9636_jA_tcp"
    assert "third_person_camera_link" not in links

    q_ref = np.zeros(s.dof)
    for joint in s.movable_joints():
        limits = joint.limits
        if limits.has_position_limit:
            q_ref[joint.dof_index] = np.clip(0.3, limits.lower, limits.upper)
    target = q.fk(s.model, q_ref, "ee_9636_jA_tcp")
    result = s.solve_ik("ee_9636_jA_tcp", target, seed=q_ref + 0.02)
    assert result.success, repr(result)


def test_mesh_obstacle_toggles_collision(session: StudioSession) -> None:
    mesh = q.load_mesh(str(FIXTURES / "meshes" / "cube.obj"))
    ee = q.fk(session.model, np.zeros(6), "wrist_3_link").translation()
    session.add_obstacle("cube", mesh, q.Transform.from_translation(ee))
    assert session.collision_state(np.zeros(6)).in_collision
    session.remove_obstacle("cube")
    assert not session.collision_state(np.zeros(6)).in_collision


def test_rbrobout_inlet_srdf_acm_makes_rest_pose_free() -> None:
    # Without the SRDF ACM the baked EE / lift contacts read as self-collision and every
    # planning problem is rejected at the start config; the SRDF must clear the rest pose.
    from quevedomp_studio.__main__ import fixture_session

    s = fixture_session("rbrobout_inlet")
    adjacent_only = len([j for j in s.model.joints])
    assert len(s.robot.acm) > adjacent_only  # SRDF pairs actually loaded
    assert not s.collision_state(np.zeros(s.dof)).in_collision


# ---- Full app, headless viser server -------------------------------------------------------------


@pytest.fixture(scope="module")
def app(session: StudioSession) -> StudioApp:
    application = StudioApp(session, host="127.0.0.1", port=8123)
    yield application
    application.server.stop()


def test_app_renders_robot(app: StudioApp) -> None:
    nodes = [n for link_nodes in app.robot_view.link_nodes.values() for n in link_nodes]
    assert len(nodes) > 0  # UR5 collision meshes resolved and added to the scene
    assert len(app.sliders) == 6


def test_interactive_ik_is_stable(session: StudioSession) -> None:
    # Seeded-only tracking: same seed + same target => identical solution, every time (no
    # random restarts to hop branches — the gizmo-flicker regression gate).
    target = q.fk(session.model, GOAL, "wrist_3_link")
    a = session.solve_ik("wrist_3_link", target, seed=GOAL + 0.05, interactive=True)
    b = session.solve_ik("wrist_3_link", target, seed=GOAL + 0.05, interactive=True)
    assert a.success and b.success
    assert a.restarts == 0 and b.restarts == 0
    assert np.allclose(a.q, b.q)


def test_app_ik_branch_picker(app: StudioApp) -> None:
    app.set_config(GOAL)
    app._snap_gizmo()  # gizmo at a reachable pose (the current EE pose)
    app._on_ik_branches()
    assert len(app._ik_branches) >= 2
    assert app.ik_branch_pick.options[0] != "—"
    assert "branches" in app.ik_status.value
    # Picking the last branch applies its config to the robot.
    app.ik_branch_pick.value = app.ik_branch_pick.options[-1]
    app._on_ik_branch_pick()
    assert np.allclose(app.session.q, app._ik_branches[-1].q)


def test_sample_path_endpoints_and_density() -> None:
    path = [np.array([0.0, 0.0]), np.array([1.0, 0.0]), np.array([1.0, 1.0])]
    dense = StudioSession.sample_path(path, samples_per_segment=8)
    assert dense.shape == (17, 2)  # 2 segments x 8 + final waypoint
    assert np.allclose(dense[0], path[0])
    assert np.allclose(dense[-1], path[-1])
    assert np.allclose(dense[8], path[1])  # waypoints are ON the dense curve


def test_app_obstacle_plan_and_scrub(app: StudioApp) -> None:
    # The scripted version of the manual session: obstacle -> start/goal -> plan -> scrub.
    # (UR5 at zeros reaches horizontally through x~0.45, z~0 — keep the obstacle above that.)
    app.add_obstacle(
        "shelf", q.BoxShape(np.array([0.05, 0.4, 0.2])),
        q.Transform.from_translation(np.array([0.45, 0.0, 0.9])),
    )
    assert not app.session.collision_state(np.zeros(6)).in_collision
    assert not app.session.collision_state(GOAL).in_collision
    app.set_config(np.zeros(6))
    app._set_start()
    app.set_config(GOAL)
    app._set_goal()

    attempt = app.plan_now(seed=2)
    assert attempt.result.ok(), attempt.result.message
    assert len(app._path_nodes) > 0  # the path IS drawn

    app.scrub.value = 0.5
    app._on_scrub()  # animates the robot to mid-path without raising
    assert app.session.q.shape == (6,)


def test_app_play_animates_to_the_end(app: StudioApp) -> None:
    assert app._last_attempt is not None  # planned by the previous test (same module fixture)
    app.scrub.value = 0.0
    app.play(blocking=True, duration=0.3)
    assert app.scrub.value == 1.0
    assert not app._playing


def test_session_v2_round_trips_the_whole_problem(tmp_path: Path, session: StudioSession) -> None:
    session.set_start(np.zeros(6))
    session.set_goal_pose("wrist_3_link", q.fk(session.model, GOAL, "wrist_3_link"), pos_tol=5e-3)
    session.planner_params.edge_resolution = 0.017
    session.planner_params.max_link_sweep = 0.005
    session.timeout = 7.5
    session.smooth = False
    file = tmp_path / "bench.qmps"
    session.save(file)

    back = StudioSession.load(file)
    assert back.goal is not None and back.goal.kind == "pose"
    assert back.goal.link == "wrist_3_link"
    assert back.goal.pos_tol == pytest.approx(5e-3)
    assert back.goal.pose.is_approx(session.goal.pose, 1e-9)
    assert back.planner_params.edge_resolution == pytest.approx(0.017)
    assert back.planner_params.max_link_sweep == pytest.approx(0.005)
    assert back.timeout == pytest.approx(7.5)
    assert back.smooth is False
    session.goal = None  # don't leak problem state into other tests
    session.planner_params.max_link_sweep = 0.0
    session.smooth = True


def test_app_save_load_rebuilds_ui(tmp_path: Path, app: StudioApp) -> None:
    # The in-UI benchmark loop: configure -> save -> load -> everything is back on screen.
    app.add_obstacle(
        "bench_box", q.BoxShape(np.array([0.1, 0.1, 0.1])),
        q.Transform.from_translation(np.array([-0.6, -0.5, 0.2])),  # behind the robot: clear of GOAL
    )
    app.set_config(GOAL)
    assert not app.session.collision_state(GOAL).in_collision
    app._set_start()
    app._set_goal()
    file = str(tmp_path / "bench_ui.qmps")
    app.session_path.value = file
    app._on_save_session()
    assert "saved" in app.session_status.value

    app.load_session(file)
    assert len(app.sliders) == 6  # UI rebuilt
    assert "bench_box" in app.session.obstacles  # collision state restored...
    assert "bench_box" in app.obstacle_view.nodes  # ...and rendered
    assert app.session.goal is not None and app.session.goal.kind == "joint"
    assert np.allclose(app.session.q, GOAL)
    attempt = app.plan_now(seed=11)  # the restored problem is immediately plannable
    assert attempt.result.ok(), attempt.result.message
