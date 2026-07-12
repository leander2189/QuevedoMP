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


def test_session_obstacle_toggles_collision(session: StudioSession) -> None:
    ee = q.fk(session.model, np.zeros(6), "wrist_3_link").translation()
    session.add_obstacle("probe", q.SphereShape(0.15), q.Transform.from_translation(ee))
    assert session.collision_state(np.zeros(6)).in_collision
    session.move_obstacle("probe", q.Transform.from_translation(ee + [0.0, 0.0, 1.0]))
    assert not session.collision_state(np.zeros(6)).in_collision
    session.remove_obstacle("probe")


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
