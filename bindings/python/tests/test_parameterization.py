"""Task 3.4 — PathSpline + time parameterization bindings.

Mirrors tests/unit/test_parameterization.cpp, plus the differential check the C++ suite cannot
run: Phase A (joint vel/acc only) against pip-installed toppra on the same densely-sampled
geometric path (skipped when toppra is not installed — it is a TEST dependency, never a library
one).
"""

from pathlib import Path

import numpy as np
import pytest

import quevedomp as q

FIXTURES = Path(__file__).resolve().parents[3] / "tests" / "fixtures"


@pytest.fixture(scope="module")
def ur5() -> q.RobotModel:
    return q.RobotModel.from_urdf((FIXTURES / "robots" / "ur5.urdf").read_text())


WAYPOINTS = [
    np.zeros(6),
    np.array([0.5, -0.9, 1.1, -0.4, 0.6, 0.3]),
    np.array([-0.3, -1.3, 1.5, -1.0, 0.2, -0.5]),
]


def test_path_spline_eval_and_derivatives(ur5: q.RobotModel) -> None:
    sp = q.PathSpline.fit(WAYPOINTS)
    assert sp.dof == 6
    assert np.allclose(sp.eval(0.0), WAYPOINTS[0], atol=1e-9)
    assert np.allclose(sp.eval(1.0), WAYPOINTS[-1], atol=1e-9)
    # d1 by finite difference.
    s, h = 0.4, 1e-6
    fd = (sp.eval(s + h) - sp.eval(s - h)) / (2 * h)
    assert np.allclose(fd, sp.d1(s), atol=1e-4)
    assert sp.d3(0.5).shape == (6,)


def test_parametrize_respects_limits(ur5: q.RobotModel) -> None:
    sp = q.PathSpline.fit(WAYPOINTS)
    lim = q.limits_from_model(ur5, default_acceleration=8.0)
    lim.tip_link = "wrist_3_link"
    lim.tip_linear_velocity = 0.5
    opt = q.ParameterizationOptions()
    opt.nodes = 300
    r = q.parametrize(ur5, sp, lim, opt)
    assert r.success, r.message
    assert r.duration > 0.0

    times = [w.time for w in r.trajectory]
    assert all(b > a for a, b in zip(times, times[1:]))
    assert np.linalg.norm(r.trajectory[0].state.vel) < 1e-6  # rest-to-rest
    assert np.linalg.norm(r.trajectory[-1].state.vel) < 1e-6
    for w in r.trajectory:
        assert np.all(np.abs(w.state.vel) <= lim.max_velocity * 1.02)
        assert np.all(np.abs(w.state.acc) <= lim.max_acceleration * 1.02)
        tip = q.jacobian(ur5, w.state.pos, "wrist_3_link") @ w.state.vel
        assert np.linalg.norm(tip[:3]) <= lim.tip_linear_velocity * 1.02


def test_fit_collision_free_avoids_wall() -> None:
    gantry = q.RobotModel.from_urdf("""<robot name="gantry2d">
      <link name="base"/>
      <joint name="jx" type="prismatic"><parent link="base"/><child link="cx"/>
        <origin xyz="0 0 0"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
      <link name="cx"/>
      <joint name="jy" type="prismatic"><parent link="cx"/><child link="ee"/>
        <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
      <link name="ee"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
    </robot>""")
    robot = q.RobotInstance(gantry)
    env = q.SceneDescription()
    env.add("wall", q.BoxShape(np.array([0.1, 1.25, 0.5])),
            q.Transform.from_translation(np.array([0.0, -0.75, 0.0])))
    scene = q.make_static_scene(gantry, env)
    ws = scene.make_workspace()

    detour = [np.array([-1.0, -1.0]), np.array([-0.25, 0.62]), np.array([0.25, 0.62]),
              np.array([1.0, -1.0])]
    disc = q.EdgeDiscretization()
    disc.joint_resolution = 0.01
    fit = q.fit_collision_free(detour, scene, robot, disc, q.QueryOptions(), ws)
    assert fit.success, fit.message
    samples = np.array([fit.spline.eval(s) for s in np.linspace(0, 1, 500)])
    br = scene.query_batch(robot, samples, q.QueryOptions(), ws)
    assert not br.in_collision.any()


def test_jerk_limited_mode_certifies(ur5: q.RobotModel) -> None:
    sp = q.PathSpline.fit(WAYPOINTS)
    lim = q.limits_from_model(ur5, default_acceleration=8.0)
    lim.max_jerk = np.full(6, 40.0)
    opt = q.ParameterizationOptions()
    opt.nodes = 300
    opt.mode = q.ParameterizationMode.JerkLimited
    r = q.parametrize(ur5, sp, lim, opt)
    assert r.success, r.message
    assert r.max_jerk_violation <= opt.jerk_tolerance

    copt = q.ParameterizationOptions()
    copt.nodes = 300
    convex = q.parametrize(ur5, sp, lim, copt)
    assert r.duration >= convex.duration - 1e-9  # jerk can only slow it down
    for w in r.trajectory:  # Phase A limits still hold after the kernel
        assert np.all(np.abs(w.state.vel) <= lim.max_velocity * 1.03)
        assert np.all(np.abs(w.state.acc) <= lim.max_acceleration * 1.03)


def test_phase_a_matches_toppra(ur5: q.RobotModel) -> None:
    """Differential vs toppra (vel+acc only): same dense geometric path, durations within 2%."""
    toppra = pytest.importorskip("toppra")
    import toppra.constraint as tc

    sp = q.PathSpline.fit(WAYPOINTS)
    ss = np.linspace(0, 1, 200)
    qs = np.array([sp.eval(s) for s in ss])

    vmax = np.full(6, 2.0)
    amax = np.full(6, 6.0)

    lim = q.ParameterizationLimits()
    lim.max_velocity = vmax
    lim.max_acceleration = amax
    opt = q.ParameterizationOptions()
    opt.nodes = 400
    ours = q.parametrize(ur5, sp, lim, opt)
    assert ours.success, ours.message

    path = toppra.SplineInterpolator(ss, qs)
    inst = toppra.algorithm.TOPPRA(
        [tc.JointVelocityConstraint(np.vstack([-vmax, vmax]).T),
         tc.JointAccelerationConstraint(np.vstack([-amax, amax]).T)],
        path, gridpoints=np.linspace(0, 1, 401))
    traj = inst.compute_trajectory(0.0, 0.0)
    assert traj is not None
    assert ours.duration == pytest.approx(traj.duration, rel=0.02)
