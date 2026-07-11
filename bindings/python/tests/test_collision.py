"""Task 4a.4 — collision scene + queries.

Mirrors tests/unit/test_collision_fcl.cpp: the same inline URDFs (sphere robot, prismatic
two-sphere robot) against closed-form primitive cases, plus the boundary-specific gates
(zero-copy BatchResult views, (N, dof) numpy batches).
"""

from pathlib import Path

import numpy as np
import pytest

import quevedomp as q

FIXTURES = Path(__file__).resolve().parents[3] / "tests" / "fixtures"

# One link, one sphere (r=0.5) at the base origin — closed-form vs environment primitives.
SPHERE_ROBOT = """<robot name="r">
  <link name="base"><collision><geometry><sphere radius="0.5"/></geometry></collision></link>
</robot>"""

# Two spheres (r=0.3) joined by a prismatic joint along +x, child nominal at x=1.0:
# q=0 -> centers 1.0 apart (clear); q=-0.5 -> 0.5 apart (self-colliding).
PRISMATIC_ROBOT = """<robot name="arm">
  <link name="base"><collision><geometry><sphere radius="0.3"/></geometry></collision></link>
  <link name="link1"><collision><geometry><sphere radius="0.3"/></geometry></collision></link>
  <joint name="j1" type="prismatic">
    <parent link="base"/><child link="link1"/>
    <origin xyz="1 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
</robot>"""


def at_x(x: float) -> q.Transform:
    return q.Transform.from_translation(np.array([x, 0.0, 0.0]))


def sphere_scene(obstacle_x: float):
    model = q.RobotModel.from_urdf(SPHERE_ROBOT)
    robot = q.RobotInstance(model)
    env = q.SceneDescription()
    env.add("obj", q.SphereShape(0.5), at_x(obstacle_x))
    scene = q.make_static_scene(model, env)
    return robot, scene, scene.make_workspace()


def no_self() -> q.QueryOptions:
    opts = q.QueryOptions()
    opts.check_self_collision = False
    return opts


# ---- Boolean queries (closed-form, mirrors the C++ cases) --------------------------------------


def test_sphere_sphere_overlap_and_clear() -> None:
    for x, expected in ((0.8, True), (1.2, False)):  # radii sum to 1.0
        robot, scene, ws = sphere_scene(x)
        assert scene.query(robot, np.zeros(0), no_self(), ws).in_collision is expected


def test_box_environment() -> None:
    model = q.RobotModel.from_urdf(SPHERE_ROBOT)
    robot = q.RobotInstance(model)
    env = q.SceneDescription()
    env.add("box", q.BoxShape(np.array([0.1, 0.1, 0.1])), at_x(0.55))
    scene = q.make_static_scene(model, env)
    ws = scene.make_workspace()
    assert scene.query(robot, np.zeros(0), no_self(), ws).in_collision


# ---- Distance ----------------------------------------------------------------------------------


def test_distance_query() -> None:
    robot, scene, ws = sphere_scene(1.5)  # surfaces 0.5 apart
    opts = no_self()
    opts.distance = True
    opts.max_distance = 2.0
    r = scene.query(robot, np.zeros(0), opts, ws)
    assert not r.in_collision
    assert r.min_distance == pytest.approx(0.5, abs=1e-3)


# ---- Batch (the primary query) + zero-copy views ------------------------------------------------


def test_query_batch_and_self_collision() -> None:
    model = q.RobotModel.from_urdf(PRISMATIC_ROBOT)
    robot = q.RobotInstance(model)
    scene = q.make_static_scene(model, q.SceneDescription())
    ws = scene.make_workspace()

    qs = np.array([[0.0], [-0.5], [-0.9]])
    r = scene.query_batch(robot, qs, q.QueryOptions(), ws)
    assert r.in_collision.shape == (3,)
    assert r.in_collision.dtype == np.uint8
    assert list(r.in_collision) == [0, 1, 1]
    assert np.shares_memory(r.in_collision, r.in_collision)  # view, not copy

    # The ACM must silence the pair, exactly as in C++.
    robot.acm.allow("base", "link1")
    r2 = scene.query_batch(robot, qs, q.QueryOptions(), ws)
    assert list(r2.in_collision) == [0, 0, 0]


# ---- check_edge (the RRT primitive) --------------------------------------------------------------


def test_check_edge() -> None:
    model = q.RobotModel.from_urdf(PRISMATIC_ROBOT)
    robot = q.RobotInstance(model)
    scene = q.make_static_scene(model, q.SceneDescription())
    ws = scene.make_workspace()

    free = q.check_edge(scene, robot, np.array([0.0]), np.array([0.2]), 0.05, q.QueryOptions(), ws)
    assert free.valid
    assert free.first_contact_t == 1.0

    hit = q.check_edge(scene, robot, np.array([0.0]), np.array([-0.9]), 0.05, q.QueryOptions(), ws)
    assert not hit.valid
    assert hit.first_contact_t < 1.0


# ---- Scene editing --------------------------------------------------------------------------------


def test_add_move_remove_object() -> None:
    model = q.RobotModel.from_urdf(SPHERE_ROBOT)
    robot = q.RobotInstance(model)
    scene = q.make_static_scene(model, q.SceneDescription())
    ws = scene.make_workspace()

    handle = scene.add_object("obs", q.SphereShape(0.5), at_x(0.8))
    assert scene.query(robot, np.zeros(0), no_self(), ws).in_collision
    scene.move_object(handle, at_x(2.0))
    assert not scene.query(robot, np.zeros(0), no_self(), ws).in_collision
    scene.move_object(handle, at_x(0.8))
    scene.remove_object(handle)
    assert not scene.query(robot, np.zeros(0), no_self(), ws).in_collision


# ---- Construction ---------------------------------------------------------------------------------


def test_force_optix_raises_when_unavailable() -> None:
    if q.optix_available():
        pytest.skip("OptiX build: ForceOptix is expected to work here")
    model = q.RobotModel.from_urdf(SPHERE_ROBOT)
    with pytest.raises(RuntimeError):
        q.make_static_scene(model, q.SceneDescription(), q.BackendHint.ForceOptix)


def test_ur5_mesh_robot_scene_builds() -> None:
    model = q.RobotModel.from_urdf((FIXTURES / "robots" / "ur5.urdf").read_text())
    robot = q.RobotInstance(model)
    meshes = q.MeshSources(
        {"example-robot-data": str(FIXTURES / "robots" / "meshes" / "example-robot-data")}
    )
    scene = q.make_static_scene(model, q.SceneDescription(), q.BackendHint.Auto, meshes)
    ws = scene.make_workspace()
    assert not scene.query(robot, np.zeros(6), no_self(), ws).in_collision
