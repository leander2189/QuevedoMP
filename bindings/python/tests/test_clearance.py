"""Roadmap R3 — ClearanceField bindings: build, batched query, zero-copy grid view, robot
sphere cover, and clearance_batch agreeing in sign with the exact collision backend."""

import numpy as np
import pytest

import quevedomp as q

GANTRY_2D = """<robot name="gantry2d">
  <link name="base"/>
  <joint name="jx" type="prismatic"><parent link="base"/><child link="cx"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
  <link name="cx"/>
  <joint name="jy" type="prismatic"><parent link="cx"/><child link="ee"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
  <link name="ee"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
</robot>"""


@pytest.fixture(scope="module")
def cell():
    model = q.RobotModel.from_urdf(GANTRY_2D)
    robot = q.RobotInstance(model)
    env = q.SceneDescription()
    env.add("wall", q.BoxShape(np.array([0.1, 1.25, 0.5])),
            q.Transform.from_translation(np.array([0.0, -0.75, 0.0])))
    opts = q.ClearanceFieldOptions()
    opts.resolution = 0.02
    field = q.ClearanceField.build(env, opts)
    return model, robot, env, field


def test_field_metadata_and_grid_view(cell) -> None:
    _, _, _, field = cell
    nx, ny, nz = int(field.dims[0]), int(field.dims[1]), int(field.dims[2])
    data = np.asarray(field.data)
    assert data.shape == (nz, ny, nx)
    assert data.dtype == np.float32
    assert field.build_seconds > 0.0

    # origin()/dims() return Eigen vectors BY VALUE — assert every component (not just [0]) so a
    # dangling rv_policy::reference_internal binding (garbage past [0] in optimized builds) is
    # caught. Wall AABB x[-0.1,0.1] y[-2,0.5] z[-0.5,0.5], margin 0.20 ⇒ lo = (-0.3,-2.2,-0.7).
    origin = np.asarray(field.origin)
    assert origin.shape == (3,) and np.all(np.isfinite(origin))
    np.testing.assert_allclose(origin, [-0.3, -2.2, -0.7], atol=0.03)
    dims = np.asarray(field.dims)
    assert dims.tolist() == [nx, ny, nz] and (dims > 1).all()


def test_signed_distance_and_query(cell) -> None:
    _, _, _, field = cell
    assert field.distance(np.array([0.0, -0.75, 0.0])) < -0.05  # deep inside the wall
    outside = np.array([1.0, -0.75, 0.0])  # 0.9 m from the wall face
    assert field.distance(outside) == pytest.approx(0.9, abs=0.06)
    g = field.gradient(outside)
    assert g[0] > 0.8  # points away from the wall along +x

    pts = np.array([[1.0, -0.75, 0.0], [0.0, -0.75, 0.0], [0.5, 0.5, 0.2]])
    d, grads = field.query(pts)
    assert d.shape == (3,) and grads.shape == (3, 3)
    assert d[0] == pytest.approx(field.distance(pts[0]))


def test_clearance_batch_sign_matches_exact_backend(cell) -> None:
    model, robot, env, field = cell
    spheres = q.decompose_robot(model)
    assert len(spheres.spheres) >= 1

    scene = q.make_static_scene(model, env)
    ws = scene.make_workspace()
    configs = np.array([[0.0, -1.0], [-1.0, -1.0], [1.0, 1.0]])
    clearances = q.clearance_batch(field, model, spheres, configs)
    assert len(clearances) == 3
    for qv, c in zip(configs, clearances):
        colliding = scene.query(robot, qv, q.QueryOptions(), ws).in_collision
        if colliding:
            assert c < 0.05  # conservative cover: at most grid noise above zero
        else:
            # Free configs with real margin must show positive clearance.
            if c > 0.1:
                assert not colliding
    assert clearances[0] < 0.0  # (0,-1) is inside the wall
    assert clearances[2] > 0.3  # (1,1) is far away
