"""Roadmap R4 — TrajectoryRefiner bindings: make_refiner over a ClearanceField, refiner vs
standalone mode, the exact-backend certificate, and the 'chomp' registry stub failing loudly."""

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
    scene = q.make_static_scene(model, env)
    opts = q.ClearanceFieldOptions()
    opts.resolution = 0.02
    field = q.ClearanceField.build(env, opts)
    spheres = q.decompose_robot(model)
    return model, robot, scene, field, spheres


def _problem(start, goal):
    prob = q.PlanningProblem()
    prob.start = np.asarray(start, dtype=float)
    prob.goal = q.JointGoal(np.asarray(goal, dtype=float), 1e-3)
    prob.timeout = 5.0
    return prob


def _params():
    p = q.RefinerParams()
    p.edge_resolution = 0.02
    p.waypoints = 32
    p.max_iterations = 120
    p.step_size = 0.15
    p.clearance_epsilon = 0.15
    return p


def _min_clearance(field, model, spheres, path, per_seg=12):
    dense = []
    for a, b in zip(path[:-1], path[1:]):
        for k in range(per_seg):
            t = k / per_seg
            dense.append((1 - t) * np.asarray(a) + t * np.asarray(b))
    dense.append(np.asarray(path[-1]))
    return float(np.min(q.clearance_batch(field, model, spheres, np.array(dense))))


def test_registry_stub_fails_loudly(cell):
    model, robot, scene, field, spheres = cell
    assert "chomp" in q.registered_planners()
    pp = q.PlannerParams()
    pp.algorithm = "chomp"
    with pytest.raises(Exception):
        q.make_planner(pp, robot, scene)


def test_standalone_solves_obstacle_free(cell):
    model, robot, scene, field, spheres = cell
    # A goal reachable by a straight line that clears the wall on the right (x=1 column is free).
    refiner = q.make_refiner(_params(), robot, scene, field, spheres)
    r = refiner.plan(_problem([1.0, -1.0], [1.0, 1.0]))
    assert r.ok(), r.message
    assert r.stats.refiner_mode == "standalone"
    assert len(r.path) >= 3


def test_refiner_raises_clearance_over_seed(cell):
    model, robot, scene, field, spheres = cell
    seed = [np.array([1.0, -1.0]), np.array([0.25, 0.0]), np.array([1.0, 1.0])]
    p = _params()
    p.seed = seed
    p.clearance_weight = 4.0
    refiner = q.make_refiner(p, robot, scene, field, spheres)
    r = refiner.plan(_problem([1.0, -1.0], [1.0, 1.0]))
    assert r.ok(), r.message
    assert r.stats.refiner_mode == "refiner"
    seed_clear = _min_clearance(field, model, spheres, seed)
    refined_clear = _min_clearance(field, model, spheres, list(r.path))
    assert refined_clear > seed_clear + 0.02, f"seed={seed_clear} refined={refined_clear}"


def test_deterministic_per_seed(cell):
    model, robot, scene, field, spheres = cell
    p = _params()
    p.seed = [np.array([1.0, -1.0]), np.array([0.25, 0.0]), np.array([1.0, 1.0])]
    refiner = q.make_refiner(p, robot, scene, field, spheres)
    a = refiner.plan(_problem([1.0, -1.0], [1.0, 1.0]))
    b = refiner.plan(_problem([1.0, -1.0], [1.0, 1.0]))
    assert len(a.path) == len(b.path)
    for pa, pb in zip(a.path, b.path):
        assert np.allclose(pa, pb, atol=1e-12)
