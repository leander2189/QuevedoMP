"""Roadmap R5 — PrmPlanner bindings: build a roadmap once (PrmBuildStats), answer many queries
around a wall, determinism per seed, and the 'prm' registry stub failing loudly."""

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
    return model, robot, scene


def _params(seed):
    p = q.PrmParams()
    p.num_nodes = 800
    p.k_neighbors = 12
    p.edge_resolution = 0.02
    p.seed = seed
    return p


def _problem(start, goal, seed):
    prob = q.PlanningProblem()
    prob.start = np.asarray(start, dtype=float)
    prob.goal = q.JointGoal(np.asarray(goal, dtype=float), 1e-3)
    prob.timeout = 5.0
    prob.seed = seed
    return prob


def _free(scene, robot, path, res=0.02):
    ws = scene.make_workspace()
    for a, b in zip(path[:-1], path[1:]):
        if not q.check_edge(scene, robot, np.asarray(a), np.asarray(b), res, q.QueryOptions(), ws).valid:
            return False
    return True


def test_registry_stub_fails_loudly(cell):
    model, robot, scene = cell
    assert "prm" in q.registered_planners()
    pp = q.PlannerParams()
    pp.algorithm = "prm"
    with pytest.raises(Exception):
        q.make_planner(pp, robot, scene)


def test_build_and_solve_around_wall(cell):
    model, robot, scene = cell
    prm, stats = q.make_prm_planner(_params(3), robot, scene)
    assert stats.nodes > 100
    assert stats.edges > stats.nodes
    assert stats.build_seconds > 0.0

    r = prm.plan(_problem([-1.0, -1.0], [1.0, -1.0], 42))
    assert r.ok(), r.message
    assert _free(scene, robot, list(r.path))
    assert max(float(w[1]) for w in r.path) > 0.4  # routes up and over the wall


def test_one_roadmap_many_queries(cell):
    model, robot, scene = cell
    prm, _ = q.make_prm_planner(_params(5), robot, scene)
    for s, g in [([-1.0, -1.0], [1.0, -1.0]), ([-1.5, 0.0], [1.5, 0.0])]:
        r = prm.plan(_problem(s, g, 1))
        assert r.ok(), r.message
        assert _free(scene, robot, list(r.path))


def test_deterministic_per_seed(cell):
    model, robot, scene = cell
    a, _ = q.make_prm_planner(_params(9), robot, scene)
    b, _ = q.make_prm_planner(_params(9), robot, scene)
    ra = a.plan(_problem([-1.0, -1.0], [1.0, -1.0], 100))
    rb = b.plan(_problem([-1.0, -1.0], [1.0, -1.0], 100))
    assert ra.ok() and len(ra.path) == len(rb.path)
    for pa, pb in zip(ra.path, rb.path):
        assert np.allclose(pa, pb, atol=1e-12)
