"""Task 4a.3 — RobotModel/RobotInstance, FK/Jacobian/IK, mesh loading.

Mirrors tests/unit/test_{robot_model,fk,jacobian,ik}.cpp against the same fixtures:
two_link.urdf for analytic FK values, ur5.urdf for the FK->IK round trip.
"""

from pathlib import Path

import numpy as np
import pytest

import quevedomp as q

FIXTURES = Path(__file__).resolve().parents[3] / "tests" / "fixtures"


def load_model(name: str) -> q.RobotModel:
    return q.RobotModel.from_urdf((FIXTURES / "robots" / name).read_text())


@pytest.fixture(scope="module")
def two_link() -> q.RobotModel:
    return load_model("two_link.urdf")


@pytest.fixture(scope="module")
def ur5() -> q.RobotModel:
    return load_model("ur5.urdf")


# ---- RobotModel ------------------------------------------------------------------------------


def test_model_topology(two_link: q.RobotModel) -> None:
    assert two_link.dof == 2
    assert two_link.root_link == "base_link"
    names = [link.name for link in two_link.links]
    assert {"base_link", "link1", "link2", "ee_link"} <= set(names)

    j1 = two_link.find_joint("joint1")
    assert j1 is not None
    assert j1.type == q.JointType.Revolute
    assert j1.is_movable()
    assert j1.dof_index == 0
    assert two_link.find_joint("joint2").type == q.JointType.Prismatic
    assert two_link.find_link("nope") is None


def test_chain_to(two_link: q.RobotModel) -> None:
    chain = two_link.chain_to("ee_link")
    assert chain.base_link == "base_link"
    assert chain.tip_link == "ee_link"
    assert len(chain.joints) == 3  # joint1, joint2, joint_ee (fixed included)
    with pytest.raises(RuntimeError):
        two_link.chain_to("nope")


def test_bad_urdf_raises() -> None:
    with pytest.raises(RuntimeError):
        q.RobotModel.from_urdf("<robot name='broken'>")


# ---- FK (mirrors test_fk.cpp analytic values) --------------------------------------------------


def test_fk_two_link_zeros(two_link: q.RobotModel) -> None:
    ee = q.fk(two_link, np.zeros(2), "ee_link")
    assert np.linalg.norm(ee.translation() - [0.2, 0.0, 0.15]) < 1e-12
    assert np.linalg.norm(ee.rotation() - np.eye(3)) < 1e-12


def test_fk_all_consistent_with_fk(two_link: q.RobotModel) -> None:
    qv = np.array([0.3, 0.05])
    poses = q.fk_all(two_link, qv)
    idx = [link.name for link in two_link.links].index("ee_link")
    assert np.linalg.norm(poses[idx].matrix() - q.fk(two_link, qv, "ee_link").matrix()) < 1e-12


def test_fk_errors(two_link: q.RobotModel) -> None:
    with pytest.raises(RuntimeError):
        q.fk_all(two_link, np.zeros(1))
    with pytest.raises(RuntimeError):
        q.fk(two_link, np.zeros(2), "nope")


# ---- Jacobian ----------------------------------------------------------------------------------


def test_jacobian_matches_finite_differences(two_link: q.RobotModel) -> None:
    qv = np.array([0.4, 0.1])
    jac = q.jacobian(two_link, qv, "ee_link")
    assert jac.shape == (6, 2)
    h = 1e-7
    for i in range(2):
        dq = np.zeros(2)
        dq[i] = h
        p_plus = q.fk(two_link, qv + dq, "ee_link").translation()
        p_minus = q.fk(two_link, qv - dq, "ee_link").translation()
        assert np.allclose(jac[:3, i], (p_plus - p_minus) / (2 * h), atol=1e-6)


# ---- IK (mirrors test_ik.cpp: FK -> IK round trip on UR5) --------------------------------------


def test_ik_round_trip_ur5(ur5: q.RobotModel) -> None:
    assert ur5.dof == 6
    q_ref = np.array([0.4, -1.1, 1.3, -0.7, 0.9, 0.2])
    target = q.fk(ur5, q_ref, "wrist_3_link")

    ik = q.make_numerical_ik(ur5)
    result = ik.solve("wrist_3_link", target, seed=q_ref + 0.05)
    assert result.success, repr(result)
    assert result.pos_error < 1e-4
    assert result.rot_error < 1e-3
    # The solution's FK must land on the target (up to the tolerances), whatever branch it found.
    reached = q.fk(ur5, result.q, "wrist_3_link")
    assert np.linalg.norm(reached.translation() - target.translation()) < 1e-4


def test_ik_deterministic_per_seed(ur5: q.RobotModel) -> None:
    target = q.fk(ur5, np.array([0.3, -0.9, 1.1, -0.5, 0.7, 0.1]), "wrist_3_link")
    opts = q.IkOptions()
    opts.seed = 1234
    a = q.make_numerical_ik(ur5, opts).solve("wrist_3_link", target)
    b = q.make_numerical_ik(ur5, opts).solve("wrist_3_link", target)
    assert a.success and b.success
    assert np.allclose(a.q, b.q)


# ---- RobotInstance / ACM -----------------------------------------------------------------------


def test_acm_symmetric_and_mutable() -> None:
    acm = q.AllowedCollisionMatrix()
    acm.allow("link1", "link2")
    assert acm.is_allowed("link2", "link1")  # order-independent
    assert len(acm) == 1
    assert ("link1", "link2") in acm.pairs()
    acm.disallow("link2", "link1")
    assert not acm.is_allowed("link1", "link2")


def test_robot_instance_holds_model_and_acm(two_link: q.RobotModel) -> None:
    inst = q.RobotInstance(two_link)
    assert inst.model.name == two_link.name
    inst.acm.allow("link1", "link2")  # mutation must stick (reference, not copy)
    assert inst.acm.is_allowed("link1", "link2")


# ---- Mesh loading ------------------------------------------------------------------------------


def test_load_mesh_cube() -> None:
    mesh = q.load_mesh(str(FIXTURES / "meshes" / "cube.obj"))
    assert mesh.vertices.shape[0] > 0
    assert mesh.triangles.shape[0] >= 12  # a cube triangulates to >= 12 faces
    with pytest.raises(RuntimeError):
        q.load_mesh(str(FIXTURES / "meshes" / "missing.obj"))


def test_resolve_mesh_uri() -> None:
    dirs = {"mypkg": "/opt/mypkg"}
    assert q.resolve_mesh_uri("package://mypkg/meshes/a.stl", dirs) == "/opt/mypkg/meshes/a.stl"
    assert q.resolve_mesh_uri("file:///tmp/a.stl", dirs) == "/tmp/a.stl"
    assert q.resolve_mesh_uri("rel/a.stl", dirs, base_dir="/base") == "/base/rel/a.stl"
    with pytest.raises(RuntimeError):
        q.resolve_mesh_uri("package://unknown/a.stl", dirs)
