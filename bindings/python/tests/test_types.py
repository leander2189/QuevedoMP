"""Task 4a.2 — core vocabulary types: Transform/Pose/Mesh/JointState/Waypoint.

Mirrors the invariants of tests/unit/test_types.cpp; adds the boundary-specific checks
(numpy round-trips, Mesh zero-copy views) the C++ suite cannot express.
"""

import numpy as np
import pytest

import quevedomp as q

IDENTITY_Q = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz


def test_version() -> None:
    assert q.__version__


# ---- Transform -------------------------------------------------------------------------------


def test_identity_is_default() -> None:
    assert q.Transform().is_approx(q.Transform.identity())
    assert np.allclose(q.Transform.identity().matrix(), np.eye(4))


def test_from_parts_roundtrip() -> None:
    t = np.array([0.1, -0.2, 0.3])
    # 90 deg about Z: w=cos(45), z=sin(45)
    qz = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    tf = q.Transform.from_parts(t, qz)
    assert np.allclose(tf.translation(), t)
    assert np.allclose(tf.quaternion(), qz)


def test_quaternion_normalized_on_entry() -> None:
    tf = q.Transform.from_rotation(2.0 * IDENTITY_Q)  # non-unit input
    assert np.allclose(tf.matrix(), np.eye(4))


def test_compose_and_apply() -> None:
    a = q.Transform.from_translation(np.array([1.0, 0.0, 0.0]))
    b = q.Transform.from_parts(
        np.array([0.0, 1.0, 0.0]), np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    )
    p = (a * b) * np.array([1.0, 0.0, 0.0])
    # b rotates x->y then translates +y; a translates +x.
    assert np.allclose(p, [1.0, 2.0, 0.0])


def test_inverse() -> None:
    tf = q.Transform.from_parts(
        np.array([0.4, 0.5, 0.6]), np.array([np.cos(0.3), np.sin(0.3), 0.0, 0.0])
    )
    assert (tf * tf.inverse()).is_approx(q.Transform.identity(), 1e-9)


def test_from_matrix_roundtrip() -> None:
    tf = q.Transform.from_parts(
        np.array([1.0, 2.0, 3.0]), np.array([np.cos(0.7), 0.0, np.sin(0.7), 0.0])
    )
    assert q.Transform.from_matrix(tf.matrix()).is_approx(tf, 1e-12)


# ---- Pose ------------------------------------------------------------------------------------


def test_pose_defaults_match_cpp() -> None:
    p = q.Pose()
    assert p.pos_tol == pytest.approx(1e-3)
    assert p.rot_tol == pytest.approx(1e-2)
    assert p.tf.is_approx(q.Transform.identity())


def test_pose_kwargs() -> None:
    p = q.Pose(q.Transform.from_translation(np.array([0.0, 0.0, 1.0])), pos_tol=1e-4)
    assert np.allclose(p.tf.translation(), [0.0, 0.0, 1.0])
    assert p.pos_tol == pytest.approx(1e-4)


# ---- Mesh (the zero-copy gate of Task 4a.2) ----------------------------------------------------


def _tri() -> q.Mesh:
    v = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    f = np.array([[0, 1, 2]], dtype=np.int32)
    return q.Mesh(v, f)


def test_mesh_roundtrip() -> None:
    m = _tri()
    assert m.vertices.shape == (3, 3)
    assert m.triangles.shape == (1, 3)
    assert m.vertices.dtype == np.float64
    assert m.triangles.dtype == np.int32
    assert np.allclose(m.vertices[1], [1.0, 0.0, 0.0])


def test_mesh_vertices_are_zero_copy_views() -> None:
    m = _tri()
    a, b = m.vertices, m.vertices
    assert np.shares_memory(a, b)  # two views over the SAME C++ buffer, not copies
    a[0, 0] = 42.0  # mutate through the view...
    assert m.vertices[0, 0] == 42.0  # ...and the mesh sees it


def test_mesh_view_outlives_mesh_object() -> None:
    v = _tri().vertices  # the view must keep the Mesh alive (owner handle)
    assert np.allclose(v[2], [0.0, 1.0, 0.0])


def test_empty_mesh() -> None:
    m = q.Mesh()
    assert m.vertices.shape == (0, 3)
    assert m.triangles.shape == (0, 3)


# ---- JointState / Waypoint ---------------------------------------------------------------------


def test_waypoint_defaults_and_numpy_joint_vectors() -> None:
    w = q.Waypoint()
    assert w.time == 0.0
    w.state.pos = np.array([0.1, 0.2, 0.3])
    assert isinstance(w.state.pos, np.ndarray)
    assert np.allclose(w.state.pos, [0.1, 0.2, 0.3])
