"""Task 4a.6 — Task 2a.5 serializer bindings (studio save/load path).

Mirrors tests/unit/test_capture_serialize.cpp round trips; adds the boundary gate that blobs
cross as bytes (binary-safe), not str.
"""

from pathlib import Path

import numpy as np
import pytest

import quevedomp as q

FIXTURES = Path(__file__).resolve().parents[3] / "tests" / "fixtures"


def test_robot_model_round_trip() -> None:
    model = q.RobotModel.from_urdf((FIXTURES / "robots" / "two_link.urdf").read_text())
    blob = q.serialize_robot_model(model)
    assert isinstance(blob, bytes) and len(blob) > 0
    back = q.deserialize_robot_model(blob)
    assert back.name == model.name
    assert back.dof == model.dof
    assert back.source_urdf == model.source_urdf


def test_robot_instance_round_trip_keeps_acm() -> None:
    model = q.RobotModel.from_urdf((FIXTURES / "robots" / "two_link.urdf").read_text())
    inst = q.RobotInstance(model)
    inst.acm.allow("link1", "link2")
    back = q.deserialize_robot_instance(q.serialize_robot_instance(inst))
    assert back.model.name == model.name
    assert back.acm.is_allowed("link2", "link1")


def test_scene_round_trip() -> None:
    env = q.SceneDescription()
    env.add("box", q.BoxShape(np.array([0.1, 0.2, 0.3])), q.Transform.from_translation(np.array([1.0, 0.0, 0.0])))
    env.add("ball", q.SphereShape(0.25))
    back = q.deserialize_scene(q.serialize_scene(env))
    assert [o.id for o in back.objects] == ["box", "ball"]
    assert back.objects[0].geometry.half_extents[2] == pytest.approx(0.3)
    assert np.allclose(back.objects[0].pose.translation(), [1.0, 0.0, 0.0])


def test_bad_blob_raises() -> None:
    with pytest.raises(RuntimeError):
        q.deserialize_robot_model(b"not a blob")
