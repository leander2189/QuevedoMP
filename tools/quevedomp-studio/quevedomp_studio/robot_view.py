"""Robot + obstacle rendering: QuevedoMP geometry -> viser scene nodes.

Renders the COLLISION geometry (what the planner actually checks — this is a planning IDE,
not a showroom). Each link's shapes become mesh nodes under /robot/<link>/<i>; update_config
re-poses them from fk_all and tints colliding links red using the query witness.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

import quevedomp as q

from .primitives import box_mesh, cylinder_mesh, sphere_mesh
from .session import Obstacle, StudioSession

ROBOT_COLOR = (170, 170, 185)
COLLIDING_COLOR = (220, 60, 50)
OBSTACLE_COLOR = (240, 170, 60)


def geometry_mesh(geom, session: Optional[StudioSession] = None) -> tuple[np.ndarray, np.ndarray]:
    """Any bound geometry (env shape or link CollisionGeometry payload) -> (vertices, faces)."""
    if isinstance(geom, q.Mesh):
        return np.asarray(geom.vertices), np.asarray(geom.triangles)
    if isinstance(geom, q.BoxShape):
        return box_mesh(geom.half_extents)
    if isinstance(geom, q.SphereShape):
        return sphere_mesh(geom.radius)
    if isinstance(geom, q.CylinderShape):
        return cylinder_mesh(geom.radius, geom.length)
    raise TypeError(f"unsupported geometry: {type(geom)!r}")


def link_geometry_mesh(cg, session: StudioSession) -> tuple[np.ndarray, np.ndarray]:
    """One URDF CollisionGeometry -> (vertices, faces), meshes resolved + scaled."""
    if cg.type == q.GeometryType.Mesh:
        path = q.resolve_mesh_uri(cg.mesh_filename, session.package_dirs, session.base_dir)
        mesh = q.load_mesh(path)
        return np.asarray(mesh.vertices) * np.asarray(cg.mesh_scale), np.asarray(mesh.triangles)
    if cg.type == q.GeometryType.Box:
        return box_mesh(cg.box_half_extents)
    if cg.type == q.GeometryType.Sphere:
        return sphere_mesh(cg.sphere_radius)
    return cylinder_mesh(cg.cylinder_radius, cg.cylinder_length)


class RobotView:
    """viser nodes for the robot's links; poses driven by fk_all."""

    def __init__(self, server, session: StudioSession) -> None:
        self.server = server
        self.session = session
        # per link: list of (node, geometry-origin Transform)
        self.link_nodes: dict[str, list] = {}
        for link in session.model.links:
            nodes = []
            for i, cg in enumerate(link.collisions):
                vertices, faces = link_geometry_mesh(cg, session)
                node = server.scene.add_mesh_simple(
                    f"/robot/{link.name}/{i}",
                    vertices=vertices,
                    faces=faces,
                    color=ROBOT_COLOR,
                    flat_shading=True,
                )
                nodes.append((node, cg.origin))
            self.link_nodes[link.name] = nodes

    def update_config(self, q_at: np.ndarray) -> "q.CollisionResult":
        """Re-pose every link at q_at and tint by collision state; returns the query result."""
        poses = self.session.link_poses(q_at)
        state = self.session.collision_state(q_at)
        colliding = set()
        if state.in_collision and state.witness is not None:
            colliding = {state.witness.a, state.witness.b}

        for link, link_pose in zip(self.session.model.links, poses):
            for node, origin in self.link_nodes[link.name]:
                world = link_pose * origin
                node.position = world.translation()
                node.wxyz = world.quaternion()
                node.color = COLLIDING_COLOR if link.name in colliding else ROBOT_COLOR
        return state


class ObstacleView:
    """One viser node + transform gizmo per obstacle."""

    def __init__(self, server, session: StudioSession) -> None:
        self.server = server
        self.session = session
        self.nodes: dict[str, object] = {}
        self.gizmos: dict[str, object] = {}

    def add(self, obstacle: Obstacle, on_moved) -> None:
        vertices, faces = geometry_mesh(obstacle.geometry)
        gizmo = self.server.scene.add_transform_controls(
            f"/obstacles/{obstacle.id}",
            scale=0.4,
            position=obstacle.pose.translation(),
            wxyz=obstacle.pose.quaternion(),
        )
        node = self.server.scene.add_mesh_simple(
            f"/obstacles/{obstacle.id}/mesh",
            vertices=vertices,
            faces=faces,
            color=OBSTACLE_COLOR,
            flat_shading=True,
        )
        self.nodes[obstacle.id] = node
        self.gizmos[obstacle.id] = gizmo

        @gizmo.on_update
        def _(_event, id=obstacle.id, g=gizmo) -> None:
            pose = q.Transform.from_parts(np.asarray(g.position), np.asarray(g.wxyz))
            self.session.move_obstacle(id, pose)
            on_moved(id)

    def remove(self, id: str) -> None:
        self.nodes.pop(id).remove()
        self.gizmos.pop(id).remove()
