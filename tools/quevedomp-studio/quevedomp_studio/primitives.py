"""Triangle-mesh generators for the collision primitives.

Everything renders through viser's add_mesh_simple, so the studio depends on exactly one
scene-object API regardless of geometry type (and matches what the collision backend sees,
since QuevedoMP renders collision geometry, not visual geometry).
"""

from __future__ import annotations

import numpy as np


def box_mesh(half_extents: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    h = np.asarray(half_extents, dtype=float)
    corners = np.array(
        [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float
    ) * h
    faces = np.array(
        [
            [0, 1, 3], [0, 3, 2],  # -x
            [4, 6, 7], [4, 7, 5],  # +x
            [0, 4, 5], [0, 5, 1],  # -y
            [2, 3, 7], [2, 7, 6],  # +y
            [0, 2, 6], [0, 6, 4],  # -z
            [1, 5, 7], [1, 7, 3],  # +z
        ],
        dtype=np.int32,
    )
    return corners, faces


def sphere_mesh(radius: float, n_lat: int = 12, n_lon: int = 24) -> tuple[np.ndarray, np.ndarray]:
    lat = np.linspace(0.0, np.pi, n_lat + 1)
    lon = np.linspace(0.0, 2.0 * np.pi, n_lon, endpoint=False)
    vertices = [np.array([0.0, 0.0, radius])]
    for t in lat[1:-1]:
        for p in lon:
            vertices.append(
                radius * np.array([np.sin(t) * np.cos(p), np.sin(t) * np.sin(p), np.cos(t)])
            )
    vertices.append(np.array([0.0, 0.0, -radius]))
    v = np.array(vertices)

    faces: list[list[int]] = []
    def ring(i: int) -> int:  # first index of latitude ring i (i in 0..n_lat-2)
        return 1 + i * n_lon

    for j in range(n_lon):  # top cap
        faces.append([0, ring(0) + j, ring(0) + (j + 1) % n_lon])
    for i in range(n_lat - 2):  # quads between rings
        for j in range(n_lon):
            a, b = ring(i) + j, ring(i) + (j + 1) % n_lon
            c, d = ring(i + 1) + j, ring(i + 1) + (j + 1) % n_lon
            faces.extend([[a, c, d], [a, d, b]])
    bottom = len(v) - 1
    for j in range(n_lon):  # bottom cap
        faces.append([bottom, ring(n_lat - 2) + (j + 1) % n_lon, ring(n_lat - 2) + j])
    return v, np.array(faces, dtype=np.int32)


def cylinder_mesh(radius: float, length: float, n: int = 24) -> tuple[np.ndarray, np.ndarray]:
    """Axis along +Z, centered at the origin (URDF/QuevedoMP convention)."""
    half = length / 2.0
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    top = [np.array([radius * np.cos(a), radius * np.sin(a), half]) for a in angles]
    bottom = [np.array([radius * np.cos(a), radius * np.sin(a), -half]) for a in angles]
    v = np.array(top + bottom + [[0.0, 0.0, half], [0.0, 0.0, -half]])
    c_top, c_bot = 2 * n, 2 * n + 1

    faces: list[list[int]] = []
    for j in range(n):
        k = (j + 1) % n
        faces.extend([[j, n + j, n + k], [j, n + k, k]])  # side quad
        faces.append([c_top, j, k])  # top cap
        faces.append([c_bot, n + k, n + j])  # bottom cap
    return v, np.array(faces, dtype=np.int32)
