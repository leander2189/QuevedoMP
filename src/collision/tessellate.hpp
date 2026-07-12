// collision/tessellate — closed triangle meshes for the collision primitives (Task 3.3d P2).
//
// The OptiX backend is triangles-only: environment primitives used to throw and robot
// primitives were silently skipped (unsound). These tessellations let every primitive ride the
// GPU path. Vertices lie ON the exact surface (inscribed): the maximal radial deficit is
// r·(1 − cos(π/n_lon)·cos(π/(2·n_lat))) for the sphere and r·(1 − cos(π/n)) for the cylinder —
// sub-millimetre at the default segment counts for decimetre-scale parts. Grazing-contact
// fidelity below that scale is QueryOptions::robot_padding / ADR-013 territory, and FCL-vs-
// OptiX agreement is asserted outside the differential boundary band as usual.
#pragma once

#include <Eigen/Core>

#include "quevedomp/core/types.hpp"

namespace quevedomp::collision {

// Axis-aligned box from half-extents, centered at the origin. 8 vertices, 12 triangles, closed.
Mesh tessellate_box(const Eigen::Vector3d &half_extents);

// UV sphere centered at the origin. Closed; poles + (n_lat-1) rings of n_lon vertices.
Mesh tessellate_sphere(double radius, int n_lat = 12, int n_lon = 24);

// Cylinder along +Z, centered at the origin (URDF convention). Closed with capped ends.
Mesh tessellate_cylinder(double radius, double length, int n = 24);

} // namespace quevedomp::collision
