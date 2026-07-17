# dtc_test_inlet fixtures — provenance

All meshes authored by Leandro (Blender, `DTC_Test.blend` — kept locally, not committed).

- `meshes/inlet_mesh_2.stl` — the v0 work object (~4k triangles), committed.
- `meshes/inlet_mesh_hires.stl` — **roadmap R1 high-poly variant: 7,320,154 triangles, 366 MB
  (binary STL), drop-in replacement for `inlet_mesh_2.stl` (same pose/scale).** Gitignored
  (too large for plain git; move to LFS if sharing is ever needed). The matching studio
  session is `sessions/inlet_hires.qmps` (also gitignored, 179 MB — embeds the mesh blob).
- **Known property (2026-07-16): `inlet_mesh_hires.stl` is NOT watertight**, so ADR-012-style
  sign/containment is unavailable on it — the ClearanceField built over it is unsigned
  (distance-to-surface only), and OptiX/FCL containment checks cannot flag full immersion in
  it. A watertight re-export (Blender remesh/solidify) would restore both.

R1 baseline (2026-07-16, seed 8, 16 threads, RTX 4060 Ti):

| scene | backend | plan | configs | per-config |
|---|---|---|---|---|
| low-poly (`benchmark.qmps`) | FCL | 0.67 s (solved) | 90k | 7.3 µs |
| hires (`inlet_hires.qmps`) | FCL | 15 s TIMEOUT | 1.95M | 7.6 µs |
| hires (`inlet_hires.qmps`) | OptiX | 15 s TIMEOUT | 2.11M | 7.0 µs |

Per-config cost barely moves with 1800× the triangles (BVH/GAS log-scaling) and the backends
are a dead heat. The hires TIMEOUT is a problem property, not a performance one — the saved
problem finds no first solution in 15 s (goal region likely collides against the hires
surface); to be investigated in the studio.
