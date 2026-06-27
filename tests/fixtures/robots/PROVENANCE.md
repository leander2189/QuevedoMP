# Robot fixture provenance & licenses

URDFs and **collision** meshes used by the test suite (Tasks 1.4 / 1.4b). Each robot's
`.urdf` plus its collision `.stl` meshes are vendored under `meshes/` (visual meshes are not —
collision geometry is what the planner uses). Mesh files are referenced from the URDFs via
`package://`/relative URIs and resolved by `resolve_mesh_uri()` (see the integration test
`tests/unit/test_robot_meshes.cpp` for the package→directory map). Files are used **verbatim**
as downloaded unless noted under "Edits".

| File | Robot | DOF (datasheet) | Movable joints (this URDF) | Retrieved |
|------|-------|-----------------|----------------------------|-----------|
| `ur5.urdf` | Universal Robots UR5 | 6 | 6 revolute | 2026-06-27 |
| `ur10.urdf` | Universal Robots UR10 | 6 | 6 revolute | 2026-06-27 |
| `panda.urdf` | Franka Emika Panda | 7 (arm) | 7 revolute + 2 finger prismatic | 2026-06-27 |
| `iiwa.urdf` | KUKA LBR iiwa | 7 | 7 revolute | 2026-06-27 |
| `irb2400.urdf` | ABB IRB 2400 | 6 | 6 revolute | 2026-06-27 |

## Sources & licenses

### `ur5.urdf`, `ur10.urdf`, `panda.urdf` — Gepetto/example-robot-data (BSD-3-Clause)
- Source: <https://github.com/Gepetto/example-robot-data> (branch `master`), files
  `robots/ur_description/urdf/ur5_robot.urdf`, `.../ur10_robot.urdf`,
  `robots/panda_description/urdf/panda.urdf`.
- Repo license: **BSD-3-Clause**.
- Upstream model origin: UR descriptions trace to ROS-Industrial `universal_robot` (BSD);
  the Panda model traces to `franka_ros`/`franka_description` (**Apache-2.0**). The Panda URDF
  here includes the hand (2 prismatic finger joints) in addition to the 7-DOF arm.

### `iiwa.urdf` — bulletphysics/bullet3 (zlib)
- Source: <https://github.com/bulletphysics/bullet3> (branch `master`),
  `data/kuka_iiwa/model.urdf`.
- Repo license: **zlib** (`LICENSE.txt`). GitHub's auto-detector reports "Other"/NOASSERTION
  because of bullet3's non-standard license header, but the project license is zlib.
- Model: KUKA LBR iiwa, as bundled in the Bullet/PyBullet data set.

### `irb2400.urdf` — Daniella1/urdf_files_dataset (MIT) ← ROS-Industrial (BSD-3-Clause)
- Source: <https://github.com/Daniella1/urdf_files_dataset> (branch `main`),
  `urdf_files/ros-industrial/abb/abb_irb2400_support/urdf/irb2400.urdf`.
- Aggregator repo license: **MIT**.
- Underlying model: ROS-Industrial `abb` / `abb_irb2400_support` (declared **BSD-3-Clause**
  in its `package.xml`), provided here as a pre-expanded (non-xacro) plain URDF.
- Note: the original `ros-industrial/abb` repo has no root `LICENSE` file (license is declared
  per-package); the MIT-licensed dataset above is used as the redistribution source.

## Collision meshes (vendored under `meshes/`)

Same repositories and licenses as the URDFs above; downloaded 2026-06-27, byte-for-byte.

| Robot | Vendored path | Count | Source |
|-------|---------------|-------|--------|
| UR5 | `meshes/example-robot-data/robots/ur_description/meshes/ur5/collision/*.stl` | 7 | example-robot-data (BSD-3-Clause) |
| UR10 | `meshes/example-robot-data/robots/ur_description/meshes/ur10/collision/*.stl` | 7 | example-robot-data (BSD-3-Clause) |
| Panda | `meshes/example-robot-data/robots/panda_description/meshes/collision/*.stl` | 9 | example-robot-data (BSD-3-Clause) |
| iiwa | `meshes/kuka_iiwa/meshes/link_*.stl` | 8 | bullet3 (zlib) |
| ABB IRB2400 | `meshes/abb_irb2400/collision/*.stl` | 7 | urdf_files_dataset (MIT) ← ROS-Industrial abb (BSD-3) |

The vendored directory layout mirrors each URDF's mesh URI so `resolve_mesh_uri()` finds them
(UR/Panda: `package://example-robot-data/...`; iiwa: relative `meshes/link_N.stl`; ABB:
`package://collision/...`). `.stl` is marked `binary` in `.gitattributes` so the (binary) STL
files are never EOL/encoding-mangled.

## Edits
None. All URDFs and meshes are byte-for-byte as downloaded on 2026-06-27.
