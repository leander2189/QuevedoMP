# Robot fixture provenance & licenses

URDFs used by the test suite (Task 1.4). **Only the `.urdf` files are vendored** — mesh assets
referenced via `package://`/relative paths are intentionally not included (URDF *parsing* needs
no meshes; mesh loading is Task 1.4b, which resolves geometry separately). Files are used
**verbatim** as downloaded unless noted under "Edits".

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

## Edits
None. All files are byte-for-byte as downloaded on 2026-06-27.
