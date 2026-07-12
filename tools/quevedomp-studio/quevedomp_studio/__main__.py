"""CLI: python -m quevedomp_studio --fixture ur5 | --urdf robot.urdf [--package-dir pkg=/dir ...]"""

from __future__ import annotations

import argparse
from pathlib import Path

from .app import StudioApp
from .session import StudioSession

# Repo fixture robots (tests/fixtures/robots), with the mesh maps the C++ tests use
# (Task 1.4b for the five arms; tests/support/dtc_scene.hpp for the two DTC cells — note the
# inlet cell deliberately points package://dtc_test at the dtc_test_inlet mesh set). The
# rbrobout cells also load their SRDF ACM (permanent contacts: baked EE, lift, dress kits).
REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURE_BASE_SUBDIR = {"iiwa": "kuka_iiwa"}
FIXTURE_PACKAGES = {
    "rbrobout": {"ur_description": "ur_description", "dtc_test": "dtc_test"},
    "rbrobout_inlet": {
        "ur_description": "ur_description",
        "dtc_test": "dtc_test_inlet",
        "ewellix_driver": "ewellix_driver",
    },
}
FIXTURE_SRDF = {"rbrobout": "rbrobout.srdf", "rbrobout_inlet": "rbrobout_inlet.srdf"}
FIXTURE_ROBOTS = ("ur5", "ur10", "panda", "iiwa", "irb2400", "rbrobout", "rbrobout_inlet")


def fixture_session(name: str) -> StudioSession:
    robots = REPO_ROOT / "tests" / "fixtures" / "robots"
    meshes = robots / "meshes"
    packages = FIXTURE_PACKAGES.get(
        name,
        {"example-robot-data": "example-robot-data", "collision": "abb_irb2400/collision"},
    )
    base_subdir = FIXTURE_BASE_SUBDIR.get(name, "")
    srdf = FIXTURE_SRDF.get(name)
    return StudioSession(
        (robots / f"{name}.urdf").read_text(),
        package_dirs={pkg: str(meshes / sub) for pkg, sub in packages.items()},
        base_dir=str(meshes / base_subdir) if base_subdir else "",
        srdf_text=(robots / srdf).read_text() if srdf else None,
    )


def main() -> None:
    parser = argparse.ArgumentParser(prog="quevedomp-studio",
                                     description="QuevedoMP Motion Planning IDE (ADR-016)")
    parser.add_argument("--fixture", choices=FIXTURE_ROBOTS,
                        help="load a repo fixture robot (mesh dirs wired automatically)")
    parser.add_argument("--urdf", help="URDF file to load")
    parser.add_argument("--load", help="a .qmps session file saved by the studio")
    parser.add_argument("--package-dir", action="append", default=[], metavar="PKG=DIR",
                        help="package:// mesh root (repeatable)")
    parser.add_argument("--base-dir", default="", help="base dir for relative mesh URIs")
    parser.add_argument("--srdf", help="optional SRDF; its <disable_collisions> pairs seed the ACM")
    parser.add_argument("--yaml", help="optional accel/jerk limits YAML extension")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--rerun-save", metavar="FILE.rrd",
                        help="log every planning attempt to this rerun recording")
    args = parser.parse_args()

    sources = [bool(args.fixture), bool(args.urdf), bool(args.load)]
    if sum(sources) != 1:
        parser.error("exactly one of --fixture, --urdf, or --load is required")

    if args.fixture:
        session = fixture_session(args.fixture)
    elif args.load:
        session = StudioSession.load(args.load)
    else:
        package_dirs = dict(spec.split("=", 1) for spec in args.package_dir)
        yaml_text = Path(args.yaml).read_text() if args.yaml else None
        srdf_text = Path(args.srdf).read_text() if args.srdf else None
        session = StudioSession(Path(args.urdf).read_text(), package_dirs=package_dirs,
                                base_dir=args.base_dir, yaml_extension=yaml_text,
                                srdf_text=srdf_text)

    app = StudioApp(session, host=args.host, port=args.port)
    if args.rerun_save:
        from .rerun_log import RerunLogger

        RerunLogger(session, args.rerun_save, ee_link=app.ik_link.value)
    app.serve_forever()


if __name__ == "__main__":
    main()
