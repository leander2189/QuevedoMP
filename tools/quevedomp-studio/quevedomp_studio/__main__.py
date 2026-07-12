"""CLI: python -m quevedomp_studio --urdf robot.urdf [--package-dir pkg=/dir ...]"""

from __future__ import annotations

import argparse
from pathlib import Path

from .app import StudioApp
from .session import StudioSession


def main() -> None:
    parser = argparse.ArgumentParser(prog="quevedomp-studio",
                                     description="QuevedoMP Motion Planning IDE (ADR-016)")
    parser.add_argument("--urdf", help="URDF file to load")
    parser.add_argument("--load", help="a .qmps session file saved by the studio")
    parser.add_argument("--package-dir", action="append", default=[], metavar="PKG=DIR",
                        help="package:// mesh root (repeatable)")
    parser.add_argument("--base-dir", default="", help="base dir for relative mesh URIs")
    parser.add_argument("--yaml", help="optional accel/jerk limits YAML extension")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--rerun-save", metavar="FILE.rrd",
                        help="log every planning attempt to this rerun recording")
    args = parser.parse_args()

    if bool(args.urdf) == bool(args.load):
        parser.error("exactly one of --urdf or --load is required")

    if args.load:
        session = StudioSession.load(args.load)
    else:
        package_dirs = dict(spec.split("=", 1) for spec in args.package_dir)
        yaml_text = Path(args.yaml).read_text() if args.yaml else None
        session = StudioSession(Path(args.urdf).read_text(), package_dirs=package_dirs,
                                base_dir=args.base_dir, yaml_extension=yaml_text)

    app = StudioApp(session, host=args.host, port=args.port)
    if args.rerun_save:
        from .rerun_log import RerunLogger

        RerunLogger(session, args.rerun_save, ee_link=app.ik_link.value)
    app.serve_forever()


if __name__ == "__main__":
    main()
