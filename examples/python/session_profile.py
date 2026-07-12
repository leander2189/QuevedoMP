#!/usr/bin/env python3
"""Plan a saved quevedomp-studio session (.qmps) headless and print the PlanningStats
attribution — the apples-to-apples benchmark runner for problems set up in the studio.

Usage:
  PYTHONPATH=build/<preset>/bindings/python:tools/quevedomp-studio \
  python3 examples/python/session_profile.py sessions/benchmark.qmps \
      [--seeds 8 9 10] [--backend keep|fcl|optix|auto] [--timeout 0=file]
"""

from __future__ import annotations

import argparse

import quevedomp as q
from quevedomp_studio.session import StudioSession


def histogram_summary(hist: dict) -> str:
    if not hist:
        return "empty"
    total = sum(hist.values())
    fat = sum(n for size, n in hist.items() if size >= 256)
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:3]
    return f"{total} queries ({fat} with batch>=256) · top sizes: " + ", ".join(
        f"{s}x{n}" for s, n in top
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", help=".qmps file saved by the studio")
    parser.add_argument("--seeds", type=int, nargs="+", default=[8])
    parser.add_argument("--backend", choices=("keep", "fcl", "optix", "auto"), default="keep",
                        help="rebuild the scene with this hint (keep = session default Auto)")
    parser.add_argument("--timeout", type=float, default=0.0, help="override; 0 = file value")
    parser.add_argument("--sweep", type=float, default=-1.0,
                        help="max link sweep in metres (P3 Cartesian-bounded edge steps); "
                        "0 disables, -1 = file value")
    args = parser.parse_args()

    session = StudioSession.load(args.session)
    if args.backend != "keep":
        hint = {"fcl": q.BackendHint.ForceCpuFcl, "optix": q.BackendHint.ForceOptix,
                "auto": q.BackendHint.Auto}[args.backend]
        session.scene = q.make_static_scene(
            session.model, session.environment(), hint, session.mesh_sources
        )
    if args.timeout > 0:
        session.timeout = args.timeout
    if args.sweep >= 0:
        session.planner_params.max_link_sweep = args.sweep

    p = session.planner_params
    print(
        f"session={args.session} · robot={session.model.name} (dof {session.dof}) ·"
        f" obstacles={len(session.obstacles)} · goal={session.goal.kind}"
    )
    edge = (f"sweep={p.max_link_sweep * 1e3:g}mm" if p.max_link_sweep > 0
            else f"edge={p.edge_resolution}")
    print(
        f"backend={args.backend} · optix_available={q.optix_available()} ·"
        f" {edge} · timeout={session.timeout}s · smooth={session.smooth}"
    )
    if p.max_link_sweep > 0:
        w = session.lever_weights()
        print("lever weights (m/rad): " + ", ".join(f"{x:.3f}" for x in w))

    for seed in args.seeds:
        attempt = session.plan(seed)
        r = attempt.result
        s = r.stats
        print(f"\nseed {r.used_seed}: {r.status} · {len(r.path)} wp -> {len(attempt.path)} smoothed"
              f"{' · ' + r.message if r.message else ''}")
        print(
            f"  total {s.time_total * 1e3:8.0f} ms · planner {s.time_planner * 1e3:6.0f} ms"
            f" · collision {s.time_collision * 1e3:8.0f} ms"
            f" · first solution {s.time_first_solution * 1e3:8.0f} ms"
        )
        print(f"  {s.collision_configs} configs · {histogram_summary(s.batch_size_histogram)}")


if __name__ == "__main__":
    main()
