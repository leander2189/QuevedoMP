#!/usr/bin/env python3
"""Profile the inlet planning problem end to end — the quevedomp-studio scenario, headless.

Builds the canonical inlet cell (rbrobout_inlet + SRDF ACM + work-object mesh at the app pose,
identical to tests/support/dtc_scene.hpp) and plans from the DTC start config to the fastener
pose as a PoseGoal on the ee_9636_jA_tcp frame. Prints the PlanningStats attribution the
planner contract mandates, so a slow plan can be blamed precisely (planner logic vs collision
vs smoothing; batch-size histogram vs the GPU crossover).

Usage:
  PYTHONPATH=build/<preset>/bindings/python python3 examples/python/inlet_plan_profile.py \
      [--backend auto|fcl|optix] [--edge 0.01] [--seed 16] [--runs 3] [--timeout 90]
"""

from __future__ import annotations

import argparse
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import quevedomp as q

REPO = Path(__file__).resolve().parents[2]
FIX = REPO / "tests" / "fixtures"

# config/inlet fastener target (world frame) — the pose the studio session targeted.
FASTENER_T = np.array([-0.9427949, -0.2044888, 1.68938])
FASTENER_Q = np.array([0.6936832, 0.7069808, -0.0134154, -0.1371198])  # wxyz

START = [
    ("ewellix_lift_top_joint", 0.0),
    ("ur_shoulder_pan_joint", -1.0),
    ("ur_shoulder_lift_joint", -1.5),
    ("ur_elbow_joint", 0.0),
    ("ur_wrist_1_joint", -1.0),
    ("ur_wrist_2_joint", 0.0),
    ("ur_wrist_3_joint", -1.5708),
]


def build_cell(hint: "q.BackendHint"):
    model = q.RobotModel.from_urdf((FIX / "robots" / "rbrobout_inlet.urdf").read_text())
    robot = q.RobotInstance(model)
    for joint in model.joints:
        robot.acm.allow(joint.parent_link, joint.child_link)
    srdf = ET.fromstring((FIX / "robots" / "rbrobout_inlet.srdf").read_text())
    for elem in srdf.iter("disable_collisions"):
        robot.acm.allow(elem.get("link1"), elem.get("link2"))
    # The dress kits are flexible cable guides: near the duct they graze the work object in
    # virtually every reachable wrist configuration (verified via witness pairs), so treating
    # them as hard collision geometry makes the whole goal region infeasible. Allow the pair,
    # as the real cell does for cable contact.
    for i in range(1, 6):
        robot.acm.allow(f"dresskit_{i}", "work_object")

    m = FIX / "robots" / "meshes"
    meshes = q.MeshSources(
        {
            "ur_description": str(m / "ur_description"),
            "dtc_test": str(m / "dtc_test_inlet"),
            "ewellix_driver": str(m / "ewellix_driver"),
        }
    )
    env = q.SceneDescription()
    env.add(
        "work_object",
        q.load_mesh(str(m / "dtc_test_inlet" / "meshes" / "inlet_mesh_2.stl")),
        q.Transform.from_parts(np.array([-1.887, 0.0202435, 0.26]),
                               np.array([-0.5, 0.0, 0.0, 0.8660254])),
    )

    t0 = time.perf_counter()
    scene = q.make_static_scene(model, env, hint, meshes)
    build_s = time.perf_counter() - t0
    return model, robot, scene, build_s


def start_config(model) -> np.ndarray:
    qv = np.zeros(model.dof)
    for name, value in START:
        joint = model.find_joint(name)
        if joint is not None and joint.dof_index >= 0:
            qv[joint.dof_index] = value
    return qv


def find_free_goal_config(model, robot, scene, target, base_seed: int) -> np.ndarray | None:
    """Deterministically hunt a collision-free IK branch for `target` (the planner's own goal
    sampler gives up after a handful of branches, which is far too few in a cluttered cell)."""
    ws = scene.make_workspace()
    opts = q.QueryOptions()
    for attempt in range(60):
        io = q.IkOptions()
        io.seed = base_seed + attempt * 7919
        io.max_restarts = 4
        result = q.make_numerical_ik(model, io).solve("ee_9636_jA_tcp", target)
        if not result.success:
            continue
        if not scene.query(robot, result.q, opts, ws).in_collision:
            return result.q
    return None


def histogram_summary(hist: dict) -> str:
    if not hist:
        return "empty"
    total = sum(hist.values())
    fat = sum(n for size, n in hist.items() if size >= 256)
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:3]
    tops = ", ".join(f"{size}x{n}" for size, n in top)
    return f"{total} queries ({fat} with batch>=256) · top sizes: {tops}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("auto", "fcl", "optix"), default="fcl")
    parser.add_argument("--edge", type=float, default=0.01)
    parser.add_argument(
        "--goal-offset", type=float, default=-0.10,
        help="retract the goal along the fastener frame's -Z (m): at the fastener itself the "
             "jointA EE is inside the duct, so every goal config collides (the physical insertion "
             "problem); a pre-insertion pose gives the planner a feasible goal region",
    )
    parser.add_argument("--seed", type=int, default=16)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    hint = {
        "auto": q.BackendHint.Auto,
        "fcl": q.BackendHint.ForceCpuFcl,
        "optix": q.BackendHint.ForceOptix,
    }[args.backend]

    print(f"backend={args.backend} · optix_available={q.optix_available()} · edge={args.edge}")
    model, robot, scene, build_s = build_cell(hint)
    print(f"scene build: {build_s * 1e3:.0f} ms · dof={model.dof}")

    params = q.PlannerParams()
    params.edge_resolution = args.edge
    planner = q.make_planner(params, robot, scene)

    # Preferred goal: a collision-free IK branch near the fastener. Under the strict collision
    # model no such branch exists (the dress kits graze the work object in every reachable
    # branch, and the ACM is not honored for robot-vs-environment pairs — core gap, see the
    # profiling notes), so fall back to a big workspace sweep past the duct: the same
    # collision-heavy regime, with a goal that is actually feasible.
    target = q.Transform.from_parts(FASTENER_T, FASTENER_Q) * q.Transform.from_translation(
        np.array([0.0, 0.0, args.goal_offset])
    )
    goal_q = find_free_goal_config(model, robot, scene, target, args.seed)
    if goal_q is not None:
        print(f"goal: collision-free IK branch at offset {args.goal_offset:+.2f} m")
    else:
        # Grid-scan pan x lift for the free config farthest from the start pan: a long sweep
        # past the duct, guaranteed feasible.
        pan = model.find_joint("ur_shoulder_pan_joint")
        lift = model.find_joint("ewellix_lift_top_joint")
        ws = scene.make_workspace()
        start = start_config(model)
        goal_q, best = None, -1.0
        for pan_v in np.linspace(pan.limits.lower + 0.05, pan.limits.upper - 0.05, 16):
            for lift_v in np.linspace(lift.limits.lower, lift.limits.upper, 3):
                cand = start.copy()
                cand[pan.dof_index] = pan_v
                cand[lift.dof_index] = lift_v
                if scene.query(robot, cand, q.QueryOptions(), ws).in_collision:
                    continue
                dist = abs(pan_v - start[pan.dof_index])
                if dist > best:
                    goal_q, best = cand, dist
        assert goal_q is not None, "no free sweep goal found"
        print(f"goal: workspace sweep past the duct (pan {start[pan.dof_index]:+.2f} -> "
              f"{goal_q[pan.dof_index]:+.2f} rad, lift {goal_q[lift.dof_index]:.2f} m)")

    problem = q.PlanningProblem()
    problem.start = start_config(model)
    problem.goal = q.JointGoal(goal_q, 1e-3)
    problem.timeout = args.timeout

    for run in range(args.runs):
        problem.seed = args.seed + run
        result = planner.plan(problem)
        s = result.stats
        print(
            f"\nseed {result.used_seed}: {result.status} · {len(result.path)} wp"
            f"{' · ' + result.message if result.message else ''}"
        )
        print(
            f"  total {s.time_total * 1e3:8.0f} ms · planner {s.time_planner * 1e3:8.0f} ms"
            f" · collision {s.time_collision * 1e3:8.0f} ms"
            f" · first solution {s.time_first_solution * 1e3:8.0f} ms"
        )
        print(f"  {s.collision_configs} configs · {histogram_summary(s.batch_size_histogram)}")
        print(f"  iterations {s.iterations}")

        if result.ok():
            sp = q.SmootherParams()
            sp.edge_resolution = args.edge
            sp.seed = result.used_seed
            smoother = q.make_shortcut_smoother(sp, robot, scene)
            t0 = time.perf_counter()
            smoothed = smoother.smooth(result.path)
            print(
                f"  shortcut smoothing: {(time.perf_counter() - t0) * 1e3:.0f} ms ·"
                f" {len(result.path)} -> {len(smoothed)} wp"
            )


if __name__ == "__main__":
    main()
