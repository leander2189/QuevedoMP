"""Optional rerun (.rrd) logging of planning attempts.

Every attempt logs the end-effector trace, start/goal markers, and a PlanningStats text block
at frame = attempt index, so a session's history scrubs in the rerun viewer — the deep-
inspection complement to the live viser view (ADR-016).
"""

from __future__ import annotations

from typing import Optional

import numpy as np

import quevedomp as q

from .session import Attempt, StudioSession


class RerunLogger:
    def __init__(self, session: StudioSession, save_path: str, ee_link: str) -> None:
        import rerun as rr  # local import: the logger is optional

        self.rr = rr
        self.session = session
        self.ee_link = ee_link
        rr.init("quevedomp-studio")
        rr.save(save_path)
        session.attempt_listeners.append(self.log_attempt)

    def _ee_positions(self, path) -> np.ndarray:
        model = self.session.model
        return np.array([q.fk(model, w, self.ee_link).translation() for w in path])

    def log_attempt(self, attempt: Attempt) -> None:
        rr = self.rr
        rr.set_time_sequence("attempt", attempt.index)
        r = attempt.result

        stats = r.stats
        rr.log(
            "attempt/summary",
            rr.TextDocument(
                f"status: {r.status}\n"
                f"used_seed: {r.used_seed}\n"
                f"waypoints: {len(attempt.path)} (smoothed: {attempt.smoothed})\n"
                f"iterations: {stats.iterations}\n"
                f"collision queries/configs: {stats.collision_queries}/{stats.collision_configs}\n"
                f"time: total {stats.time_total * 1e3:.1f} ms · planner {stats.time_planner * 1e3:.1f} ms"
                f" · collision {stats.time_collision * 1e3:.1f} ms\n"
                f"message: {r.message}"
            ),
        )
        if attempt.path:
            trace = self._ee_positions(attempt.path)
            rr.log("attempt/ee_path", rr.LineStrips3D([trace], radii=0.004))
            rr.log(
                "attempt/endpoints",
                rr.Points3D(
                    [trace[0], trace[-1]], colors=[[80, 200, 80], [80, 120, 240]], radii=0.012
                ),
            )
