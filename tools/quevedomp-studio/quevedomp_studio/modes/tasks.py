"""Tasks mode: placeholder for the MTC-lite inspector/runner (ADR-021, roadmap R7).

When `quevedomp_tasks` lands (attached objects + Sequence/Alternatives/PlanTo/IkBranches/
CartesianMove with trajectory stitching), this mode becomes its inspector: the stage tree
with per-stage status/solutions, click-to-preview, and hand-off of the stitched result to
the Trajectory mode. Tasks stay defined in Python — the studio inspects and runs them.
"""

from __future__ import annotations

from .base import Mode


class TasksMode(Mode):
    name = "tasks"
    title = "Tasks (MTC)"

    def build(self) -> None:
        self.folder = self.ctx.server.gui.add_folder(self.title)
        with self.folder:
            self.note = self.ctx.server.gui.add_text(
                "status", initial_value="arrives with roadmap R7 (quevedomp_tasks)",
                disabled=True,
            )
