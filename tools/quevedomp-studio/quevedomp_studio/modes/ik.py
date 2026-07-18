"""IK mode: gizmo-driven IK, global multi-restart solve, and the multi-branch picker (ADR-021).

Owns the EE-link dropdown — the single writer of ctx.ee_link, which Plan and Trajectory read
for markers, path curves, and tip plots. The gizmo itself lives on the context (Plan-goal
capture and pose-goal restore touch it while this mode is inactive); this mode wires its
callbacks and controls its visibility.
"""

from __future__ import annotations

import numpy as np

import quevedomp as q

from .base import Mode


class IkMode(Mode):
    name = "ik"
    title = "IK"

    def build(self) -> None:
        ctx = self.ctx
        session = ctx.session
        links = session.ik_links() or session.leaf_links() or [session.model.root_link]
        self.folder = ctx.server.gui.add_folder(self.title)
        with self.folder:
            self.link = ctx.server.gui.add_dropdown("link", options=tuple(links),
                                                    initial_value=links[0])
            self.status = ctx.server.gui.add_text("ik", initial_value="—", disabled=True)
            snap = ctx.server.gui.add_button("Snap gizmo to link")
            solve_global = ctx.server.gui.add_button("Solve (global, multi-restart)")
            self.branch_count = ctx.server.gui.add_number("branches to find", initial_value=8,
                                                          min=2, max=20, step=1)
            solve_branches = ctx.server.gui.add_button("Solve branches")
            self.branch_pick = ctx.server.gui.add_dropdown("branch", options=("—",),
                                                           initial_value="—")

        self._busy = False
        self.branches: list = []
        self.snap_gizmo()

        snap.on_click(lambda _e: self.snap_gizmo())
        solve_global.on_click(lambda _e: self.solve(interactive=False))
        solve_branches.on_click(lambda _e: self.solve_branches())
        self.branch_pick.on_update(lambda _e: self.on_branch_pick())
        ctx.ik_gizmo.on_update(lambda _e: self._on_gizmo())
        self.link.on_update(lambda _e: self._on_link())

    def set_active(self, active: bool) -> None:
        super().set_active(active)
        self.ctx.ik_gizmo.visible = active  # pose survives hiding (Plan-goal capture reads it)

    def _on_link(self) -> None:
        self.ctx.ee_link = self.link.value  # the one place the shared EE link is written
        self.snap_gizmo()

    def snap_gizmo(self) -> None:
        session = self.ctx.session
        pose = q.fk(session.model, session.q, self.link.value)
        self.ctx.ik_gizmo.position = pose.translation()
        self.ctx.ik_gizmo.wxyz = pose.quaternion()

    def solve_branches(self) -> None:
        """Solve N distinct branches for the gizmo pose and fill the picker (free ones marked)."""
        if self._busy:
            return
        self._busy = True
        try:
            ctx = self.ctx
            target = q.Transform.from_parts(
                np.asarray(ctx.ik_gizmo.position), np.asarray(ctx.ik_gizmo.wxyz)
            )
            with ctx.ui_lock:
                self.branches = ctx.session.solve_ik_branches(
                    self.link.value, target, n=int(self.branch_count.value)
                )
            if not self.branches:
                self.branch_pick.options = ("—",)
                self.status.value = "no branches found (out of reach?)"
                return
            free = sum(b.free for b in self.branches)
            # Label = index · collision state · joint-space distance from the current config
            # (the default ordering, nearest first).
            here = ctx.session.q
            options = tuple(
                f"{i + 1}: {'free' if b.free else 'COLLIDES'} · Δ{np.linalg.norm(b.q - here):.2f}"
                for i, b in enumerate(self.branches)
            )
            self.branch_pick.options = options
            self.branch_pick.value = options[0]
            self.status.value = f"{len(self.branches)} branches · {free} collision-free"
            self.apply_branch(0)
        finally:
            self._busy = False

    def on_branch_pick(self) -> None:
        label = self.branch_pick.value
        if not self.branches or label == "—":
            return
        self.apply_branch(int(label.split(":", 1)[0]) - 1)

    def apply_branch(self, index: int) -> None:
        if 0 <= index < len(self.branches):
            self.ctx.set_config(self.branches[index].q)

    def _on_gizmo(self) -> None:
        # Drop events that arrive while a solve is running (a drag emits dozens); the next
        # event re-solves from the latest gizmo pose, so tracking never falls behind.
        if self._busy:
            return
        self.solve(interactive=True)

    def solve(self, interactive: bool) -> None:
        self._busy = True
        try:
            ctx = self.ctx
            target = q.Transform.from_parts(
                np.asarray(ctx.ik_gizmo.position), np.asarray(ctx.ik_gizmo.wxyz)
            )
            with ctx.ui_lock:
                result = ctx.session.solve_ik(self.link.value, target, interactive=interactive)
            if result.success:
                mode = "track" if interactive else "global"
                self.status.value = (
                    f"{mode} ok · {result.iterations} iters · pos {result.pos_error * 1e3:.2f} mm"
                )
                ctx.set_config(result.q)
            elif interactive:
                # Tracking failure = out of reach from the current branch: hold the pose
                # instead of teleporting; "Solve (global)" is the explicit branch switch.
                self.status.value = f"out of reach · pos {result.pos_error * 1e3:.1f} mm"
            else:
                self.status.value = f"global FAILED · pos {result.pos_error * 1e3:.1f} mm"
        finally:
            self._busy = False
