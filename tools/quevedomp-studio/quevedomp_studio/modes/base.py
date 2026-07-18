"""Mode base class (ADR-021).

A mode is one working context (Scene / IK / Plan / Trajectory / Tasks): a gui folder of
widgets plus whatever scene nodes belong to that activity. Modes are constructed fresh on
every StudioApp._mount and discarded wholesale on load_session — there is no per-widget
teardown; shutdown() exists only to stop background threads before the gui/scene reset.
"""

from __future__ import annotations

from typing import Optional

from ..context import StudioContext


class Mode:
    name: str = ""  # switcher key
    title: str = ""  # folder label

    def __init__(self, ctx: StudioContext) -> None:
        self.ctx = ctx
        self.folder: Optional[object] = None

    def build(self) -> None:
        """Create self.folder, its widgets, any mode-owned scene nodes, and register ctx
        listeners. Called exactly once per mount."""
        raise NotImplementedError

    def set_active(self, active: bool) -> None:
        """Show/hide this mode. Override to also toggle mode-owned scene nodes."""
        if self.folder is not None:
            self.folder.visible = active

    def shutdown(self) -> None:
        """Stop background threads before a gui/scene reset (load_session)."""
