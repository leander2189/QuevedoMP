"""quevedomp-studio — interactive Motion Planning IDE (ADR-016, Task 4a.6).

Layering (the smoke test depends on it):
  session.py   — headless core: robot/scene state, IK, planning, save/load. No UI imports.
  app.py       — the viser UI over a StudioSession (gizmos, sliders, plan/scrub).
  rerun_log.py — optional .rrd attempt logging.
"""

from .session import Obstacle, StudioSession

__all__ = ["Obstacle", "StudioSession"]
