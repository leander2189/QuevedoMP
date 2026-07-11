"""QuevedoMP — GPU-accelerated robot-arm motion planning (Python bindings, ADR-016).

Thin re-export of the nanobind extension ``quevedomp._native``. Pythonic helpers may live
here, but no logic: the C++ library is the single source of behavior (spec section 6 Phase 4).
"""

from ._native import __version__

__all__ = ["__version__"]
