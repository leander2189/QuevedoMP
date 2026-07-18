"""The CHOMP knob set (ADR-021), built independently by two modes: Plan's standalone-CHOMP
planner section and Trajectory's refine section. Same knobs, separate widgets — each mode
keeps its own values (a coarse debug SDF in Plan shouldn't degrade a Trajectory polish)."""

from __future__ import annotations


class ChompParamsWidgets:
    """Builds the widgets into the caller's current gui container."""

    def __init__(self, gui) -> None:
        self.waypoints = gui.add_number("waypoints", initial_value=64, min=8, max=400, step=1)
        self.iterations = gui.add_number("iterations", initial_value=100, min=1, max=1000, step=1)
        self.clearance_weight = gui.add_number("clearance weight", initial_value=1.0, min=0.0,
                                               max=50.0, step=0.5)
        self.smoothness_weight = gui.add_number("smoothness weight", initial_value=1.0, min=0.0,
                                                max=50.0, step=0.5)
        self.clearance_epsilon = gui.add_number("clearance ε (m)", initial_value=0.10, min=0.01,
                                                max=1.0, step=0.01)
        self.step_size = gui.add_number("step size", initial_value=0.1, min=0.001, max=1.0,
                                        step=0.01)
        self.sdf_resolution = gui.add_number("SDF resolution (mm)", initial_value=10.0, min=2.0,
                                             max=100.0, step=1.0)

    def kwargs(self, *, standalone: bool) -> dict:
        """Keyword arguments for StudioSession.refine()."""
        return dict(
            standalone=standalone,
            waypoints=int(self.waypoints.value),
            max_iterations=int(self.iterations.value),
            clearance_weight=float(self.clearance_weight.value),
            smoothness_weight=float(self.smoothness_weight.value),
            clearance_epsilon=float(self.clearance_epsilon.value),
            step_size=float(self.step_size.value),
            resolution=float(self.sdf_resolution.value) * 1e-3,
        )
