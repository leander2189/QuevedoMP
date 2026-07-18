"""Trajectory mode: time-parameterize, CHOMP-polish, and play back the last plan (ADR-021).

Everything here consumes ctx.last_attempt / session.trajectory; nothing reads another
mode's widgets. The refine section always runs in refiner (polish) mode — the standalone
variant is a *planner* and lives in the Plan mode. Refine certifies at the fidelity the
last plan pushed onto the session (session.timeout / planner_params), not at live widget
values from another panel.
"""

from __future__ import annotations

import threading
import time

import numpy as np

import quevedomp as q

from ..session import Attempt
from .base import Mode
from .chomp_params import ChompParamsWidgets

PLOT_PALETTE = ("#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00", "#a65628", "#f781bf")


class TrajectoryMode(Mode):
    name = "trajectory"
    title = "Trajectory"

    def build(self) -> None:
        ctx = self.ctx
        gui = ctx.server.gui
        self.folder = gui.add_folder(self.title)
        with self.folder:
            self.accel = gui.add_number("default accel (rad/s²)", initial_value=8.0, min=0.1,
                                        max=100.0)
            self.tip_speed = gui.add_number("tip speed cap (m/s, 0=off)", initial_value=0.0,
                                            min=0.0, max=10.0, step=0.05)
            self.tip_accel = gui.add_number("tip accel cap (m/s², 0=off)", initial_value=0.0,
                                            min=0.0, max=100.0, step=0.5)
            self.jerk = gui.add_number("jerk limit (rad/s³, 0=off)", initial_value=0.0, min=0.0,
                                       max=1000.0, step=5.0)
            self.param_button = gui.add_button("Parameterize")
            self.status = gui.add_text("trajectory", initial_value="—", disabled=True)

            with gui.add_folder("Refine (CHOMP polish)"):
                self.chomp = ChompParamsWidgets(gui)
                self.refine_button = gui.add_button("Refine last plan")
                self.refine_status = gui.add_text("refine", initial_value="—", disabled=True)

            with gui.add_folder("Playback"):
                self.scrub = gui.add_slider("scrub", min=0.0, max=1.0, step=0.002,
                                            initial_value=0.0)
                self.play_button = gui.add_button("▶ Play")
                self.play_speed = gui.add_number("play speed (rad/s)", initial_value=0.5,
                                                 min=0.05, max=5.0)
                self.play_timed_button = gui.add_button("▶ Play (timed)")
                self.time_scale = gui.add_number("time scale ×", initial_value=1.0, min=0.1,
                                                 max=10.0, step=0.1)

            self.plots_folder = gui.add_folder("Plots")

        self._plot_handles: list = []
        self._playing = False

        self.param_button.on_click(lambda _e: self.parametrize())
        self.refine_button.on_click(lambda _e: self._on_refine_clicked())
        self.scrub.on_update(lambda _e: self._on_scrub())
        self.play_button.on_click(lambda _e: self._on_play_clicked())
        self.play_timed_button.on_click(lambda _e: self._on_play_timed_clicked())
        ctx.attempt_listeners.append(self._on_attempt)

    def shutdown(self) -> None:
        self._playing = False  # stop playback threads before the gui/scene reset

    def _on_attempt(self, attempt: Attempt) -> None:
        self.status.value = "—"  # a new attempt invalidates the previous timing
        if attempt.result.ok():
            self.scrub.value = 0.0

    # ---- parameterize (Task 3.4 / R2) --------------------------------------------------------

    def parametrize(self) -> None:
        ctx = self.ctx
        attempt = ctx.last_attempt
        if attempt is None or not attempt.result.ok():
            self.status.value = "plan first"
            return
        try:
            with ctx.ui_lock:
                tj = ctx.session.parametrize(
                    attempt,
                    default_acceleration=float(self.accel.value),
                    tip_linear_velocity=float(self.tip_speed.value),
                    tip_linear_acceleration=float(self.tip_accel.value),
                    max_jerk=float(self.jerk.value),
                    tip_link=ctx.ee_link,
                )
        except RuntimeError as error:
            self.status.value = f"FAILED: {error}"
            return
        jerk_note = (
            f" · jerk certified in {tj.jerk_passes} passes"
            if float(self.jerk.value) > 0 else ""
        )
        self.status.value = f"{tj.duration:.2f} s · {len(tj.times)} nodes{jerk_note}"
        self._draw_plots(tj)

    def _draw_plots(self, tj) -> None:
        import viser.uplot as uplot

        ctx = self.ctx
        for handle in self._plot_handles:
            handle.remove()
        self._plot_handles = []
        t = np.ascontiguousarray(tj.times)
        names = [j.name for j in ctx.session.movable_joints()]

        def joint_plot(title: str, matrix: np.ndarray):
            series = (uplot.Series(label="t (s)"),) + tuple(
                uplot.Series(label=names[i] if i < len(names) else f"j{i}",
                             stroke=PLOT_PALETTE[i % len(PLOT_PALETTE)], width=1.5)
                for i in range(matrix.shape[1])
            )
            data = (t,) + tuple(np.ascontiguousarray(matrix[:, i]) for i in range(matrix.shape[1]))
            return ctx.server.gui.add_uplot(data=data, series=series, title=title, aspect=1.8)

        with self.plots_folder:
            self._plot_handles.append(joint_plot("joint velocity (rad/s)", tj.velocities))
            self._plot_handles.append(joint_plot("joint acceleration (rad/s²)", tj.accelerations))
            tip = np.ascontiguousarray(
                np.array(
                    [
                        np.linalg.norm(
                            (q.jacobian(ctx.session.model, tj.positions[k], ctx.ee_link)
                             @ tj.velocities[k])[:3]
                        )
                        for k in range(len(t))
                    ]
                )
            )
            self._plot_handles.append(
                ctx.server.gui.add_uplot(
                    data=(t, tip),
                    series=(uplot.Series(label="t (s)"),
                            uplot.Series(label="‖v_tip‖ (m/s)", stroke="#377eb8", width=1.5)),
                    title="tip speed (m/s)", aspect=1.8,
                )
            )

    # ---- refine (CHOMP polish — R4 refiner mode) ---------------------------------------------

    def _on_refine_clicked(self) -> None:
        ctx = self.ctx
        if ctx.session.is_planning:
            return
        if ctx.session.goal is None:
            self.refine_status.value = "set a goal first"
            return
        if ctx.last_attempt is None:
            self.refine_status.value = "plan first"
            return
        self.refine_status.value = "refining…"
        self.refine_button.disabled = True
        ctx.session.refine_async(self._on_refine_done, **self.chomp.kwargs(standalone=False))

    def refine_now(self) -> Attempt:
        """Synchronous polish + display — the headless smoke-test entry point."""
        attempt = self.ctx.session.refine(**self.chomp.kwargs(standalone=False))
        self._show_refined(attempt)
        return attempt

    def _on_refine_done(self, attempt) -> None:
        try:
            if attempt is None:
                self.refine_status.value = "ERROR — see server console"
            else:
                self._show_refined(attempt)
        finally:
            self.refine_button.disabled = False

    def _show_refined(self, attempt: Attempt) -> None:
        self.ctx.show_attempt(attempt)  # redraws the path; fires _on_attempt (timing reset)
        stats = attempt.result.stats
        if attempt.result.ok():
            self.refine_status.value = (
                f"{stats.refiner_mode} · {len(attempt.path)} wp · "
                f"{stats.time_total * 1e3:.0f} ms · certified free"
            )
        else:
            self.refine_status.value = f"{attempt.result.status} · {attempt.result.message}"

    # ---- playback ----------------------------------------------------------------------------

    def _on_scrub(self) -> None:
        attempt = self.ctx.last_attempt
        if attempt is None or len(attempt.path) < 2:
            return
        t = float(self.scrub.value) * (len(attempt.path) - 1)
        i = min(int(t), len(attempt.path) - 2)
        frac = t - i
        q_at = (1.0 - frac) * attempt.path[i] + frac * attempt.path[i + 1]
        self.ctx.set_config(q_at)

    def _on_play_clicked(self) -> None:
        if self._playing:
            self._playing = False  # acts as a Stop button while running
            return
        self.play(blocking=False)

    def play(self, blocking: bool = False, duration=None) -> None:
        """Animate the robot along the last path at constant joint speed via the scrub slider."""
        attempt = self.ctx.last_attempt
        if attempt is None or len(attempt.path) < 2 or self._playing:
            return
        if duration is None:
            length = sum(
                float(np.max(np.abs(b - a))) for a, b in zip(attempt.path, attempt.path[1:])
            )
            duration = float(np.clip(length / float(self.play_speed.value), 0.5, 60.0))

        def set_label(text: str) -> None:
            try:
                self.play_button.label = text
            except AttributeError:  # older viser: label immutable — Play just re-runs
                pass

        def run() -> None:
            self._playing = True
            start = time.monotonic()
            try:
                set_label("■ Stop")
                while self._playing:
                    t = (time.monotonic() - start) / duration
                    self.scrub.value = min(t, 1.0)
                    self._on_scrub()
                    if t >= 1.0:
                        break
                    time.sleep(1.0 / 30.0)
            finally:
                self._playing = False
                set_label("▶ Play")

        if blocking:
            run()
        else:
            threading.Thread(target=run, name="quevedomp-play", daemon=True).start()

    def _on_play_timed_clicked(self) -> None:
        if self._playing:
            self._playing = False  # acts as a Stop button while running
            return
        self.play_timed(blocking=False)

    def play_timed(self, blocking: bool = False) -> None:
        """Animate the robot along the parameterized trajectory in REAL time (× time scale) —
        what the velocity profile actually looks like, unlike the constant-rate scrub."""
        tj = self.ctx.session.trajectory
        if tj is None or self._playing:
            return
        scale = max(float(self.time_scale.value), 1e-3)

        def set_label(text: str) -> None:
            try:
                self.play_timed_button.label = text
            except AttributeError:
                pass

        def run() -> None:
            self._playing = True
            start = time.monotonic()
            try:
                set_label("■ Stop")
                while self._playing:
                    t = (time.monotonic() - start) * scale
                    self.ctx.set_config(tj.sample(t))
                    if t >= tj.duration:
                        break
                    time.sleep(1.0 / 30.0)
            finally:
                self._playing = False
                set_label("▶ Play (timed)")

        if blocking:
            run()
        else:
            threading.Thread(target=run, name="quevedomp-play-timed", daemon=True).start()
