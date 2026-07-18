"""Scene mode: pose the robot and edit the static environment (ADR-021).

Robot/obstacle meshes and the obstacle drag gizmos stay visible in every mode — they are
the world the other modes operate on; only this panel's widgets are modal.
"""

from __future__ import annotations

import numpy as np

import quevedomp as q

from .base import Mode


class SceneMode(Mode):
    name = "scene"
    title = "Scene"

    def build(self) -> None:
        ctx = self.ctx
        session = ctx.session
        self._syncing_sliders = False
        self._obstacle_counter = len(session.obstacles)

        self.folder = ctx.server.gui.add_folder(self.title)
        with self.folder:
            self.status = ctx.server.gui.add_text("status", initial_value="ready", disabled=True)
            self.sliders = []
            for joint in session.movable_joints():
                limits = joint.limits
                lo, hi = (limits.lower, limits.upper) if limits.has_position_limit else (-np.pi, np.pi)
                slider = ctx.server.gui.add_slider(
                    joint.name, min=float(lo), max=float(hi), step=(hi - lo) / 200.0,
                    initial_value=float(np.clip(0.0, lo, hi)),
                )
                slider.on_update(lambda _e: self._on_sliders())
                self.sliders.append(slider)

            with ctx.server.gui.add_folder("Obstacles"):
                kind = ctx.server.gui.add_dropdown("kind", options=("box", "sphere", "cylinder"),
                                                   initial_value="box")
                size = ctx.server.gui.add_number("size (m)", initial_value=0.2, min=0.01, max=2.0)
                add = ctx.server.gui.add_button("Add obstacle")
                mesh_path = ctx.server.gui.add_text("mesh file", initial_value="")
                add_mesh = ctx.server.gui.add_button("Add mesh obstacle")
                self.obstacle_status = ctx.server.gui.add_text("obstacles", initial_value="—",
                                                               disabled=True)
                self.obstacle_pick = ctx.server.gui.add_dropdown("selected", options=("—",),
                                                                 initial_value="—")
                remove = ctx.server.gui.add_button("Remove selected")

        def on_add(_e) -> None:
            self._obstacle_counter += 1
            oid = f"{kind.value}_{self._obstacle_counter}"
            s = float(size.value)
            geometry = {
                "box": lambda: q.BoxShape(np.full(3, s / 2.0)),
                "sphere": lambda: q.SphereShape(s / 2.0),
                "cylinder": lambda: q.CylinderShape(s / 4.0, s),
            }[kind.value]()
            self.add_obstacle(oid, geometry, q.Transform.from_translation(np.array([0.5, 0.0, s / 2.0])))

        def on_add_mesh(_e) -> None:
            path = mesh_path.value.strip()
            if not path:
                self.obstacle_status.value = "enter a mesh path (STL/DAE/OBJ)"
                return
            try:
                mesh = q.load_mesh(path)  # normalized to metres, GIL released
            except RuntimeError as error:
                self.obstacle_status.value = f"load failed: {error}"
                return
            self._obstacle_counter += 1
            oid = f"mesh_{self._obstacle_counter}"
            self.add_obstacle(oid, mesh, q.Transform.from_translation(np.array([0.5, 0.0, 0.0])))
            self.obstacle_status.value = f"{oid}: {mesh.vertices.shape[0]} verts"

        def on_remove(_e) -> None:
            oid = self.obstacle_pick.value
            if oid in session.obstacles:
                session.remove_obstacle(oid)
                ctx.obstacle_view.remove(oid)
                ctx.scene_changed()
                self._sync_obstacle_pick()
                ctx.refresh()

        add.on_click(on_add)
        add_mesh.on_click(on_add_mesh)
        remove.on_click(on_remove)

        ctx.status_sink = lambda text: setattr(self.status, "value", text)
        ctx.config_listeners.append(self._sync_sliders)

        # A loaded session already carries obstacles: render them with the move hook wired.
        for obstacle in session.obstacles.values():
            ctx.obstacle_view.add(obstacle, on_moved=lambda _id: self._on_obstacle_moved())
        if session.obstacles:
            self._sync_obstacle_pick()

    # ---- robot config -----------------------------------------------------------------------

    def _on_sliders(self) -> None:
        # Programmatic slider writes (_sync_sliders) fire this too — mid-sync the slider set is
        # half new / half stale, and rebuilding the config from it walks the robot through
        # mixed configurations (the "scrub looks like IK" bug). The guard drops those events.
        if self._syncing_sliders:
            return
        self.ctx.session.set_config(np.array([s.value for s in self.sliders]))
        self.ctx.refresh()

    def _sync_sliders(self, q_new: np.ndarray) -> None:
        """Config listener: mirror a programmatic config change onto the sliders."""
        self._syncing_sliders = True
        try:
            for slider, value in zip(self.sliders, q_new):
                slider.value = float(value)
        finally:
            self._syncing_sliders = False

    # ---- obstacles --------------------------------------------------------------------------

    def add_obstacle(self, oid: str, geometry, pose) -> None:
        obstacle = self.ctx.session.add_obstacle(oid, geometry, pose)
        self.ctx.obstacle_view.add(obstacle, on_moved=lambda _id: self._on_obstacle_moved())
        self.ctx.scene_changed()
        self._sync_obstacle_pick()
        self.ctx.refresh()

    def _on_obstacle_moved(self) -> None:
        self.ctx.scene_changed()  # move_obstacle already invalidated the session caches
        self.ctx.refresh()

    def _sync_obstacle_pick(self) -> None:
        ids = tuple(self.ctx.session.obstacles) or ("—",)
        self.obstacle_pick.options = ids
        self.obstacle_pick.value = ids[-1]
