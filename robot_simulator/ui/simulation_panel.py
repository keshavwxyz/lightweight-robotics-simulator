"""Bottom panel: console, simulation statistics and ROS 2 status."""
from __future__ import annotations

import imgui

from .console import Console
from .widgets import ERROR, MUTED, OK, WARN, kv


class SimulationPanel:
    def __init__(self, ui, log_buffer):
        self.ui = ui
        self.console = Console(log_buffer)

    def draw(self) -> None:
        ui, sim = self.ui, self.ui.sim
        # Status line (always shown)
        io = imgui.get_io()
        pc = sim.config.physics
        rtf = sim.clock.real_time_factor if sim.is_playing else 0.0
        b = ui.bridge
        ros_col = OK if b.status == "connected" else (ERROR if b.status == "error" else MUTED)
        imgui.text(f"Sim time {sim.sim_time:10.3f} s")
        imgui.same_line(190)
        imgui.text(f"FPS {io.framerate:6.1f}")
        imgui.same_line(300)
        imgui.text(f"Physics {pc.frequency_hz:g} Hz (dt {pc.timestep * 1000:.3f} ms)")
        imgui.same_line(560)
        imgui.text(f"Effective {rtf * pc.frequency_hz:7.1f} Hz  RTF {rtf:4.2f}")
        imgui.same_line(800)
        imgui.text_colored(f"ROS 2: {b.status}", *ros_col)
        imgui.separator()
        if imgui.begin_tab_bar("bottom_tabs"):
            sel, _ = imgui.begin_tab_item("Console")
            if sel:
                self.console.draw()
                imgui.end_tab_item()
            sel, _ = imgui.begin_tab_item("Statistics")
            if sel:
                self._stats()
                imgui.end_tab_item()
            sel, _ = imgui.begin_tab_item("ROS 2")
            if sel:
                self._ros()
                imgui.end_tab_item()
            imgui.end_tab_bar()

    def _stats(self) -> None:
        ui, sim = self.ui, self.ui.sim
        app = ui.app
        imgui.columns(3, "stats", border=False)
        kv("State", sim.state.value)
        kv("Speed", f"{sim.speed:g}x")
        kv("Physics steps", str(sim.engine.step_count))
        kv("Steps last frame", str(sim.last_update_steps))
        imgui.next_column()
        kv("Physics time/frame", f"{sim.last_physics_wall_time * 1000:.2f} ms")
        kv("Render time/frame", f"{app.render_time * 1000:.2f} ms")
        kv("Draw calls", str(app.renderer.draw_calls if app.renderer else 0))
        kv("Dropped sim time", f"{sim.clock.dropped_time:.3f} s",
           WARN if sim.clock.dropped_time > 0 else None)
        imgui.next_column()
        kv("Robots", str(len(sim.robots)))
        kv("Objects", str(len(sim.world.objects)))
        n_sensors = sum(len(r.sensors) for r in sim.robots.values())
        kv("Sensors", str(n_sensors))
        kv("Memory (RSS)", f"{app.memory_mb():.0f} MB")
        imgui.columns(1)

    def _ros(self) -> None:
        b = self.ui.bridge
        kv("Status", b.status)
        if b.error:
            imgui.same_line()
            imgui.text_colored(b.error, *WARN)
        kv("Messages published", str(b.published))
        kv("Dropped (backlog)", str(b.dropped))
        if b.status == "connected":
            imgui.begin_child("topics", 0, 0, border=True)
            for topic, typ in b.topics():
                imgui.text(topic)
                imgui.same_line(360)
                imgui.text_colored(typ, *MUTED)
            imgui.end_child()
