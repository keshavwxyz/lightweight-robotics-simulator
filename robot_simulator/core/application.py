"""Application loops.

GUI mode (`Application`): The main thread does the input, the simulation
scheduler (fixed-timestep physics and sensors) and the rendering, in this
sequence. The ROS 2 bridge has its own thread. The physics time follows the
wall-clock time. Thus, the frame rate does not change the physics timestep.

Headless mode (`HeadlessRunner`): The same simulation loop runs without a
window and stays at real time. If possible, the cameras use a hidden OpenGL
context.
"""
from __future__ import annotations

import logging
import os
import resource
import signal
import time
from pathlib import Path
from typing import Optional

from ..project.project_manager import ProjectManager
from ..ros2_bridge.bridge_manager import Ros2Bridge
from .configuration import AppConfig
from .event_manager import install_log_buffer
from .simulation_manager import SimulationManager

log = logging.getLogger(__name__)


def _rss_mb() -> float:
    try:
        with open("/proc/self/statm") as f:
            pages = int(f.read().split()[1])
        return pages * os.sysconf("SC_PAGE_SIZE") / 1e6
    except Exception:
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


class Application:
    def __init__(self, config: Optional[AppConfig] = None, project: Optional[str] = None,
                 urdfs: tuple[str, ...] = (), enable_ros: Optional[bool] = None, autoplay: bool = False):
        self.config = config or AppConfig()
        self.log_buffer = install_log_buffer()
        self.sim = SimulationManager(self.config)
        self.bridge = Ros2Bridge(self.sim)
        self.project = ProjectManager(self.sim)
        self.renderer = None
        self.window = None
        self.render_time = 0.0
        self._quit = False
        self._initial_project = project
        self._initial_urdfs = urdfs
        self._enable_ros = self.config.ros2.enabled if enable_ros is None else enable_ros
        self._autoplay = autoplay
        self.screenshot_path: Optional[str] = None   # For tests: save one frame, then stop.
        self._frame = 0

    def request_quit(self) -> None:
        self._quit = True

    def memory_mb(self) -> float:
        return _rss_mb()

    def run(self) -> int:
        import glfw
        import imgui
        from imgui.integrations.glfw import GlfwRenderer

        from ..rendering.offscreen_context import create_gl_window
        from ..rendering.renderer import Renderer
        from ..ui.main_window import MainWindow
        from ..ui.ui_context import UIContext
        from ..ui.widgets import apply_theme

        rc = self.config.render
        self.window = create_gl_window(rc.window_width, rc.window_height, "Lightweight Robotics Simulator")
        glfw.swap_interval(1 if rc.vsync else 0)
        imgui.create_context()
        io = imgui.get_io()
        io.ini_file_name = None
        font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
        xscale, _ = glfw.get_window_content_scale(self.window)
        if font_path.is_file():
            io.fonts.add_font_from_file_ttf(str(font_path), round(15 * max(xscale, 1.0)))
        impl = GlfwRenderer(self.window)
        apply_theme()

        self.renderer = Renderer(rc.background)
        self.sim.set_image_backend(self.renderer.make_image_backend(self.sim))
        ui = UIContext(self)
        for k in ("show_grid", "show_frames", "show_collision", "show_sensors", "show_joint_axes"):
            setattr(ui.view, k, getattr(rc, k))
        main = MainWindow(ui, self.log_buffer)
        self.sim.events.subscribe("robot_added", lambda *_: self.renderer.builder.forget())
        self.sim.events.subscribe("robot_removed", lambda *_: self.renderer.builder.forget())

        self._startup(ui)
        last = time.perf_counter()
        last_idle_publish = 0.0
        try:
            while not glfw.window_should_close(self.window) and not self._quit and not self.sim.quit_requested:
                glfw.poll_events()
                impl.process_inputs()
                now = time.perf_counter()
                dt = now - last
                last = now
                # Simulation scheduler: fixed-step physics, sensors and ROS queues.
                self.sim.update(dt)
                if not self.sim.is_playing and self.bridge.status == "connected" and now - last_idle_publish > 1.0:
                    self.sim.publish_state_now()   # While paused, publish the state again so that TF stays available.
                    last_idle_publish = now
                # Rendering and user interface.
                t0 = time.perf_counter()
                imgui.new_frame()
                main.draw()
                fw, fh = glfw.get_framebuffer_size(self.window)
                from ..rendering.gl import GL
                GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
                GL.glViewport(0, 0, fw, fh)
                GL.glClearColor(0.08, 0.08, 0.09, 1)
                GL.glClear(GL.GL_COLOR_BUFFER_BIT)
                imgui.render()
                impl.render(imgui.get_draw_data())
                self.render_time = time.perf_counter() - t0
                self._frame += 1
                if self.screenshot_path and self._frame == 120:
                    self._save_screenshot(fw, fh)
                    self._quit = True
                glfw.swap_buffers(self.window)
                if glfw.get_window_attrib(self.window, glfw.ICONIFIED):
                    time.sleep(0.02)
        finally:
            self.bridge.stop()
            self.renderer.release()
            impl.shutdown()
            glfw.terminate()
            self.sim.shutdown()
        return 0

    def _save_screenshot(self, w: int, h: int) -> None:
        import numpy as np
        from ..rendering.gl import GL
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
        raw = GL.glReadPixels(0, 0, w, h, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        img = np.frombuffer(raw, np.uint8).reshape(h, w, 3)[::-1]
        try:
            from PIL import Image
            Image.fromarray(img).save(self.screenshot_path)
        except ImportError:
            np.save(self.screenshot_path, img)
        log.info("Screenshot saved to %s", self.screenshot_path)

    def _startup(self, ui) -> None:
        if self._initial_project:
            ui.open_project(self._initial_project)
        for u in self._initial_urdfs:
            ui.import_urdf(u)
        if self._enable_ros:
            if self.bridge.available():
                self.bridge.start()
            else:
                log.warning("ROS 2 is not available (%s); running without the bridge",
                            "source /opt/ros/<distro>/setup.bash")
        if self._autoplay:
            self.sim.play()


class HeadlessRunner:
    """Run the simulation without a GUI (for example, on a server or in CI)."""

    def __init__(self, config: Optional[AppConfig] = None, project: Optional[str] = None,
                 urdfs: tuple[str, ...] = (), enable_ros: bool = True, cameras: bool = True):
        self.config = config or AppConfig()
        self.sim = SimulationManager(self.config)
        self.bridge = Ros2Bridge(self.sim)
        self.project = ProjectManager(self.sim)
        self._ctx = None
        self._renderer = None
        if cameras:
            try:
                from ..rendering.offscreen_context import OffscreenContext
                from ..rendering.renderer import Renderer
                self._ctx = OffscreenContext()
                self._renderer = Renderer()
                self.sim.set_image_backend(self._renderer.make_image_backend(self.sim))
            except Exception as exc:
                log.warning("No OpenGL context for camera sensors (%s); cameras disabled", exc)
        if project:
            report = self.project.open(project)
            for e in report.errors:
                log.error("%s", e)
        for u in urdfs:
            from .robot_instance import RobotConfig
            self.sim.spawn_robot(RobotConfig(urdf_path=u))
        if enable_ros:
            self.bridge.start()
        self._stop = False

    def run(self, duration: Optional[float] = None) -> None:
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "_stop", True))
        signal.signal(signal.SIGTERM, lambda *_: setattr(self, "_stop", True))
        self.sim.play()
        start = last = time.perf_counter()
        dt_target = self.sim.timestep
        log.info("Headless simulation running (Ctrl+C to stop)")
        try:
            while not self._stop and not self.sim.quit_requested:
                now = time.perf_counter()
                self.sim.update(now - last)
                last = now
                if duration is not None and self.sim.sim_time >= duration:
                    break
                # Wait until approximately the time of the next physics step.
                remaining = dt_target - self.sim.clock.accumulator
                time.sleep(max(0.0005, min(remaining / max(self.sim.speed, 1e-3), 0.01)))
        finally:
            log.info("Stopping after %.2f s simulation time (%.2f s wall)", self.sim.sim_time,
                     time.perf_counter() - start)
            self.bridge.stop()
            if self._renderer:
                self._renderer.release()
            if self._ctx:
                self._ctx.close()
            self.sim.shutdown()
