"""Main window layout.

Toolbar at the top, scene hierarchy at the left, 3D viewport in the center,
properties at the right, console and statistics at the bottom.
"""
from __future__ import annotations

import imgui

from .camera_view import CameraViews
from .properties_panel import PropertiesPanel
from .scene_hierarchy import draw_scene_hierarchy
from .simulation_panel import SimulationPanel
from .toolbar import draw_menu_bar, draw_toolbar
from .widgets import MUTED

FIXED = (imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_NO_SAVED_SETTINGS |
         imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_BRING_TO_FRONT_ON_FOCUS)

CONTROLS_TEXT = """Viewport
  Left drag         orbit
  Right / middle    pan
  Wheel             zoom
  Click             select (click again to select a link)
  F                 focus selection

Keyboard
  Space             play / pause
  S                 single step (paused)
  Ctrl+R            reset
  G / C             toggle grid / collision geometry
  Ctrl+N/O/S/I      new / open / save / import URDF
  Delete            remove selected robot / object / sensor"""


class MainWindow:
    def __init__(self, ui, log_buffer):
        self.ui = ui
        self.properties = PropertiesPanel(ui)
        self.bottom = SimulationPanel(ui, log_buffer)
        self.cameras = CameraViews(ui)
        self.left_w = 290.0
        self.right_w = 380.0
        self.bottom_h = 220.0
        self._drag_moved = False

    def draw(self) -> None:
        ui = self.ui
        io = imgui.get_io()
        W, H = io.display_size.x, io.display_size.y
        menu_h = draw_menu_bar(ui)
        tb_h = 40.0
        draw_toolbar(ui, 0, menu_h, W, tb_h)
        top = menu_h + tb_h
        bottom_y = H - self.bottom_h
        mid_h = max(bottom_y - top, 50)

        imgui.set_next_window_position(0, top)
        imgui.set_next_window_size(self.left_w, mid_h)
        imgui.begin("Scene##hier", False, FIXED | imgui.WINDOW_HORIZONTAL_SCROLLING_BAR)
        draw_scene_hierarchy(ui)
        imgui.end()

        imgui.set_next_window_position(W - self.right_w, top)
        imgui.set_next_window_size(self.right_w, mid_h)
        imgui.begin("Properties##props", False, FIXED)
        self.properties.draw()
        imgui.end()

        imgui.set_next_window_position(0, bottom_y)
        imgui.set_next_window_size(W, self.bottom_h)
        imgui.begin("Bottom##bottom", False, FIXED)
        self.bottom.draw()
        imgui.end()

        self._viewport(self.left_w, top, W - self.left_w - self.right_w, mid_h)
        self.cameras.draw()
        self._dialogs()
        self._shortcuts()

    def _viewport(self, x: float, y: float, w: float, h: float) -> None:
        ui = self.ui
        imgui.set_next_window_position(x, y)
        imgui.set_next_window_size(w, h)
        imgui.push_style_var(imgui.STYLE_WINDOW_PADDING, (0, 0))
        imgui.begin("Viewport##vp", False, FIXED | imgui.WINDOW_NO_SCROLLBAR | imgui.WINDOW_NO_SCROLL_WITH_MOUSE)
        avail = imgui.get_content_region_available()
        vw, vh = int(max(avail.x, 1)), int(max(avail.y, 1))
        tex = ui.app.renderer.render_viewport(ui.sim, ui.camera, vw, vh, ui.view, ui.selection)
        origin = imgui.get_cursor_screen_position()
        imgui.image(tex, vw, vh, (0, 1), (1, 0))
        hovered = imgui.is_item_hovered()
        io = imgui.get_io()
        if hovered:
            if io.mouse_wheel:
                ui.camera.zoom(io.mouse_wheel)
            if imgui.is_mouse_clicked(0) or imgui.is_mouse_clicked(1) or imgui.is_mouse_clicked(2):
                self._drag_moved = False
        if imgui.is_mouse_dragging(0, 2.0) and (hovered or self._drag_moved) and imgui.is_window_focused():
            ui.camera.orbit(io.mouse_delta.x, io.mouse_delta.y)
            self._drag_moved = True
        if (imgui.is_mouse_dragging(1, 2.0) or imgui.is_mouse_dragging(2, 2.0)) and (hovered or self._drag_moved) \
                and imgui.is_window_focused():
            ui.camera.pan(io.mouse_delta.x, io.mouse_delta.y, vh)
            self._drag_moved = True
        if hovered and imgui.is_mouse_released(0) and not self._drag_moved:
            mx, my = io.mouse_pos.x - origin.x, io.mouse_pos.y - origin.y
            ui.pick(2.0 * mx / vw - 1.0, 1.0 - 2.0 * my / vh, vw / vh)
        if not (imgui.is_mouse_down(0) or imgui.is_mouse_down(1) or imgui.is_mouse_down(2)):
            self._drag_moved = False
        # overlay hint
        imgui.set_cursor_screen_pos((origin.x + 8, origin.y + 6))
        sel = ui.selection
        path = "/".join(x for x in (sel.robot, sel.name) if x)
        label = f"{sel.kind}: {path}" if sel.kind else "nothing selected"
        imgui.text_colored(label, *MUTED)
        imgui.end()
        imgui.pop_style_var()

    def _dialogs(self) -> None:
        ui = self.ui
        ui.file_dialog.draw()
        if ui.error_popup is not None:
            imgui.open_popup("Error##err")
        io = imgui.get_io()
        imgui.set_next_window_position(io.display_size.x / 2, io.display_size.y / 2, imgui.APPEARING, 0.5, 0.5)
        imgui.set_next_window_size(560, 0, imgui.APPEARING)
        opened, _ = imgui.begin_popup_modal("Error##err", None, imgui.WINDOW_ALWAYS_AUTO_RESIZE)
        if opened:
            imgui.push_text_wrap_pos(540)
            imgui.text(ui.error_popup or "")
            imgui.pop_text_wrap_pos()
            if imgui.button("OK", 120):
                ui.error_popup = None
                imgui.close_current_popup()
            imgui.end_popup()
        for flag, title, body in (("show_controls", "Viewport controls", CONTROLS_TEXT),
                                  ("show_about", "About", ABOUT_TEXT)):
            if getattr(ui, flag):
                imgui.set_next_window_size(520, 0, imgui.APPEARING)
                expanded, opened = imgui.begin(title, True, imgui.WINDOW_ALWAYS_AUTO_RESIZE)
                if expanded:
                    imgui.text(body)
                imgui.end()
                if not opened:
                    setattr(ui, flag, False)
        if ui.show_topics:
            imgui.set_next_window_size(640, 420, imgui.FIRST_USE_EVER)
            expanded, opened = imgui.begin("ROS 2 topics and frames", True)
            if expanded:
                self._topics_window()
            imgui.end()
            if not opened:
                ui.show_topics = False

    def _topics_window(self) -> None:
        ui = self.ui
        b = ui.bridge
        if b.status != "connected":
            imgui.text_colored("Bridge is not running (ROS 2 menu -> Enable bridge).", *MUTED)
            return
        imgui.text("Topics")
        imgui.separator()
        for topic, typ in b.topics():
            imgui.text(topic)
            imgui.same_line(330)
            imgui.text_colored(typ, *MUTED)
        imgui.spacing()
        imgui.text("TF frames (parent -> child)")
        imgui.separator()
        for parent, child, _ in b.tf.all_static():
            imgui.text(f"{parent} -> {child}")
            imgui.same_line(330)
            imgui.text_colored("static", *MUTED)
        for name, d in b.tf.descriptors.items():
            p = d.tf_prefix
            if d.odom_enabled:
                imgui.text(f"{p}{d.odom_frame} -> {p}{d.root_link}")
            elif not d.fixed_base:
                imgui.text(f"world -> {p}{d.root_link}")
            for j in d.joints:
                if j.type != "fixed":
                    imgui.text(f"{p}{j.parent} -> {p}{j.child}")
                    imgui.same_line(330)
                    imgui.text_colored(f"dynamic ({j.type})", *MUTED)

    def _shortcuts(self) -> None:
        ui = self.ui
        io = imgui.get_io()
        if io.want_text_input or ui.file_dialog.visible:
            return
        import glfw
        ctrl = io.key_ctrl

        def pressed(key):
            return imgui.is_key_pressed(key, False)

        sim = ui.sim
        if ctrl:
            if pressed(glfw.KEY_S):
                ui.save_project() if ui.project.path else ui.ask_save_as()
            elif pressed(glfw.KEY_O):
                ui.ask_open_project()
            elif pressed(glfw.KEY_N):
                ui.new_project()
            elif pressed(glfw.KEY_I):
                ui.ask_import_urdf()
            elif pressed(glfw.KEY_R):
                sim.reset()
            elif pressed(glfw.KEY_Q):
                ui.app.request_quit()
            return
        if pressed(glfw.KEY_SPACE):
            sim.pause() if sim.is_playing else sim.play()
        elif pressed(glfw.KEY_S) and not sim.is_playing:
            sim.single_step()
        elif pressed(glfw.KEY_F):
            ui.focus_selection()
        elif pressed(glfw.KEY_G):
            ui.view.show_grid = not ui.view.show_grid
        elif pressed(glfw.KEY_C):
            ui.view.show_collision = not ui.view.show_collision
        elif pressed(glfw.KEY_DELETE):
            sel = ui.selection
            if sel.kind == "robot":
                ui.remove_robot(sel.robot)
            elif sel.kind == "object":
                ui.remove_object(sel.name)
            elif sel.kind == "sensor" and ui.selected_robot():
                ui.selected_robot().remove_sensor(sel.name)
                ui.select("robot", sel.robot)


ABOUT_TEXT = """Lightweight Robotics Simulator

Physics:   PyBullet (DIRECT mode)
Rendering: custom OpenGL 3.3 renderer + Dear ImGui
Robots:    URDF only
Sensors:   RGB camera, perfect IMU, 2D LiDAR
Motors:    rotational, linear
Middleware: ROS 2 (rclpy), optional"""
