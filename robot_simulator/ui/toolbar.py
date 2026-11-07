"""Main menu bar and top toolbar (project, import, environment, simulation controls)."""
from __future__ import annotations

import imgui

from ..core.simulation_manager import SPEED_PRESETS, SimState
from ..environment.environment_objects import PRESETS
from .widgets import ACCENT, MUTED, OK, WARN, toggle_button

ENV_LABELS = {"ground_plane": "Ground plane", "box": "Box", "cylinder": "Cylinder", "sphere": "Sphere",
              "wall": "Static wall", "dynamic_box": "Dynamic box"}


def draw_menu_bar(ui) -> float:
    sim = ui.sim
    height = 0.0
    if imgui.begin_main_menu_bar():
        height = imgui.get_window_height()
        if imgui.begin_menu("File"):
            if imgui.menu_item("New Project", "Ctrl+N")[0]:
                ui.new_project()
            if imgui.menu_item("Open Project...", "Ctrl+O")[0]:
                ui.ask_open_project()
            if imgui.menu_item("Save Project", "Ctrl+S")[0]:
                ui.save_project() if ui.project.path else ui.ask_save_as()
            if imgui.menu_item("Save Project As...")[0]:
                ui.ask_save_as()
            imgui.separator()
            if imgui.menu_item("Import URDF...", "Ctrl+I")[0]:
                ui.ask_import_urdf()
            _, ui.import_fixed_base = imgui.menu_item("Import with fixed base", None, ui.import_fixed_base)
            imgui.separator()
            if imgui.menu_item("Quit", "Ctrl+Q")[0]:
                ui.app.request_quit()
            imgui.end_menu()
        if imgui.begin_menu("Environment"):
            for k in PRESETS:
                if imgui.menu_item(f"Add {ENV_LABELS[k]}")[0]:
                    ui.add_object(k)
            imgui.end_menu()
        if imgui.begin_menu("Simulation"):
            if imgui.menu_item("Play", "Space", sim.is_playing, not sim.is_playing)[0]:
                sim.play()
            if imgui.menu_item("Pause", "Space", False, sim.is_playing)[0]:
                sim.pause()
            if imgui.menu_item("Stop")[0]:
                sim.stop()
            if imgui.menu_item("Reset", "Ctrl+R")[0]:
                sim.reset()
            if imgui.menu_item("Single step", "S", False, not sim.is_playing)[0]:
                sim.single_step()
            imgui.separator()
            for s in SPEED_PRESETS:
                if imgui.menu_item(f"Speed {s:g}x", None, abs(sim.speed - s) < 1e-9)[0]:
                    sim.set_speed(s)
            imgui.end_menu()
        if imgui.begin_menu("View"):
            v = ui.view
            _, v.show_grid = imgui.menu_item("Grid", "G", v.show_grid)
            _, v.show_frames = imgui.menu_item("Link frames", None, v.show_frames)
            _, v.show_joint_axes = imgui.menu_item("Joint axes", None, v.show_joint_axes)
            _, v.show_collision = imgui.menu_item("Collision geometry", "C", v.show_collision)
            _, v.show_visuals = imgui.menu_item("Visual geometry", None, v.show_visuals)
            _, v.show_sensors = imgui.menu_item("Sensor visualisation", None, v.show_sensors)
            imgui.separator()
            if imgui.menu_item("Focus selection", "F")[0]:
                ui.focus_selection()
            imgui.end_menu()
        if imgui.begin_menu("ROS 2"):
            connected = ui.bridge.status == "connected"
            clicked, _ = imgui.menu_item("Enable bridge", None, connected, ui.bridge.available())
            if clicked:
                ui.set_ros_enabled(not connected)
            if imgui.menu_item("Topics and frames...")[0]:
                ui.show_topics = True
            imgui.end_menu()
        if imgui.begin_menu("Help"):
            if imgui.menu_item("Viewport controls")[0]:
                ui.show_controls = True
            if imgui.menu_item("About")[0]:
                ui.show_about = True
            imgui.end_menu()
        title = ui.project.name + (" *" if ui.project.dirty else "")
        imgui.same_line(imgui.get_window_width() - imgui.calc_text_size(title).x - 20)
        imgui.text_colored(title, *MUTED)
        imgui.end_main_menu_bar()
    return height


def draw_toolbar(ui, x: float, y: float, w: float, h: float) -> None:
    sim = ui.sim
    imgui.set_next_window_position(x, y)
    imgui.set_next_window_size(w, h)
    flags = (imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE |
             imgui.WINDOW_NO_SCROLLBAR | imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_NO_SAVED_SETTINGS)
    imgui.begin("##toolbar", False, flags)
    if imgui.button("New"):
        ui.new_project()
    imgui.same_line()
    if imgui.button("Open"):
        ui.ask_open_project()
    imgui.same_line()
    if imgui.button("Save"):
        ui.save_project() if ui.project.path else ui.ask_save_as()
    imgui.same_line()
    imgui.text_colored("|", *MUTED)
    imgui.same_line()
    if imgui.button("Import URDF"):
        ui.ask_import_urdf()
    imgui.same_line()
    if imgui.button("Add Environment"):
        imgui.open_popup("add_env")
    if imgui.begin_popup("add_env"):
        for k in PRESETS:
            if imgui.selectable(ENV_LABELS[k])[0]:
                ui.add_object(k)
        imgui.end_popup()
    imgui.same_line()
    imgui.text_colored("|", *MUTED)
    imgui.same_line()

    playing = sim.state is SimState.PLAYING
    if toggle_button("Play" if not playing else "Playing", playing, 70):
        sim.play()
    imgui.same_line()
    if toggle_button("Pause", sim.state is SimState.PAUSED, 60):
        sim.pause()
    imgui.same_line()
    if imgui.button("Stop", 50):
        sim.stop()
    imgui.same_line()
    if imgui.button("Reset", 55):
        sim.reset()
    imgui.same_line()
    if imgui.button("Step", 50):
        sim.single_step()
    imgui.same_line()
    imgui.push_item_width(80)
    labels = [f"{s:g}x" for s in SPEED_PRESETS]
    idx = min(range(len(SPEED_PRESETS)), key=lambda i: abs(SPEED_PRESETS[i] - sim.speed))
    changed, idx = imgui.combo("Speed", idx, labels)
    if changed:
        sim.set_speed(SPEED_PRESETS[idx])
    imgui.pop_item_width()
    imgui.same_line()
    state_col = {SimState.PLAYING: OK, SimState.PAUSED: WARN, SimState.STOPPED: MUTED}[sim.state]
    imgui.text_colored(f"  {sim.state.value.upper()}", *state_col)
    imgui.same_line()
    imgui.text(f"  t = {sim.sim_time:9.3f} s")
    imgui.same_line()
    rtf = sim.clock.real_time_factor if sim.is_playing else 0.0
    imgui.text_colored(f"  RTF {rtf:4.2f}", *ACCENT)
    if ui.status_message:
        imgui.same_line()
        imgui.text_colored("   " + ui.status_message, *MUTED)
    imgui.end()
