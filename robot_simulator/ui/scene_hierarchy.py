"""Left panel: scene hierarchy (environment, robots, links, joints, sensors)."""
from __future__ import annotations

import imgui

from .widgets import MUTED

LEAF = imgui.TREE_NODE_LEAF | imgui.TREE_NODE_NO_TREE_PUSH_ON_OPEN
BASE = imgui.TREE_NODE_OPEN_ON_ARROW | imgui.TREE_NODE_SPAN_AVAILABLE_WIDTH
SENSOR_LABEL = {"camera": "Camera", "imu": "IMU", "lidar_2d": "LiDAR 2D"}


def _flags(selected: bool, leaf: bool = False, open_default: bool = False) -> int:
    f = BASE | (LEAF if leaf else 0)
    if selected:
        f |= imgui.TREE_NODE_SELECTED
    if open_default:
        f |= imgui.TREE_NODE_DEFAULT_OPEN
    return f


def draw_scene_hierarchy(ui) -> None:
    sim = ui.sim
    sel = ui.selection
    imgui.text_colored("SCENE", *MUTED)
    imgui.separator()
    if imgui.tree_node("Scene", _flags(sel.kind == "", open_default=True)):
        if imgui.is_item_clicked():
            ui.select()
        if imgui.tree_node(f"Environment ({len(sim.world.objects)})", _flags(False, open_default=True)):
            for name, obj in sim.world.objects.items():
                tag = "static" if obj.config.is_static else "dynamic"
                imgui.tree_node(f"{name}  [{obj.config.shape}, {tag}]##obj_{name}",
                                _flags(sel.kind == "object" and sel.name == name, leaf=True))
                if imgui.is_item_clicked():
                    ui.select("object", "", name)
                if imgui.is_item_hovered() and imgui.is_mouse_double_clicked(0):
                    ui.focus_selection()
            imgui.tree_pop()
        for rname, robot in list(sim.robots.items()):
            opened = imgui.tree_node(f"{rname}##robot_{rname}",
                                     _flags(sel.kind == "robot" and sel.robot == rname, open_default=True))
            if imgui.is_item_clicked():
                ui.select("robot", rname)
            if imgui.is_item_hovered() and imgui.is_mouse_double_clicked(0):
                ui.focus_selection()
            if not opened:
                continue
            _draw_link(ui, robot, robot.model.root_link)
            if robot.sensors:
                if imgui.tree_node(f"Sensors ({len(robot.sensors)})##sens_{rname}", _flags(False, open_default=True)):
                    for sname, s in robot.sensors.items():
                        imgui.tree_node(f"{SENSOR_LABEL[s.TYPE]}: {sname}  @{s.config.link}##s_{rname}_{sname}",
                                        _flags(sel.kind == "sensor" and sel.robot == rname and sel.name == sname,
                                               leaf=True))
                        if imgui.is_item_clicked():
                            ui.select("sensor", rname, sname)
                    imgui.tree_pop()
            imgui.tree_pop()
        imgui.tree_pop()


def _draw_link(ui, robot, link: str) -> None:
    sel = ui.selection
    rname = robot.name
    child_joints = robot.model.child_joints_of[link]
    is_root = link == robot.model.root_link
    label = f"{link}{'  (base)' if is_root else ''}##l_{rname}_{link}"
    opened = imgui.tree_node(label, _flags(sel.kind == "link" and sel.robot == rname and sel.name == link,
                                           leaf=not child_joints, open_default=True))
    if imgui.is_item_clicked():
        ui.select("link", rname, link)
    if not child_joints or not opened:
        return
    for j in child_joints:
        motor = robot.motors.get(j.name)
        badge = f" [{motor.TYPE[:3]} motor]" if motor else (" [mimic]" if j.mimic else "")
        jopen = imgui.tree_node(f"{j.name} ({j.type}){badge}##j_{rname}_{j.name}",
                                _flags(sel.kind == "joint" and sel.robot == rname and sel.name == j.name,
                                       open_default=True))
        if imgui.is_item_clicked():
            ui.select("joint", rname, j.name)
        if jopen:
            _draw_link(ui, robot, j.child)
            imgui.tree_pop()
    imgui.tree_pop()
