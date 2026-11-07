"""Small ImGui helpers shared by the panels."""
from __future__ import annotations

import math
from typing import Optional, Sequence

import imgui

ACCENT = (0.33, 0.62, 0.98, 1.0)
MUTED = (0.62, 0.64, 0.68, 1.0)
WARN = (0.98, 0.75, 0.30, 1.0)
ERROR = (0.98, 0.42, 0.40, 1.0)
OK = (0.45, 0.85, 0.50, 1.0)


def heading(text: str) -> None:
    imgui.spacing()
    imgui.text_colored(text.upper(), *ACCENT)
    imgui.separator()


def kv(key: str, value: str, color: Optional[tuple] = None) -> None:
    imgui.text_colored(key, *MUTED)
    imgui.same_line(150)
    if color:
        imgui.text_colored(value, *color)
    else:
        imgui.text(value)


def fmt_vec(v: Sequence[float], digits: int = 3) -> str:
    return "(" + ", ".join(f"{x:.{digits}f}" for x in v) + ")"


def vec3(label: str, v: Sequence[float], speed: float = 0.01, fmt: str = "%.3f") -> tuple[bool, tuple]:
    changed, out = imgui.drag_float3(label, float(v[0]), float(v[1]), float(v[2]), speed, 0.0, 0.0, fmt)
    return changed, tuple(float(x) for x in out)


def float_input(label: str, value: float, fmt: str = "%.4f", min_value: Optional[float] = None,
                max_value: Optional[float] = None) -> tuple[bool, float]:
    changed, v = imgui.input_float(label, float(value), 0.0, 0.0, fmt, imgui.INPUT_TEXT_ENTER_RETURNS_TRUE)
    if changed:
        if not math.isfinite(v):
            return False, value
        if min_value is not None:
            v = max(min_value, v)
        if max_value is not None:
            v = min(max_value, v)
    return changed, v


def combo_value(label: str, current: str, options: Sequence[str]) -> tuple[bool, str]:
    opts = list(options)
    idx = opts.index(current) if current in opts else 0
    if not opts:
        imgui.text_colored(f"{label}: (none available)", *MUTED)
        return False, current
    changed, idx = imgui.combo(label, idx, opts)
    return changed, opts[idx]


def help_marker(text: str) -> None:
    imgui.same_line()
    imgui.text_disabled("(?)")
    if imgui.is_item_hovered():
        imgui.begin_tooltip()
        imgui.push_text_wrap_pos(imgui.get_font_size() * 30)
        imgui.text(text)
        imgui.pop_text_wrap_pos()
        imgui.end_tooltip()


def danger_button(label: str) -> bool:
    imgui.push_style_color(imgui.COLOR_BUTTON, 0.55, 0.18, 0.18, 1.0)
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.70, 0.22, 0.22, 1.0)
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.80, 0.25, 0.25, 1.0)
    clicked = imgui.button(label)
    imgui.pop_style_color(3)
    return clicked


def toggle_button(label: str, active: bool, width: float = 0) -> bool:
    if active:
        imgui.push_style_color(imgui.COLOR_BUTTON, *ACCENT)
    clicked = imgui.button(label, width)
    if active:
        imgui.pop_style_color(1)
    return clicked


def apply_theme() -> None:
    style = imgui.get_style()
    style.window_rounding = 4.0
    style.frame_rounding = 3.0
    style.grab_rounding = 3.0
    style.tab_rounding = 3.0
    style.scrollbar_rounding = 3.0
    style.window_border_size = 1.0
    style.frame_padding = (6, 4)
    style.item_spacing = (8, 6)
    style.window_padding = (10, 8)
    style.indent_spacing = 14.0
    c = style.colors
    def set_(idx, r, g, b, a=1.0):
        c[idx] = (r, g, b, a)
    set_(imgui.COLOR_WINDOW_BACKGROUND, 0.105, 0.11, 0.125)
    set_(imgui.COLOR_CHILD_BACKGROUND, 0.105, 0.11, 0.125)
    set_(imgui.COLOR_POPUP_BACKGROUND, 0.12, 0.125, 0.14, 0.98)
    set_(imgui.COLOR_MENUBAR_BACKGROUND, 0.13, 0.135, 0.155)
    set_(imgui.COLOR_BORDER, 0.22, 0.23, 0.26)
    set_(imgui.COLOR_FRAME_BACKGROUND, 0.17, 0.18, 0.20)
    set_(imgui.COLOR_FRAME_BACKGROUND_HOVERED, 0.22, 0.23, 0.26)
    set_(imgui.COLOR_FRAME_BACKGROUND_ACTIVE, 0.25, 0.27, 0.31)
    set_(imgui.COLOR_TITLE_BACKGROUND, 0.13, 0.135, 0.155)
    set_(imgui.COLOR_TITLE_BACKGROUND_ACTIVE, 0.15, 0.16, 0.19)
    set_(imgui.COLOR_BUTTON, 0.20, 0.22, 0.26)
    set_(imgui.COLOR_BUTTON_HOVERED, 0.27, 0.33, 0.43)
    set_(imgui.COLOR_BUTTON_ACTIVE, 0.30, 0.45, 0.70)
    set_(imgui.COLOR_HEADER, 0.22, 0.30, 0.45)
    set_(imgui.COLOR_HEADER_HOVERED, 0.27, 0.38, 0.58)
    set_(imgui.COLOR_HEADER_ACTIVE, 0.30, 0.45, 0.70)
    set_(imgui.COLOR_TAB, 0.16, 0.17, 0.20)
    set_(imgui.COLOR_TAB_HOVERED, 0.27, 0.38, 0.58)
    set_(imgui.COLOR_TAB_ACTIVE, 0.24, 0.34, 0.52)
    set_(imgui.COLOR_CHECK_MARK, *ACCENT[:3])
    set_(imgui.COLOR_SLIDER_GRAB, 0.33, 0.55, 0.90)
    set_(imgui.COLOR_SLIDER_GRAB_ACTIVE, *ACCENT[:3])
    set_(imgui.COLOR_SEPARATOR, 0.22, 0.23, 0.26)
    set_(imgui.COLOR_TEXT, 0.90, 0.91, 0.93)
