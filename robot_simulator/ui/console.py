"""Console. It shows the log records of core.event_manager.LogBuffer."""
from __future__ import annotations

import logging
import time

import imgui

from .widgets import ERROR, MUTED, WARN

LEVELS = [("All", logging.DEBUG), ("Info", logging.INFO), ("Warnings", logging.WARNING), ("Errors", logging.ERROR)]


class Console:
    def __init__(self, buffer):
        self.buffer = buffer
        self.level_idx = 0
        self.filter = ""
        self.autoscroll = True
        self._last_version = -1

    def draw(self) -> None:
        imgui.push_item_width(110)
        _, self.level_idx = imgui.combo("##level", self.level_idx, [l[0] for l in LEVELS])
        imgui.pop_item_width()
        imgui.same_line()
        imgui.push_item_width(200)
        _, self.filter = imgui.input_text("Filter", self.filter, 128)
        imgui.pop_item_width()
        imgui.same_line()
        if imgui.button("Clear"):
            self.buffer.clear()
        imgui.same_line()
        _, self.autoscroll = imgui.checkbox("Auto-scroll", self.autoscroll)
        imgui.begin_child("console_log", 0, 0, border=True)
        min_level = LEVELS[self.level_idx][1]
        flt = self.filter.lower()
        for ts, level, src, msg in self.buffer.snapshot():
            if level < min_level or (flt and flt not in msg.lower() and flt not in src.lower()):
                continue
            stamp = time.strftime("%H:%M:%S", time.localtime(ts))
            line = f"{stamp} [{src}] {msg}"
            if level >= logging.ERROR:
                imgui.text_colored(line, *ERROR)
            elif level >= logging.WARNING:
                imgui.text_colored(line, *WARN)
            else:
                imgui.text_colored(line, *MUTED) if level < logging.INFO else imgui.text(line)
        if self.autoscroll and self.buffer.version != self._last_version:
            imgui.set_scroll_here_y(1.0)
            self._last_version = self.buffer.version
        imgui.end_child()
