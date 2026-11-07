"""Simple ImGui file browser. It does not need an external GUI toolkit."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Callable, Optional

import imgui

from .widgets import MUTED, WARN


class FileDialog:
    def __init__(self):
        self.visible = False
        self._request_open = False
        self.title = ""
        self.mode = "open"            # open | save
        self.extensions: tuple[str, ...] = ()
        self.callback: Optional[Callable[[str], None]] = None
        self.cwd = Path.cwd()
        self.filename = ""
        self.error = ""
        self._path_text = str(self.cwd)

    def open(self, title: str, extensions: tuple[str, ...], callback: Callable[[str], None],
             mode: str = "open", start_dir: Optional[str] = None, default_name: str = "") -> None:
        self.title, self.extensions, self.callback, self.mode = title, extensions, callback, mode
        if start_dir and Path(start_dir).is_dir():
            self.cwd = Path(start_dir).resolve()
        self._path_text = str(self.cwd)
        self.filename = default_name
        self.error = ""
        self.visible = True
        self._request_open = True

    def _entries(self):
        try:
            items = sorted(os.scandir(self.cwd), key=lambda e: (not e.is_dir(), e.name.lower()))
        except OSError as exc:
            self.error = str(exc)
            return [], []
        dirs = [e.name for e in items if e.is_dir() and not e.name.startswith(".")]
        files = [e.name for e in items if e.is_file() and not e.name.startswith(".")
                 and (not self.extensions or e.name.lower().endswith(self.extensions))]
        return dirs, files

    def draw(self) -> None:
        if not self.visible:
            return
        if self._request_open:
            imgui.open_popup(self.title)
            self._request_open = False
        io = imgui.get_io()
        imgui.set_next_window_size(min(760, io.display_size.x - 40), min(520, io.display_size.y - 40),
                                   imgui.ALWAYS)
        imgui.set_next_window_position(io.display_size.x / 2, io.display_size.y / 2, imgui.ALWAYS, 0.5, 0.5)
        opened, _ = imgui.begin_popup_modal(self.title, None, imgui.WINDOW_NO_RESIZE)
        if not opened:
            return
        if imgui.button("Up"):
            self.cwd = self.cwd.parent
            self._path_text = str(self.cwd)
        imgui.same_line()
        if imgui.button("Home"):
            self.cwd = Path.home()
            self._path_text = str(self.cwd)
        imgui.same_line()
        imgui.push_item_width(-1)
        changed, self._path_text = imgui.input_text("##path", self._path_text, 1024,
                                                    imgui.INPUT_TEXT_ENTER_RETURNS_TRUE)
        imgui.pop_item_width()
        if changed:
            p = Path(self._path_text).expanduser()
            if p.is_dir():
                self.cwd = p.resolve()
            elif p.is_file():
                self.cwd = p.parent.resolve()
                self.filename = p.name
            else:
                self.error = f"'{p}' does not exist"
        dirs, files = self._entries()
        imgui.begin_child("entries", 0, -70, border=True)
        for d in dirs:
            clicked, _ = imgui.selectable(f"[dir]  {d}", False, imgui.SELECTABLE_ALLOW_DOUBLE_CLICK)
            if clicked:
                self.cwd = (self.cwd / d).resolve()
                self._path_text = str(self.cwd)
        for f in files:
            clicked, _ = imgui.selectable(f"       {f}", f == self.filename, imgui.SELECTABLE_ALLOW_DOUBLE_CLICK)
            if clicked:
                self.filename = f
                if imgui.is_mouse_double_clicked(0):
                    self._accept()
        if not dirs and not files:
            imgui.text_colored("(empty)", *MUTED)
        imgui.end_child()
        imgui.push_item_width(-180)
        _, self.filename = imgui.input_text("File name", self.filename, 512)
        imgui.pop_item_width()
        if self.error:
            imgui.text_colored(self.error, *WARN)
        label = "Open" if self.mode == "open" else "Save"
        if imgui.button(label, 100):
            self._accept()
        imgui.same_line()
        if imgui.button("Cancel", 100):
            self.visible = False
            imgui.close_current_popup()
        if self.extensions:
            imgui.same_line()
            imgui.text_colored("Filter: " + ", ".join(self.extensions), *MUTED)
        if not self.visible:
            pass
        imgui.end_popup()

    def _accept(self) -> None:
        if not self.filename and self.mode == "open":
            self.error = "Select a file"
            return
        path = self.cwd / self.filename if self.filename else self.cwd
        if self.mode == "open" and not path.exists():
            self.error = f"'{path}' does not exist"
            return
        self.visible = False
        imgui.close_current_popup()
        if self.callback:
            self.callback(str(path))
