"""Hidden GLFW window with an OpenGL 3.3 core context.

Use it without the GUI: for camera sensors in headless mode and for tests.
"""
from __future__ import annotations

import logging

log = logging.getLogger(__name__)


def create_gl_window(width: int, height: int, title: str, visible: bool = True):
    import glfw

    if not glfw.init():
        raise RuntimeError("GLFW initialisation failed (is a display available?)")
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, True)
    glfw.window_hint(glfw.VISIBLE, visible)
    window = glfw.create_window(width, height, title, None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("Could not create an OpenGL 3.3 core window")
    glfw.make_context_current(window)
    return window


class OffscreenContext:
    def __init__(self):
        import glfw
        self._glfw = glfw
        self.window = create_gl_window(64, 64, "offscreen", visible=False)

    def make_current(self) -> None:
        self._glfw.make_context_current(self.window)

    def close(self) -> None:
        if self.window:
            self._glfw.destroy_window(self.window)
            self.window = None
