"""Single import point for PyOpenGL.

PyOpenGL error checks add one glGetError() call to each OpenGL call. Thus,
the error checks are off. Set ROBOT_SIM_GL_DEBUG=1 to turn them on.
"""
import os

import OpenGL

OpenGL.ERROR_CHECKING = os.environ.get("ROBOT_SIM_GL_DEBUG", "0") == "1"
OpenGL.ERROR_LOGGING = OpenGL.ERROR_CHECKING

from OpenGL import GL  # noqa: E402

__all__ = ["GL"]
