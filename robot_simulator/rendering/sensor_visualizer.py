"""Sends lines and points to the GPU in one buffer for each frame.

These are frames, joint axes, LiDAR scans and camera frustums.
"""
from __future__ import annotations

import ctypes

import numpy as np

from .gl import GL
from .shader_manager import ShaderManager


class DynamicBatch:
    """Vertex buffer with position and colour. Each frame fills it again."""

    def __init__(self):
        self.vao = GL.glGenVertexArrays(1)
        self.vbo = GL.glGenBuffers(1)
        self.capacity = 0
        GL.glBindVertexArray(self.vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, 28, ctypes.c_void_p(0))
        GL.glEnableVertexAttribArray(1)
        GL.glVertexAttribPointer(1, 4, GL.GL_FLOAT, GL.GL_FALSE, 28, ctypes.c_void_p(12))
        GL.glBindVertexArray(0)

    def upload(self, data: np.ndarray) -> None:
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
        if data.nbytes > self.capacity:
            self.capacity = max(data.nbytes, self.capacity * 2, 4096)
            GL.glBufferData(GL.GL_ARRAY_BUFFER, self.capacity, None, GL.GL_STREAM_DRAW)
        else:
            GL.glBufferData(GL.GL_ARRAY_BUFFER, self.capacity, None, GL.GL_STREAM_DRAW)  # release the old storage
        GL.glBufferSubData(GL.GL_ARRAY_BUFFER, 0, data.nbytes, data)

    def draw(self, mode, count: int) -> None:
        GL.glBindVertexArray(self.vao)
        GL.glDrawArrays(mode, 0, count)


class SensorVisualizer:
    def __init__(self, shaders: ShaderManager):
        self.shaders = shaders
        self.lines = DynamicBatch()
        self.points = DynamicBatch()

    def draw(self, lines: list[tuple], points: list[tuple], view, proj) -> None:
        if not lines and not points:
            return
        prog = self.shaders.line
        prog.use()
        prog.mat4("u_view", view)
        prog.mat4("u_proj", proj)
        if lines:
            data = np.empty((len(lines) * 2, 7), dtype=np.float32)
            for k, (a, b, col) in enumerate(lines):
                data[2 * k, :3] = a
                data[2 * k + 1, :3] = b
                data[2 * k, 3:] = col
                data[2 * k + 1, 3:] = col
            self.lines.upload(data)
            prog.float("u_point_size", 1.0)
            self.lines.draw(GL.GL_LINES, len(data))
        if points:
            chunks = []
            for pts, col in points:
                arr = np.empty((len(pts), 7), dtype=np.float32)
                arr[:, :3] = pts
                arr[:, 3:] = col
                chunks.append(arr)
            data = np.concatenate(chunks)
            self.points.upload(data)
            prog.float("u_point_size", 4.0)
            self.points.draw(GL.GL_POINTS, len(data))
