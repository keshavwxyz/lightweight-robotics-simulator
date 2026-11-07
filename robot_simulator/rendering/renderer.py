"""Custom OpenGL 3.3 core renderer.

The renderer draws a `RenderScene` into an off-screen framebuffer. The
viewport and the camera sensors use the same code. The viewport uses a
multisampled framebuffer and shows it as an ImGui image. The camera sensors
read the image back to CPU memory (`camera_image`). An OpenGL 3.3 core context
must be current.
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from .gl import GL
from .grid_renderer import GridRenderer
from .mesh_renderer import MeshLibrary
from .scene_builder import RenderScene, SceneBuilder, Selection, ViewOptions
from .sensor_visualizer import SensorVisualizer
from .shader_manager import ShaderManager

log = logging.getLogger(__name__)

LIGHT_DIR = np.array([-0.4, -0.3, -1.0])


class Framebuffer:
    def __init__(self, width: int, height: int, samples: int = 0):
        self.width, self.height, self.samples = width, height, samples
        self.fbo = GL.glGenFramebuffers(1)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.fbo)
        self.color_rb = self.depth_rb = None
        self.texture = None
        if samples > 0:
            self.color_rb, self.depth_rb = GL.glGenRenderbuffers(2)
            GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, self.color_rb)
            GL.glRenderbufferStorageMultisample(GL.GL_RENDERBUFFER, samples, GL.GL_RGBA8, width, height)
            GL.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0, GL.GL_RENDERBUFFER, self.color_rb)
            GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, self.depth_rb)
            GL.glRenderbufferStorageMultisample(GL.GL_RENDERBUFFER, samples, GL.GL_DEPTH_COMPONENT24, width, height)
            GL.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL.GL_DEPTH_ATTACHMENT, GL.GL_RENDERBUFFER, self.depth_rb)
            # resolve target
            self.resolve = Framebuffer(width, height, 0)
        else:
            self.resolve = None
            self.texture = GL.glGenTextures(1)
            GL.glBindTexture(GL.GL_TEXTURE_2D, self.texture)
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA8, width, height, 0, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, None)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
            GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0, GL.GL_TEXTURE_2D, self.texture, 0)
            self.depth_rb = GL.glGenRenderbuffers(1)
            GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, self.depth_rb)
            GL.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL.GL_DEPTH_COMPONENT24, width, height)
            GL.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL.GL_DEPTH_ATTACHMENT, GL.GL_RENDERBUFFER, self.depth_rb)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.fbo)
        status = GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        if status != GL.GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError(f"Incomplete framebuffer (status {status})")

    @property
    def color_texture(self) -> int:
        return self.resolve.texture if self.resolve else self.texture  # type: ignore[return-value]

    def resolve_samples(self) -> None:
        if self.resolve is None:
            return
        GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, self.fbo)
        GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, self.resolve.fbo)
        GL.glBlitFramebuffer(0, 0, self.width, self.height, 0, 0, self.width, self.height,
                             GL.GL_COLOR_BUFFER_BIT, GL.GL_NEAREST)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

    def delete(self) -> None:
        if self.resolve:
            self.resolve.delete()
        if self.texture:
            GL.glDeleteTextures([self.texture])
        rbs = [r for r in (self.color_rb, self.depth_rb) if r]
        if rbs:
            GL.glDeleteRenderbuffers(len(rbs), rbs)
        GL.glDeleteFramebuffers(1, [self.fbo])


class Renderer:
    def __init__(self, background=(0.16, 0.17, 0.20)):
        self.background = background
        self.shaders = ShaderManager()
        self.meshes = MeshLibrary()
        self.grid = GridRenderer(self.shaders, self.meshes)
        self.sensor_viz = SensorVisualizer(self.shaders)
        self.builder = SceneBuilder(self.meshes)
        self._viewport_fbo: Optional[Framebuffer] = None
        self._camera_fbos: dict[tuple[int, int], Framebuffer] = {}
        self.draw_calls = 0
        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)

    def draw_scene(self, scene: RenderScene, view: np.ndarray, proj: np.ndarray, eye: np.ndarray,
                   width: int, height: int, show_grid: bool = True, far: float = 100.0,
                   background=None) -> None:
        bg = background or self.background
        GL.glViewport(0, 0, width, height)
        GL.glClearColor(bg[0], bg[1], bg[2], 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LESS)
        GL.glDisable(GL.GL_CULL_FACE)
        GL.glDisable(GL.GL_SCISSOR_TEST)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
        calls = 0

        prog = self.shaders.mesh
        prog.use()
        prog.mat4("u_view", view)
        prog.mat4("u_proj", proj)
        prog.vec3("u_light_dir", LIGHT_DIR)
        prog.vec3("u_eye", eye)
        prog.int("u_unlit", 0)
        prog.int("u_tex", 0)
        for it in scene.items:
            calls += self._draw_item(prog, it)

        self.grid.draw_grounds(scene.grounds, view, proj, eye, show_grid, far)
        calls += len(scene.grounds)

        if scene.transparent:
            prog.use()
            prog.int("u_unlit", 1)
            GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
            for it in scene.transparent:
                calls += self._draw_item(prog, it)
            GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
            prog.int("u_unlit", 0)

        lines = scene.lines
        if show_grid and not scene.grounds:
            lines = lines + self.grid.fallback_grid_lines()
        if lines or scene.points:
            self.sensor_viz.draw(lines, scene.points, view, proj)
            calls += 2
        self.draw_calls = calls

    def _draw_item(self, prog, it) -> int:
        mesh = self.meshes.get(it.mesh_key)
        if mesh is None:
            return 0
        M = it.model
        prog.mat4("u_model", M)
        R = M[:3, :3]
        try:
            N = np.linalg.inv(R).T
        except np.linalg.LinAlgError:
            return 0
        prog.mat3("u_normal", N)
        prog.vec4("u_color", it.color)
        textured = bool(it.use_vcolor and mesh.texture is not None)
        prog.int("u_use_vcolor", 1 if it.use_vcolor and not textured else 0)
        prog.int("u_use_texture", 1 if textured else 0)
        if textured:
            mesh.bind_texture()
        prog.float("u_highlight", it.highlight)
        mesh.draw()
        return 1

    def render_viewport(self, sim, camera, width: int, height: int, opts: ViewOptions,
                        selection: Selection) -> int:
        width, height = max(int(width), 1), max(int(height), 1)
        fb = self._viewport_fbo
        if fb is None or fb.width != width or fb.height != height:
            if fb is not None:
                fb.delete()
            fb = self._viewport_fbo = Framebuffer(width, height, samples=4)
        scene = self.builder.build(sim, opts, selection, overlays=True)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fb.fbo)
        self.draw_scene(scene, camera.view_matrix(), camera.projection_matrix(width / height), camera.eye,
                        width, height, opts.show_grid, camera.far)
        fb.resolve_samples()
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        return fb.color_texture

    def make_image_backend(self, sim):
        """Return a callable implementing sensors.camera.ImageBackend."""
        def backend(view: np.ndarray, proj: np.ndarray, width: int, height: int) -> Optional[np.ndarray]:
            scene = self.builder.build(sim, ViewOptions(show_grid=False), None, overlays=False)
            return self.camera_image(scene, view, proj, width, height)
        return backend

    def camera_image(self, scene: RenderScene, view, proj, width: int, height: int) -> np.ndarray:
        key = (width, height)
        fb = self._camera_fbos.get(key)
        if fb is None:
            if len(self._camera_fbos) > 8:
                for f in self._camera_fbos.values():
                    f.delete()
                self._camera_fbos.clear()
            fb = self._camera_fbos[key] = Framebuffer(width, height, samples=0)
        prev = GL.glGetIntegerv(GL.GL_FRAMEBUFFER_BINDING)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, fb.fbo)
        eye = np.linalg.inv(view)[:3, 3]
        self.draw_scene(scene, view, proj, eye, width, height, show_grid=False, far=100.0,
                        background=(0.62, 0.70, 0.80))
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
        raw = GL.glReadPixels(0, 0, width, height, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, int(prev))
        img = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)
        return np.ascontiguousarray(img[::-1])   # The OpenGL origin is at the bottom left. Image rows start at the top.

    def release(self) -> None:
        if self._viewport_fbo:
            self._viewport_fbo.delete()
        for f in self._camera_fbos.values():
            f.delete()
        self._camera_fbos.clear()
        self.meshes.release()
