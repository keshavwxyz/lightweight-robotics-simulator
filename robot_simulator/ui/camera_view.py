"""Windows that show the last image of each camera sensor."""
from __future__ import annotations

import imgui

from ..rendering.gl import GL


class CameraViews:
    def __init__(self, ui):
        self.ui = ui
        self._textures: dict[tuple[str, str], tuple[int, int, int, int]] = {}  # key -> (tex, w, h, update_count)

    def draw(self) -> None:
        ui = self.ui
        for key in list(ui.camera_windows):
            rname, sname = key
            r = ui.sim.robots.get(rname)
            s = r.sensors.get(sname) if r else None
            if s is None:
                ui.camera_windows.discard(key)
                continue
            tex = self._update_texture(key, s)
            imgui.set_next_window_size(360, 300, imgui.FIRST_USE_EVER)
            expanded, opened = imgui.begin(f"Camera: {rname}/{sname}", True)
            if expanded and tex:
                avail = imgui.get_content_region_available()
                img = s.latest
                aspect = img.width / img.height
                w = avail.x
                h = w / aspect
                if h > avail.y:
                    h = avail.y
                    w = h * aspect
                imgui.image(tex, w, h)
            elif expanded:
                imgui.text("No image yet - start the simulation.")
            imgui.end()
            if not opened:
                ui.camera_windows.discard(key)

    def _update_texture(self, key, sensor) -> int:
        img = sensor.latest
        entry = self._textures.get(key)
        if img is None:
            return entry[0] if entry else 0
        if entry is not None and entry[3] == sensor.update_count:
            return entry[0]
        if entry is None or entry[1] != img.width or entry[2] != img.height:
            if entry is not None:
                GL.glDeleteTextures([entry[0]])
            tex = GL.glGenTextures(1)
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
            GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGB8, img.width, img.height, 0, GL.GL_RGB,
                            GL.GL_UNSIGNED_BYTE, img.data)
        else:
            tex = entry[0]
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
            GL.glTexSubImage2D(GL.GL_TEXTURE_2D, 0, 0, 0, img.width, img.height, GL.GL_RGB,
                               GL.GL_UNSIGNED_BYTE, img.data)
        self._textures[key] = (tex, img.width, img.height, sensor.update_count)
        return tex
