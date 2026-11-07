"""GPU meshes, primitive shapes and a mesh cache.

Each primitive shape is made one time as a unit shape: a cube with edge 1, a
cylinder with radius 1 and height 1 along Z, and a sphere with radius 1. The
model matrix scales the unit shape. Thus, all boxes, cylinders and spheres
share one buffer for each shape. trimesh loads each mesh file one time. The
cache then keeps it.
"""
from __future__ import annotations

import ctypes
import logging
import math
from typing import Optional

import numpy as np

from .gl import GL

log = logging.getLogger(__name__)

STRIDE = 12 * 4  # pos(3) normal(3) color(4) uv(2) float32


class GpuMesh:
    def __init__(self, vertices: np.ndarray, normals: np.ndarray, indices: np.ndarray,
                 colors: Optional[np.ndarray] = None, name: str = "",
                 uvs: Optional[np.ndarray] = None, image: Optional[np.ndarray] = None):
        n = len(vertices)
        if colors is None:
            colors = np.ones((n, 4), dtype=np.float32)
        if uvs is None:
            uvs = np.zeros((n, 2), dtype=np.float32)
        data = np.hstack([vertices.astype(np.float32), normals.astype(np.float32),
                          colors.astype(np.float32), uvs.astype(np.float32)]).astype(np.float32)
        self.name = name
        self.has_vertex_colors = colors is not None and not np.allclose(colors, 1.0)
        self.texture = self._upload_texture(image) if image is not None else None
        self.count = int(indices.size)
        self.bounds = (vertices.min(axis=0), vertices.max(axis=0)) if n else (np.zeros(3), np.zeros(3))
        self.vao = GL.glGenVertexArrays(1)
        self.vbo, self.ebo = GL.glGenBuffers(2)
        GL.glBindVertexArray(self.vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STATIC_DRAW)
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        idx = indices.astype(np.uint32).ravel()
        GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, idx.nbytes, idx, GL.GL_STATIC_DRAW)
        for loc, size, off in ((0, 3, 0), (1, 3, 12), (2, 4, 24), (3, 2, 40)):
            GL.glEnableVertexAttribArray(loc)
            GL.glVertexAttribPointer(loc, size, GL.GL_FLOAT, GL.GL_FALSE, STRIDE, ctypes.c_void_p(off))
        GL.glBindVertexArray(0)

    @staticmethod
    def _upload_texture(image: np.ndarray) -> int:
        """Send an (H, W, 4) uint8 RGBA image (row 0 = top) to the GPU, with mipmaps."""
        img = np.ascontiguousarray(image[::-1])          # OpenGL: row 0 = bottom (v = 0)
        tex = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA8, img.shape[1], img.shape[0], 0, GL.GL_RGBA,
                        GL.GL_UNSIGNED_BYTE, img)
        GL.glGenerateMipmap(GL.GL_TEXTURE_2D)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR_MIPMAP_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_REPEAT)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_REPEAT)
        return int(tex)

    def bind_texture(self) -> None:
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.texture)

    def draw(self) -> None:
        GL.glBindVertexArray(self.vao)
        GL.glDrawElements(GL.GL_TRIANGLES, self.count, GL.GL_UNSIGNED_INT, None)

    def delete(self) -> None:
        if self.texture is not None:
            GL.glDeleteTextures([self.texture])
        GL.glDeleteVertexArrays(1, [self.vao])
        GL.glDeleteBuffers(2, [self.vbo, self.ebo])


def unit_cube() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    faces = [((1, 0, 0), (0, 1, 0), (0, 0, 1)), ((-1, 0, 0), (0, 0, 1), (0, 1, 0)),
             ((0, 1, 0), (0, 0, 1), (1, 0, 0)), ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
             ((0, 0, 1), (1, 0, 0), (0, 1, 0)), ((0, 0, -1), (0, 1, 0), (1, 0, 0))]
    v, n, i = [], [], []
    for normal, u, w in faces:
        normal, u, w = np.array(normal, float), np.array(u, float), np.array(w, float)
        c = normal * 0.5
        base = len(v)
        for su, sw in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)):
            v.append(c + su * u + sw * w)
            n.append(normal)
        i += [base, base + 1, base + 2, base, base + 2, base + 3]
    return np.array(v), np.array(n), np.array(i)


def unit_cylinder(segments: int = 32) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    v, n, i = [], [], []
    for k in range(segments + 1):
        a = 2 * math.pi * k / segments
        c, s = math.cos(a), math.sin(a)
        v += [(c, s, -0.5), (c, s, 0.5)]
        n += [(c, s, 0), (c, s, 0)]
    for k in range(segments):
        a0 = 2 * k
        i += [a0, a0 + 2, a0 + 1, a0 + 1, a0 + 2, a0 + 3]
    for z, nz in ((0.5, 1.0), (-0.5, -1.0)):
        center = len(v)
        v.append((0, 0, z))
        n.append((0, 0, nz))
        for k in range(segments + 1):
            a = 2 * math.pi * k / segments
            v.append((math.cos(a), math.sin(a), z))
            n.append((0, 0, nz))
        for k in range(segments):
            if nz > 0:
                i += [center, center + 1 + k, center + 2 + k]
            else:
                i += [center, center + 2 + k, center + 1 + k]
    return np.array(v, float), np.array(n, float), np.array(i)


def unit_sphere(rings: int = 16, sectors: int = 24) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    v = []
    for r in range(rings + 1):
        phi = math.pi * r / rings
        for s in range(sectors + 1):
            th = 2 * math.pi * s / sectors
            v.append((math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th), math.cos(phi)))
    i = []
    for r in range(rings):
        for s in range(sectors):
            a = r * (sectors + 1) + s
            b = a + sectors + 1
            i += [a, b, a + 1, a + 1, b, b + 1]
    va = np.array(v, float)
    return va, va.copy(), np.array(i)


def unit_quad() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    v = np.array([(-0.5, -0.5, 0), (0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, 0.5, 0)], float)
    return v, np.tile((0, 0, 1.0), (4, 1)), np.array([0, 1, 2, 0, 2, 3])


def _texture_of(mesh) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Return (uv for each vertex, RGBA image) if the mesh has an image texture (OBJ+MTL, DAE, glTF)."""
    vis = getattr(mesh, "visual", None)
    if vis is None or getattr(vis, "kind", None) != "texture" or getattr(vis, "uv", None) is None:
        return None, None
    mat = getattr(vis, "material", None)
    img = getattr(mat, "image", None) or getattr(mat, "baseColorTexture", None)
    if img is None or len(vis.uv) != len(mesh.vertices):
        return None, None
    return np.asarray(vis.uv, dtype=np.float32), np.asarray(img.convert("RGBA"), dtype=np.uint8)


def load_mesh_file(path: str):
    """Load a mesh with trimesh and return flat-shaded geometry.

    The result is (vertices, normals, indices, vertex colours or None, uvs or
    None, RGBA texture or None).
    """
    import trimesh

    loaded = trimesh.load(path, process=False)
    if isinstance(loaded, trimesh.Scene) and len(loaded.geometry) == 1:
        node = loaded.graph.nodes_geometry[0]
        T, gname = loaded.graph[node]
        single = loaded.geometry[gname].copy()
        single.apply_transform(T)
        loaded = single
    if isinstance(loaded, trimesh.Scene):
        parts = []
        for node in loaded.graph.nodes_geometry:
            T, gname = loaded.graph[node]
            g = loaded.geometry[gname]
            if not isinstance(g, trimesh.Trimesh):
                continue
            g = g.copy()
            g.apply_transform(T)
            parts.append(g)
        if not parts:
            raise ValueError("file contains no triangle meshes")
        mesh = trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
        # concatenate can remove the colours of textured parts; keep the remaining colours
    else:
        mesh = loaded
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError("file does not contain a triangle mesh")

    uv, image = _texture_of(mesh)
    colors = None
    try:
        vis = mesh.visual
        if vis is not None and vis.kind is not None:
            cv = vis.to_color() if vis.kind == "texture" else vis
            fc = np.asarray(cv.face_colors, dtype=np.float32) / 255.0
            # trimesh gives grey (102, 102, 102) to meshes that have no colour data
            if fc.size and not np.allclose(fc, [102 / 255, 102 / 255, 102 / 255, 1.0], atol=0.02):
                colors = fc
    except Exception:
        colors = None

    faces = np.asarray(mesh.faces)
    verts = np.asarray(mesh.vertices)[faces].reshape(-1, 3)     # separate vertices for each face give flat shading
    fn = np.asarray(mesh.face_normals)
    normals = np.repeat(fn, 3, axis=0)
    vcolors = np.repeat(colors, 3, axis=0) if colors is not None and image is None else None
    uvs = uv[faces].reshape(-1, 2) if uv is not None else None
    indices = np.arange(len(verts))
    return verts, normals, indices, vcolors, uvs, image


class MeshLibrary:
    """Cache of GPU meshes. The keys are 'box', 'cylinder', 'sphere', 'quad' or an absolute file path."""

    def __init__(self):
        self._meshes: dict[str, Optional[GpuMesh]] = {}
        self._failed: set[str] = set()
        for key, fn in (("box", unit_cube), ("cylinder", unit_cylinder), ("sphere", unit_sphere),
                        ("quad", unit_quad)):
            v, n, i = fn()
            self._meshes[key] = GpuMesh(v, n, i, name=key)

    def get(self, key: str) -> Optional[GpuMesh]:
        if key in self._meshes:
            return self._meshes[key]
        if key in self._failed:
            return None
        try:
            v, n, i, c, uv, img = load_mesh_file(key)
            mesh = GpuMesh(v, n, i, c, name=key, uvs=uv, image=img)
            self._meshes[key] = mesh
            log.info("Loaded mesh %s (%d triangles%s)", key, len(i) // 3,
                     f", texture {img.shape[1]}x{img.shape[0]}" if img is not None else "")
            return mesh
        except Exception as exc:
            log.error("Could not load mesh '%s' for rendering: %s", key, exc)
            self._failed.add(key)
            return None

    def release(self) -> None:
        for m in self._meshes.values():
            if m is not None:
                m.delete()
        self._meshes.clear()
