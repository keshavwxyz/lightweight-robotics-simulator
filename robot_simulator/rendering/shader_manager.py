"""GLSL 330 core shader programs and a cache of uniform locations."""
from __future__ import annotations

import numpy as np

from .gl import GL

MESH_VS = """
#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_nrm;
layout(location = 2) in vec4 a_col;
layout(location = 3) in vec2 a_uv;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;
uniform mat3 u_normal;
out vec3 v_nrm;
out vec3 v_wpos;
out vec4 v_col;
out vec2 v_uv;
void main() {
    vec4 wp = u_model * vec4(a_pos, 1.0);
    v_wpos = wp.xyz;
    v_nrm = u_normal * a_nrm;
    v_col = a_col;
    v_uv = a_uv;
    gl_Position = u_proj * u_view * wp;
}
"""

MESH_FS = """
#version 330 core
in vec3 v_nrm;
in vec3 v_wpos;
in vec4 v_col;
in vec2 v_uv;
uniform vec4 u_color;
uniform int u_use_vcolor;
uniform int u_use_texture;
uniform sampler2D u_tex;
uniform int u_unlit;
uniform vec3 u_light_dir;
uniform vec3 u_eye;
uniform float u_highlight;
out vec4 frag;
void main() {
    vec4 base = (u_use_vcolor == 1) ? v_col : u_color;
    if (u_use_texture == 1) base = texture(u_tex, v_uv);
    if (u_unlit == 1) { frag = base; return; }
    vec3 n = normalize(v_nrm);
    if (!gl_FrontFacing) n = -n;
    vec3 l = normalize(-u_light_dir);
    float diff = max(dot(n, l), 0.0);
    vec3 v = normalize(u_eye - v_wpos);
    vec3 h = normalize(l + v);
    float spec = pow(max(dot(n, h), 0.0), 32.0) * 0.20;
    float hemi = 0.5 + 0.5 * n.z;
    float fill = max(dot(n, v), 0.0);   // fill light from the direction of the camera
    vec3 c = base.rgb * (0.20 + 0.15 * hemi) + base.rgb * (diff * 0.55 + fill * 0.30) + vec3(spec);
    c = mix(c, vec3(1.0, 0.75, 0.15), u_highlight * 0.45);
    frag = vec4(c, base.a);
}
"""

GROUND_VS = """
#version 330 core
layout(location = 0) in vec3 a_pos;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec3 v_wpos;
void main() {
    vec4 wp = u_model * vec4(a_pos, 1.0);
    v_wpos = wp.xyz;
    gl_Position = u_proj * u_view * wp;
}
"""

GROUND_FS = """
#version 330 core
in vec3 v_wpos;
uniform vec4 u_color;
uniform int u_grid;
uniform vec3 u_eye;
uniform float u_fade;
out vec4 frag;
float grid_line(vec2 p, float spacing) {
    vec2 q = p / spacing;
    vec2 g = abs(fract(q - 0.5) - 0.5) / fwidth(q);
    return 1.0 - min(min(g.x, g.y), 1.0);
}
void main() {
    vec2 p = v_wpos.xy;
    float checker = mod(floor(p.x) + floor(p.y), 2.0);
    vec3 c = u_color.rgb * (0.92 + 0.10 * checker);
    if (u_grid == 1) {
        float minor = grid_line(p, 0.1) * 0.25;
        float major = grid_line(p, 1.0) * 0.6;
        c = mix(c, vec3(0.75), max(minor, major) * 0.35);
        float ax = 1.0 - min(abs(p.y) / fwidth(p.y), 1.0);
        float ay = 1.0 - min(abs(p.x) / fwidth(p.x), 1.0);
        c = mix(c, vec3(0.85, 0.25, 0.25), ax * 0.8);
        c = mix(c, vec3(0.25, 0.80, 0.30), ay * 0.8);
    }
    float d = length(v_wpos - u_eye);
    float fog = clamp((d - u_fade * 0.5) / (u_fade * 0.5), 0.0, 1.0);
    frag = vec4(c, 1.0 - fog);
}
"""

LINE_VS = """
#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec4 a_col;
uniform mat4 u_view;
uniform mat4 u_proj;
uniform float u_point_size;
out vec4 v_col;
void main() {
    v_col = a_col;
    gl_Position = u_proj * u_view * vec4(a_pos, 1.0);
    gl_PointSize = u_point_size;
}
"""

LINE_FS = """
#version 330 core
in vec4 v_col;
out vec4 frag;
void main() { frag = v_col; }
"""


class ShaderProgram:
    def __init__(self, vs: str, fs: str, name: str = "program"):
        self.name = name
        self.id = GL.glCreateProgram()
        shaders = [self._compile(vs, GL.GL_VERTEX_SHADER), self._compile(fs, GL.GL_FRAGMENT_SHADER)]
        for s in shaders:
            GL.glAttachShader(self.id, s)
        GL.glLinkProgram(self.id)
        if not GL.glGetProgramiv(self.id, GL.GL_LINK_STATUS):
            raise RuntimeError(f"Shader link error ({name}): {GL.glGetProgramInfoLog(self.id).decode()}")
        for s in shaders:
            GL.glDeleteShader(s)
        self._locs: dict[str, int] = {}

    def _compile(self, src: str, kind) -> int:
        s = GL.glCreateShader(kind)
        GL.glShaderSource(s, src)
        GL.glCompileShader(s)
        if not GL.glGetShaderiv(s, GL.GL_COMPILE_STATUS):
            raise RuntimeError(f"Shader compile error ({self.name}): {GL.glGetShaderInfoLog(s).decode()}")
        return s

    def use(self) -> None:
        GL.glUseProgram(self.id)

    def loc(self, name: str) -> int:
        l = self._locs.get(name)
        if l is None:
            l = GL.glGetUniformLocation(self.id, name)
            self._locs[name] = l
        return l

    def mat4(self, name: str, m: np.ndarray) -> None:
        GL.glUniformMatrix4fv(self.loc(name), 1, GL.GL_TRUE, np.ascontiguousarray(m, dtype=np.float32))

    def mat3(self, name: str, m: np.ndarray) -> None:
        GL.glUniformMatrix3fv(self.loc(name), 1, GL.GL_TRUE, np.ascontiguousarray(m, dtype=np.float32))

    def vec3(self, name: str, v) -> None:
        GL.glUniform3f(self.loc(name), float(v[0]), float(v[1]), float(v[2]))

    def vec4(self, name: str, v) -> None:
        GL.glUniform4f(self.loc(name), float(v[0]), float(v[1]), float(v[2]), float(v[3]))

    def int(self, name: str, v: int) -> None:
        GL.glUniform1i(self.loc(name), int(v))

    def float(self, name: str, v: float) -> None:
        GL.glUniform1f(self.loc(name), float(v))


class ShaderManager:
    def __init__(self):
        self.mesh = ShaderProgram(MESH_VS, MESH_FS, "mesh")
        self.ground = ShaderProgram(GROUND_VS, GROUND_FS, "ground")
        self.line = ShaderProgram(LINE_VS, LINE_FS, "line")
