"""URDF parser.

The parser reads a URDF document into data structures (`URDFDescription`).
It adds each unsupported element to `warnings`. It does not ignore elements
without a warning. It does no physics or rendering work.
"""
from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional

from .joint import Joint, JointDynamics, JointLimit, Mimic, SUPPORTED_JOINT_TYPES
from .link import Collision, Geometry, Inertial, Link, Material, Origin, Visual
from .resource_resolver import resolve_uri

log = logging.getLogger(__name__)

KNOWN_IGNORED_TOP_LEVEL = {
    "transmission": "transmissions are not simulated; assign motors to joints instead",
    "gazebo": "Gazebo extensions are not supported",
    "ros2_control": "ros2_control tags are not supported; use the simulator motor configuration",
}


class URDFError(Exception):
    """The URDF does not give a valid robot."""


@dataclass
class URDFDescription:
    name: str
    links: dict[str, Link]
    joints: dict[str, Joint]
    materials: dict[str, Material]
    source_path: Optional[Path]
    warnings: list[str] = field(default_factory=list)
    unresolved_meshes: list[str] = field(default_factory=list)


def _floats(text: Optional[str], n: int, default: tuple, what: str, warnings: list[str]) -> tuple:
    if text is None:
        return default
    try:
        vals = tuple(float(v) for v in text.split())
    except ValueError:
        warnings.append(f"{what}: could not parse '{text}', using default {default}")
        return default
    if len(vals) != n:
        warnings.append(f"{what}: expected {n} values but got '{text}', using default {default}")
        return default
    return vals


def _float_attr(el: ET.Element, attr: str, default: float, what: str, warnings: list[str]) -> float:
    v = el.get(attr)
    if v is None:
        return default
    try:
        return float(v)
    except ValueError:
        warnings.append(f"{what}: attribute {attr}='{v}' is not a number, using {default}")
        return default


def _origin(el: Optional[ET.Element], what: str, warnings: list[str]) -> Origin:
    if el is None:
        return Origin()
    return Origin(xyz=_floats(el.get("xyz"), 3, (0.0, 0.0, 0.0), f"{what} origin xyz", warnings),
                  rpy=_floats(el.get("rpy"), 3, (0.0, 0.0, 0.0), f"{what} origin rpy", warnings))


def _warn_unknown_children(el: ET.Element, allowed: Iterable[str], what: str, warnings: list[str]) -> None:
    allowed = set(allowed)
    for child in el:
        if not isinstance(child.tag, str):
            continue  # comments
        if child.tag not in allowed:
            warnings.append(f"{what}: unsupported element <{child.tag}> ignored")


class URDFParser:
    def __init__(self, package_paths: Iterable[str] = ()):
        self.package_paths = tuple(package_paths)

    def parse_file(self, path: str | Path) -> URDFDescription:
        path = Path(path).expanduser().resolve()
        if not path.is_file():
            raise URDFError(f"URDF file not found: {path}")
        if path.suffix.lower() not in (".urdf", ".xml"):
            # Only URDF is supported. Convert xacro, SDF or MJCF files first.
            raise URDFError(f"'{path.name}' is not a URDF file (.urdf). Only URDF is supported; "
                            f"convert xacro with `xacro file.xacro > file.urdf`.")
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError as exc:
            raise URDFError(f"Cannot read {path}: {exc}") from exc
        return self.parse_string(text, path.parent, source_path=path)

    def parse_string(self, xml_text: str, base_dir: Optional[Path] = None,
                     source_path: Optional[Path] = None) -> URDFDescription:
        try:
            root = ET.fromstring(xml_text)
        except ET.ParseError as exc:
            raise URDFError(f"Malformed URDF XML: {exc}") from exc
        if root.tag != "robot":
            raise URDFError(f"Root element must be <robot>, found <{root.tag}> (only URDF is supported)")
        warnings: list[str] = []
        name = root.get("name") or "robot"

        materials: dict[str, Material] = {}
        for m in root.findall("material"):
            mat = self._material(m, "robot", warnings, {})
            if mat.name:
                materials[mat.name] = mat

        links: dict[str, Link] = {}
        unresolved: list[str] = []
        for l_el in root.findall("link"):
            link = self._link(l_el, materials, base_dir, warnings, unresolved)
            if link.name in links:
                raise URDFError(f"Duplicate link name '{link.name}'")
            links[link.name] = link

        joints: dict[str, Joint] = {}
        for j_el in root.findall("joint"):
            joint = self._joint(j_el, warnings)
            if joint.name in joints:
                raise URDFError(f"Duplicate joint name '{joint.name}'")
            joints[joint.name] = joint

        for child in root:
            if not isinstance(child.tag, str) or child.tag in ("link", "joint", "material"):
                continue
            reason = KNOWN_IGNORED_TOP_LEVEL.get(child.tag, "element is not supported")
            warnings.append(f"<{child.tag}> {child.get('name', '')}: ignored ({reason})".replace(" :", ":"))

        self._validate(links, joints, warnings)
        desc = URDFDescription(name=name, links=links, joints=joints, materials=materials,
                               source_path=source_path, warnings=warnings, unresolved_meshes=unresolved)
        for w in warnings:
            log.warning("[URDF %s] %s", name, w)
        return desc

    def _material(self, el: ET.Element, owner: str, warnings: list[str],
                  materials: dict[str, Material]) -> Material:
        name = el.get("name", "")
        color = el.find("color")
        texture = el.find("texture")
        if color is None and texture is None and name in materials:
            return materials[name]  # reference to a global material
        rgba = None
        if color is not None:
            rgba = _floats(color.get("rgba"), 4, (0.7, 0.7, 0.7, 1.0), f"{owner} material '{name}'", warnings)
        tex = None
        if texture is not None:
            tex = texture.get("filename")
            warnings.append(f"{owner}: material '{name}' texture '{tex}' is not supported; "
                            f"using the material colour instead")
        if rgba is None and name in materials:
            rgba = materials[name].rgba
        return Material(name=name, rgba=rgba, texture=tex)

    def _geometry(self, el: Optional[ET.Element], what: str, base_dir: Optional[Path],
                  warnings: list[str], unresolved: list[str]) -> Optional[Geometry]:
        if el is None:
            warnings.append(f"{what}: missing <geometry>, element skipped")
            return None
        shapes = [c for c in el if isinstance(c.tag, str)]
        if not shapes:
            warnings.append(f"{what}: empty <geometry>, element skipped")
            return None
        g = shapes[0]
        if len(shapes) > 1:
            warnings.append(f"{what}: <geometry> has {len(shapes)} shapes; only <{g.tag}> is used")
        if g.tag == "box":
            size = _floats(g.get("size"), 3, (0.0, 0.0, 0.0), f"{what} box size", warnings)
            return Geometry(type="box", size=size)
        if g.tag == "cylinder":
            return Geometry(type="cylinder", radius=_float_attr(g, "radius", 0.0, what, warnings),
                            length=_float_attr(g, "length", 0.0, what, warnings))
        if g.tag == "sphere":
            return Geometry(type="sphere", radius=_float_attr(g, "radius", 0.0, what, warnings))
        if g.tag == "mesh":
            fn = g.get("filename", "")
            scale = _floats(g.get("scale"), 3, (1.0, 1.0, 1.0), f"{what} mesh scale", warnings)
            path, reason = resolve_uri(fn, base_dir, self.package_paths) if fn else (None, "no filename")
            if path is None:
                msg = f"{what}: mesh '{fn}' could not be resolved: {reason}"
                warnings.append(msg)
                unresolved.append(fn)
            return Geometry(type="mesh", filename=fn, resolved_path=str(path) if path else None, scale=scale)
        warnings.append(f"{what}: geometry type <{g.tag}> is not supported (box, cylinder, sphere, mesh only); "
                        f"element skipped")
        return None

    def _link(self, el: ET.Element, materials: dict[str, Material], base_dir: Optional[Path],
              warnings: list[str], unresolved: list[str]) -> Link:
        name = el.get("name")
        if not name:
            raise URDFError("<link> without a name")
        what = f"link '{name}'"
        _warn_unknown_children(el, ("inertial", "visual", "collision"), what, warnings)
        link = Link(name=name)

        inert = el.find("inertial")
        if inert is not None:
            mass_el = inert.find("mass")
            mass = _float_attr(mass_el, "value", 0.0, what, warnings) if mass_el is not None else 0.0
            i_el = inert.find("inertia")
            inertia = (0.0,) * 6
            if i_el is not None:
                inertia = tuple(_float_attr(i_el, k, 0.0, what, warnings)
                                for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
            if mass < 0:
                warnings.append(f"{what}: negative mass {mass} clamped to 0")
                mass = 0.0
            link.inertial = Inertial(mass=mass, origin=_origin(inert.find("origin"), what, warnings),
                                     inertia=inertia)  # type: ignore[arg-type]

        for i, v in enumerate(el.findall("visual")):
            vwhat = f"{what} visual {i}"
            geom = self._geometry(v.find("geometry"), vwhat, base_dir, warnings, unresolved)
            if geom is None:
                continue
            mat_el = v.find("material")
            mat = self._material(mat_el, what, warnings, materials) if mat_el is not None else None
            link.visuals.append(Visual(geometry=geom, origin=_origin(v.find("origin"), vwhat, warnings),
                                       material=mat, name=v.get("name", "")))
        for i, c in enumerate(el.findall("collision")):
            cwhat = f"{what} collision {i}"
            geom = self._geometry(c.find("geometry"), cwhat, base_dir, warnings, unresolved)
            if geom is None:
                continue
            link.collisions.append(Collision(geometry=geom, origin=_origin(c.find("origin"), cwhat, warnings),
                                             name=c.get("name", "")))
        return link

    def _joint(self, el: ET.Element, warnings: list[str]) -> Joint:
        name = el.get("name")
        jtype = el.get("type")
        if not name or not jtype:
            raise URDFError("<joint> requires both 'name' and 'type'")
        what = f"joint '{name}'"
        parent_el, child_el = el.find("parent"), el.find("child")
        if parent_el is None or child_el is None or not parent_el.get("link") or not child_el.get("link"):
            raise URDFError(f"{what}: requires <parent link=..> and <child link=..>")
        _warn_unknown_children(el, ("parent", "child", "origin", "axis", "limit", "dynamics", "mimic",
                                    "safety_controller", "calibration"), what, warnings)
        for ignored in ("safety_controller", "calibration"):
            if el.find(ignored) is not None:
                warnings.append(f"{what}: <{ignored}> is not supported and is ignored")

        effective_type = jtype
        if jtype not in SUPPORTED_JOINT_TYPES:
            warnings.append(f"{what}: joint type '{jtype}' is not supported "
                            f"(fixed, revolute, continuous, prismatic only); it is treated as FIXED")
            effective_type = "fixed"

        axis_el = el.find("axis")
        axis = _floats(axis_el.get("xyz") if axis_el is not None else None, 3, (1.0, 0.0, 0.0),
                       f"{what} axis", warnings)
        norm = sum(a * a for a in axis) ** 0.5
        if effective_type != "fixed":
            if norm < 1e-9:
                warnings.append(f"{what}: zero-length axis, using (1, 0, 0)")
                axis = (1.0, 0.0, 0.0)
            else:
                axis = tuple(a / norm for a in axis)

        limit = None
        lim_el = el.find("limit")
        if lim_el is not None:
            limit = JointLimit(lower=_float_attr(lim_el, "lower", 0.0, what, warnings),
                               upper=_float_attr(lim_el, "upper", 0.0, what, warnings),
                               effort=_float_attr(lim_el, "effort", 0.0, what, warnings),
                               velocity=_float_attr(lim_el, "velocity", 0.0, what, warnings))
        elif effective_type in ("revolute", "prismatic"):
            warnings.append(f"{what}: {effective_type} joint without <limit>; limits default to 0 "
                            f"which makes the joint immobile in many tools")
            limit = JointLimit()

        dyn = JointDynamics()
        dyn_el = el.find("dynamics")
        if dyn_el is not None:
            dyn = JointDynamics(damping=_float_attr(dyn_el, "damping", 0.0, what, warnings),
                                friction=_float_attr(dyn_el, "friction", 0.0, what, warnings))

        mimic = None
        m_el = el.find("mimic")
        if m_el is not None:
            if effective_type == "fixed":
                warnings.append(f"{what}: <mimic> on a fixed joint ignored")
            elif not m_el.get("joint"):
                warnings.append(f"{what}: <mimic> without joint attribute ignored")
            else:
                mimic = Mimic(joint=m_el.get("joint", ""),
                              multiplier=_float_attr(m_el, "multiplier", 1.0, what, warnings),
                              offset=_float_attr(m_el, "offset", 0.0, what, warnings))

        return Joint(name=name, type=effective_type, parent=parent_el.get("link", ""),
                     child=child_el.get("link", ""), origin=_origin(el.find("origin"), what, warnings),
                     axis=axis, limit=limit, dynamics=dyn, mimic=mimic,  # type: ignore[arg-type]
                     original_type=jtype)

    @staticmethod
    def _validate(links: dict[str, Link], joints: dict[str, Joint], warnings: list[str]) -> None:
        if not links:
            raise URDFError("URDF contains no links")
        children: dict[str, str] = {}
        for j in joints.values():
            if j.parent not in links:
                raise URDFError(f"joint '{j.name}': parent link '{j.parent}' does not exist")
            if j.child not in links:
                raise URDFError(f"joint '{j.name}': child link '{j.child}' does not exist")
            if j.child in children:
                raise URDFError(f"link '{j.child}' has two parent joints ('{children[j.child]}' and '{j.name}')")
            children[j.child] = j.name
        roots = [n for n in links if n not in children]
        if len(roots) != 1:
            raise URDFError(f"URDF must form a single tree; found root links {roots}")
        # loop check: go from each link to the root
        parent_of = {j.child: j.parent for j in joints.values()}
        for start in links:
            seen, cur = set(), start
            while cur in parent_of:
                if cur in seen:
                    raise URDFError(f"kinematic loop detected at link '{cur}'")
                seen.add(cur)
                cur = parent_of[cur]
        for j in joints.values():
            if j.mimic and j.mimic.joint not in joints:
                warnings.append(f"joint '{j.name}': mimics unknown joint '{j.mimic.joint}', mimic ignored")
                j.mimic = None
