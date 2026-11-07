"""Robot loading: URDF file -> RobotModel and a URDF for PyBullet.

PyBullet cannot find `package://` URIs. It also reads only some mesh formats.
Thus, the loader writes a changed copy of the URDF into a cache directory:
  * Each mesh filename is an absolute path.
  * Collision meshes in formats that PyBullet cannot read become OBJ files.
  * Unsupported joint types (floating, planar) become "fixed". PyBullet and
    the internal model then have the same hierarchy.
  * Meshes that are not found are removed. The parser already logged them.
The loader does not change or copy the original URDF and its files.
"""
from __future__ import annotations

import hashlib
import logging
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

from .resource_resolver import resolve_uri
from .robot_model import RobotModel
from .urdf_parser import URDFDescription, URDFError, URDFParser

log = logging.getLogger(__name__)

PYBULLET_MESH_FORMATS = {".obj", ".stl", ".dae"}
CACHE_DIR = Path(os.environ.get("ROBOT_SIM_CACHE", Path(tempfile.gettempdir()) / "robot_simulator_cache"))


@dataclass
class LoadedRobotDescription:
    model: RobotModel
    pybullet_urdf: Path          # changed URDF for PyBullet
    warnings: list[str]


def _convert_mesh_to_obj(src: Path) -> Optional[Path]:
    try:
        import trimesh
    except ImportError:
        return None
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha1(f"{src}:{src.stat().st_mtime_ns}".encode()).hexdigest()[:16]
    out = CACHE_DIR / f"{src.stem}_{digest}.obj"
    if out.exists():
        return out
    try:
        mesh = trimesh.load(str(src), force="mesh")
        mesh.export(str(out))
        return out
    except Exception as exc:  # pragma: no cover - depends on file
        log.warning("Could not convert mesh %s to OBJ: %s", src, exc)
        return None


class RobotLoader:
    def __init__(self, package_paths: Iterable[str] = ()):
        self.package_paths = tuple(package_paths)
        self.parser = URDFParser(self.package_paths)

    def load(self, urdf_path: str | Path) -> LoadedRobotDescription:
        urdf_path = Path(urdf_path).expanduser().resolve()
        desc = self.parser.parse_file(urdf_path)
        model = RobotModel(desc)
        warnings = list(desc.warnings)
        resolved = self._write_resolved_urdf(urdf_path, desc, warnings)
        model.warnings = warnings
        return LoadedRobotDescription(model=model, pybullet_urdf=resolved, warnings=warnings)

    def _write_resolved_urdf(self, urdf_path: Path, desc: URDFDescription, warnings: list[str]) -> Path:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        base_dir = urdf_path.parent

        for link_el in root.findall("link"):
            if link_el.find("inertial") is None and link_el.get("name") != "world":
                # In URDF, a link without <inertial> is a frame without mass.
                # PyBullet gives such a link 1 kg and unit inertia. This changes the dynamics.
                inertial = ET.SubElement(link_el, "inertial")
                ET.SubElement(inertial, "mass", value="0.001")
                ET.SubElement(inertial, "inertia", ixx="1e-7", ixy="0", ixz="0", iyy="1e-7", iyz="0", izz="1e-7")
            for kind in ("visual", "collision"):
                for el in list(link_el.findall(kind)):
                    mesh = el.find("geometry/mesh")
                    if mesh is None:
                        continue
                    fn = mesh.get("filename", "")
                    path, _ = resolve_uri(fn, base_dir, self.package_paths) if fn else (None, "")
                    if path is None:
                        link_el.remove(el)  # the parser already logged this mesh
                        continue
                    if path.suffix.lower() not in PYBULLET_MESH_FORMATS:
                        if kind == "visual":
                            # PyBullet runs in DIRECT mode. Only the custom renderer draws visuals.
                            link_el.remove(el)
                            continue
                        conv = _convert_mesh_to_obj(path)
                        if conv is None:
                            warnings.append(f"link '{link_el.get('name')}': collision mesh '{fn}' "
                                            f"format '{path.suffix}' is not supported by the physics engine; "
                                            f"collision shape skipped")
                            link_el.remove(el)
                            continue
                        warnings.append(f"link '{link_el.get('name')}': collision mesh '{fn}' converted to OBJ "
                                        f"for the physics engine")
                        path = conv
                    mesh.set("filename", str(path))

        for j_el in root.findall("joint"):
            jt = j_el.get("type")
            if jt not in ("fixed", "revolute", "continuous", "prismatic"):
                j_el.set("type", "fixed")
            if desc.joints.get(j_el.get("name", "")) is not None and desc.joints[j_el.get("name", "")].mimic is None:
                m = j_el.find("mimic")
                if m is not None:
                    j_el.remove(m)

        # Remove elements that PyBullet does not use.
        for tag in ("transmission", "gazebo", "ros2_control"):
            for el in root.findall(tag):
                root.remove(el)

        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha1(f"{urdf_path}:{urdf_path.stat().st_mtime_ns}".encode()).hexdigest()[:16]
        out = CACHE_DIR / f"{urdf_path.stem}_{digest}.urdf"
        tree.write(out, encoding="utf-8", xml_declaration=True)
        return out


__all__ = ["RobotLoader", "LoadedRobotDescription", "URDFError"]
