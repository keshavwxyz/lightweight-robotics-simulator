"""Find the files of URDF mesh references on disk.

Supported reference types:
  * absolute paths                       /home/me/robot/meshes/base.stl
  * file:// URIs                         file:///home/me/robot/meshes/base.stl
  * paths relative to the URDF file      meshes/base.stl
  * package://<pkg>/<relative path>      resolved locally, see `find_package`
"""
from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path
from typing import Iterable, Optional


def _package_xml_name(pkg_dir: Path) -> Optional[str]:
    xml = pkg_dir / "package.xml"
    if not xml.is_file():
        return None
    try:
        import xml.etree.ElementTree as ET
        node = ET.parse(xml).getroot().find("name")
        return node.text.strip() if node is not None and node.text else None
    except Exception:
        return None


def _search_roots(urdf_dir: Optional[Path], extra: Iterable[str]) -> list[Path]:
    roots: list[Path] = [Path(p) for p in extra if p]
    for var in ("ROS_PACKAGE_PATH",):
        roots += [Path(p) for p in os.environ.get(var, "").split(os.pathsep) if p]
    # ament index: <prefix>/share/<pkg>
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            roots.append(Path(prefix) / "share")
    # Also search the directories above the URDF. A robot in a source tree is then
    # found without a sourced ROS environment.
    if urdf_dir is not None:
        d = urdf_dir.resolve()
        for _ in range(6):
            roots.append(d)
            if d.parent == d:
                break
            d = d.parent
    return roots


@lru_cache(maxsize=256)
def _find_package_cached(name: str, urdf_dir: Optional[str], extra: tuple[str, ...]) -> Optional[str]:
    for root in _search_roots(Path(urdf_dir) if urdf_dir else None, extra):
        if not root.is_dir():
            continue
        # The root directory can be the package.
        if root.name == name or _package_xml_name(root) == name:
            return str(root)
        cand = root / name
        if cand.is_dir():
            return str(cand)
        # Search one level of subdirectories for a package.xml with this name.
        try:
            for child in root.iterdir():
                if child.is_dir() and (child / "package.xml").is_file() and _package_xml_name(child) == name:
                    return str(child)
        except OSError:
            continue
    return None


def find_package(name: str, urdf_dir: Optional[Path] = None, extra_paths: Iterable[str] = ()) -> Optional[Path]:
    found = _find_package_cached(name, str(urdf_dir) if urdf_dir else None, tuple(extra_paths))
    return Path(found) if found else None


def resolve_uri(uri: str, urdf_dir: Optional[Path], extra_paths: Iterable[str] = ()) -> tuple[Optional[Path], str]:
    """Return (absolute path or None, reason if the file is not found)."""
    uri = uri.strip()
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        pkg, _, rel = rest.partition("/")
        pkg_dir = find_package(pkg, urdf_dir, extra_paths)
        if pkg_dir is None:
            return None, (f"package '{pkg}' could not be found (searched ROS_PACKAGE_PATH, "
                          f"AMENT_PREFIX_PATH and the directories above the URDF)")
        path = pkg_dir / rel
    elif uri.startswith("file://"):
        path = Path(uri[len("file://"):])
    elif uri.startswith(("http://", "https://", "model://")):
        return None, f"URI scheme of '{uri}' is not supported"
    else:
        path = Path(uri)
        if not path.is_absolute():
            path = (urdf_dir or Path.cwd()) / path
    path = path.resolve()
    if not path.is_file():
        return None, f"file '{path}' does not exist"
    return path, ""
