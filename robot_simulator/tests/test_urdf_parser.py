"""Robot description manager: URDF parsing, mesh resolution, hierarchy."""
import math
from pathlib import Path

import numpy as np
import pytest

from robot_simulator.robot.robot_loader import RobotLoader
from robot_simulator.robot.robot_model import RobotModel
from robot_simulator.robot.urdf_parser import URDFError, URDFParser

from .conftest import ARM_URDF, DIFF_URDF, MANIP_URDF, MESH_URDF


def parse(xml: str):
    return URDFParser().parse_string(xml)


def test_parse_differential_drive():
    d = URDFParser().parse_file(DIFF_URDF)
    assert d.name == "diff_bot"
    assert len(d.links) == 8 and len(d.joints) == 7
    lw = d.joints["left_wheel_joint"]
    assert lw.type == "continuous" and lw.parent == "base_link" and lw.child == "left_wheel"
    assert lw.axis == (0.0, 1.0, 0.0)
    assert lw.origin.xyz == (0.0, 0.18, 0.0)
    assert lw.limit.effort == 5.0 and lw.limit.velocity == 20.0
    base = d.links["base_link"]
    assert base.inertial.mass == pytest.approx(4.0)
    assert base.visuals[0].geometry.type == "box"
    assert base.visuals[0].material.rgba == (0.20, 0.40, 0.75, 1.0)   # global material reference
    assert d.warnings == []


def test_six_dof_arm_hierarchy_and_limits():
    m = RobotModel(URDFParser().parse_file(ARM_URDF))
    assert m.root_link == "base_link"
    assert [j.name for j in m.movable_joints] == [f"joint{i}" for i in range(1, 7)]
    j2 = m.joints["joint2"]
    assert (j2.limit.lower, j2.limit.upper, j2.limit.effort, j2.limit.velocity) == (-2.0, 2.0, 80.0, 2.0)
    chain = [j.name for j in m.chain_to_root("tool0")]
    assert chain == ["tool0_joint", "joint6", "joint5", "joint4", "joint3", "joint2", "joint1"]
    assert "tool0" in m.format_tree()


def test_mesh_robot_multiple_visuals_and_package_uri():
    d = URDFParser().parse_file(MESH_URDF)
    visuals = d.links["base_link"].visuals
    assert len(visuals) == 3
    assert all(v.geometry.type == "mesh" and v.geometry.resolved_path for v in visuals)
    assert Path(visuals[0].geometry.resolved_path).name == "base.stl"        # package://
    assert Path(visuals[1].geometry.resolved_path).name == "knob.stl"        # relative path
    assert visuals[2].geometry.scale == (0.5, 0.5, 0.5)
    arm = d.links["arm_link"].visuals
    assert [v.geometry.type for v in arm] == ["mesh", "sphere"]
    assert d.unresolved_meshes == []


def test_absolute_and_file_uri(tmp_path):
    mesh = Path(MESH_URDF).parents[1] / "meshes" / "knob.stl"
    xml = f"""<robot name="r"><link name="a">
      <visual><geometry><mesh filename="{mesh}"/></geometry></visual>
      <visual><geometry><mesh filename="file://{mesh}"/></geometry></visual></link></robot>"""
    d = parse(xml)
    assert [Path(v.geometry.resolved_path) for v in d.links["a"].visuals] == [mesh, mesh]


def test_unresolved_mesh_is_reported_not_silently_ignored():
    d = parse('<robot name="r"><link name="a"><visual><geometry>'
              '<mesh filename="package://no_such_pkg/m.stl"/></geometry></visual></link></robot>')
    assert d.unresolved_meshes == ["package://no_such_pkg/m.stl"]
    assert any("no_such_pkg" in w for w in d.warnings)


def test_unsupported_elements_produce_warnings():
    xml = """<robot name="r">
      <link name="a"><visual><geometry><capsule radius="1" length="2"/></geometry></visual>
        <visual><geometry><box size="1 1 1"/></geometry>
          <material name="m"><texture filename="t.png"/></material></visual></link>
      <link name="b"/>
      <joint name="j" type="floating"><parent link="a"/><child link="b"/></joint>
      <transmission name="t"/>
      <gazebo reference="a"/>
    </robot>"""
    d = parse(xml)
    text = "\n".join(d.warnings)
    assert "capsule" in text
    assert "texture" in text
    assert "floating" in text and d.joints["j"].type == "fixed" and d.joints["j"].original_type == "floating"
    assert "<transmission>" in text and "<gazebo>" in text
    assert len(d.links["a"].visuals) == 1


def test_axis_is_normalised_and_mimic_parsed():
    d = URDFParser().parse_file(MANIP_URDF)
    rf = d.joints["right_finger_joint"]
    assert rf.mimic.joint == "left_finger_joint" and rf.mimic.multiplier == 1.0
    xml = """<robot name="r"><link name="a"/><link name="b"/>
      <joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 2"/>
      <limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"""
    assert parse(xml).joints["j"].axis == (0.0, 0.0, 1.0)


@pytest.mark.parametrize("xml,msg", [
    ("<robot name='r'><link name='a'/><link name='b'/></robot>", "single tree"),
    ("<robot name='r'><link name='a'/><joint name='j' type='fixed'><parent link='a'/><child link='x'/></joint></robot>",
     "does not exist"),
    ("<robot name='r'><link name='a'/><link name='a'/></robot>", "Duplicate link"),
    ("<sdf/>", "Root element"),
    ("<robot><link", "Malformed"),
])
def test_invalid_urdfs_raise_clear_errors(xml, msg):
    with pytest.raises(URDFError, match=msg):
        parse(xml)


def test_only_urdf_files_accepted(tmp_path):
    p = tmp_path / "robot.sdf"
    p.write_text("<sdf/>")
    with pytest.raises(URDFError, match="Only URDF"):
        URDFParser().parse_file(p)


def test_forward_kinematics_matches_joint_definition():
    m = RobotModel(URDFParser().parse_file(ARM_URDF))
    T = m.forward_kinematics({"joint1": math.pi / 2})
    # joint1 rotates about Z: shoulder origin unchanged, upper arm frame rotated by 90 deg
    np.testing.assert_allclose(T["shoulder_link"][:3, 3], [0, 0, 0.1], atol=1e-12)
    np.testing.assert_allclose(T["shoulder_link"][:3, 0], [0, 1, 0], atol=1e-12)


def test_loader_writes_resolved_urdf_for_physics():
    loaded = RobotLoader().load(MESH_URDF)
    text = loaded.pybullet_urdf.read_text()
    assert "package://" not in text
    assert str(Path(MESH_URDF).parents[1] / "meshes" / "base.stl") in text
