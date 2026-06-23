import math

from ultra_scanning_system.asm_tool_kinematics import (
    default_dynamic_transforms,
    default_static_transforms,
)


def test_default_static_transforms_include_tool_chain_endpoints():
    transforms = default_static_transforms("tool0_controller")
    pairs = {(item.parent, item.child) for item in transforms}

    assert ("tool0_controller", "asm_base") in pairs
    assert ("asm_tool_link2", "asm_ee_site") in pairs


def test_dynamic_joint_transforms_use_xml_axes():
    joint1 = 0.2
    joint2 = -0.4
    transforms = default_dynamic_transforms(joint1, joint2)
    by_child = {item.child: item for item in transforms}

    assert by_child["asm_tool_link1"].xyz == (0.0, 0.0, 0.074)
    assert by_child["asm_tool_link2"].xyz == (0.0, 0.0, 0.0495)

    link1_quat = by_child["asm_tool_link1"].quat_xyzw
    link2_quat = by_child["asm_tool_link2"].quat_xyzw
    assert math.isclose(link1_quat[1], -math.sin(joint1 / 2.0))
    assert math.isclose(link1_quat[3], math.cos(joint1 / 2.0))
    assert math.isclose(link2_quat[0], math.sin(joint2 / 2.0))
    assert math.isclose(link2_quat[3], math.cos(joint2 / 2.0))


def test_v2_static_transforms_include_linear_stage_and_tool_tip():
    transforms = default_static_transforms("tool0_controller", asm_version=2)
    by_child = {item.child: item for item in transforms}

    assert by_child["asm_base"].parent == "tool0_controller"
    assert by_child["camera_link"].xyz == (0.1, -0.0011879, 0.005)
    assert by_child["asm_linear_fixed_link"].parent == "asm_force_sensor_link"
    assert by_child["asm_linear_fixed_link"].xyz == (0.0, 0.0, 0.0095)
    assert by_child["asm_force_sensor_link"].quat_xyzw == (0.0, 0.0, -1.0, 0.0)
    assert by_child["asm_linear_fixed_link"].quat_xyzw == (0.0, 0.0, 1.0, 0.0)
    assert by_child["asm_waterproof_link"].parent == "asm_tool_link3"
    assert by_child["asm_ee_site"].parent == "asm_waterproof_link"


def test_v2_dynamic_transforms_use_slide_and_three_tool_joints():
    joint1_rad = 0.2
    joint2_rad = -0.3
    joint3_m = 0.01
    transforms = default_dynamic_transforms(
        joint1_rad,
        joint2_rad,
        joint3_m,
        asm_version=2,
    )
    by_child = {item.child: item for item in transforms}

    assert by_child["asm_tool_link1"].parent == "asm_linear_fixed_link"
    assert by_child["asm_tool_link1"].xyz == (0.0, 0.0, 0.11171)
    assert by_child["asm_tool_link2"].xyz == (0.0, 0.0, 0.068)
    assert by_child["asm_tool_link3"].xyz == (0.00034889, 0.0, 0.0495)

    link2_quat = by_child["asm_tool_link2"].quat_xyzw
    link3_quat = by_child["asm_tool_link3"].quat_xyzw
    assert math.isclose(link2_quat[1], -math.sin(joint1_rad / 2.0))
    assert math.isclose(link2_quat[3], math.cos(joint1_rad / 2.0))
    assert math.isclose(link3_quat[0], math.sin((-0.039683 + joint2_rad) / 2.0))
    assert math.isclose(link3_quat[3], math.cos((-0.039683 + joint2_rad) / 2.0))


def test_v3_static_transforms_fix_linear_frame_to_tool_link1():
    transforms = default_static_transforms("tool0_controller", asm_version=3)
    by_child = {item.child: item for item in transforms}

    link1 = by_child["asm_tool_link1"]
    assert link1.parent == "asm_linear_fixed_link"
    assert link1.xyz == (0.0, 0.0, 0.0)
    assert link1.quat_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_v3_dynamic_transforms_ignore_joint3_and_keep_v2_rotary_joints():
    transforms_a = default_dynamic_transforms(0.2, -0.3, 0.0, asm_version=3)
    transforms_b = default_dynamic_transforms(0.2, -0.3, 0.5, asm_version=3)
    by_child = {item.child: item for item in transforms_a}

    assert transforms_a == transforms_b
    assert "asm_tool_link1" not in by_child
    assert by_child["asm_tool_link2"].parent == "asm_tool_link1"
    assert by_child["asm_tool_link2"].xyz == (0.0, 0.0, 0.068)
    assert by_child["asm_tool_link3"].parent == "asm_tool_link2"
    assert by_child["asm_tool_link3"].xyz == (0.00034889, 0.0, 0.0495)
