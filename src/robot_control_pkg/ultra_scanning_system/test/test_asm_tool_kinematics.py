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
