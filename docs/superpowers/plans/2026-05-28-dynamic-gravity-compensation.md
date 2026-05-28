# Dynamic Gravity Compensation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add hardware dynamic gravity calibration and compensation launch paths for the ASM tool.

**Architecture:** Put the reusable least-squares model and wrench prediction in a pure Python helper module. Add hardware ROS nodes for service-based calibration and online compensation, then wire them through config files, launch files, and `setup.py` entry points.

**Tech Stack:** ROS 2 `rclpy`, `geometry_msgs/WrenchStamped`, `std_srvs/Trigger`, `tf2_ros`, NumPy, pytest.

---

### Task 1: Dynamic Model Tests And Helper

**Files:**
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/test/test_dynamic_gravity_model.py`
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/tool_gravity_compensation/dynamic_gravity_model.py`

- [ ] **Step 1: Write failing tests**

Add tests for recovering two link masses/COM vectors from synthetic TF samples and predicting the same wrench.

- [ ] **Step 2: Run failing tests**

Run: `PYTHONPATH=src/project_pkgs/robot_control_pkg/tool_gravity_compensation python3 -m pytest src/project_pkgs/robot_control_pkg/tool_gravity_compensation/test/test_dynamic_gravity_model.py`

Expected: fails because `dynamic_gravity_model` does not exist.

- [ ] **Step 3: Implement helper**

Implement `solve_dynamic_gravity_params(samples, link_count)` and `predict_dynamic_gravity_wrench(g_sensor, transforms, params)`.

- [ ] **Step 4: Verify helper tests pass**

Run the same pytest command.

### Task 2: Hardware Dynamic Nodes

**Files:**
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/tool_gravity_compensation/dynamic_gravity_calibration_node.py`
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/tool_gravity_compensation/dynamic_gravity_compensation_node.py`
- Modify: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/setup.py`

- [ ] **Step 1: Add calibration node**

Implement service-based sample collection with `~/clear_samples`, `~/collect_current_pose`, and `~/solve_and_save`.

- [ ] **Step 2: Add compensation node**

Load dynamic calibration JSON and publish compensated/model wrench from current TF.

- [ ] **Step 3: Register entry points**

Add both nodes to `console_scripts`.

### Task 3: Launch And Config

**Files:**
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/config/gravity_calibration_dynamic_params.yaml`
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/config/gravity_compensation_dynamic_params.yaml`
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/launch/gravity_calibration_dynamic.launch.py`
- Create: `src/project_pkgs/robot_control_pkg/tool_gravity_compensation/launch/gravity_compensation_dynamic.launch.py`

- [ ] **Step 1: Add hardware params**

Use `/external_force_torque_wrench`, `/external_force_torque_wrench_compensated`, `base`, `asm_force_sensor_link`, and the ASM link frames from the approved design.

- [ ] **Step 2: Add launch files**

Follow the existing package launch style and load each params file from package share.

### Task 4: Verification

- [ ] **Step 1: Run unit tests**

Run: `PYTHONPATH=src/project_pkgs/robot_control_pkg/tool_gravity_compensation python3 -m pytest src/project_pkgs/robot_control_pkg/tool_gravity_compensation/test/test_dynamic_gravity_model.py`

- [ ] **Step 2: Check Python imports**

Run: `PYTHONPATH=src/project_pkgs/robot_control_pkg/tool_gravity_compensation python3 -m py_compile src/project_pkgs/robot_control_pkg/tool_gravity_compensation/tool_gravity_compensation/dynamic_gravity_model.py src/project_pkgs/robot_control_pkg/tool_gravity_compensation/tool_gravity_compensation/dynamic_gravity_calibration_node.py src/project_pkgs/robot_control_pkg/tool_gravity_compensation/tool_gravity_compensation/dynamic_gravity_compensation_node.py`
