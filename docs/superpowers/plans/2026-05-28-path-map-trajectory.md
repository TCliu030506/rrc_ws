# Path Map Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `robot_trajectory_planner` node that reads `data/path_map.txt`, blends from the current `asm_ee_site` pose to the first path point, then publishes the path as desired pose/twist/accel.

**Architecture:** Keep trajectory math in a pure Python logic module with unit tests. The ROS node only handles parameters, file lookup, subscriptions, publishers, and timer callbacks. Do not modify `ultra_scanning_system.launch.py`.

**Tech Stack:** ROS 2 Python (`rclpy`), `geometry_msgs`, package share lookup through `ament_index_python`, Python `math` only for rotation-vector/quaternion math.

---

### Task 1: Pure Path Logic

**Files:**
- Create: `src/project_pkgs/robot_control_pkg/robot_trajectory_planner/robot_trajectory_planner/path_map_trajectory_logic.py`
- Create: `src/project_pkgs/robot_control_pkg/robot_trajectory_planner/test/test_path_map_trajectory_logic.py`

- [ ] **Step 1: Write failing tests**

Cover parsing `x y z rx ry rz`, invalid rows, rotation-vector quaternion conversion, and rate-limited interpolation from current pose toward the first point.

- [ ] **Step 2: Run tests to verify RED**

Run: `PYTHONPATH=src/project_pkgs/robot_control_pkg/robot_trajectory_planner python3 -m pytest src/project_pkgs/robot_control_pkg/robot_trajectory_planner/test/test_path_map_trajectory_logic.py -q`

Expected: import failure for `path_map_trajectory_logic`.

- [ ] **Step 3: Implement minimal pure logic**

Implement path parsing, quaternion helpers, `PathMapTrajectory` state, and desired pose/twist/accel calculation without ROS dependencies.

- [ ] **Step 4: Run tests to verify GREEN**

Run the same pytest command and expect all tests to pass.

### Task 2: ROS Node, Data Install, Launch

**Files:**
- Create: `src/project_pkgs/robot_control_pkg/robot_trajectory_planner/robot_trajectory_planner/path_map_trajectory_node.py`
- Create: `src/project_pkgs/robot_control_pkg/robot_trajectory_planner/launch/path_map_trajectory.launch.py`
- Create: `src/project_pkgs/robot_control_pkg/robot_trajectory_planner/data/path_map.txt`
- Modify: `src/project_pkgs/robot_control_pkg/robot_trajectory_planner/setup.py`
- Modify: `src/project_pkgs/robot_control_pkg/robot_trajectory_planner/package.xml`

- [ ] **Step 1: Add node**

Subscribe to `/asm_ee_site/pose`, publish `/scan/desired_pose`, `/scan/desired_twist`, `/scan/desired_accel`, load package-share `data/path_map.txt` by default, and expose parameters for topics, speeds, publish rate, path file, and looping.

- [ ] **Step 2: Register/install**

Install launch files and data files through `setup.py`, register `path_map_trajectory_node`, and add `ament_index_python` runtime dependency.

- [ ] **Step 3: Verify syntax and package build**

Run `python3 -m py_compile` for the new modules, then `colcon build --packages-select robot_trajectory_planner --symlink-install`.

### Task 3: Final Verification

**Files:**
- Use changed files only.

- [ ] **Step 1: Run focused tests**

Run the new pytest file.

- [ ] **Step 2: Inspect worktree**

Run `git status --short` and report only files relevant to this task.
