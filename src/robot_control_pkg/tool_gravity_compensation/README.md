# tool_gravity_compensation

ROS 2 Python package for:
- multi-pose least-squares calibration of tool gravity parameters (mass and CoM)
- online force/torque gravity compensation

## Nodes
- `gravity_calibration_node`
- `gravity_compensation_node`

## Identification Model

For each pose, with gravity projected in sensor frame as `g_s`:

- `f = b_f + m * g_s`
- `tau = b_tau + (r_com x (m * g_s))`

Unknowns are solved by least squares from multiple poses:

- tool mass `m`
- CoM in sensor frame `r_com`
- force bias `b_f`
- torque bias `b_tau`

## Quick Start

1. Build package:

```bash
cd ~/rrc_ws
colcon build --packages-select tool_gravity_compensation
source install/setup.bash
```

2. Run calibration node:

```bash
ros2 launch tool_gravity_compensation gravity_calibration.launch.py
```

3. Start collection, move robot through diverse orientations, then stop:

```bash
ros2 service call /start_collection std_srvs/srv/Trigger {}
ros2 service call /stop_collection std_srvs/srv/Trigger {}
```

4. Solve and save calibration result:

```bash
ros2 service call /solve_and_save std_srvs/srv/Trigger {}
```

5. Run compensation node (uses saved JSON by default):

```bash
ros2 launch tool_gravity_compensation gravity_compensation.launch.py
```

## Config Files

- `config/gravity_calibration_params.yaml`
- `config/gravity_compensation_params.yaml`
