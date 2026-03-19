[![Build badge](https://github.com)

# force_sensor
Provide support to pub force sensor msg

## Guide
run launch file: launch/forcesensor.launch
set pub_rate (determine the pub rate of topics: "forcesensor" & "probeforce_real")
set the topic the node: gravity_compensation_node subscribe

## Troubleshooting

This section will cover some previously raised issues.

### Unable to open port when rosrun serialport.cpp.
maybe the node do not have the permission to write serialport
run "sudo chmod 666 /dev/ttyUSB0" first
or "sudo usermod -aG dialout rus(username)"
check: "ls -l /dev/ttyUSB*"

- ros2 launch force_sensor forcesensor.launch.py 
