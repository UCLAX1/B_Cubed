# Reduced ROS 2 Pi Experiment

This branch is an experiment to reduce ROS 2 usage on the Raspberry Pi drive path.

The goal is to keep ROS 2 for communication between the Jetson and Pi, while avoiding
extra ROS 2 hops inside the Pi motor-control loop. In balanced drive mode, the RL
balance controller computes motor powers and sends CAN commands directly in the same
process. The low-level runner remains available as shared motor/CAN code and as an
optional standalone direct motor-test path.

Expected Pi command path:

```text
Jetson/manual ROS command -> Pi balance controller -> direct CAN motor output
```

Logs and reports should stay lightweight and go to stdout or the launch log files
rather than motor-control ROS topics.
