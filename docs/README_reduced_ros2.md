# Reduced ROS 2 Pi Experiment

This branch is an experiment to reduce ROS 2 usage on the Raspberry Pi drive path.

Assume the Raspberry Pi is running Linux kernel `5.15` with SocketCAN support.

The goal is to keep ROS 2 for communication between the Jetson and Pi, while avoiding
extra ROS 2 hops inside the Pi motor-control loop. In balanced drive mode, the RL
balance controller sends lightweight Unix-domain socket commands to a separate
non-ROS motor process. That motor process computes wheel powers and sends CAN
commands directly.

Expected Pi command path:

```text
Jetson/manual ROS command -> Pi balance controller -> Unix socket -> motor IPC server -> direct CAN motor output
```

Logs and reports should stay lightweight and go to stdout or the launch log files
rather than motor-control ROS topics.

The motor IPC server uses `can0` with the `socketcan` interface by default.
