# low_level_runner

ROS 2 low-level control package for the Kiwi drive base.

The active controller is:

```bash
ros2 launch low_level_runner kiwi_drive_controller.launch.py
```

For the Raspberry Pi motor-control host, use the Pi-facing launch file:

```bash
ros2 launch low_level_runner pi_low_level_motor_control.launch.py
```

Or use the workspace helper script:

```bash
./launch_pi_motor_control.sh
```

That launch starts the `low_level_motor_control` ROS node, which uses the same
Kiwi drive controller implementation and the Pi hardware defaults in
`config/pi_low_level_motor_control.yaml`.

It subscribes to:

- `cmd_vel` for gated Nav2 velocity commands
- `cmd_vel_manual` for web-console manual commands

Manual commands take priority while they are fresh. The node publishes the
calculated wheel powers on `kiwi_drive/motor_powers` and, when hardware is
enabled, sends those powers to the CAN motor controllers.

For a dry run:

```bash
HARDWARE_ENABLED=false ./launch_pi_motor_control.sh
```

The previous low-level runner implementation is preserved in
`low_level_runner/legacy_low_level_runner.py` for reference.
