# lowlevelrunner

ROS 2 low-level control package for the Kiwi drive base.

The active controller is:

```bash
ros2 launch lowlevelrunner kiwi_drive_controller.launch.py
```

It subscribes to:

- `cmd_vel` for gated Nav2 velocity commands
- `cmd_vel_manual` for web-console manual commands

Manual commands take priority while they are fresh. The node publishes the
calculated wheel powers on `kiwi_drive/motor_powers` and, when hardware is
enabled, sends those powers to the CAN motor controllers.

For a dry run:

```bash
ros2 launch lowlevelrunner kiwi_drive_controller.launch.py hardware_enabled:=false
```

The previous low-level runner implementation is preserved in
`lowlevelrunner/legacy_low_level_runner.py` for reference.
