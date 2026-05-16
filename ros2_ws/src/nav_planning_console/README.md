# nav_planning_console

Standalone ROS 2 web console for viewing an `OccupancyGrid`, requesting Nav2
plans, and sending/canceling `NavigateToPose` goals.

Run it after the workspace is built and sourced:

```bash
ros2 launch nav_planning_console nav_planning_console.launch.py
```

The ROS bridge node lives in `nav_planning_console/node.py`. Static browser
assets live in `web/` and are installed into the package share directory.

The manual control panel publishes `geometry_msgs/Twist` commands to
`cmd_vel_manual` by default. The low-level Kiwi drive controller can subscribe
to that topic and temporarily prioritize manual commands over Nav2 output.

When the live robot/camera pose falls inside an occupied map cell, the console
can adjust only the planner start pose to the nearest nearby free cell before
calling `ComputePathToPose`. Tune that behavior with:

- `start_pose_rescue_enabled`
- `start_pose_rescue_radius`
- `start_pose_rescue_clearance`
- `start_pose_occupied_threshold`
- `start_pose_unknown_is_occupied`
