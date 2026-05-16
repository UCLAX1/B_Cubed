# nav_planning_console

Standalone ROS 2 web console for viewing an `OccupancyGrid`, requesting Nav2
plans, and sending/canceling `NavigateToPose` goals.

Run it after the workspace is built and sourced:

```bash
ros2 launch nav_planning_console nav_planning_console.launch.py
```

The ROS bridge node lives in `nav_planning_console/node.py`. Static browser
assets live in `web/` and are installed into the package share directory.
