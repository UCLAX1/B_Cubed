# Headless Drive Test

This is the boot-safe launch path for the split B_Cubed robot.

## Jetson

Install the unit from the repository:

```bash
cd /home/jetson-nano-x1/Documents/B_Cubed/ros2_ws
sudo cp systemd/b_cubed_jetson.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now b_cubed_jetson.service
```

Mapping is the default mode. To force localization, create the flag file with
the saved posegraph prefix:

```bash
echo /home/jetson-nano-x1/Documents/B_Cubed/ros2_ws/maps/lab_a > .b_cubed_localization
sudo systemctl restart b_cubed_jetson.service
```

The launcher can also be run manually:

```bash
./launch_headless.sh --localization --map-file /absolute/path/to/posegraph_prefix
```

## Raspberry Pi

Copy `systemd/b_cubed_pi.service` to `/etc/systemd/system/` on the Pi and enable
it there. The service brings up `can0`, starts the Sense HAT IMU, starts the RL
balance controller, and starts low-level motor control with
`HARDWARE_ENABLED=false` until the quickstart checks pass.

## Final Check

Run this before enabling drivetrain power:

```bash
./final_drive_test_quickstart.sh
```

The script checks both hosts, the two systemd services, the shared ROS graph,
and `/balance/status` for an active learned-policy backend.
