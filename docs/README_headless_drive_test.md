# B_Cubed Headless Drive Quickstart

This guide is the copy-paste startup path for a two-device B_Cubed drive test.

- Jetson `ubuntu`: ZED camera, SLAM/Nav2, map, planning web console.
- Raspberry Pi `ubuntu-1`: Sense HAT IMU, RL balance controller, ROS command listener, CAN motor output.
- ROS 2 uses CycloneDDS with unicast peers. The Pi may be on Tailscale (`100.80.7.37`) or the Jetson hotspot (`10.42.0.166`).

## Network Addresses

| Device | SSH alias | Tailscale | Hotspot |
| --- | --- | --- | --- |
| Jetson | `ubuntu` | `100.86.7.33` | `10.42.0.1` |
| Raspberry Pi | `ubuntu-1` | `100.80.7.37` | `10.42.0.166` |

The launch scripts write `~/cyclonedds.xml` at startup. They bind DDS only to an IP that exists on the device, which avoids CycloneDDS errors like `does not match an available interface`.

## 1. Jetson Startup

Use this for normal Tailscale/Wi-Fi mode:

```bash
ssh ubuntu
cd ~/Documents/B_Cubed/ros2_ws

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
PLANNING_CONSOLE_HOST=0.0.0.0 \
NETWORK_MODE=tailscale \
./launch_headless.sh --mapping
```

Open the web console from another computer:

```text
http://100.86.7.33:8080/
```

Use this when the Jetson should switch to hotspot mode:

```bash
ssh ubuntu
cd ~/Documents/B_Cubed/ros2_ws

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
PLANNING_CONSOLE_HOST=0.0.0.0 \
./launch_headless.sh --hotspot --mapping
```

Open the web console on hotspot:

```text
http://10.42.0.1:8080/
```

To run localization with a saved posegraph prefix:

```bash
ssh ubuntu
cd ~/Documents/B_Cubed/ros2_ws

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
PLANNING_CONSOLE_HOST=0.0.0.0 \
NETWORK_MODE=tailscale \
./launch_headless.sh --localization --map-file /home/jetson-nano-x1/Documents/B_Cubed/ros2_ws/maps/MAP_PREFIX
```

Stop the Jetson launch with `Ctrl-C`. The script sends a clean shutdown to the ZED wrapper and navigation launch processes. If hotspot mode was used, it attempts to restore Wi-Fi/Tailscale DDS configuration during cleanup.

## 2. Raspberry Pi Startup

The Pi launcher auto-detects whether the Pi currently has `100.80.7.37` or `10.42.0.166`, writes a matching CycloneDDS config, detaches Sense HAT kernel drivers, starts the IMU, starts the RL controller, and starts low-level motor control.

Run this on the Pi for the real drive stack:

```bash
ssh ubuntu-1
cd ~/B_Cubed/ros2_ws

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
START_SENSE_HAT=true \
SENSE_HAT_DETACH_KERNEL=true \
START_MOTOR_CONTROL=true \
HARDWARE_ENABLED=true \
SETUP_CAN=false \
STATUS_INTERVAL_SEC=5 \
COMMAND_MONITOR_SHOW_MISSING=summary \
./launch_pi_balanced_drive.sh --auto
```

Force Tailscale mode:

```bash
./launch_pi_balanced_drive.sh --tailscale
```

Force hotspot mode:

```bash
./launch_pi_balanced_drive.sh --hotspot
```

Stop the Pi launch with `Ctrl-C`. The script sends a clean shutdown to the ROS launch process.

## 3. CAN Setup, If Needed

If `can0` is not already up, initialize the CANable before running the Pi launch:

```bash
ssh ubuntu-1

if ! ip link show can0 >/dev/null 2>&1; then
  sudo slcand -o -c -s8 /dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_205530883541-if00 can0
fi
sudo ip link set can0 up
ip -details link show can0
```

## 4. Quick Health Checks

Check that the Jetson can see its ROS graph:

```bash
ssh ubuntu
source /opt/ros/humble/setup.bash
source ~/Documents/B_Cubed/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson-nano-x1/cyclonedds.xml

ros2 topic list | egrep '/cmd_vel_manual|/map|/scan|/zed/zed_node/pose|/zed/zed_node/point_cloud/cloud_registered'
```

Check that the Pi can see both its own topics and the Jetson topics:

```bash
ssh ubuntu-1
source /opt/ros/humble/setup.bash
source ~/B_Cubed/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml

ros2 topic list | egrep '/cmd_vel_manual|/cmd_vel_balanced|/jet_cmd|/kiwi_drive/motor_powers|/balance/status|/imu/data|/map|/scan'
```

Check that the RL controller is alive:

```bash
ssh ubuntu-1
source /opt/ros/humble/setup.bash
source ~/B_Cubed/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml

ros2 topic echo /balance/status
```

Expected while idle:

```text
backend=npz
command_source=idle
reason=stale_command
out_v=(0.000,0.000)
```

Expected while moving the web console joystick:

```text
safe=True
backend=npz
command_source=manual
balance_v=(...)
out_v=(...)
```

If it stays `stale_command` while moving the web console, the Pi is not receiving fresh `/cmd_vel_manual` samples.

## 5. Command Chain Watch

Run this on the Pi while moving the web console joystick:

```bash
ssh ubuntu-1
source /opt/ros/humble/setup.bash
source ~/B_Cubed/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml

ros2 topic echo /cmd_vel_manual
```

In separate Pi terminals:

```bash
ros2 topic echo /cmd_vel_balanced
ros2 topic echo /jet_cmd
ros2 topic echo /kiwi_drive/motor_powers
```

The expected flow is:

```text
web console -> /cmd_vel_manual -> RL balance controller -> /cmd_vel_balanced and /jet_cmd -> low-level motor control -> /kiwi_drive/motor_powers -> CAN
```

## 6. ZED Camera Check

The Jetson launch needs the ZED to appear as a USB 3 camera, not just the HID interface. If the launch reports `CAMERA STREAM FAILED TO START`, check:

```bash
ssh ubuntu
lsusb -t
lsusb | grep -i stereolabs
ls /dev/video*
```

Healthy camera enumeration should show a SuperSpeed USB path and video devices. If only `STEREOLABS ZED-M HID Interface` appears at 12M/USB2 and there is no `/dev/video*`, fix the physical USB connection before debugging ROS.

## 7. Useful Logs

Jetson:

```bash
tail -f ~/Documents/B_Cubed/ros2_ws/log/headless/zed_wrapper.log
tail -f ~/Documents/B_Cubed/ros2_ws/log/headless/zed_slam_nav.log
```

Pi:

```bash
find ~/B_Cubed/ros2_ws/log/pi_balanced_drive -maxdepth 3 -type f | sort | tail
```

## 8. Systemd Optional Install

Jetson:

```bash
ssh ubuntu
cd ~/Documents/B_Cubed/ros2_ws
sudo cp systemd/b_cubed_jetson.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now b_cubed_jetson.service
sudo journalctl -u b_cubed_jetson.service -f
```

Pi:

```bash
ssh ubuntu-1
cd ~/B_Cubed/ros2_ws
sudo cp systemd/b_cubed_pi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now b_cubed_pi.service
sudo journalctl -u b_cubed_pi.service -f
```
