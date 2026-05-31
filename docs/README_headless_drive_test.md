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

## 0. One-Time Hotspot Setup

The runtime scripts already know how to use either Tailscale or hotspot DDS. By default, the Jetson launch auto-detects the current address and prefers the hotspot if `10.42.0.1` is already active; it does not start the hotspot or switch the Jetson back to Tailscale on startup or shutdown. The one-time setup is making sure the Jetson has a NetworkManager hotspot profile named `Hotspot`, and the Pi joins that hotspot at the expected static IP.

On the Jetson, create or verify the hotspot profile:

```bash
ssh ubuntu

nmcli -t -f NAME connection show | grep -Fx Hotspot || \
sudo nmcli connection add type wifi ifname wlan0 con-name Hotspot autoconnect no ssid x1-jetson

sudo nmcli connection modify Hotspot \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  ipv4.method shared \
  ipv4.addresses 10.42.0.1/24 \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk 'CHANGE_ME_B_CUBED_PASSWORD'
```

If the Wi-Fi device is not `wlan0`, find it with:

```bash
nmcli device status
```

To make the SSID available for Pi setup, bring the hotspot up once. Run this from a local terminal or inside `tmux` if your current SSH session depends on Wi-Fi/Tailscale:

```bash
sudo nmcli connection up Hotspot
```

On the Pi, connect to the Jetson hotspot and pin the Pi to `10.42.0.166`:

```bash
ssh ubuntu-1

nmcli -t -f NAME connection show | grep -Fx B_Cubed-Hotspot || \
sudo nmcli dev wifi connect x1-jetson password 'CHANGE_ME_B_CUBED_PASSWORD' name B_Cubed-Hotspot
sudo nmcli connection modify B_Cubed-Hotspot \
  ipv4.method manual \
  ipv4.addresses 10.42.0.166/24 \
  ipv4.gateway 10.42.0.1 \
  ipv4.dns 10.42.0.1
sudo nmcli connection up B_Cubed-Hotspot
```

If the Pi is not using NetworkManager, set the same static IPv4 values in its active Wi-Fi config instead:

```text
address: 10.42.0.166/24
gateway: 10.42.0.1
dns: 10.42.0.1
ssid: x1-jetson
```

Basic network checks after the Jetson hotspot is up and the Pi has joined:

```bash
# Jetson
ip -4 addr show | grep 10.42.0.1
ping -c 3 10.42.0.166

# Pi
hostname -I
ping -c 3 10.42.0.1

# Laptop connected to x1-jetson Wi-Fi
ping -c 3 10.42.0.1
ping -c 3 10.42.0.166
```

If the Pi gets a different `10.42.0.x` address, either fix the Pi static IP or pass the same override on both devices:

```bash
PI_HOTSPOT_IP=10.42.0.X ./launch_headless.sh --hotspot --mapping
PI_HOTSPOT_IP=10.42.0.X ./launch_pi_balanced_drive.sh --hotspot
```

## 1. Jetson Startup

Use this for whichever Jetson network is already active. If the hotspot is up, this stays on hotspot; otherwise it uses the Tailscale address if present:

```bash
ssh ubuntu
cd ~/Documents/B_Cubed/ros2_ws

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
PLANNING_CONSOLE_HOST=0.0.0.0 \
./launch_headless.sh --mapping
```


Use this when the Jetson hotspot is already active:
```bash
ssh ubuntu
cd ~/Documents/B_Cubed/ros2_ws

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
PLANNING_CONSOLE_HOST=0.0.0.0 \
NETWORK_MODE=hotspot \
./launch_headless.sh --mapping
```

Open the web console from another computer:

```text
http://10.42.0.1:8080/
```

The Jetson launch does not bring up the hotspot. Start the `Hotspot` NetworkManager profile before running the launch.

Important: if you start the hotspot itself through a Tailscale SSH session, the SSH connection may drop as soon as the Jetson changes Wi-Fi modes. Start the launch after reconnecting through the hotspot, or run it inside `tmux`, with systemd, or from a local terminal.

```bash
ssh ubuntu
tmux new -s b-cubed-hotspot
```

Inside the `tmux` session:

```bash
cd ~/Documents/B_Cubed/ros2_ws
sudo -v

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
PLANNING_CONSOLE_HOST=0.0.0.0 \
./launch_headless.sh --hotspot --mapping
```

After the hotspot comes up, connect the laptop to the `x1-jetson` Wi-Fi network and use the hotspot address below. If you need another Jetson shell while in hotspot mode, connect with:

```bash
ssh jetson-nano-x1@10.42.0.1
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

Stop the Jetson launch with `Ctrl-C`. The script sends a clean shutdown to the ZED wrapper and navigation launch processes. It leaves the Jetson network alone.

## 2. Raspberry Pi Startup

The Pi launcher auto-detects whether the Pi currently has `100.80.7.37` or `10.42.0.166`, writes a matching CycloneDDS config, detaches Sense HAT kernel drivers, starts the IMU, starts the RL controller, and starts low-level motor control.

Run this on the Pi for the real drive stack:

```bash
ssh ubuntu@10.42.0.166
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

When `HARDWARE_ENABLED=true`, the Pi launch turns the MOSFET on with BCM GPIO `16` (physical pin `36`) before starting the ROS motor stack. On `Ctrl-C`, systemd stop, or launch exit, it turns the MOSFET off. Override with:

```bash
MOSFET_POWER_ENABLED=false ./launch_pi_balanced_drive.sh --hotspot
MOSFET_GPIO_PIN=16 ./launch_pi_balanced_drive.sh --hotspot
```

Force Tailscale mode:

```bash
./launch_pi_balanced_drive.sh --tailscale
```

Force hotspot mode:

```bash
./launch_pi_balanced_drive.sh --hotspot
```

Bypass the RL balance controller for direct motor testing:

```bash
ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
START_SENSE_HAT=false \
START_BALANCE_CONTROLLER=false \
START_MOTOR_CONTROL=true \
HARDWARE_ENABLED=true \
SETUP_CAN=false \
STATUS_INTERVAL_SEC=5 \
COMMAND_MONITOR_SHOW_MISSING=summary \
./launch_pi_balanced_drive.sh --hotspot
```

With `START_BALANCE_CONTROLLER=false`, the Pi does not start `bb8_balance_controller`. The low-level motor controller listens directly to `cmd_vel` and `cmd_vel_manual`, so the web console joystick goes straight to Kiwi wheel powers. Use this only for supported/benchtop motor testing; the robot is not balancing in this mode.

Use `ssh ubuntu-1` while the Pi is still on Tailscale. Use `ssh ubuntu@10.42.0.166` after it has joined the Jetson hotspot.

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

The examples below use the hotspot IPs. On Tailscale, use `ssh ubuntu` and `ssh ubuntu-1` instead.

Check that the Jetson can see its ROS graph:

```bash
ssh jetson-nano-x1@10.42.0.1
source /opt/ros/humble/setup.bash
source ~/Documents/B_Cubed/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson-nano-x1/cyclonedds.xml

ros2 topic list | egrep '/cmd_vel_manual|/map|/scan|/zed/zed_node/pose|/zed/zed_node/point_cloud/cloud_registered'
```

Check that the Pi can see both its own topics and the Jetson topics:

```bash
ssh ubuntu@10.42.0.166
source /opt/ros/humble/setup.bash
source ~/B_Cubed/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml

ros2 topic list | egrep '/cmd_vel_manual|/cmd_vel_balanced|/jet_cmd|/kiwi_drive/motor_powers|/balance/status|/imu/data|/map|/scan'
```

Check that the RL controller is alive:

```bash
ssh ubuntu@10.42.0.166
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
ssh ubuntu@10.42.0.166
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

## 8. Hotspot Troubleshooting

If the Jetson launch says `NETWORK_MODE=hotspot requested, but 10.42.0.1 is not present`, the Jetson hotspot is not active yet. Start the `Hotspot` NetworkManager profile first, then rerun the launch.

If the Pi launch says `NETWORK_MODE=hotspot requested, but 10.42.0.166 is not present`, the Pi did not join the hotspot with the expected static IP. Check `hostname -I` on the Pi and fix the Pi Wi-Fi profile or override `PI_HOTSPOT_IP` on both launch commands.

If `http://10.42.0.1:8080/` does not open from the laptop, verify the laptop is on the Jetson hotspot, then run:

```bash
ping -c 3 10.42.0.1
curl -I http://10.42.0.1:8080/
```

If ping works but ROS topics do not cross between Jetson and Pi, check the generated DDS configs:

```bash
# Jetson should show local_address=10.42.0.1 and peer 10.42.0.166
cat ~/cyclonedds.xml

# Pi should show local_address=10.42.0.166 and peer 10.42.0.1
cat ~/cyclonedds.xml
```

The expected hotspot command flow is unchanged:

```text
laptop web console -> Jetson 10.42.0.1:8080 -> /cmd_vel_manual -> Pi 10.42.0.166 -> RL balance controller -> low-level motor control -> CAN
```

In direct motor-test mode, the command flow is:

```text
laptop web console -> Jetson 10.42.0.1:8080 -> /cmd_vel_manual -> Pi 10.42.0.166 -> low-level motor control -> CAN
```

`kiwi_drive/status` should not alternate between `hardware_active=true` and `hardware_active=false` from one healthy motor node. If it flickers, first check for duplicate publishers or a restarting launch:

```bash
ros2 topic info -v /kiwi_drive/status
ros2 node list | egrep 'low_level_motor_control|kiwi_drive_controller'
systemctl is-active b_cubed_pi.service
```

If both systemd and a manual Pi launch are running, stop one before testing the motors:

```bash
sudo systemctl stop b_cubed_pi.service
```

## 9. Systemd Optional Install

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

The Jetson systemd unit defaults to network auto-detection. For a standalone hotspot robot, make sure the `Hotspot` NetworkManager profile is already active before the service starts. You can also force the launch to require the hotspot address with a systemd override:

```bash
sudo systemctl edit b_cubed_jetson.service
```

```ini
[Service]
Environment=NETWORK_MODE=hotspot
```

The Pi systemd unit defaults to DDS auto-detection, but it is conservative about hardware. For a real drive test through systemd, confirm the service has:

```ini
[Service]
Environment=HARDWARE_ENABLED=true
Environment=START_MOTOR_CONTROL=true
Environment=SETUP_CAN=false
```
