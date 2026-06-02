okay so 


# ssh stuffs
#### pi:
on hotspot:
```
ssh ubuntu@10.42.0.166
```
on tailscale:
```
ssh ubuntu@ubuntu-1
```
#### jetson
on hotspot:
```
ssh jetson-nano-x1@10.42.0.1
```
on tailscale
```
ssh jetson-nano-x1@ubuntu
```

# Raspberry Pi

imu init (in home directory):
1.
```
./run_sense.sh
```

2. note: make sure that the egg script actually spits out imu data. if it doesn't, run the shell script again
```
python3 egg.py
```


if you need to have the pi rebuild on launch
```
BUILD_FIRST=true \
ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
START_SENSE_HAT=false \
START_BALANCE_CONTROLLER=false \
START_MOTOR_CONTROL=true \
HARDWARE_ENABLED=true \
SETUP_CAN=true \
MOTOR_POWER_SLEW_RATE_PER_SEC=2.0 \
MOTOR_POWER_DEADBAND=0.08 \
MOTOR_MIN_POWER=0.0 \
./launch_pi_balanced_drive.sh --hotspot
```


if  you dont:
```
BUILD_FIRST=false \
ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
START_SENSE_HAT=false \
START_BALANCE_CONTROLLER=false \
START_MOTOR_CONTROL=true \
HARDWARE_ENABLED=true \
SETUP_CAN=true \
MOTOR_POWER_SLEW_RATE_PER_SEC=2.0 \
MOTOR_POWER_DEADBAND=0.08 \
MOTOR_MIN_POWER=0.0 \
./launch_pi_balanced_drive.sh --hotspot
```


connecting pi to hotspot
```
sudo nmcli connection up x1-jetson
```

# Jetson

launch the script as it is rn 

1.
```
cd Documents/B_Cubed/ros2_ws
```
2.
```
sudo -v
```
3.
```
ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
PLANNING_CONSOLE_HOST=0.0.0.0 \
NETWORK_MODE=hotspot \
./launch_headless.sh --mapping
```


github why wont you work