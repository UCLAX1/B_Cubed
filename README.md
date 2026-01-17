# B_Cubed: Motors Testing

HOW TO TEST THE MOTORS:

1. plug the jhoinrch CAN bus into the SPARK MAX (green to green).\
a. DO NOT TOUCH THE CAN BUS SWITCHES. the 12 ohm switch should be down, and the boot switch should be up.
2. plug the motor into the big box. don't plug into the middle one on the big box, only the left and right ones.
3. plug your computer into the can bus using the usb-c hole.
4. turn on the box and set it to 12 volts and 1.8 amps
5. run the program

if these steps do not work, skip the CAN bus and plug your computer into the SPARK MAX using usb-c

use REV Hardware Client to see if the motors spin

UCLA X1 Robotics 2025-26 B^3 Project

1. Install ROS2 Humble
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. Install the ZED SDK:
https://www.stereolabs.com/developers/release/5.0#82af3640d775

3. Install the ROS2 Wrapper for the ZED SDK
https://www.stereolabs.com/docs/ros2

