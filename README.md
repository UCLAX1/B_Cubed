# B_Cubed: Motors Testing

## how to use the motors
- STEP 1: look at motors_test.py and run it for an example
- STEP 2: make a can bus like this:
`bus = CanBus(channel='COM5', interface='slcan', bitrate=1000000)`\
(you'll have to figure out the channel, interface, and bitrate depending on your device)\
(for example, if you're on windows, the channel can be found in Device Manager)\
and call `bus.start()`
- STEP 3: make your motor: `motor0 = Motor(bus, 0)`, `motor1 = Motor(bus, 1)`, etc.\
(the ids are saved to a list)
- STEP 4: in your while loop, repeatedly call `motor.send_heartbeat()` to make it not break and `motor.set_power(your_power)` to set the power
 


## What the heck are the files in this branch
- ServoEx is the class for the servos. It should have everything.
- HardwareInterface is the class for the Motors. It uses can.

robot.py and test.py are separate programs, and are a bit antequated.

I fixed a lot of stupid stuff in the code to fit it to B-Cubed and to make it more usable


HardwareInterface.py and motor_test.py are ripped from Akhilesh's code from the BruinBear github.\


## HOW TO TEST THE MOTORS:

1. plug the jhoinrch CAN bus into the SPARK MAX (green to green).\
a. DO NOT TOUCH THE CAN BUS SWITCHES. the 12 ohm switch should be down, and the boot switch should be up.
2. plug the motor into the big box. don't plug into the middle one on the big box, only the left and right ones.
3. plug your computer into the can bus using the usb-c hole.
4. turn on the box and set it to 12 volts and 1.8 amps
5. run the program

if these steps do not work, skip the CAN bus and plug your computer into the SPARK MAX using usb-c

use REV Hardware Client to see if the motors spin



## original readme description VVV

UCLA X1 Robotics 2025-26 B^3 Project

1. Install ROS2 Humble
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. Install the ZED SDK:
https://www.stereolabs.com/developers/release/5.0#82af3640d775

3. Install the ROS2 Wrapper for the ZED SDK
https://www.stereolabs.com/docs/ros2

