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



-----------------------------------------------------


# B_Cubed: Servo Testing

## How to use the servos
- STEP 1: look at `sim/servo_test.py` for an example
- STEP 2: make a servo: `servo = ServoEx(servo_pin=16, encoder_pin_a=27, encoder_pin_b=22, absolute_encoder_pin=17)`
(use BCM pin numbering — see `sim/pin_mapping.txt` for physical-to-BCM conversions)
- STEP 3: in your while loop, call `servo.update()` to keep encoders in sync
- STEP 4: use `servo.get_position()` (rotations) or `servo.get_position_radians()` to read position
- STEP 5: on exit, call `servo.save_encoder_position()` to save calibration to `servo_init_pos.json`

## How the encoder system works
Two encoders are used together:
- **Relative (quadrature)**: 2048 counts/rev, high resolution, but drifts
- **Absolute**: reads 0.0–1.0 from PWM pulse width, never drifts, but has a deadzone near 0.0/1.0

Every 0.25s, `update()` re-centers the relative encoder against the absolute to correct drift. Centering is skipped when either encoder is within 0.08 rotations of the deadzone boundary.

## GPIO pin mapping

### Arm Servo (70kg, servo_pin BCM=13, physical=33)
| Signal | Physical | BCM |
|--------|----------|-----|
| Power     | 17 | 3.3V |
| Ground    | 25 | GND |
| Encoder A | 37 | 26 |
| Encoder B | 31 | 6 |
| Absolute  | 29 | 5 |

### Head Servo (5.5kg, servo_pin BCM=18, physical=12)
| Signal | Physical | BCM |
|--------|----------|-----|
| Power     | 1  | 3.3V |
| Ground    | 39 | GND |
| Encoder A | 13 | 27 |
| Encoder B | 15 | 22 |
| Absolute  | 11 | 17 |

### Other
| Signal | Physical | BCM |
|--------|----------|-----|
| Servo 1 data | 32 | 12 |
| MOSFET       | 36 | 16 |

## HOW TO TEST THE SERVOS:

1. connect the servo signal, encoder A/B, and absolute wires to the correct GPIO pins
2. power the Pi and SSH in (e.g. via Tailscale)
3. run `python3 sim/servo_test.py` on the Pi
4. rotate the servo by hand — `normal` and `absolute` positions should update in the terminal

if the relative encoder reads 0 while the servo spins but absolute works, try swapping A and B pins — the channels may be reversed on the connector (see `sim/pin_mapping.txt`)

## What the heck are the files
- `main/ServoEx.py` — the main servo class. Use this.
- `sim/servo_test.py` — basic test: prints position in a loop
- `sim/servo_control_filters.py` — PD damping filter to reduce jitter near a setpoint
- `sim/pin_mapping.txt` — physical-to-BCM GPIO pin reference

## original readme description VVV

UCLA X1 Robotics 2025-26 B^3 Project

1. Install ROS2 Humble
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. Install the ZED SDK:
https://www.stereolabs.com/developers/release/5.0#82af3640d775

3. Install the ROS2 Wrapper for the ZED SDK
https://www.stereolabs.com/docs/ros2

