# listens to ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np

import B_Cubed.main.HardwareInterface as interface

# ROS2 Config
TOPIC_NAME: str = 'jet_cmd'
QUEUE_SIZE: int = 10

# Motor Config
CHANNEL: str = "COM5"
INTERFACE: str = "slcan"
BITRATE: int = 1000000

# units per second
SPEED_CHANGE_RATE: float = 0.5

TOP_LEFT_VEC: np.ndarray = np.array([-0.5, -np.sqrt(3) / 2])
TOP_RIGHT_VEC: np.ndarray = np.array([-0.5, np.sqrt(3) / 2])
BOTTOM_VEC: np.ndarray = np.array([1.0, 0.0])

#CAN IDs (set in the speed controllers)
TOP_LEFT_MOTOR_CAN_ID: int = 3
TOP_RIGHT_MOTOR_CAN_ID: int = 7
BOTTOM_MOTOR_CAN_ID: int = 9

class LowLevelController(Node):
    def __init__(self):
        # Set up subscriber
        super().__init__("low_level_controller")
        self.subscription = self.create_subscription(
            Float32MultiArray, TOPIC_NAME, self.command_callback, QUEUE_SIZE
        )

        # Setup bus
        self.bus = interface.CanBus(channel=CHANNEL, interface=INTERFACE, bitrate=BITRATE)
        self.bus.start()

        # Setup motor state
        self.velocity: np.ndarray = np.array([0.0, 0.0])
        self.speed: float = 1.0
        self.angular_velocity: float = 0.0

        self.top_left_speed: float = 0.0
        self.top_right_speed: float = 0.0
        self.bottom_speed: float = 0.0

        # Setup motors
        if self.bus.started_successfully():
            self.top_left_motor = interface.Motor(self.bus, TOP_LEFT_MOTOR_CAN_ID)
            self.top_left_motor.set_power(0)
            self.top_left_motor.reset_encoder()

            self.top_right_motor = interface.Motor(self.bus, TOP_RIGHT_MOTOR_CAN_ID)
            self.top_right_motor.set_power(0)
            self.top_right_motor.reset_encoder()

            self.bottom_motor = interface.Motor(self.bus, BOTTOM_MOTOR_CAN_ID)
            self.bottom_motor.set_power(0)
            self.bottom_motor.reset_encoder()

        # Setup Servo
        self.servo = []

        # Latest command from ROS — [x_velocity, y_velocity, angular_velocity,head_direction, state]
        self.latest_command: list = [0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0]

        # Setup Timing
        self.control_timer = self.create_timer(0.02, self.control_loop)

    def command_callback(self, msg: Float32MultiArray):
        """Store the latest command from the jet_cmd topic."""
        # msg.data is an array.array; convert to a plain list for easy unpacking.
        if len(msg.data) >= 4:
            self.latest_command = list(msg.data)
        else:
            self.get_logger().warn(
                f"Received command with {len(msg.data)} values, expected 4. Ignoring."
            )

    def control_loop(self):
        x_velocity, y_velocity, angular_velocity, servo_angle_1, servo_angle_2, servo_angle_3, state = self.latest_command
        
        self.velocity = np.array([x_velocity, y_velocity])
        self.angular_velocity = angular_velocity

        self.top_left_speed = float(np.dot(self.velocity, TOP_LEFT_VEC)) + self.angular_velocity
        self.top_right_speed = float(np.dot(self.velocity, TOP_RIGHT_VEC)) + self.angular_velocity
        self.bottom_speed = float(np.dot(self.velocity, BOTTOM_VEC)) + self.angular_velocity

        if self.bus.started_successfully():
            # Keep motors alive
            self.top_left_motor.send_heartbeat()
            self.top_right_motor.send_heartbeat()
            self.bottom_motor.send_heartbeat()

            # Set motor speeds
            self.top_left_motor.set_power(self.top_left_speed)
            self.top_right_motor.set_power(self.top_right_speed)
            self.bottom_motor.set_power(self.bottom_speed)


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()