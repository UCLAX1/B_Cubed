#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from sense_hat import SenseHat
import time


class SenseHatRawPublisher(Node):
    def __init__(self):
        super().__init__('sense_hat_raw_publisher')

        self.pub = self.create_publisher(
            Float32MultiArray,
            'sense_hat/raw',
            10
        )

        self.sense = SenseHat()
        self.timer = self.create_timer(0.1, self.publish_raw)

        self.get_logger().info("Publishing raw Sense HAT data")

    def publish_raw(self):
        accel = self.sense.get_accelerometer_raw()
        gyro = self.sense.get_gyroscope_raw()
        orientation = self.sense.get_orientation()
        data = [
            accel['x'], accel['y'], accel['z'],
            gyro['x'], gyro['y'], gyro['z'],
            orientation['roll'],
            orientation['pitch'],
            orientation['yaw']
        ]


        msg = Float32MultiArray()

        msg.data = [
            orientation['roll'],
            orientation['pitch'],
            orientation['yaw']
        ]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SenseHatRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
