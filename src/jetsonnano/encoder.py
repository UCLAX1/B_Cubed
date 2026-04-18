import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Publisher(Node):
    def __init__(self):
        super().__init__("publisher")
        self.publisher_ = self.create_publisher(Float32MultiArray, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.servo_angle_1 = 0.0
        self.servo_angle_2 = 0.0
        self.servo_angle_3 = 0.0
        self.state = 0.0

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.x_velocity),
            float(self.y_velocity),
            float(self.servo_angle_1),
            float(self.servo_angle_2),
            float(self.servo_angle_3),
            float(self.state),
        ]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
