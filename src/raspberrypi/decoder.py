# listens to ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.subscription = self.create_subscription(
            Float32MultiArray, "topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Recieved: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
