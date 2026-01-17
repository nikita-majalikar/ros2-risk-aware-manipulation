import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class UncertaintyPublisher(Node):

    def __init__(self):
        super().__init__('uncertainty_publisher')

        self.pub = self.create_publisher(
            Float32,
            '/object_pose_uncertain',
            10
        )

        self.timer = self.create_timer(0.5, self.publish_uncertainty)

        self.get_logger().info('Uncertainty Publisher started')

    def publish_uncertainty(self):
        msg = Float32()
        msg.data = random.uniform(0.0, 1.0)

        self.pub.publish(msg)

        self.get_logger().info(f'Published uncertainty: {msg.data:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = UncertaintyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
