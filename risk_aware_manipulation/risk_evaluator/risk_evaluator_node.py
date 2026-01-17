import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RiskEvaluatorNode(Node):

    def __init__(self):
        super().__init__('risk_evaluator_node')

        self.subscription = self.create_subscription(
            Float32,
            '/object_pose_uncertain',
            self.uncertainty_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32,
            '/grasp_risk',
            10
        )

        self.get_logger().info('Risk Evaluator Node started')

    def uncertainty_callback(self, msg):
        uncertainty = float(msg.data)

        # simple passthrough + clamp
        risk = max(0.0, min(uncertainty, 1.0))

        out = Float32()
        out.data = risk
        self.publisher.publish(out)

        self.get_logger().info(
            f"uncertainty={uncertainty:.4f} -> risk={risk:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RiskEvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

