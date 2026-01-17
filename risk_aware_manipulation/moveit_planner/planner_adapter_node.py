import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PlannerAdapterNode(Node):

    def __init__(self):
        super().__init__('planner_adapter_node')

        self.current_mode = "normal"

        self.create_subscription(
            String,
            '/planner_mode',
            self.mode_callback,
            10
        )

        self.constraint_pub = self.create_publisher(
            String,
            '/planner_constraints',
            10
        )

        self.timer = self.create_timer(0.5, self.publish_constraints)

        self.get_logger().info("Planner Adapter Node started")

    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f"Planner mode updated: {self.current_mode}")

    def publish_constraints(self):
        out = String()

        if self.current_mode == "normal":
            out.data = "fast_plan"
        elif self.current_mode == "conservative":
            out.data = "slow_plan"
        elif self.current_mode == "abort":
            out.data = "stop"
        else:
            out.data = "unknown"

        self.constraint_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


