import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String


class RiskSupervisor(Node):

    def __init__(self):
        super().__init__('risk_supervisor_node')

        self.current_risk = None
        self.execution_state = "idle"

        self.create_subscription(
            Float32,
            '/grasp_risk',
            self.risk_callback,
            10
        )

        self.create_subscription(
            String,
            '/execution_status',
            self.execution_callback,
            10
        )

        self.command_pub = self.create_publisher(
            String,
            '/planner_mode',
            10
        )

        self.timer = self.create_timer(0.5, self.evaluate)

        self.get_logger().info("Risk Supervisor started")

    def risk_callback(self, msg):
        self.current_risk = msg.data

    def execution_callback(self, msg):
        self.execution_state = msg.data

    def evaluate(self):
        if self.current_risk is None:
            return

        command = String()

        if self.execution_state in ["failed", "timeout", "cancelled"]:
            command.data = "abort"
        elif self.current_risk < 0.3:
            command.data = "normal"
        elif self.current_risk < 0.6:
            command.data = "conservative"
        else:
            command.data = "abort"

        self.command_pub.publish(command)

        self.get_logger().info(
            f"Decision: {command.data} | risk={self.current_risk:.2f} | exec={self.execution_state}"
        )


def main():
    rclpy.init()
    node = RiskSupervisor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
