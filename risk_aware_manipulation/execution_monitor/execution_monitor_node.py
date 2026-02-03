import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time

class ExecutionMonitor(Node):

    def __init__(self):
        super().__init__('execution_monitor_node')

        self.progress_sub = self.create_subscription(
            Float32,
            '/fake_execution_progress',
            self.progress_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/execution_status',
            10
        )

        self.last_progress = 0.0
        self.last_progress_time = time.time()
        self.current_status = "idle"

        self.timer = self.create_timer(0.5, self.evaluate_execution)

        self.get_logger().info("Execution Monitor Node started")

    def progress_callback(self, msg):
        if msg.data > self.last_progress:
            self.last_progress = msg.data
            self.last_progress_time = time.time()

    def evaluate_execution(self):
        now = time.time()
        dt = now - self.last_progress_time

        if self.last_progress == 0.0:
            status = "idle"
        elif self.last_progress >= 1.0:
            status = "completed"
        elif dt > 3.0:
            status = "stalled"
        elif dt > 1.5:
            status = "slow"
        else:
            status = "executing"

        if status != self.current_status:
            self.current_status = status
            msg = String()
            msg.data = status
            self.status_pub.publish(msg)
            self.get_logger().warn(f"Execution status → {status}")

def main():
    rclpy.init()
    node = ExecutionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
