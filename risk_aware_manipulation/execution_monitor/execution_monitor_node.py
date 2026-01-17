import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
import csv
import time
from pathlib import Path

class ExecutionMonitorNode(Node):
    def __init__(self):
        super().__init__('execution_monitor_node')

        self.current_mode = "unknown"
        self.start_time = None

        self.mode_sub = self.create_subscription(
            String,
            '/planning_mode',
            self.mode_callback,
            10
        )

        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/execute_trajectory/_action/status',
            self.status_callback,
            10
        )

        log_dir = Path.home() / "ros2_logs"
        log_dir.mkdir(exist_ok=True)
        self.log_file = log_dir / "execution_log.csv"

        if not self.log_file.exists():
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp",
                    "planning_mode",
                    "execution_status",
                    "duration_sec"
                ])

        self.get_logger().info("Execution monitor started")

    def mode_callback(self, msg: String):
        self.current_mode = msg.data

    def status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        status = msg.status_list[-1].status
        status_map = {
            0: "unknown",
            1: "accepted",
            2: "executing",
            3: "succeeded",
            4: "canceled",
            5: "aborted"
        }

        status_str = status_map.get(status, "other")

        if status_str == "executing" and self.start_time is None:
            self.start_time = time.time()

        if status_str in ["succeeded", "aborted", "canceled"]:
            duration = 0.0
            if self.start_time:
                duration = time.time() - self.start_time

            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(),
                    self.current_mode,
                    status_str,
                    round(duration, 3)
                ])

            self.get_logger().info(
                f"Execution {status_str} | mode={self.current_mode} | duration={duration:.2f}s"
            )

            self.start_time = None

def main():
    rclpy.init()
    node = ExecutionMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
