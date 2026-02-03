import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class FakeExecutionProgress(Node):

    def __init__(self):
        super().__init__('fake_execution_progress')
        self.pub = self.create_publisher(Float32, '/fake_execution_progress', 10)
        self.timer = self.create_timer(0.5, self.publish_progress)

        self.progress = 0.0
        self.mode = "normal"  # normal | slow | stall

        self.get_logger().info("Fake Execution Progress Publisher started")

    def publish_progress(self):
        # Randomly change behavior
        if random.random() < 0.1:
            self.mode = random.choice(["normal", "slow", "stall"])

        if self.mode == "normal":
            self.progress += random.uniform(0.05, 0.1)
        elif self.mode == "slow":
            self.progress += random.uniform(0.005, 0.02)
        elif self.mode == "stall":
            self.progress += 0.0

        self.progress = min(self.progress, 1.0)

        msg = Float32()
        msg.data = self.progress
        self.pub.publish(msg)

        self.get_logger().info(f"Progress={self.progress:.2f} mode={self.mode}")

        if self.progress >= 1.0:
            self.get_logger().warn("Execution completed, resetting")
            self.progress = 0.0

def main():
    rclpy.init()
    node = FakeExecutionProgress()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
