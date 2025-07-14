import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Use Float64 for simple position controllers

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher_ = self.create_publisher(Float64, '/joint1_position_controller/command', 10)

        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position = 0.0
        self.direction = 1.0
        self.get_logger().info("JointMover node has been started.")

    def timer_callback(self):
        # Alternate the joint position between -1.0 and 1.0
        self.position += 0.5 * self.direction
        if self.position > 1.0 or self.position < -1.0:
            self.direction *= -1.0

        msg = Float64()
        msg.data = self.position
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing joint position: {msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = JointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()