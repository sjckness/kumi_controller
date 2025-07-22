import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/front_sh_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.send_trajectory)

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['front_sh']

        point = JointTrajectoryPoint()
        point.positions = [1.0]  # posizione target in radianti
        point.time_from_start.sec = 2

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory sent!')

        self.timer.cancel()  # manda solo una volta

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
