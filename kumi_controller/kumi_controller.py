import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/multi_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.send_trajectory)

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['front_sh', 'front_knee', 'front_ank', 'back_sh', 'back_knee','back_ank',]

        p1 = JointTrajectoryPoint()
        p1.positions = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]   # posizione target in radianti
        p1.time_from_start.sec = 2

        p2 = JointTrajectoryPoint()
        p2.positions = [0.0, 0.0, 0.0, 1.0, 1.0, 0.0]  # posizione target in radianti
        p2.time_from_start.sec = 4

        msg.points = [p1, p2]
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
