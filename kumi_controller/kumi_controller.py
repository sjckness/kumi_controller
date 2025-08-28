import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os
import tempfile
import xacro
from launch_ros.substitutions import FindPackageShare

pkg_share = FindPackageShare("kumi_controller").find("kumi_controller")
xacro_file = os.path.join(pkg_share, 'description', 'kumi_core.xacro')
#process xacro file to a str
doc = xacro.parse(open(xacro_file))
xacro.process_doc(doc)
robot_description_inline = doc.toxml()

#creates a temp .urdf file -> needed for node spown_entity.py
with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
    f.write(robot_description_inline)
    f.flush()
    f.close()
    urdf_temp_path = f.name



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
        p1.positions = [-0.9, 0.0, 0.9, -0.9, 0.0, 0.9]   # posizione target in radianti
        p1.time_from_start.sec = 5

        p2 = JointTrajectoryPoint()
        p2.positions = [1.57, 0.0, 0.0, -4.0, 0.0, 0.0]  # posizione target in radianti
        p2.time_from_start.sec = 5
        p2.time_from_start.nanosec = int(0.7 * 1e9)  # cioè 500000000

        p3 = JointTrajectoryPoint()
        p3.positions = [-0.0, 0.0, 0.0, -0.0, 0.0, 0.0]  # posizione target in radianti
        p3.time_from_start.sec = 9

        msg.points = [p1,p2]
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
