import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os
import csv
from builtin_interfaces.msg import Duration
import math
import tempfile
import xacro
from launch_ros.substitutions import FindPackageShare

position = [ [ 1, 2, 3], 
             [ 4, 5, 6],
             [ 7, 8, 9] ]

pkg_share = FindPackageShare("kumi").find("kumi")
xacro_file = os.path.join(pkg_share, 'description', 'kumi_core.xacro')
traj_file = os.path.join(pkg_share, 'kumi_controller', 'targets.csv')
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
        msg.joint_names = ['front_sh', 'front_knee', 'front_ank', 'back_sh', 'back_knee','back_ank']

        points = []
        with open(traj_file, newline="") as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # salta l’intestazione
            
            for row in reader:
                time_float = float(row[0])  # prima colonna = tempo
                secs = int(math.floor(time_float))
                nanosecs = int((time_float - secs) * 1e9)
                duration = Duration()
                duration.sec = secs
                duration.nanosec = nanosecs

                positions_deg = [float(x) for x in row[1:]]  # resto = posizioni giunti
                positions_rads = [x / 180 * 3.14159265 for x in positions_deg] 
                p = JointTrajectoryPoint()
                p.positions = positions_rads
                p.time_from_start = duration
                points.append(p)

        print(points)
        msg.points = points
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
