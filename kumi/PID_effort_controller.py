import rclpy
import os
from launch_ros.substitutions import FindPackageShare
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import csv
import math
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def load_trajectory_from_csv(traj_file):
    points = []

    with open(traj_file, newline="") as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # salta intestazione

        for row in reader:
            time_float = float(row[0])
            secs = int(math.floor(time_float))
            nanosecs = int((time_float - secs) * 1e9)

            duration = Duration()
            duration.sec = secs
            duration.nanosec = nanosecs

            positions_deg = [float(x) for x in row[1:]]
            positions_rads = [x / 180.0 * math.pi for x in positions_deg]

            p = JointTrajectoryPoint()
            p.positions = positions_rads
            p.time_from_start = duration
            points.append(p)

    return points

pkg_share = FindPackageShare("kumi").find("kumi")
traj_file = os.path.join(pkg_share, 'cntr_files', 'targets.csv')

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Parametri
        self.declare_parameter('joints', ['front_sh', 'front_knee', 'front_ank',
                                          'back_sh', 'back_knee','back_ank'])
        self.declare_parameter('kp', [0.0] * 6)
        self.declare_parameter('ki', [0.0] * 6)
        self.declare_parameter('kd', [0.0] * 6)
        self.declare_parameter('max_effort', 10.0)
        self.declare_parameter('max_delta', 1.0)
        self.declare_parameter('use_pid', True)

        self.joints = self.get_parameter('joints').get_parameter_value().string_array_value
        self.kp = np.array(self.get_parameter('kp').get_parameter_value().double_array_value)
        self.ki = np.array(self.get_parameter('ki').get_parameter_value().double_array_value)
        self.kd = np.array(self.get_parameter('kd').get_parameter_value().double_array_value)
        self.max_effort = self.get_parameter('max_effort').get_parameter_value().double_value
        self.max_delta = self.get_parameter('max_delta').get_parameter_value().double_value
        self.use_pid = self.get_parameter('use_pid').get_parameter_value().bool_value

        # Stato dei giunti
        self.target_positions = np.zeros(6)
        self.last_positions = np.zeros(6)
        self.last_velocities = np.zeros(6)
        self.integrals = np.zeros(6)
        self.last_efforts = np.zeros(6)

        # Traiettoria
        self.traj_points = load_trajectory_from_csv(traj_file)
        self.current_step = 0

        # Subscriber giunti
        self.subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Subscriber PID e target positions
        self.sub_pid = self.create_subscription(
            Float64MultiArray,
            '/pid_params',
            self.pid_callback,
            10
        )
        self.sub_target = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_callback,
            10
        )

        # Publisher effort
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_joint_controller/commands',
            10
        )
        # Publisher stato giunti
        self.publisher_state = self.create_publisher(
            Float64MultiArray,
            '/joint_state_array',
            10
        )

        # Timer a 1ms
        self.timer = self.create_timer(0.001, self.control_loop)

    # --- Callback Subscriber ---
    def joint_state_callback(self, msg):
        joint_names = msg.name
        joint_positions = np.array(msg.position)
        joint_velocities = np.array(msg.velocity)
        joint_indices = [joint_names.index(j) for j in self.joints]
        self.last_positions = joint_positions[joint_indices]
        self.last_velocities = joint_velocities[joint_indices]

    def pid_callback(self, msg):
        data = np.array(msg.data)
        if len(data) == 3:
            self.kp[:] = data[0]
            self.ki[:] = data[1]
            self.kd[:] = data[2]
            self.get_logger().info(f"Updated PID params: P={self.kp}, I={self.ki}, D={self.kd}")
        elif len(data) == 18:
            self.kp[:] = data[0:6]
            self.ki[:] = data[6:12]
            self.kd[:] = data[12:18]
            self.get_logger().info("Updated PID params per joint")

    def target_callback(self, msg):
        data = np.array(msg.data)
        if len(data) == 6:
            self.target_positions = data
            self.get_logger().info(f"Updated target positions: {self.target_positions}")

    # --- Controllo ---
    def control_loop(self):
        # Aggiorna target dalla traiettoria se PID attivo
        if self.current_step < len(self.traj_points):
            target_positions = self.traj_points[self.current_step].positions
            self.current_step += 1
            self.target_positions = np.array(target_positions)

        # Calcolo efforts
        errors = self.target_positions - self.last_positions
        self.integrals += errors
        derivatives = self.last_velocities

        efforts = self.kp * errors + self.ki * self.integrals - self.kd * derivatives
        efforts = np.clip(efforts, -self.max_effort, self.max_effort)

        delta_efforts = efforts - self.last_efforts
        delta_efforts = np.clip(delta_efforts, -self.max_delta, self.max_delta)
        efforts = self.last_efforts + delta_efforts
        self.last_efforts = efforts

        # --- Pubblica ---
        msg_effort = Float64MultiArray()
        msg_effort.data = efforts.tolist()
        self.publisher.publish(msg_effort)

        msg_target = Float64MultiArray()
        msg_target.data = self.target_positions.tolist()
        self.publisher_state.publish(msg_target)

        msg_state = Float64MultiArray()
        msg_state.data = self.last_positions.tolist()
        self.publisher_state.publish(msg_state)

        self.get_logger().info(f"Efforts: {efforts}")
        self.get_logger().info(f"Target positions: {self.target_positions}")


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

