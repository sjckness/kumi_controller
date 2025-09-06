import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import pinocchio as pin



class ComCalculator(Node):
    def __init__(self):
        super().__init__('com_calculator')

        # Parametro URDF
        self.declare_parameter('urdf_file', '')
        self.urdf_path = self.get_parameter('urdf_file').get_parameter_value().string_value
        if not self.urdf_path:
            self.get_logger().error("Parametro 'urdf_file' non fornito!")
            raise RuntimeError("Parametro 'urdf_file' mancante")

        # Carica modello Pinocchio
        self.robot_model, _, _ = pin.buildModelsFromUrdf(self.urdf_path)
        self.robot_data = self.robot_model.createData()

        self.latest_joint_msg = None

        # Subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer (20 Hz)
        self.timer = self.create_timer(10, self.timer_callback)

    def joint_state_callback(self, msg):
        self.latest_joint_msg = msg

    def timer_callback(self):
        if self.latest_joint_msg is None:
            return

        # Crea configurazione q inizialmente nulla
        q = np.zeros(self.robot_model.nq)

        # Riempi i valori in base ai giunti
        for i, joint in enumerate(self.robot_model.names[1:]):  # salta "universe"
            if joint in self.latest_joint_msg.name:
                idx_msg = self.latest_joint_msg.name.index(joint)
                q[i] = self.latest_joint_msg.position[idx_msg]

        # Calcola centro di massa
        com = pin.centerOfMass(self.robot_model, self.robot_data, q)
        self.get_logger().info(f"Centro di massa: {com}")

def main(args=None):
    rclpy.init(args=args)
    node = ComCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

