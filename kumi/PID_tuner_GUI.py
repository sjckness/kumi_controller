import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np

class PIDTunerGUI(Node):
    def __init__(self):
        super().__init__('pid_tuner_gui')

        # ROS Publishers
        self.pub_target = self.create_publisher(Float64MultiArray, '/target_positions', 10)
        self.pub_pid = self.create_publisher(Float64MultiArray, '/pid_params', 10)

        # ROS Subscriber
        self.sub_joint_state = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Dati giunti
        self.joint_names = [
            'front_sh', 'back_sh', 'front_ank',
            'back_sh', 'back_knee', 'back_ank'
        ]
        self.current_positions = np.zeros(6)
        self.target_positions = np.zeros(6)
        self.pid_params = np.array([[0.0, 0.0, 0.0] for _ in range(6)])  # P,I,D per giunto

        # QSettings per salvataggio automatico
        self.settings = QtCore.QSettings("MyOrg", "PIDTunerGUI")
        self.load_settings()

        # GUI
        self.app = QtWidgets.QApplication(sys.argv)
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle('PID Tuner')
        self.main_layout = QtWidgets.QVBoxLayout()
        self.window.setLayout(self.main_layout)

        # --- Selettore giunto ---
        self.joint_selector = QtWidgets.QComboBox()
        self.joint_selector.addItems(self.joint_names + ['All'])
        self.joint_selector.currentIndexChanged.connect(self.update_joint_selection)
        self.selected_joint = 0
        self.main_layout.addWidget(QtWidgets.QLabel("Select Joint:"))
        self.main_layout.addWidget(self.joint_selector)

        # --- PID Sliders + spinbox ---
        self.sliders_pid = []
        self.spin_pid = []
        for i, name in enumerate(['P','I','D']):
            layout = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(name)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            slider.setValue(int(self.pid_params[0,i]*100))
            slider.valueChanged.connect(self.update_pid_from_slider)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setMinimum(0)
            spin.setMaximum(10)
            spin.setSingleStep(0.01)
            spin.setValue(self.pid_params[0,i])
            spin.valueChanged.connect(self.update_pid_from_spinbox)
            layout.addWidget(label)
            layout.addWidget(slider)
            layout.addWidget(spin)
            self.main_layout.addLayout(layout)
            self.sliders_pid.append(slider)
            self.spin_pid.append(spin)

        # --- Target slider + spinbox ---
        layout_target = QtWidgets.QHBoxLayout()
        self.slider_target = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_target.setMinimum(-180)
        self.slider_target.setMaximum(180)
        self.slider_target.setValue(0)
        self.slider_target.valueChanged.connect(self.update_target_from_slider)
        self.spin_target = QtWidgets.QDoubleSpinBox()
        self.spin_target.setMinimum(-180)
        self.spin_target.setMaximum(180)
        self.spin_target.setSingleStep(1)
        self.spin_target.setValue(0)
        self.spin_target.valueChanged.connect(self.update_target_from_spinbox)
        self.label_error = QtWidgets.QLabel("Err: 0%")
        layout_target.addWidget(QtWidgets.QLabel("Target"))
        layout_target.addWidget(self.slider_target)
        layout_target.addWidget(self.spin_target)
        layout_target.addWidget(self.label_error)
        self.main_layout.addLayout(layout_target)

        # --- Radial gauges per joint ---
        self.dials = []
        self.dial_labels = []
        self.dial_value_labels = []
        dial_layout = QtWidgets.QHBoxLayout()
        for jname in self.joint_names:
            dial_widget = QtWidgets.QVBoxLayout()
            dial = QtWidgets.QDial()
            dial.setMinimum(-180)
            dial.setMaximum(180)
            dial.setNotchesVisible(True)
            dial.setEnabled(False)
            label_name = QtWidgets.QLabel(jname)
            label_name.setAlignment(QtCore.Qt.AlignCenter)
            label_value = QtWidgets.QLabel("0.0°")
            label_value.setAlignment(QtCore.Qt.AlignCenter)
            dial_widget.addWidget(dial)
            dial_widget.addWidget(label_name)
            dial_widget.addWidget(label_value)
            dial_layout.addLayout(dial_widget)
            self.dials.append(dial)
            self.dial_labels.append(label_name)
            self.dial_value_labels.append(label_value)
        self.main_layout.addLayout(dial_layout)

        # --- Timer GUI ---
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(50)

        self.window.show()
        self.window.closeEvent = self.closeEvent  # intercetta chiusura finestra

    # --- ROS callback ---
    def joint_state_callback(self, msg: JointState):
        indices = [msg.name.index(j) for j in self.joint_names if j in msg.name]
        self.current_positions = np.array([msg.position[i] for i in indices])

    # --- Update sliders/spinbox ---
    def update_joint_selection(self):
        self.selected_joint = self.joint_selector.currentIndex()
        idx = self.selected_joint if self.selected_joint < 6 else 0
        for i in range(3):
            self.sliders_pid[i].setValue(int(self.pid_params[idx,i]*100))
            self.spin_pid[i].setValue(self.pid_params[idx,i])
        self.slider_target.setValue(int(self.target_positions[idx]*180/np.pi))
        self.spin_target.setValue(self.target_positions[idx]*180/np.pi)

    def update_pid_from_slider(self):
        for i in range(3):
            val = self.sliders_pid[i].value()/100
            if self.selected_joint == 6:
                self.pid_params[:,i] = val
            else:
                self.pid_params[self.selected_joint,i] = val
            self.spin_pid[i].setValue(val)

    def update_pid_from_spinbox(self):
        for i in range(3):
            val = self.spin_pid[i].value()
            if self.selected_joint == 6:
                self.pid_params[:,i] = val
            else:
                self.pid_params[self.selected_joint,i] = val
            self.sliders_pid[i].setValue(int(val*100))

    def update_target_from_slider(self):
        val_rad = self.slider_target.value()*np.pi/180
        if self.selected_joint == 6:
            self.target_positions[:] = val_rad
        else:
            self.target_positions[self.selected_joint] = val_rad
        self.spin_target.setValue(float(self.slider_target.value()))

    def update_target_from_spinbox(self):
        val_rad = self.spin_target.value() * np.pi / 180
        if self.selected_joint == 6:
            self.target_positions[:] = val_rad
        else:
            self.target_positions[self.selected_joint] = val_rad
        self.slider_target.setValue(int(self.spin_target.value()))

    # --- Aggiorna display e pubblica ---
    def update_all(self):
        for i, dial in enumerate(self.dials):
            val_deg = int(self.current_positions[i] * 180 / np.pi)
            val_deg = max(dial.minimum(), min(dial.maximum(), val_deg))
            dial.setValue(val_deg)
            error = abs(self.target_positions[i] - self.current_positions[i])
            color = QtGui.QColor(0,255,0) if error < 0.05 else QtGui.QColor(255,0,0)
            self.dial_value_labels[i].setText(f"{val_deg:.1f}°")
            self.dial_value_labels[i].setStyleSheet(f"color: {color.name()};")

        idx = self.selected_joint if self.selected_joint < 6 else 0
        target_deg = self.target_positions[idx] * 180 / np.pi
        current_deg = self.current_positions[idx] * 180 / np.pi
        error_pct = abs(target_deg - current_deg)/abs(target_deg)*100 if target_deg !=0 else 0
        self.label_error.setText(f"Err: {error_pct:.1f}%")

        self.publish_pid()
        self.publish_target()

    def publish_pid(self):
        msg = Float64MultiArray()
        msg.data = self.pid_params.flatten().tolist()
        self.pub_pid.publish(msg)

    def publish_target(self):
        msg = Float64MultiArray()
        msg.data = self.target_positions.tolist()
        self.pub_target.publish(msg)

    # --- Salvataggio e caricamento ---
    def save_settings(self):
        for i, name in enumerate(self.joint_names):
            self.settings.setValue(f"{name}/P", self.pid_params[i,0])
            self.settings.setValue(f"{name}/I", self.pid_params[i,1])
            self.settings.setValue(f"{name}/D", self.pid_params[i,2])
            self.settings.setValue(f"{name}/Target", self.target_positions[i])
        self.settings.sync()

    def load_settings(self):
        for i, name in enumerate(self.joint_names):
            self.pid_params[i,0] = float(self.settings.value(f"{name}/P", 0.0))
            self.pid_params[i,1] = float(self.settings.value(f"{name}/I", 0.0))
            self.pid_params[i,2] = float(self.settings.value(f"{name}/D", 0.0))
            self.target_positions[i] = float(self.settings.value(f"{name}/Target", 0.0))

    def closeEvent(self, event):
        self.save_settings()
        event.accept()

    def run(self):
        sys.exit(self.app.exec_())

def main(args=None):
    rclpy.init(args=args)
    gui_node = PIDTunerGUI()
    try:
        gui_node.run()
    finally:
        gui_node.save_settings()
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
