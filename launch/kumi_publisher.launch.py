import os
import tempfile

from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import xacro

from ament_index_python.packages import get_package_share_directory



pkg_share = FindPackageShare("kumi_controller").find("kumi_controller")
xacro_file = os.path.join(pkg_share, 'description', 'kumi.xacro')

robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str  #Indica esplicitamente che è una stringa
    )
robot_description = {'robot_description': robot_description_content}


#Launchdescription automatically called by ros while running this launch file
def generate_launch_description():
    return LaunchDescription([
        # publish robot description (urdf)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
    ])