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
yaml_path= os.path.join(pkg_share, "config", "position_cntr.yaml")

robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str  #Indica esplicitamente che è una stringa
    )
robot_description = {'robot_description': robot_description_content}

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

print('temp:   ' + urdf_temp_path)

urdf_path = LaunchConfiguration('urdf_path')


#define ros nodes to be launched
def launch_setup(context, *args, **kwargs):
    #list of nodes to run
    return [
        DeclareLaunchArgument(
            'urdf_path',
            default_value = urdf_temp_path,
            description = 'Path to the URDF file'
        ),

        # wwntity spown delayed in order to wait for plugins 
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_my_entity',
                    arguments=['-file', urdf_path, '-z', '5', '-entity', 'kumi'],
                    output='screen',
                )
            ]
        ),
        
        #wait before running the controller
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[yaml_path],
                    output='screen',
                )
            ]
        ),

        #wait before running controller nodes
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager-timeout", "50"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint1_trajectory_controller", "--controller-manager-timeout", "50"],
                    output="screen",
                )
            ]
        ),
    ]

#Launchdescription automatically called by ros while running this launch file
def generate_launch_description():
    return LaunchDescription([
        # publish robot description (urdf)
        #Node(
        #    package='robot_state_publisher',
        #    executable='robot_state_publisher',
        #    name='robot_state_publisher',
        #    parameters=[robot_description],
        #    output='screen',
        #),

        #launch an external process
        TimerAction(
            period=3.0,
            actions=[
            ExecuteProcess(
                cmd=['gazebo', '--verbose', 
                     '/usr/share/gazebo-11/worlds/empty.world',
                    '-s', 'libgazebo_ros_factory.so'
                ],
                output='screen',
            )

            ]
        ),
        #run an arbitrary py function with actions ore nodes to launch
        OpaqueFunction(function=launch_setup)
    ])
