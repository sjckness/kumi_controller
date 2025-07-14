from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction

from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue

import tempfile

#define ros nodes to be launched
def launch_setup(context, *args, **kwargs):
    # xacro file path
    xacro_file = os.path.join(
        get_package_share_directory('kumi_controller'),
        'description',
        'kumi.xacro'
    )

    pkg_share = FindPackageShare("kumi_controller").find("kumi_controller")
    yaml_path= os.path.join(pkg_share, "config", "position_cntr.yaml")
    
    #process xacro file to a str
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    #creates a temp .urdf file -> needed for node spown_entity.py
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(robot_description)
        f.flush()
        f.close()
        urdf_temp_path = f.name

    #list of nodes to run
    return [
        # publish robot description (urdf)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description':robot_description}],
            output='screen',
        ),

        # wwntity spown delayed in order to wait for plugins 
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_my_entity',
                    arguments=['-file', urdf_temp_path, '-z', '5', '-entity', 'kumi'],
                    output='screen',
                )
            ]
        ),
        
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[yaml_path],
            output='screen',
        ),
    ]

#Launchdescription automatically called by ros while running this launch file
def generate_launch_description():
    return LaunchDescription([
        #launch an external process
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 
                 '/usr/share/gazebo-11/worlds/empty.world',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen',
        ),

        #run an arbitrary py function with actions ore nodes to launch
        OpaqueFunction(function=launch_setup)
    ])
