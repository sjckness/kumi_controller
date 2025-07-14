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

world_path = os.path.join('/home/andreas/dev_ws/src/kumi_controller', 'worlds', 'test_world.world')

def launch_setup(context, *args, **kwargs):
    # Path al file Xacro
    xacro_file = os.path.join(
        get_package_share_directory('kumi_controller'),
        'description',
        'kumi.xacro'
    )

    print("xacrofile:" + xacro_file)

    pkg_share = FindPackageShare("kumi_controller").find("kumi_controller")
    #urdf_file = os.path.join(pkg_share, "description", "kumi.xacro")
    yaml_path= os.path.join(pkg_share, "config", "position_cntr.yaml")
    # Converte Xacro a URDF string
    #robot_description = xacro.process_file(urdf_file)
    

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(robot_description)
        f.flush()
        f.close()
        urdf_temp_path = f.name

    return [
        # Pubblica il robot URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description':robot_description}],
            output='screen',
        ),

        # Nodo custom che pubblica su /world/default/create
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
            #executable='ros2_control_node',
            parameters=[yaml_path],
            output='screen',
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 
                 '/usr/share/gazebo-11/worlds/empty.world',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen',
        ),
    
        OpaqueFunction(function=launch_setup)
    ])
