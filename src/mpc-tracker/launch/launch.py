from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    aruco_config = os.path.join(
        get_package_share_directory('mpc-tracker'),
        'rviz_config_v2.rviz'
    )
    
    turtlebot_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'empty_world.launch.py'
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='burger'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot_launch)
        ),
        Node(
            package="mpc-tracker",
            executable="path_planner",
            name="path_publisher",
            output="screen"
        ),
        Node(
            package="mpc-tracker",
            executable="controller",
            name="mpc_node",
            output="screen"
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', aruco_config
                 ],
            output='screen'
        ),
        Node(
            package="mpc-tracker",
            executable="travelled_path",
            name="travelled_path",
            output="screen"
        ),
        ExecuteProcess(
            cmd=['rqt'],
            output='screen'
        ),
])