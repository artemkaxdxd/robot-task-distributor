from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directories
    robot_launch = os.path.join(
        FindPackageShare('task_offloading_core').find('task_offloading_core'),
        'launch',
        'robot_nodes.launch.py'
    )
    
    edge_launch = os.path.join(
        FindPackageShare('edge_server').find('edge_server'),
        'launch',
        'edge_station.launch.py'
    )

    return LaunchDescription([
        # Launch robot nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch)
        ),

        # Launch edge station
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(edge_launch)
        ),
    ])