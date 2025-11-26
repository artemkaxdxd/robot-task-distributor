from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'num_workers',
            default_value='2',
            description='Number of worker threads for task execution'
        ),

        # Edge server node
        Node(
            package='edge_server',
            executable='edge_server_node',
            name='edge_server',
            output='screen',
            parameters=[{
                'num_workers': LaunchConfiguration('num_workers'),
                'status_publish_rate_hz': 1.0,
            }]
        ),

        # Station monitor node
        Node(
            package='edge_server',
            executable='station_monitor_node',
            name='station_monitor',
            output='screen',
        ),
    ])