from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'edge_address',
            default_value='127.0.0.1',
            description='IP address of edge station'
        ),
        DeclareLaunchArgument(
            'edge_port',
            default_value='50051',
            description='Port of edge station'
        ),

        # System monitor node
        Node(
            package='system_monitor',
            executable='system_monitor_node',
            name='system_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),

        # Network monitor node
        Node(
            package='network_monitor',
            executable='network_monitor_node',
            name='network_monitor',
            output='screen',
            parameters=[{
                'edge_address': LaunchConfiguration('edge_address'),
                'edge_port': LaunchConfiguration('edge_port'),
                'publish_rate_hz': 0.5,
            }]
        ),

        # Decision maker node
        Node(
            package='decision_maker',
            executable='decision_maker_node',
            name='decision_maker',
            output='screen',
        ),

        # Task executor node
        Node(
            package='task_executor',
            executable='task_executor_node',
            name='task_executor',
            output='screen',
        ),
    ])