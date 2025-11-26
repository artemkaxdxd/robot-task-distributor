from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rate',
            default_value='0.2',
            description='Task generation rate in Hz'
        ),

        Node(
            package='task_executor',
            executable='task_generator_node',
            name='task_generator',
            output='screen',
            parameters=[{
                'generation_rate_hz': LaunchConfiguration('rate'),
                'task_types': ['object_detection', 'path_planning', 'pointcloud_processing'],
            }]
        ),
    ])