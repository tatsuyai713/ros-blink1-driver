import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fade_time',
            default_value='1.0',
            description='Fade time in seconds'
        ),
        Node(
            package='ros_blink1_driver',
            executable='ros_blink1_driver_node',
            name='ros_blink1_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'fade_time': launch.substitutions.LaunchConfiguration('fade_time')}],
        )
    ])
