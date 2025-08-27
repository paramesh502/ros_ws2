from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_system_tests',
            executable='test_manager_node',
            output='screen',
            emulate_tty=True,
            parameters=[]
        )
    ])
