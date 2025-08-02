from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='simple_test_system', executable='tester_node', name='tester_node', output='screen'),
        Node(package='simple_test_system', executable='responder_node', name='node1', output='screen'),
        Node(package='simple_test_system', executable='responder_node', name='node2', output='screen'),
        Node(package='simple_test_system', executable='responder_node', name='node3', output='screen'),
        Node(package='simple_test_system', executable='responder_node', name='node4', output='screen'),
    ])
