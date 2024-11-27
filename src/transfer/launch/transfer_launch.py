
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    jetson2app = Node(
        package="transfer",
        executable="jetson2app",
        parameters=[
            {'local_port': 43899}
        ],
        output='screen'
    )
    jetson2motion = Node(
        package="transfer",
        executable="jetson2motion",
        parameters=[
            {'target_ip': "192.168.1.120"},
            {'target_port': 43893},
            {'local_port': 43897}
        ],
        output='screen'
    )
    sensor_checker = Node(
        package="transfer",
        executable="sensor_checker",
        parameters=[
            {'is_debug': True}
        ],
        output='screen'
    )
    launch_description = LaunchDescription(
        [jetson2app, jetson2motion, sensor_checker])
    return launch_description
