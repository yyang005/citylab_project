from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='robot_patrol',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen')
    ])