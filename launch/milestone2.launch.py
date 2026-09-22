"""Launch the Milestone 2 PX4 offboard mission node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_energy_inspection',
            executable='offboard_mission',
            name='offboard_mission',
            output='screen',
        ),
    ])
