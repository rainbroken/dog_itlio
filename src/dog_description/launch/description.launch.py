from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = Path(get_package_share_directory("dog_description"))
    robot_description = (share_directory / "urdf" / "bpx.urdf").read_text()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "process_prefix", default_value="env"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                prefix=LaunchConfiguration("process_prefix"),
                parameters=[{"robot_description": robot_description}],
            )
        ]
    )
