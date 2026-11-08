from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    driver_share = Path(get_package_share_directory("dog_driver"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_odom_tf", default_value="false"),
            DeclareLaunchArgument("odom_topic", default_value="/dog/odom"),
            DeclareLaunchArgument("robot_ip", default_value="10.21.20.1"),
            DeclareLaunchArgument("robot_state_port", default_value="9873"),
            DeclareLaunchArgument("process_prefix", default_value="env"),
            DeclareLaunchArgument("start_velocity_guard", default_value="true"),
            Node(package="dog_navigation", executable="velocity_guard", name="velocity_guard",
                 condition=IfCondition(LaunchConfiguration("start_velocity_guard")), output="screen"),
            Node(
                package="dog_driver",
                executable="dog_driver_node",
                name="dog_driver",
                output="screen",
                prefix=LaunchConfiguration("process_prefix"),
                parameters=[
                    str(driver_share / "config" / "driver.yaml"),
                    {
                        "publish_odom_tf": ParameterValue(
                            LaunchConfiguration("publish_odom_tf"), value_type=bool
                        ),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "robot_ip": LaunchConfiguration("robot_ip"),
                        "robot_state_port": ParameterValue(
                            LaunchConfiguration("robot_state_port"), value_type=int
                        ),
                    },
                ],
            ),
        ]
    )
