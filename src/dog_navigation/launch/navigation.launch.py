"""Planning only. Start map localization independently; no hardware or motion output."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("dog_navigation"))
    return LaunchDescription([
        DeclareLaunchArgument("map_root", default_value="~/.local/share/dog/maps"),
        DeclareLaunchArgument("map_id", default_value="full_map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config", default_value=str(share / "config/navigation.yaml")),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        Node(package="dog_navigation", executable="local_navigation", name="local_navigation", output="screen",
             parameters=[LaunchConfiguration("config"), {
                 key: ParameterValue(LaunchConfiguration(key), value_type=str) for key in ("map_root", "map_id")
             }, {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                 "goal_topic": LaunchConfiguration("goal_topic")}]),
    ])
