"""Existing 3D map localization; no robot motion or hardware driver is started."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("dog_slam"))
    description = Path(get_package_share_directory("dog_description")) / "urdf" / "bpx.urdf"
    sensor = Path(get_package_share_directory("itera_lio")) / "config" / "mid360.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("map_path", default_value="",
                              description="PCD or map project directory; empty waits for reload_map service"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_lio", default_value="true"),
        DeclareLaunchArgument("bridge", default_value="true"),
        DeclareLaunchArgument("robot_description_file", default_value=str(description)),
        DeclareLaunchArgument("sensor_config", default_value=str(sensor)),
        DeclareLaunchArgument("localization_config", default_value=str(share / "config/localization.yaml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch/mapping.launch.py")),
            condition=IfCondition(LaunchConfiguration("start_lio")),
            launch_arguments={
                "operation_mode": "localization", "loop_closure": "false",
                **{key: LaunchConfiguration(key) for key in
                   ("use_sim_time", "bridge", "robot_description_file", "sensor_config")},
            }.items()),
        Node(package="dog_slam", executable="map_localizer", name="map_localizer", output="screen",
            parameters=[LaunchConfiguration("localization_config"), {
                "map_path": ParameterValue(LaunchConfiguration("map_path"), value_type=str),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }]),
    ])
