# Copyright 2026 rainbroken
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz_default_from_config(output_config_path: Path) -> str:
    """Read the profile's RViz default while keeping rviz:=... as an override."""
    with output_config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    try:
        value = config["itera_lio_node"]["ros__parameters"]["debug"]["launch_rviz"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            "output.yaml must define itera_lio_node.ros__parameters.debug.launch_rviz"
        ) from exc
    if not isinstance(value, bool):
        raise RuntimeError("debug.launch_rviz must be a boolean")
    return "true" if value else "false"


def generate_launch_description():
    config = Path(get_package_share_directory("itera_lio")) / "config" / "mid360.yaml"
    bridge_config = (
        Path(get_package_share_directory("interfaces"))
        / "config"
        / "livox_bridge.yaml"
    )
    frontend_config = (
        Path(get_package_share_directory("itera_lio")) / "config" / "frontend.yaml"
    )
    backend_config = (
        Path(get_package_share_directory("itera_lio")) / "config" / "backend.yaml"
    )
    loop_config = (
        Path(get_package_share_directory("itera_lio")) / "config" / "loop.yaml"
    )
    output_config = (
        Path(get_package_share_directory("itera_lio")) / "config" / "output.yaml"
    )
    camera_config = (
        Path(get_package_share_directory("itera_lio")) / "config" / "camera.yaml"
    )
    evaluation_config = (
        Path(get_package_share_directory("itera_lio")) / "config" / "evaluation.yaml"
    )
    rviz_config = Path(get_package_share_directory("itera_lio")) / "rviz" / "itera_lio.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value=_rviz_default_from_config(output_config),
                description=(
                    "Start RViz2; defaults to debug.launch_rviz from output.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "bridge",
                default_value="true",
                description="Start the Livox-to-PointCloud2 and IMU bridge",
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=str(camera_config),
                description="ROS2 parameter file for camera calibration and synchronization",
            ),
            DeclareLaunchArgument(
                "evaluation_config",
                default_value=str(evaluation_config),
                description="ROS2 parameter file for evaluation recording and comparison",
            ),
            Node(
                package="interfaces",
                executable="livox_to_pointcloud2_node",
                name="livox_to_pointcloud2",
                output="screen",
                parameters=[str(bridge_config)],
                condition=IfCondition(LaunchConfiguration("bridge")),
            ),
            Node(
                package="itera_lio", executable="itera_lio_node", name="itera_lio_node",
                output="screen", sigterm_timeout="90",
                parameters=[
                    str(config), str(frontend_config), str(backend_config), str(loop_config),
                    str(output_config), LaunchConfiguration("camera_config"),
                    LaunchConfiguration("evaluation_config"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
