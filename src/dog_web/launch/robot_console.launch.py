"""Console + optional robot driver. Mapping is managed by the console or externally."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('motion_config', default_value=''),
        DeclareLaunchArgument('map_root', default_value='~/.local/share/dog/maps'),
        DeclareLaunchArgument('slam_output', default_value='~/.local/share/dog/slam'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('driver', default_value='false'),
        DeclareLaunchArgument('robot_ip', default_value='10.21.20.1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dog_web'), 'launch', 'planning_preview.launch.py'])),
            launch_arguments={key: LaunchConfiguration(key) for key in
                              ['host', 'port', 'map_root', 'slam_output', 'use_sim_time', 'motion_config']}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dog_driver'), 'launch', 'driver.launch.py'])),
            condition=IfCondition(LaunchConfiguration('driver')),
            launch_arguments={'robot_ip': LaunchConfiguration('robot_ip'),
                              'publish_odom_tf': 'false'}.items()),
    ])
