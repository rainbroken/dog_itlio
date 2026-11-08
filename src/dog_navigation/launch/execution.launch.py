"""Terrain planner, task action server and optionally the sole velocity guard."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('dog_navigation'))
    common = {k: ParameterValue(LaunchConfiguration(k), value_type=str) for k in ('map_root', 'map_id')}
    common['use_sim_time'] = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    enabled = ParameterValue(LaunchConfiguration('execute'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('map_root', default_value='~/.local/share/dog/maps'),
        DeclareLaunchArgument('map_id', default_value='full_map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('guard', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('execution_config', default_value=str(share / 'config/execution.yaml')),
        DeclareLaunchArgument('global_planner_config', default_value=str(share / 'config/global_planner.yaml')),
        DeclareLaunchArgument('local_planner_config', default_value=str(share / 'config/local_planner.yaml')),
        DeclareLaunchArgument('decision_config', default_value=str(share / 'config/decision.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/navigation.launch.py')),
            launch_arguments={**{k: LaunchConfiguration(k) for k in (
                'map_root', 'map_id', 'use_sim_time', 'config', 'global_planner_config', 'local_planner_config')},
                              'goal_topic': PythonExpression(["'/navigation/planner_goal' if '",
                                  LaunchConfiguration('execute'), "'.lower() == 'true' else '/goal_pose'"])}.items()),
        Node(package='dog_navigation', executable='navigation_executor', name='navigation_executor', output='screen',
             parameters=[LaunchConfiguration('local_planner_config'), LaunchConfiguration('decision_config'),
                         LaunchConfiguration('execution_config'), common, {'enabled': enabled}]),
        Node(package='dog_navigation', executable='velocity_guard', name='velocity_guard', output='screen',
             condition=IfCondition(LaunchConfiguration('guard')),
             parameters=[LaunchConfiguration('execution_config'), common, {'allow_autonomous': enabled}]),
    ])
