from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackagePrefix
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('map_root', default_value='~/.local/share/dog/maps'),
        DeclareLaunchArgument('ros', default_value='true'),
        DeclareLaunchArgument('allow_mapping', default_value='true'),
        DeclareLaunchArgument('mapping_bridge', default_value='true',
                              description='Standalone console starts conversion with mapping; bringup owns it separately.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_output', default_value='~/.local/share/dog/slam'),
        DeclareLaunchArgument('motion_config', default_value='', description='Persistent Web speeds; empty uses map_root/.motion.json'),
        ExecuteProcess(cmd=[PathJoinSubstitution([FindPackagePrefix('dog_web'), 'lib',
                                                 'dog_web', 'planning_preview']),
                            '--host', LaunchConfiguration('host'),
                            '--port', LaunchConfiguration('port'),
                            '--map-root', LaunchConfiguration('map_root'),
                            '--ros', LaunchConfiguration('ros'),
                            '--allow-mapping', LaunchConfiguration('allow_mapping'),
                            '--mapping-bridge', LaunchConfiguration('mapping_bridge'),
                            '--use-sim-time', LaunchConfiguration('use_sim_time'),
                            '--slam-output', LaunchConfiguration('slam_output'),
                            '--motion-config', LaunchConfiguration('motion_config')], output='screen',
                       sigterm_timeout='110'),
    ])
