"""Explicit mapping or localization/replanning ownership; no navigation actuator."""
from pathlib import Path
import re
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import SetParameter, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def configuration_defaults(context):
    path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or set(document) != {'bringup'} or not isinstance(document['bringup'], dict):
        raise ValueError('Bringup config must contain a bringup mapping: ' + str(path))
    booleans = {'lidar', 'driver', 'start_lio', 'bridge', 'use_sim_time', 'execute'}
    ports = {'port', 'robot_state_port'}
    files = {'livox_config', 'localization_config', 'navigation_config', 'motion_config', 'execution_config'}
    strings = {'mode', 'map_id', 'robot_ip', 'host', 'map_root', 'slam_output'} | files
    actions = [LogInfo(msg='Bringup configuration: ' + str(path))]
    for key, value in document['bringup'].items():
        if key in booleans:
            if type(value) is not bool:
                raise ValueError(key + ' must be a YAML boolean')
            value = str(value).lower()
        elif key in ports:
            if type(value) is not int or not (0 if key == 'port' else 1) <= value <= 65535:
                raise ValueError(key + ' must be a valid port number')
            value = str(value)
        elif key in strings:
            if not isinstance(value, str) or (not value.strip() and key not in files):
                raise ValueError(key + ' must be a nonempty string')
            if key in files:
                if not value.strip():
                    continue
                candidate = Path(value).expanduser()
                value = str((path.parent / candidate).resolve())
        else:
            raise ValueError('Unknown bringup setting: ' + str(key))
        # CLI values are already present in the launch context. YAML fills only
        # unset keys, before DeclareLaunchArgument applies package fallbacks.
        if key not in context.launch_configurations:
            actions.append(SetLaunchConfiguration(key, value))
    return actions


def navigation_child_exit(event, context):
    if not context.is_shutdown:
        raise RuntimeError(f'Navigation bringup child exited unexpectedly: {event.returncode}')
    return []


def mode_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'mapping':
        return [LogInfo(msg='Mapping mode: LIO starts only from the web mapping action.')]
    if mode != 'navigation':
        raise ValueError('mode must be mapping or navigation')
    root = Path(LaunchConfiguration('map_root').perform(context)).expanduser().resolve()
    map_id = LaunchConfiguration('map_id').perform(context)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', map_id):
        raise ValueError('Invalid map_id')
    shared = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return [
        RegisterEventHandler(OnProcessExit(on_exit=navigation_child_exit)),
        # Includes set launch configurations in their enclosing scope. Keep
        # child bridge/config overrides away from bringup's sensor conversion.
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dog_slam'), 'launch', 'localization.launch.py'])),
            launch_arguments={**shared, 'map_path': str(root / map_id),
                **{key: LaunchConfiguration(key) for key in ('start_lio', 'localization_config')},
                'bridge': 'false',  # Conversion is owned by bringup in both modes.
            }.items())]),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dog_navigation'), 'launch', 'execution.launch.py'])),
            launch_arguments={**shared, 'map_root': str(root), 'map_id': map_id,
                              'config': LaunchConfiguration('navigation_config'),
                              'execution_config': LaunchConfiguration('execution_config'),
                              'execute': LaunchConfiguration('execute'), 'guard': 'false'}.items())]),
        LogInfo(msg='Navigation mode: localization, planning and task execution share one map project. '
                    'Initialize localization, then submit /navigate_terrain or use the web console. '
                    'Autonomous output requires execute=true and confirmed calibration.'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('dog_bringup'), 'config', 'bringup.yaml'])),
        OpaqueFunction(function=configuration_defaults),
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'navigation']),
        DeclareLaunchArgument('map_id', default_value='full_map'),
        DeclareLaunchArgument('start_lio', default_value='true', description='Navigation mode: disable only for external/replayed LIO and TF.'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('localization_config', default_value=PathJoinSubstitution([
            FindPackageShare('dog_slam'), 'config', 'localization.yaml'])),
        DeclareLaunchArgument('navigation_config', default_value=PathJoinSubstitution([
            FindPackageShare('dog_navigation'), 'config', 'navigation.yaml'])),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Start MID360s hardware driver (LiDAR + internal IMU).'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='Start BPX robot driver; disable if already running or replaying.'),
        DeclareLaunchArgument('livox_config', default_value=PathJoinSubstitution([
            FindPackageShare('livox_ros_driver2'), 'config', 'MID360s_config.json']),
            description='Livox device/host IP and UDP ports; match the installed sensor.'),
        DeclareLaunchArgument('robot_ip', default_value='10.21.20.1'),
        DeclareLaunchArgument('robot_state_port', default_value='9873'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('map_root', default_value='~/.local/share/dog/maps'),
        DeclareLaunchArgument('slam_output', default_value='~/.local/share/dog/slam'),
        DeclareLaunchArgument('motion_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('execution_config', default_value=PathJoinSubstitution([
            FindPackageShare('dog_navigation'), 'config', 'execution.yaml'])),
        OpaqueFunction(function=mode_nodes),
        GroupAction([
            SetParameter(name='use_sim_time', value=ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)),
            Node(package='interfaces', executable='livox_to_pointcloud2_node',
                name='livox_to_pointcloud2', output='screen',
                condition=IfCondition(LaunchConfiguration('bridge')),
                parameters=[PathJoinSubstitution([FindPackageShare('interfaces'), 'config', 'livox_bridge.yaml']),
                            {'frame_id': 'livox_frame', 'imu_frame_id': 'livox_imu'}]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py'])),
                condition=IfCondition(LaunchConfiguration('lidar')),
                launch_arguments={'user_config_path': LaunchConfiguration('livox_config')}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('dog_driver'), 'launch', 'driver.launch.py'])),
                condition=IfCondition(LaunchConfiguration('driver')),
                launch_arguments={
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'robot_state_port': LaunchConfiguration('robot_state_port'),
                    # odom -> base_link is owned by LIO while mapping.
                    'publish_odom_tf': 'false',
                    'start_velocity_guard': 'false',
                }.items()),
        ]),
        Node(package='dog_navigation', executable='velocity_guard', name='velocity_guard', output='screen',
             parameters=[LaunchConfiguration('execution_config'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'map_root': LaunchConfiguration('map_root'),
                 'map_id': PythonExpression(["'", LaunchConfiguration('map_id'), "' if '", LaunchConfiguration('mode'), "' == 'navigation' else ''"]),
                 'allow_autonomous': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
             }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dog_web'), 'launch', 'planning_preview.launch.py'])),
            launch_arguments={
                **{key: LaunchConfiguration(key) for key in
                   ['host', 'port', 'map_root', 'slam_output', 'use_sim_time', 'motion_config']},
                'ros': 'true',
                'mapping_bridge': 'false',
                'allow_mapping': PythonExpression(["'true' if '", LaunchConfiguration('mode'), "' == 'mapping' else 'false'"]),
            }.items()),
        LogInfo(msg='Hardware starts only as configured by lidar/driver. '
                    'Autonomous navigation defaults to disabled and requires confirmed calibration.'),
    ])
