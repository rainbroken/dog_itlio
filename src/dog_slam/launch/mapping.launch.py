"""Robot LIO mapping entry point. URDF is the single mounting calibration source."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def compose(a, b):
    ra, ta = a
    rb, tb = b
    return (
        [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)],
        [ta[i] + sum(ra[i][k] * tb[k] for k in range(3)) for i in range(3)],
    )


def inverse(transform):
    r, t = transform
    rt = [list(row) for row in zip(*r)]
    return rt, [-sum(rt[i][k] * t[k] for k in range(3)) for i in range(3)]


def fixed_transform(robot, root, child):
    """Return T_root_child, rejecting missing, cyclic or articulated chains."""
    identity = ([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], [0., 0., 0.])
    links = {link.attrib["name"] for link in robot.findall("link")}
    if root not in links or child not in links:
        raise ValueError(f"URDF must contain {root} and {child}")
    parents = {}
    for joint in robot.findall("joint"):
        name = joint.find("child").attrib["link"]
        if name in parents:
            raise ValueError(f"Multiple URDF parents for {name}")
        parents[name] = joint

    def resolve(name, visited):
        if name == root:
            return identity
        if name in visited or name not in parents:
            raise ValueError(f"No fixed URDF chain from {root} to {child}")
        joint = parents[name]
        if joint.attrib["type"] != "fixed":
            raise ValueError(f"Sensor mount {joint.attrib['name']} must be fixed")
        origin = joint.find("origin")
        xyz = [float(x) for x in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(x) for x in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(x) for x in xyz + rpy):
            raise ValueError("URDF origin requires finite xyz/rpy triples")
        sr, sp, sy = map(math.sin, rpy)
        cr, cp, cy = map(math.cos, rpy)
        rotation = [
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr],
        ]
        return compose(resolve(joint.find("parent").attrib["link"], visited | {name}),
                       (rotation, xyz))

    return resolve(child, set())


def launch_nodes(context):
    share = Path(get_package_share_directory("dog_slam"))
    lio = Path(get_package_share_directory("itera_lio")) / "config"
    bridge = Path(get_package_share_directory("interfaces")) / "config" / "livox_bridge.yaml"
    model = Path(LaunchConfiguration("robot_description_file").perform(context)).expanduser().read_text()
    robot = ET.fromstring(model)
    base_to_imu = fixed_transform(robot, "base_link", "livox_imu")
    base_to_lidar = fixed_transform(robot, "base_link", "livox_frame")
    imu_to_lidar = compose(inverse(base_to_imu), base_to_lidar)
    flatten = lambda r: [value for row in r for value in row]
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    output_dir = str(Path(LaunchConfiguration("output_dir").perform(context)).expanduser().resolve())
    operation_mode = LaunchConfiguration("operation_mode").perform(context)
    if operation_mode not in ("mapping", "localization"):
        raise ValueError("operation_mode must be mapping or localization")
    localizing = operation_mode == "localization"
    mode_overrides = {
        "tf.publish_map_to_odom": not localizing,
    }
    if localizing:
        mode_overrides.update({
            "backend.enabled": False, "loop_closure.enabled": False,
            "debug.publish_deskewed": True, "debug.publish_registered": False,
            "debug.publish_global_map": False, "debug.publish_optimized_map": False,
            "save_pcd.save_pcd_on_shutdown": False, "evaluation.enabled": False,
        })
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": model, "use_sim_time": sim_time}]),
        Node(package="interfaces", executable="livox_to_pointcloud2_node",
             name="livox_to_pointcloud2", output="screen",
             condition=IfCondition(LaunchConfiguration("bridge")),
             parameters=[str(bridge), {"frame_id": "livox_frame", "imu_frame_id": "livox_imu",
                                       "use_sim_time": sim_time}]),
        Node(package="itera_lio", executable="itera_lio_node", name="itera_lio_node",
             output="screen", sigterm_timeout="90",
             additional_env={"ITERA_LIO_NO_ARCHIVE": "1"} if localizing else {},
             parameters=[
                 LaunchConfiguration("sensor_config"), str(lio / "frontend.yaml"),
                 str(lio / "backend.yaml"), str(lio / "loop.yaml"),
                 str(lio / "output.yaml"), str(lio / "evaluation.yaml"),
                 str(share / "config" / "mapping.yaml"),
                 {
                     "use_sim_time": sim_time,
                     "mode.type": "lio",
                     "frames.world": "map", "frames.odom": "odom",
                     "frames.imu": "livox_imu", "frames.lidar": "livox_frame",
                     "topics.odometry": "/lio/odom",
                     "extrin_calib.extrinsic_T": imu_to_lidar[1],
                     "extrin_calib.extrinsic_R": flatten(imu_to_lidar[0]),
                     "tf.publish_map_to_odom": True,
                     "tf.publish_odom_to_imu": False,
                     "tf.publish_imu_to_lidar": False,
                     "loop_closure.enabled": ParameterValue(
                         LaunchConfiguration("loop_closure"), value_type=bool),
                     "save_pcd.pcd_directory": output_dir,
                     "evaluation.output_dir": output_dir,
                     "loop_closure.storage.archive_directory": str(Path(output_dir) / "loop_keyframes"),
                 },
                 mode_overrides,
             ]),
        Node(package="dog_slam", executable="lio_tf_adapter", name="lio_tf_adapter",
             output="screen", parameters=[{
                 "use_sim_time": sim_time,
                 "odom_frame": "odom", "base_frame": "base_link", "imu_frame": "livox_imu",
                 "odometry_topic": "/lio/odom",
                 "base_to_imu_translation": base_to_imu[1],
                 "base_to_imu_rotation": flatten(base_to_imu[0]),
             }]),
    ]


def generate_launch_description():
    description = Path(get_package_share_directory("dog_description")) / "urdf" / "bpx.urdf"
    sensor = Path(get_package_share_directory("itera_lio")) / "config" / "mid360.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("operation_mode", default_value="mapping",
                              description="localization.launch.py selects localization: exclusive map TF and no archive"),
        DeclareLaunchArgument("bridge", default_value="true",
                              description="Convert /livox/lidar and /livox/imu; no hardware driver"),
        DeclareLaunchArgument("loop_closure", default_value="false",
                              description="Enable experimental IteraLIO loop correction"),
        DeclareLaunchArgument("robot_description_file", default_value=str(description)),
        DeclareLaunchArgument("sensor_config", default_value=str(sensor),
                              description="Sensor timing/noise/topics; URDF overrides extrinsics"),
        DeclareLaunchArgument("output_dir", default_value="~/.local/share/dog/slam"),
        OpaqueFunction(function=launch_nodes),
    ])
