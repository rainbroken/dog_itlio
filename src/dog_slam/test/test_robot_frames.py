import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

spec = importlib.util.spec_from_file_location(
    "mapping", Path(__file__).resolve().parents[1] / "launch" / "mapping.launch.py")
mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)


def test_robot_extrinsics_consistent():
    robot = ET.parse(Path(get_package_share_directory("dog_description")) / "urdf" / "bpx.urdf").getroot()
    bi = mapping.fixed_transform(robot, "base_link", "livox_imu")
    bl = mapping.fixed_transform(robot, "base_link", "livox_frame")
    il = mapping.compose(mapping.inverse(bi), bl)
    recovered = mapping.compose(bi, il)
    for row, expected in zip(recovered[0], bl[0]):
        assert row == pytest.approx(expected, abs=1e-12)
    assert recovered[1] == pytest.approx(bl[1], abs=1e-12)
    # URDF has a nonzero IMU mounting rotation, unlike the old LIO identity.
    assert abs(il[0][0][2]) > 0.01


@pytest.mark.parametrize("joint_type", ["revolute", "continuous"])
def test_reject_articulated_mount(joint_type):
    robot = ET.fromstring(f'''<robot name="test"><link name="base"/><link name="imu"/>
      <joint name="mount" type="{joint_type}"><parent link="base"/>
      <child link="imu"/></joint></robot>''')
    with pytest.raises(ValueError, match="must be fixed"):
        mapping.fixed_transform(robot, "base", "imu")


def test_reject_disconnected_sensor():
    robot = ET.fromstring('<robot name="test"><link name="base"/><link name="imu"/></robot>')
    with pytest.raises(ValueError, match="No fixed URDF chain"):
        mapping.fixed_transform(robot, "base", "imu")


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_mode_has_exclusive_map_tf(mode):
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    context = LaunchContext()
    share = Path(get_package_share_directory("dog_description"))
    context.launch_configurations.update({
        "robot_description_file": str(share / "urdf/bpx.urdf"),
        "operation_mode": mode, "use_sim_time": "false", "bridge": "false",
        "output_dir": "/tmp/dog-launch-test-unused", "loop_closure": "false",
        "sensor_config": str(Path(get_package_share_directory("itera_lio")) / "config/mid360.yaml"),
    })
    nodes = mapping.launch_nodes(context)
    lio = next(n for n in nodes if n.node_executable == "itera_lio_node")
    evaluated = evaluate_parameters(context, lio._Node__parameters)
    params = {}
    for item in evaluated:
        if isinstance(item, dict): params.update(item)
    assert params["tf.publish_map_to_odom"] == (mode == "mapping")
    assert not params["tf.publish_odom_to_imu"]
    assert not params["tf.publish_imu_to_lidar"]
    if mode == "localization":
        assert not params["backend.enabled"] and not params["loop_closure.enabled"]
        assert params["debug.publish_deskewed"]
        assert not params["save_pcd.save_pcd_on_shutdown"]
