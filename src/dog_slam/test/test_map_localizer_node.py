"""Synthetic DDS integration; no robot, driver or motion commands."""
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import time
import pytest

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy


@pytest.mark.parametrize("global_mode,registration_rate", [(False,8.), (True,8.), ("history",8.), (False,.5)])
def test_localization_lifecycle(monkeypatch, tmp_path, global_mode, registration_rate):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(130 + os.getpid() % 80))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    points = []
    for i in range(61):
        for j in range(41):
            x, y = -3+i*.1, -2+j*.1
            points.extend(((x, y, 0.), (x, -2., j*.075), (-3., x*.65, j*.075)))
            if i < 20:
                points.append((1+i*.05, .7, j*.05))
    pcd = tmp_path / "cloud.pcd"
    if global_mode == "history":
        (tmp_path / "localization_keyframes.csv").write_text(
            "id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,.35,.1,.65,0,0,0,1\n")
    pcd.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                   f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n" +
                   "".join(f"{x} {y} {z}\n" for x,y,z in points))
    executable = Path(get_package_prefix("dog_slam")) / "lib/dog_slam/map_localizer"
    config = Path(get_package_prefix("dog_slam")) / "share/dog_slam/config/localization.yaml"
    log = (tmp_path / "node.log").open("w+")
    process = subprocess.Popen([str(executable), "--ros-args", "--params-file", str(config),
        # Sparse registration must still accumulate confirmations while scan
        # input is continuous. It must not bypass the output validity timeout.
        "-p", f"map_path:={pcd}", "-p", f"registration_rate:={registration_rate}", "-p", "data_timeout:=1.0",
        "-p", "recovery_interval:=0.2",
        "-p", f"global_relocalization.enabled:={str(bool(global_mode)).lower()}",
        "-p", f"global_relocalization.on_startup:={str(global_mode == 'history').lower()}",
        # Allow candidate verification on the ARM test host. The native timeout
        # test separately verifies that an insufficient budget cannot succeed.
        "-p", "global_relocalization.time_budget:=5.0",
        "-p", "max_result_age:=6.0"], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("test_map_localization")
    status, poses, transforms = [], [], []
    node.create_subscription(String, "/map_localizer/status", lambda m: status.append(json.loads(m.data)), 10)
    from geometry_msgs.msg import PoseStamped
    node.create_subscription(PoseStamped, "/map_localizer/pose", poses.append, 10)
    node.create_subscription(TFMessage, "/tf", lambda m: transforms.extend(
        t for t in m.transforms if t.header.frame_id == "map"), 10)
    tf_pub = node.create_publisher(TFMessage, "/tf", 10)
    static_pub = node.create_publisher(TFMessage, "/tf_static", QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cloud_pub = node.create_publisher(PointCloud2, "/lio/cloud_deskewed", 10)
    seed_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    # Body at (.2,.1,.65), sensor mounting translated .15 in body X.
    body = (.2, .1, .65)
    payload = b"".join(struct.pack("fff", x-body[0]-.15, y-body[1], z-body[2]) for x,y,z in points)
    mounting = TransformStamped()
    mounting.header.frame_id = "base_link"; mounting.child_frame_id = "livox_frame"
    mounting.transform.translation.x = .15; mounting.transform.rotation.w = 1.
    static_pub.publish(TFMessage(transforms=[mounting]))

    def step(send=True):
        stamp = node.get_clock().now().to_msg()
        odom = TransformStamped(); odom.header.stamp = stamp
        odom.header.frame_id = "odom"; odom.child_frame_id = "base_link"
        odom.transform.translation.x = 2.; odom.transform.translation.z = 1.
        odom.transform.rotation.w = 1.; tf_pub.publish(TFMessage(transforms=[odom]))
        if send:
            msg = PointCloud2(); msg.header.stamp = stamp; msg.header.frame_id = "livox_frame"
            msg.height = 1; msg.width = len(points); msg.point_step = 12; msg.row_step = msg.width*12
            msg.fields = [PointField(name=n, offset=i*4, datatype=7, count=1) for i,n in enumerate("xyz")]
            msg.data = payload; msg.is_dense = True; cloud_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.03)
        time.sleep(.025)

    def until(predicate, seconds=12., send=True):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, (log.seek(0), log.read())
            step(send)
            if predicate(): return True
        return False

    try:
        assert until(lambda: status and status[-1]["map_ready"])
        if global_mode != "history":
            assert not transforms  # No fake identity map TF before initialization.
        malformed = PointCloud2(); malformed.header.frame_id = "livox_frame"
        malformed.header.stamp.sec = -1
        cloud_pub.publish(malformed)  # Reject without throwing out of the ROS callback.
        global_client=node.create_client(Trigger,"/map_localizer/global_relocalize")
        assert global_client.wait_for_service(timeout_sec=3)
        if global_mode != "history":
            requested=global_client.call_async(Trigger.Request());assert until(requested.done)
            assert requested.result().success == global_mode
        if not global_mode:
            seed = PoseWithCovarianceStamped(); seed.header.frame_id = "map"
            seed.pose.pose.position.x, seed.pose.pose.position.y, seed.pose.pose.position.z = body
            seed.pose.pose.orientation.w = 1.; seed_pub.publish(seed)
        assert until(lambda: status and status[-1]["valid"] and poses), json.dumps(status[-1])
        assert status[-1]["input_fresh"]
        assert isinstance(status[-1]["registration_running"],bool)
        if global_mode:
            assert status[-1]["global_timing_state"]=="succeeded"
            assert status[-1]["global_relocalization_elapsed_ms"]>0
        assert status[-1]["localization_source"] == ("historical_keyframes" if global_mode == "history" else
            "global_map_search" if global_mode else "initial_pose")
        p = poses[-1].pose.position
        # Independently voxelized source/target have different centroid phases;
        # allow half a voxel diagonal in 3D, not a claimed real accuracy.
        tolerance=.15*math.sqrt(3)/2
        assert math.dist((p.x,p.y,p.z), body) < tolerance
        assert until(lambda: bool(transforms))
        t = transforms[-1].transform.translation
        assert math.dist((t.x,t.y,t.z), (-1.8,.1,-.35)) < tolerance
        # Stop clouds while odometry continues: validity and TF refresh must stop.
        assert until(lambda: not status[-1]["valid"] and status[-1]["state"] == "lost" and
                     not status[-1]["input_fresh"], send=False)
        until(lambda: False, seconds=.4, send=False)
        count = len(transforms)
        until(lambda: False, seconds=.4, send=False)
        assert len(transforms) == count
        assert until(lambda: status[-1]["valid"]), status[-1]
        reset = node.create_client(Trigger, "/map_localizer/reset")
        assert reset.wait_for_service(timeout_sec=3)
        future = reset.call_async(Trigger.Request())
        if global_mode == "history":
            assert until(lambda: future.done() and not status[-1]["valid"],send=False)
            assert until(lambda: future.done() and status[-1]["valid"] and
                status[-1]["localization_source"] == "historical_keyframes")
        else:
            assert until(lambda: future.done() and status[-1]["state"] == "waiting_initial_pose")
            assert not status[-1]["valid"]
        # Invalid map reload clears localization rather than retaining a valid old correction.
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParameters
        client = node.create_client(SetParameters, "/map_localizer/set_parameters")
        assert client.wait_for_service(timeout_sec=3)
        req = SetParameters.Request(parameters=[Parameter("map_path", value="/missing.pcd").to_parameter_msg()])
        changed = client.call_async(req); assert until(changed.done)
        reload = node.create_client(Trigger, "/map_localizer/reload_map")
        assert reload.wait_for_service(timeout_sec=3)
        loaded = reload.call_async(Trigger.Request())
        assert until(lambda: loaded.done() and status[-1]["state"] == "map_error")
        assert not status[-1]["map_ready"] and not status[-1]["valid"]
    finally:
        node.destroy_node(); rclpy.shutdown(); process.terminate()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        log.close()
