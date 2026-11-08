"""Exercise the built node over DDS; no sensor or robot connection required."""

import math
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg
import pytest
import rclpy
from tf2_msgs.msg import TFMessage


def spin_until(node, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_body_tf_and_invalid_input_rejection(monkeypatch, tmp_path):
    # Isolate synthetic odometry from any robot processes on the host.
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    executable = Path(get_package_prefix("dog_slam")) / "lib/dog_slam/lio_tf_adapter"
    log = (tmp_path / "adapter.log").open("w+")
    process = subprocess.Popen([
        str(executable), "--ros-args",
        "-p", "base_to_imu_translation:=[1.0, 0.0, 0.0]",
        "-p", "base_to_imu_rotation:=[0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0]",
    ], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("test_lio_tf_adapter")
    messages = []
    subscription = node.create_subscription(TFMessage, "/tf", messages.append, 10)
    publisher = node.create_publisher(Odometry, "/lio/odom", 10)
    try:
        assert spin_until(node, lambda: publisher.get_subscription_count() == 1 and
                          node.count_publishers("/tf") == 1), "DDS discovery failed"
        assert process.poll() is None
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "livox_imu"
        msg.header.stamp.sec = 123
        msg.header.stamp.nanosec = 456000000
        msg.pose.pose.position.x = 4.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.position.z = 3.0
        msg.pose.pose.orientation.y = math.sqrt(0.5)
        msg.pose.pose.orientation.w = math.sqrt(0.5)
        # Identity body rotation, body position (3,2,3), sensor pitch +90 degrees.
        publisher.publish(msg)
        assert spin_until(node, lambda: bool(messages))
        transform = messages.pop().transforms[0]
        assert transform.header == msg.header
        assert transform.child_frame_id == "base_link"
        t = transform.transform.translation
        assert (t.x, t.y, t.z) == pytest.approx((3., 2., 3.), abs=1e-10)
        q = transform.transform.rotation
        assert (q.x, q.y, q.z, abs(q.w)) == pytest.approx((0., 0., 0., 1.), abs=1e-10)
        for invalid in ("frame", "quaternion", "nonfinite", "timestamp"):
            msg.child_frame_id = "wrong_imu" if invalid == "frame" else "livox_imu"
            msg.pose.pose.orientation.w = 0.0 if invalid == "quaternion" else math.sqrt(0.5)
            msg.pose.pose.position.x = float("nan") if invalid == "nonfinite" else 4.0
            msg.header.stamp.sec = 0 if invalid == "timestamp" else 124
            msg.header.stamp.nanosec = 0
            publisher.publish(msg)
            assert not spin_until(node, lambda: bool(messages), timeout=0.3), invalid
        assert process.poll() is None
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()


def test_bridge_sensor_frames_timestamps_and_units(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    executable = Path(get_package_prefix("interfaces")) / "lib/interfaces/livox_to_pointcloud2_node"
    log = (tmp_path / "bridge.log").open("w+")
    process = subprocess.Popen([
        str(executable), "--ros-args", "-p", "frame_id:=livox_frame",
        "-p", "imu_frame_id:=livox_imu", "-p", "qos.reliability:=reliable",
    ], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("test_slam_bridge")
    imu_messages, clouds = [], []
    node.create_subscription(Imu, "/interfaces/imu", imu_messages.append, 10)
    node.create_subscription(PointCloud2, "/interfaces/lidar", clouds.append, 10)
    imu_pub = node.create_publisher(Imu, "/livox/imu", 10)
    lidar_pub = node.create_publisher(CustomMsg, "/livox/lidar", 10)
    try:
        assert spin_until(node, lambda: imu_pub.get_subscription_count() == 1 and
                          lidar_pub.get_subscription_count() == 1 and
                          node.count_publishers("/interfaces/imu") == 1 and
                          node.count_publishers("/interfaces/lidar") == 1)
        imu = Imu()
        imu.header.frame_id = "livox_frame"
        imu.header.stamp.sec = 123
        imu.header.stamp.nanosec = 456
        imu.linear_acceleration.x = 1.0
        imu.angular_velocity.y = 0.5
        cloud = CustomMsg()
        cloud.header = imu.header
        cloud.timebase = 123000000456
        imu_pub.publish(imu)
        lidar_pub.publish(cloud)
        assert spin_until(node, lambda: bool(imu_messages) and bool(clouds))
        assert imu_messages[0].header.frame_id == "livox_imu"
        assert clouds[0].header.frame_id == "livox_frame"
        assert imu_messages[0].header.stamp == imu.header.stamp
        assert clouds[0].header.stamp == cloud.header.stamp
        assert imu_messages[0].linear_acceleration.x == pytest.approx(9.80665)
        assert imu_messages[0].angular_velocity.y == 0.5
        assert process.poll() is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
