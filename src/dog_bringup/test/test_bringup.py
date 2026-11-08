"""Exercise the installed entry point without connecting to robot/sensor hardware."""
import json
import os
import signal
import socket
import subprocess
import time
import urllib.error
import urllib.request
import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu


def test_mapping_is_deferred_until_web_request_and_shuts_down(tmp_path, monkeypatch):
    with socket.socket() as allocation:
        allocation.bind(('127.0.0.1', 0))
        port = allocation.getsockname()[1]
    environment = dict(os.environ, ROS_DOMAIN_ID='195', ROS_LOCALHOST_ONLY='1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '195')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('test_startup_sensor_input')
    lidar = node.create_publisher(CustomMsg, '/livox/lidar', 10)
    imu = node.create_publisher(Imu, '/livox/imu', 10)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    log_path = tmp_path / 'bringup.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'dog_bringup', 'dog_bringup.launch.py',
            'mode:=mapping',
            'lidar:=false', 'driver:=false', f'port:={port}',
            f'map_root:={tmp_path / "maps"}', f'slam_output:={tmp_path / "slam"}',
            f'motion_config:={tmp_path / "motion.json"}',
        ], env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def api(path, body=None):
            request = urllib.request.Request(
                f'http://127.0.0.1:{port}{path}',
                data=None if body is None else json.dumps(body).encode(),
                headers={'Content-Type': 'application/json'})
            with opener.open(request, timeout=3) as response:
                return json.load(response)

        def wait(predicate):
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                stamp = node.get_clock().now()
                cloud = CustomMsg(); cloud.header.stamp = stamp.to_msg()
                cloud.header.frame_id = 'livox_frame'; cloud.timebase = stamp.nanoseconds
                cloud.point_num = 1; cloud.points = [CustomPoint(x=2., y=.5, z=.1)]
                lidar.publish(cloud)
                inertial = Imu(); inertial.header.stamp = stamp.to_msg()
                inertial.header.frame_id = 'livox_frame'; inertial.linear_acceleration.z = 1.
                imu.publish(inertial)
                rclpy.spin_once(node, timeout_sec=.01)
                try:
                    if predicate():
                        return
                except (OSError, urllib.error.URLError):
                    pass
                time.sleep(.1)
            raise AssertionError(log_path.read_text())

        try:
            wait(lambda: api('/api/robot')['ros'])
            initial = api('/api/robot')
            assert not initial['mapping']['owned']
            assert not initial['mapping']['save_available']
            assert 'lio' not in initial['streams']
            # Keep observing after startup, rather than relying on one early sample.
            time.sleep(.5)
            assert not api('/api/robot')['mapping']['save_available']
            assert 'LIO started:' not in log_path.read_text()
            wait(lambda: all(api('/api/robot')['streams'].get(key, {}).get('fresh')
                             for key in ('lidar', 'imu')))
            assert log_path.read_text().count('Bridging lidar') == 1
            assert api('/api/slam/start', {})['success']
            wait(lambda: api('/api/robot')['mapping']['save_available'])
            assert api('/api/robot')['mapping']['owned']
            assert 'LIO started:' in log_path.read_text()
            assert log_path.read_text().count('Bridging lidar') == 1
            assert not api('/api/robot')['navigation_ready']
            assert api('/api/slam/stop', {})['success']
            wait(lambda: not api('/api/robot')['mapping']['owned'])
            assert api('/api/robot')['mapping']['exit_code'] == 0
            wait(lambda: all(api('/api/robot')['streams'].get(key, {}).get('fresh')
                             for key in ('lidar', 'imu')))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
                raise AssertionError('Bringup did not shut down cleanly: ' + log_path.read_text())
            finally:
                node.destroy_node(); rclpy.shutdown()
    assert process.returncode == 0, log_path.read_text()
