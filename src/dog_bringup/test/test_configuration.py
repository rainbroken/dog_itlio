"""Exercise YAML defaults and the workspace script without robot hardware."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import urllib.request

import pytest
import yaml


def test_script_loads_yaml_from_workspace(tmp_path):
    config = tmp_path / 'bringup.yaml'
    config.write_text(yaml.safe_dump({'bringup': {
        'driver': False, 'lidar': False, 'port': 0, 'host': '127.0.0.1',
        'map_root': str(tmp_path / 'maps'), 'slam_output': str(tmp_path / 'slam'),
    }}))
    script = Path(__file__).resolve().parents[3] / 'bringup.sh'
    log_path = tmp_path / 'launch.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(['bash', str(script), 'config:=' + str(config)],
            cwd=script.parent, env=dict(os.environ, ROS_DOMAIN_ID='193', ROS_LOCALHOST_ONLY='1'),
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                text = log_path.read_text()
                assert process.poll() is None, text
                address = re.search(r'http://127\.0\.0\.1:\d+', text)
                if address:
                    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                    with opener.open(address[0] + '/api/robot', timeout=3) as response:
                        status = json.load(response)
                    assert status['ros'] and not status['mapping']['owned']
                    assert 'livox_ros_driver2_node' not in text and 'dog_driver_node' not in text
                    assert str(config) in text
                    break
                time.sleep(.1)
            else:
                raise AssertionError(log_path.read_text())
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    assert process.returncode == 0, log_path.read_text()


def test_saved_motion_speeds_survive_bringup_restart(tmp_path):
    """Use the real launch/YAML wiring, without robot or lidar hardware."""
    settings = tmp_path / 'config' / 'motion.json'
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    expected = {'x': .53, 'y': .24, 'yaw': .78}
    for run in range(2):
        log_path = tmp_path / f'restart-{run}.log'
        with log_path.open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'dog_bringup', 'dog_bringup.launch.py',
                'lidar:=false', 'driver:=false', 'bridge:=false', 'host:=127.0.0.1', 'port:=0',
                f'map_root:={tmp_path / "maps"}', f'slam_output:={tmp_path / "slam"}',
                f'motion_config:={settings}'],
                env=dict(os.environ, ROS_DOMAIN_ID='193', ROS_LOCALHOST_ONLY='1'),
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    text = log_path.read_text()
                    assert process.poll() is None, text
                    address = re.search(r'http://127\.0\.0\.1:\d+', text)
                    if address:
                        url = address[0] + '/api/settings/motion'
                        if run == 0:
                            request = urllib.request.Request(url, data=json.dumps(expected).encode(),
                                headers={'Content-Type': 'application/json'})
                            with opener.open(request, timeout=3) as response:
                                saved = json.load(response)
                            assert all(saved[key] == value for key, value in expected.items())
                        with opener.open(url, timeout=3) as response:
                            restored = json.load(response)
                        assert all(restored[key] == value for key, value in expected.items())
                        assert all(json.loads(settings.read_text())[key] == value for key, value in expected.items())
                        break
                    time.sleep(.1)
                else:
                    raise AssertionError(log_path.read_text())
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        assert process.returncode == 0, log_path.read_text()


@pytest.mark.parametrize('settings', [{'driver': 'false'}, {'unknown_setting': True}])
def test_invalid_yaml_fails_before_starting_hardware(tmp_path, settings):
    config = tmp_path / 'invalid.yaml'
    config.write_text(yaml.safe_dump({'bringup': settings}))
    result = subprocess.run(['ros2', 'launch', 'dog_bringup', 'dog_bringup.launch.py',
        'config:=' + str(config)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=15)
    assert result.returncode != 0
    assert 'process started' not in result.stdout
    assert 'must be a YAML boolean' in result.stdout or 'Unknown bringup setting' in result.stdout
