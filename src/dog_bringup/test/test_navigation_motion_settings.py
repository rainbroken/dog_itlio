"""Isolated host/API/startup profile integration; no robot driver or motion task."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request


def test_navigation_motion_profile_requires_restart_and_reaches_both_nodes(monkeypatch,tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID',str(120+os.getpid()%70))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    workspace=Path(__file__).resolve().parents[3]
    subprocess.run([str(workspace/'build/dog_navigation/test_local_planner'),'--gtest_filter=LocalPlanner.ProjectFixture'],
                   env=dict(os.environ,DOG_NAV_TEST_MAP_ROOT=str(tmp_path)),check=True,capture_output=True)
    with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
    processes=[];logs=[]
    def start(package,executable,args):
        log=(tmp_path/(executable+'-'+str(len(logs))+'.log')).open('w');logs.append(log)
        p=subprocess.Popen([str(workspace/f'install/{package}/lib/{package}/{executable}'),*args],stdout=log,stderr=subprocess.STDOUT)
        processes.append(p);return p
    def nodes():
        return [start('dog_navigation',exe,['--ros-args','-p',f'map_root:={tmp_path}','-p','map_id:=scene',
                '--params-file',str(workspace/'src/dog_navigation/config/local_planner.yaml')])
                for exe in ('navigation_executor','velocity_guard')]
    def request(path,body=None):
        data=None if body is None else json.dumps(body).encode()
        with urllib.request.urlopen(urllib.request.Request(f'http://127.0.0.1:{port}'+path,data=data,
                headers={'Content-Type':'application/json'}),timeout=3) as response:return json.load(response)
    def wait(check):
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            try:
                value=check()
                if value:return value
            except (OSError,ValueError):pass
            time.sleep(.1)
        raise AssertionError('Isolated navigation profile did not become ready')
    try:
        running=nodes()
        start('dog_web','planning_preview',['--host','127.0.0.1','--port',str(port),'--map-root',str(tmp_path),'--ros','true'])
        first=wait(lambda: (lambda j:j if j['active'] is not None else None)(request('/api/settings/navigation')))
        desired=dict(first['saved'],walk_speed=.4,acceleration=.10,yaw_acceleration=.30)
        saved=request('/api/settings/navigation',desired)
        assert saved['restart_required'] and saved['active']==first['active']
        for p in running:p.send_signal(signal.SIGINT);p.wait(timeout=5)
        nodes()
        effective=wait(lambda: (lambda j:j if j['active']==desired else None)(request('/api/settings/navigation')))
        assert not effective['restart_required']
        state=request('/api/robot')['navigation']
        assert state['enabled'] is False and state['guard_auto_enabled'] is False
        # Inspect the independent guard's actual sampled startup values.
        import rclpy
        from std_msgs.msg import String
        rclpy.init();node=rclpy.create_node('navigation_profile_check');guard=[]
        from rclpy.qos import QoSProfile,DurabilityPolicy
        node.create_subscription(String,'/velocity_guard/status',lambda msg:guard.append(json.loads(msg.data)),
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline=time.monotonic()+5
        while not guard and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        node.destroy_node();rclpy.shutdown()
        assert guard and guard[-1]['braking_deceleration']==.10
        assert guard[-1]['yaw_braking_deceleration']==.30
        assert guard[-1]['max_linear_speed']==desired['walk_speed']
        assert guard[-1]['max_yaw_rate']==desired['yaw_rate']
    finally:
        for p in processes:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for log in logs:log.close()
