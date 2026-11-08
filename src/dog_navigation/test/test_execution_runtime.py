"""Isolated DDS ideal-plant execution and safety test; never connects a driver."""
import json
import math
import os
import signal
from pathlib import Path
import struct
import subprocess
import time
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from dog_interfaces.action import NavigateTerrain
from dog_interfaces.msg import RobotStatus
from dog_interfaces.srv import SetMotion
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_prefix


def test_configured_timer_frequencies(monkeypatch, tmp_path):
    """Check actual DDS publication rates from launch-loaded nondefault YAML."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(110+os.getpid()%80))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    prefix=Path(get_package_prefix('dog_navigation'))
    fixture=Path(__file__).resolve().parents[3]/'build/dog_navigation/test_local_planner'
    subprocess.run([str(fixture),'--gtest_filter=LocalPlanner.ProjectFixture'],
                   env=dict(os.environ,DOG_NAV_TEST_MAP_ROOT=str(tmp_path)),check=True)
    for executable,key in (('navigation_executor','controller_frequency'),
                           ('velocity_guard','guard_frequency')):
        for value in ('0.0', '-1.0', '5.0', '.nan', '.inf'):
            result=subprocess.run([str(prefix/'lib/dog_navigation'/executable),
                '--ros-args','-p',f'{key}:={value}'],capture_output=True,text=True,timeout=5)
            assert result.returncode != 0 and key in result.stderr, result.stderr
    files={
        'global_planner_config': {'local_navigation': {'planning_rate':8.0}},
        'local_planner_config': {'navigation_executor': {'controller_frequency':12.5}},
        'execution_config': {'velocity_guard': {'guard_frequency':80.0}},
    }
    args=['ros2','launch','dog_navigation','execution.launch.py',
          f'map_root:={tmp_path}','map_id:=scene']
    for argument,nodes in files.items():
        config=tmp_path/(argument+'.yaml')
        config.write_text(yaml.safe_dump({name:{'ros__parameters':params}
                                        for name,params in nodes.items()}))
        args.append(f'{argument}:={config}')
    with (tmp_path/'frequency-launch.log').open('w+') as log:
        process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        rclpy.init();node=rclpy.create_node('frequency_fixture')
        samples={name:[] for name in ('local_navigation','navigation_executor','velocity_guard')}
        for name in samples:
            node.create_subscription(String,f'/{name}/status',
                lambda m,key=name:samples[key].append(time.monotonic()),100)
        try:
            deadline=time.monotonic()+10
            while not all(samples.values()) and time.monotonic()<deadline:
                assert process.poll() is None
                rclpy.spin_once(node,timeout_sec=.02)
            assert all(samples.values())
            start=time.monotonic()
            while time.monotonic()-start<2.0:
                assert process.poll() is None
                rclpy.spin_once(node,timeout_sec=.01)
            elapsed=time.monotonic()-start
            for name,expected in (('local_navigation',8.),('navigation_executor',12.5),('velocity_guard',80.)):
                measured=sum(t>=start for t in samples[name])/elapsed
                assert expected*.75 < measured < expected*1.25, (name,measured,expected)
        finally:
            node.destroy_node();rclpy.shutdown();process.send_signal(signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)


def test_execution_and_fail_closed_inputs(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(110+os.getpid()%80))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    prefix=Path(get_package_prefix('dog_navigation'))
    fixture=Path(__file__).resolve().parents[3]/'build/dog_navigation/test_local_planner'
    subprocess.run([str(fixture),'--gtest_filter=LocalPlanner.ProjectFixture'],
                   env=dict(os.environ,DOG_NAV_TEST_MAP_ROOT=str(tmp_path),DOG_NAV_TEST_RECT='1'),check=True)
    processes=[];logs=[]
    for executable,params in (
        ('local_navigation',{'goal_topic':'/navigation/planner_goal','processing_budget':'2.0'}),
        ('navigation_executor',{'enabled':'true'}),
        ('velocity_guard',{'allow_autonomous':'true','calibration_confirmed':'true'})):
        log=(tmp_path/(executable+'.log')).open('w+');logs.append(log)
        args=[str(prefix/'lib/dog_navigation'/executable),'--ros-args',
              '-p',f'map_root:={tmp_path}','-p','map_id:=scene']
        for name in ('global_planner', 'local_planner', 'decision', 'navigation', 'execution'):
            args += ['--params-file', str(prefix/f'share/dog_navigation/config/{name}.yaml')]
        # Named-node YAML wins over CLI wildcard overrides in Humble. Match the
        # launch Node dictionary's final named-node parameter file instead.
        overrides=tmp_path/(executable+'-overrides.yaml')
        overrides.write_text(yaml.safe_dump({executable: {'ros__parameters': {
            key: yaml.safe_load(value) for key,value in params.items()}}}))
        args+=['--params-file',str(overrides)]
        processes.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT))
    rclpy.init();node=rclpy.create_node('execution_fixture')
    tf=node.create_publisher(TFMessage,'/tf',10)
    status=node.create_publisher(RobotStatus,'/dog/status',10)
    odom=node.create_publisher(Odometry,'/dog/odom',10)
    loc=node.create_publisher(String,'/map_localizer/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    scan=node.create_publisher(PointCloud2,'/lio/cloud_deskewed',10)
    manual=node.create_publisher(Twist,'/cmd_vel',10)
    velocity=Twist();states=[];guards=[];x=-1.5;y=0.;yaw=0.;located=True;motion=0;walk_requests=[];send_scan=True
    def walk(request,response):
        nonlocal motion
        assert request.motion==SetMotion.Request.WALK and request.confirm_restricted_motion
        walk_requests.append(request.motion);motion=6;response.accepted=True;return response
    node.create_service(SetMotion,'/dog/set_motion',walk)
    def command(m):
        nonlocal velocity
        velocity=m
    node.create_subscription(Twist,'/dog/safe_cmd_vel',command,10)
    node.create_subscription(String,'/navigation_executor/status',lambda m:states.append(json.loads(m.data)),10)
    node.create_subscription(String,'/velocity_guard/status',lambda m:guards.append(json.loads(m.data)),10)
    client=ActionClient(node,NavigateTerrain,'/navigate_terrain')
    last=time.monotonic()

    def step():
        nonlocal x,y,yaw,last
        now=time.monotonic();dt=min(.06,now-last);last=now
        x+=velocity.linear.x*math.cos(yaw)*dt;y+=velocity.linear.x*math.sin(yaw)*dt;yaw+=velocity.angular.z*dt
        stamp=node.get_clock().now().to_msg();transforms=[]
        for parent,child,px,py,pz,angle in (
            ('map','odom',0.,0.,0.,0.),('odom','base_link',x,y,.65,yaw),
            ('base_link','livox_frame',0.,0.,0.,0.)):
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp
            t.transform.translation.x=px;t.transform.translation.y=py;t.transform.translation.z=pz
            t.transform.rotation.z=math.sin(angle/2);t.transform.rotation.w=math.cos(angle/2);transforms.append(t)
        tf.publish(TFMessage(transforms=transforms))
        s=RobotStatus();s.header.stamp=stamp;s.connected=True;s.state_fresh=True;s.current_motion_state=motion;s.current_gait=0;status.publish(s)
        o=Odometry();o.header.stamp=stamp;o.twist.twist=velocity;odom.publish(o)
        loc.publish(String(data=json.dumps({'valid':located,'map_ready':True,'map_path':str(tmp_path/'scene')})))
        points=[(-1.8,-1.,0.),(-1.,1.,0.),(0.,-1.,0.),(1.,1.,0.)]
        data=b''.join(struct.pack('fff',(px-x)*math.cos(yaw)+(py-y)*math.sin(yaw),
                     -(px-x)*math.sin(yaw)+(py-y)*math.cos(yaw),pz-.65) for px,py,pz in points)
        cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id='livox_frame';cloud.height=1;cloud.width=len(points)
        cloud.point_step=12;cloud.row_step=len(data);cloud.data=data
        cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
        if send_scan:scan.publish(cloud)
        rclpy.spin_once(node,timeout_sec=.01);time.sleep(.02)

    def until(predicate,seconds=8):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes), 'Node exited; inspect '+str(tmp_path)
            step()
            if predicate():return True
        return False

    def service(name,kind=Trigger,value=None):
        c=node.create_client(kind,name);assert until(c.service_is_ready)
        req=kind.Request()
        if value is not None:req.data=value
        future=c.call_async(req);assert until(future.done);assert future.result().success

    def start(goal_x):
        goal=NavigateTerrain.Goal();goal.goal.header.frame_id='map';goal.goal.pose.position.x=goal_x;goal.goal.pose.orientation.w=1.
        future=client.send_goal_async(goal);assert until(future.done);handle=future.result();assert handle.accepted
        return handle

    try:
        assert until(lambda:states and guards and client.server_is_ready())
        handle=start(-.6)
        assert until(lambda:velocity.linear.x>.01), json.dumps({'executor':states[-1],'guard':guards[-1]},indent=2)
        service('/navigation_executor/pause');assert until(lambda:states[-1]['task_state']=='paused' and abs(velocity.linear.x)<1e-8)
        service('/navigation_executor/resume');assert until(lambda:velocity.linear.x>.01), (states[-1],guards[-1])
        located=False;assert until(lambda:states[-1]['task_state']=='relocalizing' and abs(velocity.linear.x)<1e-8)
        located=True;assert until(lambda:velocity.linear.x>.01), (states[-1],guards[-1])
        send_scan=False;assert until(lambda:states[-1]['task_state']=='waiting_route' and abs(velocity.linear.x)<1e-8)
        send_scan=True;assert until(lambda:velocity.linear.x>.01), (states[-1],guards[-1])
        result=handle.get_result_async();assert until(result.done,35), (x,states[-1],guards[-1])
        assert result.result().result.success
        assert walk_requests
        assert abs(x+.6)<.16 and abs(velocity.linear.x)<1e-8
        handle=start(.7);assert until(lambda:velocity.linear.x>.01)
        service('/navigation_executor/cancel')
        result=handle.get_result_async();assert until(result.done)
        assert not result.result().result.success
        assert until(lambda:abs(velocity.linear.x)<1e-8)
        handle=start(.7);assert until(lambda:velocity.linear.x>.01)
        manual.publish(Twist());assert until(lambda:states[-1]['task_state']=='aborted')
        result=handle.get_result_async();assert until(result.done);assert not result.result().result.success
        assert until(lambda:abs(velocity.linear.x)<1e-8)
        assert until(lambda:guards[-1]['reason']=='manual_takeover_requires_new_task')
        handle=start(.7);assert until(lambda:velocity.linear.x>.01)
        service('/velocity_guard/estop',SetBool,True)
        assert until(lambda:guards[-1]['estop'] and abs(velocity.linear.x)<1e-8)
        service('/velocity_guard/estop',SetBool,False)
        assert until(lambda:states[-1]['task_state']=='aborted')
        result=handle.get_result_async();assert until(result.done);assert not result.result().result.success
        # The revoked task cannot restart when the emergency latch is cleared.
        for _ in range(10):step();assert abs(velocity.linear.x)<1e-8
        handle=start(.7);assert until(lambda:velocity.linear.x>.01), json.dumps({'executor':states[-1],'guard':guards[-1],'x':x},indent=2)
        executor=processes.pop(1);executor.terminate();executor.wait(timeout=5)
        assert until(lambda:abs(velocity.linear.x)<1e-8 and guards[-1]['owner']=='none'
                     and guards[-1]['reason']=='autonomous_command_expired',1)
        if os.environ.get('DOG_NAV_PERF_RUNTIME_REPORT'):
            def summary(values):
                values=sorted(values)
                return {'count':len(values),'p50_ms':values[len(values)//2],
                        'p95_ms':values[math.ceil(.95*len(values))-1],'max_ms':values[-1]}
            executor_times=[s['processing_ms'] for s in states if 'processing_ms' in s and s['task_state']=='following']
            guard_times=[s['processing_ms'] for s in guards if s['owner']=='autonomous']
            assert executor_times and guard_times
            Path(os.environ['DOG_NAV_PERF_RUNTIME_REPORT']).write_text(json.dumps({
                'model':'DDS ideal velocity plant on synthetic rectangular floor; no driver',
                'executor_following':summary(executor_times),'guard_autonomous':summary(guard_times),
                'physical_robot_verified':False},indent=2)+'\n')
    finally:
        node.destroy_node();rclpy.shutdown()
        for p in processes:
            p.terminate()
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=5)
        for log in logs:log.close()
