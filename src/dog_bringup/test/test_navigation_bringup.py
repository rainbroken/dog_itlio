"""Actual bringup + localizer + replanner over synthetic DDS, without hardware."""
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time
import urllib.error
import urllib.request
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Path as PathMsg, Odometry
from dog_interfaces.msg import TerrainRoute
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def fixture(root):
    executable=Path(__file__).resolve().parents[3]/"build/dog_navigation/test_local_planner"
    subprocess.run([str(executable),"--gtest_filter=LocalPlanner.ProjectFixture"],check=True,
                   env=dict(os.environ,DOG_NAV_TEST_MAP_ROOT=str(root),DOG_NAV_TEST_ROOM="1"))


def start(root, start_lio, bridge=False):
    # This test exercises manual initialization and the disabled-global API.
    # Production defaults now enable automatic historical relocalization.
    config=yaml.safe_load((Path(get_package_share_directory("dog_slam"))/"config/localization.yaml").read_text())
    config["map_localizer"]["ros__parameters"]["global_relocalization"]["enabled"]=False
    config["map_localizer"]["ros__parameters"]["global_relocalization"]["body_height"]=.65
    localization_config=root/"manual-localization.yaml"
    localization_config.write_text(yaml.safe_dump(config))
    # Synthetic fixtures deliberately use a .65 m body, independent of hardware.
    nav_share=Path(get_package_share_directory('dog_navigation'))
    extra=[]
    for argument,name,nodes in [('navigation_config','navigation',('local_navigation',)),
                                ('execution_config','execution',('navigation_executor','velocity_guard'))]:
        document=yaml.safe_load((nav_share/f'config/{name}.yaml').read_text())
        for node in nodes:document[node]['ros__parameters']['base_ground_offset']=.65
        path=root/(name+'-fixture.yaml');path.write_text(yaml.safe_dump(document));extra.append(f'{argument}:={path}')
    with socket.socket() as s:
        s.bind(("127.0.0.1",0));port=s.getsockname()[1]
    log_path=root/"bringup.log";log=log_path.open("w")
    p=subprocess.Popen(["ros2","launch","dog_bringup","dog_bringup.launch.py",
        # This fixture previews /goal_pose; do not inherit the user's hardware
        # execute switch from bringup.yaml.
        "mode:=navigation","execute:=false","lidar:=false","driver:=false",f"bridge:={str(bridge).lower()}",
        f"start_lio:={str(start_lio).lower()}",f"port:={port}",f"map_root:={root}",f"motion_config:={root}/motion.json", "map_id:=scene",
        f"localization_config:={localization_config}",*extra],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    return p,port,log,log_path


def stop(p,log):
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
        raise AssertionError("Bringup shutdown timeout")
    finally:log.close()


def api(port,path,body=None):
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request=urllib.request.Request(f"http://127.0.0.1:{port}{path}",
        data=None if body is None else json.dumps(body).encode(),headers={"Content-Type":"application/json"})
    with opener.open(request,timeout=2) as response:return json.load(response)


def test_navigation_bringup_localizes_replans_and_invalidates(monkeypatch,tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID",str(205+os.getpid()%15));monkeypatch.setenv("ROS_LOCALHOST_ONLY","1")
    fixture(tmp_path);process,port,log,log_path=start(tmp_path,False)
    # Source is a binary XYZ project saved by the real C++ map store.
    header,data=(tmp_path/"scene/cloud.pcd").read_bytes().split(b"DATA binary\n",1)
    assert b"FIELDS x y z" in header
    static=b"".join(struct.pack("fff",x+1.35,y,z-.65) for x,y,z in struct.iter_unpack("fff",data))
    rclpy.init();node=rclpy.create_node("bringup_pipeline_test")
    localized=[];planning=[];paths=[];map_tf=[];velocities=[];routes=[]
    node.create_subscription(TerrainRoute,"/local_navigation/terrain_route",routes.append,20)
    node.create_subscription(String,"/map_localizer/status",lambda m:localized.append(json.loads(m.data)),20)
    node.create_subscription(String,"/local_navigation/status",lambda m:planning.append(json.loads(m.data)),20)
    node.create_subscription(PathMsg,"/local_navigation/global_path",paths.append,10)
    node.create_subscription(TFMessage,"/tf",lambda m:map_tf.extend(t for t in m.transforms if t.header.frame_id=="map"),20)
    node.create_subscription(Twist,"/cmd_vel",velocities.append,10)
    tf=node.create_publisher(TFMessage,"/tf",10)
    scan=node.create_publisher(PointCloud2,"/lio/cloud_deskewed",10)
    goal_pub=node.create_publisher(PoseStamped,"/goal_pose",10)
    lio_odom=node.create_publisher(Odometry,"/lio/odom",10)
    mode="clear"

    def step(send=True):
        stamp=node.get_clock().now().to_msg();transforms=[]
        for parent,child,x,z in (("odom","base_link",-1.5,.65),("base_link","livox_frame",.15,0.)):
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp
            t.transform.translation.x=x;t.transform.translation.z=z;t.transform.rotation.w=1.;transforms.append(t)
        tf.publish(TFMessage(transforms=transforms))
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id="odom";odom.child_frame_id="base_link"
        odom.pose.pose.position.x=-1.5;odom.pose.pose.position.z=.65;odom.pose.pose.orientation.w=1.;lio_odom.publish(odom)
        if send:
            novel=[]
            if mode=="obstacle":novel=[(0.,0.,z) for z in (.2,.4,.6,.8)]
            if mode=="wall":novel=[(0.,i*.08,z) for i in range(-18,19) for z in (.2,.4,.6,.8)]
            payload=static+b"".join(struct.pack("fff",x+1.35,y,z-.65) for x,y,z in novel)
            cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id="livox_frame"
            cloud.height=1;cloud.width=len(payload)//12;cloud.point_step=12;cloud.row_step=len(payload);cloud.data=payload
            cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate("xyz")]
            scan.publish(cloud)
        for _ in range(6):rclpy.spin_once(node,timeout_sec=.002)
        time.sleep(.04)

    def until(predicate,seconds=15,send=True):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            assert process.poll() is None,log_path.read_text()
            step(send)
            try:
                if predicate():return True
            except (OSError,urllib.error.URLError):pass
        return False

    try:
        assert until(lambda:localized and localized[-1]["map_ready"] and planning and
                     goal_pub.get_subscription_count()==1 and api(port,"/api/robot").get("localization",{}).get("map_ready"))
        assert not map_tf and not planning[-1]["valid"]
        assert not api(port,"/api/robot")["mapping"]["start_allowed"]
        try:api(port,"/api/slam/start",{})
        except urllib.error.HTTPError as error:assert error.code==400
        else:raise AssertionError("Navigation mode allowed a competing LIO mapping process")
        load=api(port,"/api/load",{"name":"scene"})
        assert until(lambda:api(port,f'/api/jobs/{load["job"]}')["state"]=="complete")
        metadata=api(port,"/api/map")
        initial={"map_id":"scene","revision":metadata["revision"],"ground_position":[-1.5,0,0],"yaw":0.,"body_height":.65}
        # Map readiness does not imply discovery of the synthetic odom TF in
        # the web bridge. Keep publishing while waiting for seed acceptance.
        assert until(lambda:api(port,"/api/localization/initialize",initial)["success"])
        try:api(port,"/api/localization/global_relocalize",initial)
        except urllib.error.HTTPError as error:assert error.code==400
        else:raise AssertionError("Web enabled global search despite disabled YAML flag")
        goal=PoseStamped();goal.header.frame_id="map";goal.pose.position.x=1.5;goal.pose.orientation.w=1.;goal_pub.publish(goal)
        assert until(lambda:localized[-1]["valid"] and planning[-1]["valid"] and paths and paths[-1].poses), (localized[-1],planning[-1])
        # A web preview must derive its start from fresh localization, even if
        # a client supplies a conflicting manual start alongside the flag.
        preview_result={}
        preview_errors=[]
        def live_preview():
            try:
                preview_result.update(api(port,"/api/plan",{"map_id":"scene","revision":metadata["revision"],
                    "use_robot_start":True,"start":[1000,1000,1000],"goal":[1.5,0,0]}))
            except urllib.error.HTTPError as error:
                preview_errors[:]=[error.read().decode()];return False
            return preview_result.get("start_source")=="localization"
        assert until(live_preview),preview_errors
        assert sum((a-b)**2 for a,b in zip(preview_result["points"][0],[-1.5,0,0]))<.25**2
        assert map_tf and node.count_publishers("/tf")==2
        # Refreshing the same route must not revoke an in-flight command.
        routes.clear()
        assert until(lambda:len(routes)>=6)
        stable_pairs=0
        for a,b in zip(routes,routes[1:]):
            if a.valid and b.valid and a.node_ids==b.node_ids and a.stairs==b.stairs and a.goal_stamp==b.goal_stamp:
                assert a.generation==b.generation
                stable_pairs+=1
        assert stable_pairs>=3
        mode="obstacle"
        assert until(lambda:planning[-1]["valid"] and planning[-1]["novel_obstacles"]>0),planning[-1]
        assert until(lambda:paths[-1].poses and all(p.pose.position.x**2+p.pose.position.y**2>.17 for p in paths[-1].poses))
        mode="wall"
        assert until(lambda:planning[-1]["state"]=="blocked_no_route"),planning[-1]
        assert until(lambda:not paths[-1].poses)
        start_index=len(planning)
        assert until(lambda:any(s["state"]=="scan_unavailable" for s in planning[start_index:]) and
                     not localized[-1]["valid"],seconds=6,send=False),(localized[-1],planning[-1])
        assert planning[-1]["stop_required"]
        assert not any(v.linear.x or v.linear.y or v.angular.z for v in velocities)
        assert "LIO started:" not in log_path.read_text()  # External LIO mode has a single synthetic owner.
    finally:
        node.destroy_node();rclpy.shutdown();stop(process,log)
    assert process.returncode==0,log_path.read_text()


def test_navigation_bringup_starts_real_lio_with_exclusive_map_tf(monkeypatch,tmp_path):
    # Do not discover a just-terminated previous test's parameter service.
    monkeypatch.setenv("ROS_DOMAIN_ID",str(225+os.getpid()%5));monkeypatch.setenv("ROS_LOCALHOST_ONLY","1")
    fixture(tmp_path);process,port,log,log_path=start(tmp_path,True,bridge=True)
    rclpy.init();node=rclpy.create_node("bringup_lio_probe");statuses=[]
    node.create_subscription(String,"/map_localizer/status",lambda m:statuses.append(json.loads(m.data)),10)
    try:
        client=node.create_client(GetParameters,"/itera_lio_node/get_parameters")
        assert client.wait_for_service(timeout_sec=12),log_path.read_text()
        names=["tf.publish_map_to_odom","tf.publish_odom_to_imu","tf.publish_imu_to_lidar","backend.enabled","debug.publish_deskewed"]
        # A freshly discovered Fast DDS service can receive a request before
        # discovery of its reply reader finishes (logged as send-response timeout).
        # Retry this read-only query within the original five-second budget.
        for _ in range(5):
            future=client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(node,future,timeout_sec=1)
            if future.done():break
            client.remove_pending_request(future)
        assert future.done()
        assert [v.bool_value for v in future.result().values]==[False,False,False,False,True]
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not(statuses and statuses[-1]["map_ready"]):rclpy.spin_once(node,timeout_sec=.1)
        assert statuses and statuses[-1]["map_ready"] and not statuses[-1]["valid"]
        # The localization child disables its own bridge; this must not disable
        # bringup's bridge or leak into sibling launch configurations.
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if node.count_publishers("/interfaces/lidar")==1 and node.count_publishers("/interfaces/imu")==1:
                break
        assert node.count_publishers("/interfaces/lidar")==1,log_path.read_text()
        assert node.count_publishers("/interfaces/imu")==1,log_path.read_text()
        assert node.get_node_names().count("livox_to_pointcloud2")==1
        deadline=time.monotonic()+8
        while True:
            try:assert not api(port,"/api/robot")["mapping"]["start_allowed"];break
            except (OSError,urllib.error.URLError):
                assert time.monotonic()<deadline;time.sleep(.1)
    finally:
        node.destroy_node();rclpy.shutdown();stop(process,log)
    assert process.returncode==0,log_path.read_text()


def test_missing_map_fails_whole_navigation_bringup(monkeypatch,tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID",str(205+os.getpid()%15));monkeypatch.setenv("ROS_LOCALHOST_ONLY","1")
    process,_,log,log_path=start(tmp_path,False)
    try:
        process.wait(timeout=12)
        assert process.returncode!=0,log_path.read_text()
        assert "local_navigation" in log_path.read_text() and "exited unexpectedly" in log_path.read_text()
        assert not (tmp_path/"scene").exists()
    finally:
        if process.poll() is None:stop(process,log)
        else:log.close()
