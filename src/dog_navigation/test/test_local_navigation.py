"""Node-level synthetic inputs over DDS; no robot or velocity publisher."""
import json
import os
from pathlib import Path
import struct
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def test_live_replanning_and_invalidations(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100+os.getpid()%100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    prefix=Path(get_package_prefix("dog_navigation"))
    fixture=Path(__file__).resolve().parents[3]/"build/dog_navigation/test_local_planner"
    env=dict(os.environ, DOG_NAV_TEST_MAP_ROOT=str(tmp_path))
    subprocess.run([str(fixture),"--gtest_filter=LocalPlanner.ProjectFixture"],env=env,check=True)
    log=(tmp_path/"node.log").open("w+")
    process=subprocess.Popen([str(prefix/"lib/dog_navigation/local_navigation"),"--ros-args",
        "--params-file",str(prefix/"share/dog_navigation/config/navigation.yaml"),
        "-p",f"map_root:={tmp_path}","-p","map_id:=scene","-p","processing_budget:=2.0"],
        stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node("navigation_test")
    statuses=[];paths=[];local=[]
    node.create_subscription(String,"/local_navigation/status",lambda m:statuses.append(json.loads(m.data)),10)
    node.create_subscription(PathMsg,"/local_navigation/global_path",paths.append,10)
    node.create_subscription(PathMsg,"/local_navigation/local_path",local.append,10)
    tf=node.create_publisher(TFMessage,"/tf",10)
    scan=node.create_publisher(PointCloud2,"/lio/cloud_deskewed",10)
    localization=node.create_publisher(String,"/map_localizer/status",10)
    goal_pub=node.create_publisher(PoseStamped,"/goal_pose",10)
    mode="clear";located=True;same=True

    def step(send=True):
        stamp=node.get_clock().now().to_msg()
        transforms=[]
        for parent,child,x,z in (("map","odom",0.,0.),("odom","base_link",-1.5,.65),("base_link","livox_frame",0.,0.)):
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp
            t.transform.translation.x=x;t.transform.translation.z=z;t.transform.rotation.w=1.;transforms.append(t)
        tf.publish(TFMessage(transforms=transforms))
        status=String();status.data=json.dumps({"valid":located,"map_ready":True,
            "map_path":str(tmp_path/("scene" if same else "other"))});localization.publish(status)
        if send:
            points=[(x,y,0.) for x in (-1.8,-1.,0.,1.,1.8) for y in (-1.,0.,1.)]
            if mode=="obstacle":points.extend((0.,0.,z) for z in (.2,.4,.6,.8))
            if mode=="wall":points.extend((0.,i*.08,z) for i in range(-18,19) for z in (.2,.4,.6,.8))
            # Origin is (-1.5,0,.65); publish actual sensor-frame coordinates.
            data=b"".join(struct.pack("fff",x+1.5,y,z-.65) for x,y,z in points)
            cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id="livox_frame"
            cloud.height=1;cloud.width=len(points);cloud.point_step=12;cloud.row_step=len(data);cloud.data=data
            cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate("xyz")]
            scan.publish(cloud)
        rclpy.spin_once(node,timeout_sec=.02);time.sleep(.03)

    def until(predicate,seconds=10,send=True):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            assert process.poll() is None,(log.seek(0),log.read())
            step(send)
            if predicate():return True
        return False

    try:
        assert until(lambda:goal_pub.get_subscription_count()==1 and statuses)
        assert not statuses[-1]["valid"]
        goal=PoseStamped();goal.header.frame_id="map";goal.pose.position.x=1.5;goal.pose.orientation.w=1.
        goal_pub.publish(goal)
        assert until(lambda:statuses[-1]["valid"] and paths and paths[-1].poses),statuses[-1]
        mode="obstacle"
        assert until(lambda:statuses[-1]["valid"] and statuses[-1]["novel_obstacles"]>0),statuses[-1]
        # Wait for the matching path generation to arrive over its separate topic.
        assert until(lambda:paths[-1].poses and all(p.pose.position.x**2+p.pose.position.y**2>.17 for p in paths[-1].poses))
        assert local[-1].poses and statuses[-1]["planning_only"]
        mode="wall"
        assert until(lambda:statuses[-1]["state"]=="blocked_no_route"),statuses[-1]
        assert statuses[-1]["novel_obstacles"]>0
        assert until(lambda:paths and not paths[-1].poses and local and not local[-1].poses)
        assert statuses[-1]["stop_required"]
        located=False
        assert until(lambda:not statuses[-1]["valid"] and statuses[-1]["state"].startswith("localization"))
        located=True;same=False;start=len(statuses)
        assert until(lambda:any(s["state"]=="map_mismatch" for s in statuses[start:]))
        same=True
        assert until(lambda:statuses[-1]["state"]=="scan_unavailable",send=False),statuses[-1]
        cancel=node.create_client(Trigger,"/local_navigation/cancel");assert cancel.wait_for_service(timeout_sec=3)
        f=cancel.call_async(Trigger.Request());assert until(f.done)
        assert f.result().success
        mode="clear";goal.pose.position.x=-1.5;start=len(statuses);goal_pub.publish(goal)
        assert until(lambda:any(s["state"]=="goal_reached" for s in statuses[start:])),statuses[-1]
    finally:
        node.destroy_node();rclpy.shutdown();process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        log.close()
