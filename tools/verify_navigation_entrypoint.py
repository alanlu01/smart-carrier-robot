"""Native installed launch + service observer + process transcripts; no hardware/goals."""
import json,os,resource,signal,subprocess,time,tempfile,sys,math
from pathlib import Path
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped,Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    rclpy.init();node=rclpy.create_node('codex_navigation_entrypoint_probe')
    bootstrap='--bootstrap' in sys.argv
    broadcaster=StaticTransformBroadcaster(node);transforms=[];health=[];commands=[];seeds=[];poses=[];states=[]
    pairs=[('odom','base_footprint'),('base_footprint','laser')]
    if not bootstrap:pairs.insert(0,('map','odom'))
    for parent,child in pairs:
        tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child
        tr.header.stamp=node.get_clock().now().to_msg();tr.transform.rotation.w=1.;transforms.append(tr)
    broadcaster.sendTransform(transforms)
    node.create_subscription(String,'/navigation/pipeline_health',lambda m:health.append(json.loads(m.data)),1)
    node.create_subscription(Twist,'/cmd_vel',commands.append,10)
    state_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped,'/initialpose',seeds.append,state_qos)
    node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',poses.append,state_qos)
    node.create_subscription(String,'/localization/state',lambda m:states.append(json.loads(m.data)),10)
    base=Path(os.environ['ROS_LOG_DIR']);base.mkdir(parents=True,exist_ok=True)
    root=Path(tempfile.mkdtemp(prefix='entrypoint_',dir=base))
    # ROS domain isolation does not isolate files: never touch the production pose cache.
    parameters=yaml.safe_load((Path(get_package_share_directory('smart_delivery_core'))/
                               'config/my_nav2_params.yaml').read_text())
    parameters['localization_manager']['ros__parameters']['pose_cache_path']=str(root/'pose_cache.json')
    parameters['localization_manager']['ros__parameters']['automatic_motion_recovery']=False
    if bootstrap:
        import numpy as np
        from PIL import Image
        share=Path(get_package_share_directory('smart_delivery_core'))
        map_path=share/'maps/3F_NEW.yaml';meta=yaml.safe_load(map_path.read_text())
        pixels=np.array(Image.open(map_path.parent/meta['image']).convert('L'))
        origin=meta['origin'];resolution=meta['resolution']
        distances=np.arange(.15,12.,.025);ranges=[]
        for angle in np.linspace(-math.pi,math.pi,720,endpoint=False):
            xs=np.floor((distances*np.cos(angle)-origin[0])/resolution).astype(int)
            ys=pixels.shape[0]-1-np.floor((distances*np.sin(angle)-origin[1])/resolution).astype(int)
            valid=(xs>=0)&(ys>=0)&(xs<pixels.shape[1])&(ys<pixels.shape[0])
            hits=np.where(valid & (pixels[np.clip(ys,0,pixels.shape[0]-1),np.clip(xs,0,pixels.shape[1]-1)]<90))[0]
            ranges.append(float(distances[hits[0]]) if len(hits) else float('inf'))
        scans=[node.create_publisher(LaserScan,name,qos_profile_sensor_data) for name in ('/scan','/scan_filtered')]
        odom=node.create_publisher(Odometry,'/odom',10)
        wheels=node.create_publisher(String,'/chassis/wheel_sample',qos_profile_sensor_data)
        sequence=0
        def sensors():
            nonlocal sequence
            sequence+=1;stamp=node.get_clock().now()
            scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp=stamp.to_msg()
            scan.angle_min=-math.pi;scan.angle_max=math.pi-2*math.pi/720;scan.angle_increment=2*math.pi/720
            scan.range_min=.15;scan.range_max=12.;scan.ranges=ranges
            for publisher in scans:publisher.publish(scan)
            message=Odometry();message.header.frame_id='odom';message.child_frame_id='base_footprint'
            message.header.stamp=stamp.to_msg();message.pose.pose.orientation.w=1.;odom.publish(message)
            if sequence%5==0:
                wheels.publish(String(data=json.dumps(dict(schema_version=1,source='serial_rx',session='isolated-test',
                    sequence=sequence,sample_ros_stamp_ns=stamp.nanoseconds,wheels=dict.fromkeys('ABCD',0.)))))
        node.create_timer(.1,sensors)
    test_params=root/'nav_params.yaml'
    test_params.write_text(yaml.safe_dump(parameters,allow_unicode=True))
    logpath=root/'entrypoint.log'
    try:
        with logpath.open('w') as log:
            child=subprocess.Popen(['ros2','launch','smart_delivery_core','navigation.launch.py',
                'auto_initialize:='+str(bootstrap).lower(),'params_file:='+str(test_params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                env=dict(os.environ,ROS_LOG_DIR=str(root)),
                preexec_fn=lambda:resource.setrlimit(resource.RLIMIT_CORE,(0,0)))
            try:
                end=time.monotonic()+50.
                while time.monotonic()<end and child.poll() is None:
                    rclpy.spin_once(node,timeout_sec=.05)
                    localized=bool(states and states[-1].get('state')=='LOCALIZED')
                    if health and health[-1]['state']=='ready' and (not bootstrap or poses and seeds and localized):break
                ready=bool(health and health[-1]['state']=='ready')
                child.send_signal(signal.SIGINT)  # Let launch forward exactly once.
                try:child.wait(timeout=32.)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=4.)
            finally:
                try:os.killpg(child.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                if child.poll() is None:child.wait(timeout=4.)
        text=logpath.read_text(errors='replace')
        exits=list(root.glob('*/nav2_*.exit.json'))
        parsed=list({p['pid']:p for p in (json.loads(x.read_text()) for x in exits)}.values())
        assert ready,'navigation observer never reported ready'
        if bootstrap:assert seeds and poses, f'AMCL bootstrap failed: seeds={len(seeds)}, poses={len(poses)}, states={states[-1:]}'
        if bootstrap:assert localized, f'Manager did not verify startup: {states[-1:]}'
        assert 'SIGSEGV' not in text and 'exit code -6' not in text
        assert parsed and all(p['returncode']==0 for p in parsed),parsed
        assert len(parsed)==1, 'Expected exactly one composed navigation container'
        assert all(all(v==0. for v in (m.linear.x,m.linear.y,m.angular.z)) for m in commands)
        print(json.dumps(dict(actual_entrypoint=True,navigation_health_ready=True,
              nav_process_exits=len(parsed),all_child_exits_clean=True,goals=0,nonzero_commands=0,
              localization_verified=bootstrap and localized,
              amcl_bootstrap_without_external_map_tf=bootstrap and bool(seeds and poses),
              weak_remote_network_validated=False,log=str(logpath))))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
