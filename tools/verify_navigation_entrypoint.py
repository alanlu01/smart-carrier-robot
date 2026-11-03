"""Native installed launch + service observer + process transcripts; no hardware/goals."""
import json,os,resource,signal,subprocess,time,tempfile
from pathlib import Path
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped,Twist
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    rclpy.init();node=rclpy.create_node('codex_navigation_entrypoint_probe')
    broadcaster=StaticTransformBroadcaster(node);transforms=[];health=[];commands=[]
    for parent,child in (('map','odom'),('odom','base_footprint'),('base_footprint','laser')):
        tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child
        tr.header.stamp=node.get_clock().now().to_msg();tr.transform.rotation.w=1.;transforms.append(tr)
    broadcaster.sendTransform(transforms)
    node.create_subscription(String,'/navigation/pipeline_health',lambda m:health.append(json.loads(m.data)),1)
    node.create_subscription(Twist,'/cmd_vel',commands.append,10)
    base=Path(os.environ['ROS_LOG_DIR']);base.mkdir(parents=True,exist_ok=True)
    root=Path(tempfile.mkdtemp(prefix='entrypoint_',dir=base))
    # ROS domain isolation does not isolate files: never touch the production pose cache.
    parameters=yaml.safe_load((Path(get_package_share_directory('smart_delivery_core'))/
                               'config/my_nav2_params.yaml').read_text())
    parameters['localization_manager']['ros__parameters']['pose_cache_path']=str(root/'pose_cache.json')
    test_params=root/'nav_params.yaml'
    test_params.write_text(yaml.safe_dump(parameters,allow_unicode=True))
    logpath=root/'entrypoint.log'
    try:
        with logpath.open('w') as log:
            child=subprocess.Popen(['ros2','launch','smart_delivery_core','navigation.launch.py',
                'auto_initialize:=false','params_file:='+str(test_params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                env=dict(os.environ,ROS_LOG_DIR=str(root)),
                preexec_fn=lambda:resource.setrlimit(resource.RLIMIT_CORE,(0,0)))
            try:
                end=time.monotonic()+35.
                while time.monotonic()<end and child.poll() is None:
                    rclpy.spin_once(node,timeout_sec=.05)
                    if health and health[-1]['state']=='ready':break
                ready=bool(health and health[-1]['state']=='ready')
                child.send_signal(signal.SIGINT)  # Let launch forward exactly once.
                try:child.wait(timeout=12.)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=4.)
            finally:
                try:os.killpg(child.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                if child.poll() is None:child.wait(timeout=4.)
        text=logpath.read_text(errors='replace')
        exits=list(root.glob('*/nav2_*.exit.json'))
        parsed=list({p['pid']:p for p in (json.loads(x.read_text()) for x in exits)}.values())
        assert ready,'navigation observer never reported ready'
        assert 'SIGSEGV' not in text and 'exit code -6' not in text
        assert parsed and all(p['returncode']==0 for p in parsed),parsed
        assert all(all(v==0. for v in (m.linear.x,m.linear.y,m.angular.z)) for m in commands)
        print(json.dumps(dict(actual_entrypoint=True,navigation_health_ready=True,
              standalone_exits=len(parsed),all_child_exits_clean=True,goals=0,nonzero_commands=0,
              weak_remote_network_validated=False,log=str(logpath))))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
