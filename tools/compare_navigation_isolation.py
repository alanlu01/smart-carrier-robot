"""Localhost/domain 231 Nav2 startup/shutdown comparison. No goals or motion."""
import json,os,resource,signal,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped,Twist
from lifecycle_msgs.srv import GetState
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from smart_delivery_core.navigation_health import NavigationHealth


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    root=Path(__file__).resolve().parents[1]/'packages/smart_delivery_core'
    share=Path(get_package_share_directory('smart_delivery_core'))
    output=Path(os.environ['ROS_LOG_DIR']);output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('codex_nav2_isolation_probe')
    monitor=NavigationHealth();executor=SingleThreadedExecutor()
    executor.add_node(node);executor.add_node(monitor)
    broadcaster=StaticTransformBroadcaster(node);transforms=[];commands=[]
    for parent,child in (('map','odom'),('odom','base_footprint'),('base_footprint','laser')):
        tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child
        tf.header.stamp=node.get_clock().now().to_msg();tf.transform.rotation.w=1.;transforms.append(tf)
    broadcaster.sendTransform(transforms)
    node.create_subscription(Twist,'/cmd_vel',commands.append,10)
    try:
        for name,composition,profile in (('composed_current','True','current'),
                                          ('composed_async','True','async')):
            env=dict(os.environ)
            path=root/'config'/('fastdds_nav_async.xml' if profile=='async' else 'fastdds_robot.xml')
            env['FASTRTPS_DEFAULT_PROFILES_FILE']=env['FASTDDS_DEFAULT_PROFILES_FILE']=str(path)
            if profile=='async':
                env['RMW_FASTRTPS_USE_QOS_FROM_XML']='1';env['RMW_FASTRTPS_PUBLICATION_MODE']='ASYNCHRONOUS'
            else:
                env.pop('RMW_FASTRTPS_USE_QOS_FROM_XML',None);env.pop('RMW_FASTRTPS_PUBLICATION_MODE',None)
            logpath=output/(name+'.log');start=time.monotonic();ready=False;duration=None
            with logpath.open('w') as log:
                # Exercise the deployed scoped plugin-lifetime wrapper too.
                child=subprocess.Popen(['ros2','launch',str(root/'launch/nav2_composed_bringup.launch.py'),
                    'map:='+str(share/'maps/3F_NEW.yaml'),'params_file:='+str(share/'config/my_nav2_params.yaml'),
                    'autostart:=true','use_sim_time:=false','use_composition:='+composition],env=env,
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                    preexec_fn=lambda:resource.setrlimit(resource.RLIMIT_CORE,(0,0)))
                client=node.create_client(GetState,'/bt_navigator/get_state')
                try:
                    while time.monotonic()-start<30. and child.poll() is None:
                        if client.service_is_ready():
                            future=client.call_async(GetState.Request())
                            wait=time.monotonic()+.3
                            while not future.done() and time.monotonic()<wait:
                                executor.spin_once(timeout_sec=.02)
                            if future.done() and future.exception() is None and future.result().current_state.id==3:
                                ready=True;duration=time.monotonic()-start;break
                            if not future.done():future.cancel();client.remove_pending_request(future)
                        executor.spin_once(timeout_sec=.05)
                    settle=time.monotonic()+5.
                    while time.monotonic()<settle:executor.spin_once(timeout_sec=.05)
                    diagnostics=dict(bond_instances=len(monitor.bonds),service_timeouts=dict(monitor.timeouts),
                                     monitor_wall_durations=monitor.duration_metrics.snapshot(time.monotonic()))
                    stop=time.monotonic();os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=12.)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGTERM)
                        try:child.wait(timeout=4.)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=4.)
                    stop_duration=time.monotonic()-stop
                finally:
                    node.destroy_client(client)
                    # Kill only this isolated child process group if it left descendants.
                    try:os.killpg(child.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    if child.poll() is None:child.wait(timeout=4.)
            text=logpath.read_text(errors='replace')
            startup=text.split("sending signal 'SIGINT'",1)[0]
            print(json.dumps(dict(mode=name,active=ready,startup_sec=duration,stop_sec=stop_duration,
                returncode=child.returncode,child_abort='SIGSEGV' in text or 'exit code -6' in text,
                startup_transition_errors=startup.count('Unable to start transition'),
                shutdown_transition_errors=text.count('Unable to start transition')-startup.count('Unable to start transition'),
                diagnostics=diagnostics,remote_weak_network_tested=False,
                goals=0,nonzero_commands=0,log=str(logpath))),flush=True)
            assert ready,(name,'Nav2 never activated')
            assert 'CRITICAL FAILURE' not in startup and 'Unable to start transition' not in startup
            assert diagnostics['bond_instances']==0
            assert all(all(v==0. for v in (m.linear.x,m.linear.y,m.angular.z)) for m in commands)
    finally:
        executor.shutdown();monitor.destroy_node();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
