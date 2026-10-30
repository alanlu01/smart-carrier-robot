"""Reproduce the unresolved Nav2 shutdown fault in domain 231, without goals."""
import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.qos import qos_profile_system_default


def stop_managers(node, timeout=20.):
    """Diagnostic experiment, not a production shutdown command."""
    deadline=time.monotonic()+timeout;results=[]
    for name in ('lifecycle_manager_navigation','lifecycle_manager_localization'):
        client=node.create_client(ManageLifecycleNodes,'/'+name+'/manage_nodes',
                                 qos_profile=qos_profile_system_default)
        try:
            remaining=deadline-time.monotonic()
            if remaining<=0 or not client.wait_for_service(timeout_sec=min(1.,remaining)):
                results.append((name,False,'service_unavailable'));continue
            request=ManageLifecycleNodes.Request();request.command=request.SHUTDOWN
            future=client.call_async(request)
            while rclpy.ok() and not future.done() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=min(.1,max(0.,deadline-time.monotonic())))
            success=future.done() and future.exception() is None and future.result().success
            results.append((name,bool(success),'complete' if success else 'timeout_or_rejected'))
        finally:node.destroy_client(client)
    return results


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    output=Path(os.environ['ROS_LOG_DIR']);output.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('smart_delivery_core'))
    rclpy.init();node=Node('codex_nav2_shutdown_probe')
    broadcaster=StaticTransformBroadcaster(node)
    transforms=[]
    for parent,child in (('map','odom'),('odom','base_footprint'),('base_footprint','laser')):
        tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child
        tr.header.stamp=node.get_clock().now().to_msg();tr.transform.rotation.w=1.
        transforms.append(tr)
    broadcaster.sendTransform(transforms)
    results=[]
    try:
        for mode in ('signal_only','lifecycle_first'):
            path=output/(mode+'.log')
            with path.open('w') as log:
                child=subprocess.Popen(['ros2','launch','nav2_bringup','bringup_launch.py',
                    'map:='+str(share/'maps/3F_NEW.yaml'),
                    'params_file:='+str(share/'config/my_nav2_params.yaml'),
                    'use_sim_time:=false','autostart:=true'],stdout=log,stderr=subprocess.STDOUT,
                    start_new_session=True)
                ready=False;stop=[]
                try:
                    client=node.create_client(GetState,'/bt_navigator/get_state')
                    deadline=time.monotonic()+25.
                    while time.monotonic()<deadline and child.poll() is None:
                        if client.wait_for_service(timeout_sec=.2):
                            future=client.call_async(GetState.Request())
                            rclpy.spin_until_future_complete(node,future,timeout_sec=.5)
                            if future.done() and future.exception() is None and future.result().current_state.id==3:
                                ready=True;break
                        rclpy.spin_once(node,timeout_sec=.05)
                    node.destroy_client(client)
                    if mode=='lifecycle_first':
                        settle=time.monotonic()+2.
                        while time.monotonic()<settle:rclpy.spin_once(node,timeout_sec=.05)
                        stop=stop_managers(node)
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=12.)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5.)
                finally:
                    if child.poll() is None:
                        os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5.)
            text=path.read_text()
            result=dict(mode=mode,ready=ready,returncode=child.returncode,lifecycle=stop,
                child_abort=('exit code -6' in text or 'SIGSEGV' in text),
                transition_errors=text.count('Unable to start transition'),log=str(path))
            results.append(result);print(json.dumps(result),flush=True)
            # Diagnostic capture only: successful completion is not a fixed crash.
    finally:
        node.destroy_node();rclpy.shutdown()
    print('Loopback shutdown comparison finished; no goals or motor commands sent.')


if __name__=='__main__':main()
