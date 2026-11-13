"""Exercise native composed planner/controller/BT handoffs on a software plant."""
import json
from copy import deepcopy
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from smart_delivery_core.station_nav_profile import station_parameters
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    root = Path(tempfile.mkdtemp(prefix='handoff_', dir=os.environ['ROS_LOG_DIR']))
    share = Path(get_package_share_directory('smart_delivery_core'))
    parameters = station_parameters(yaml.safe_load((share/'config/my_nav2_params.yaml').read_text()))
    if '--twirling-2' in sys.argv:
        parameters['controller_server']['ros__parameters']['FollowPath']['TwirlingCritic']['cost_weight']=2.
    normal = share/'behavior_trees/smart_delivery_navigate.xml'
    parameters['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(normal)
    parameters['amcl']['ros__parameters']['tf_broadcast'] = False
    # A simple bounded free room, with the production footprint and inflation.
    width = 200
    image = bytes(0 if x in (0, width-1) or y in (0, width-1) else 254
                  for y in range(width) for x in range(width))
    (root/'room.pgm').write_bytes(b'P5\n200 200\n255\n'+image)
    (root/'room.yaml').write_text(yaml.safe_dump(dict(image='room.pgm', resolution=.05,
        origin=[-5., -5., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))
    (root/'parameters.yaml').write_text(yaml.safe_dump(parameters))
    rclpy.init()
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from smart_delivery_core.station_navigation import StationNavigation
    from tf2_ros import Buffer, TransformListener
    node = BasicNavigator(node_name='codex_navigation_handoff_plant')
    state = dict(x=0., y=0., yaw=0., command=Twist(), received=-math.inf, last=time.monotonic(), map_shift=0.)
    frames = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    tr = TransformStamped(); tr.header.frame_id='map'; tr.child_frame_id='odom'
    tr.transform.rotation.w=1.; tr.header.stamp=node.get_clock().now().to_msg()
    odom = node.create_publisher(Odometry, '/odom', 10)
    scan = node.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
    paths = []
    command_peaks={}
    command_trace={}
    for topic in ('/cmd_vel_nav','/cmd_vel_smoothed'):
        node.create_subscription(Twist,topic,lambda m,t=topic:command_trace.update({t:[m.linear.x,m.linear.y,m.angular.z]}),10)
    phase={'tree':'none','stress':False,'feedback':0.,'feedback_at':0.,'rate':0.}
    health=[]
    node.create_subscription(String,'/navigation/pipeline_health',lambda m:health.append(json.loads(m.data)),1)
    lease_pub=node.create_publisher(Bool,'/smart_carrier/delivery_navigation_active',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    feedback=node.create_publisher(String,'/chassis/serial_rx_health',qos_profile_sensor_data)
    proof={'enabled':True,'lease':False}
    def publish_proof():
        lease_pub.publish(Bool(data=proof['lease']))
        if not proof['enabled']:return
        stamp=node.get_clock().now().nanoseconds
        v=state['command'];stopped=math.hypot(v.linear.x,v.linear.y)<.01 and abs(v.angular.z)<.01
        feedback.publish(String(data=json.dumps(dict(healthy=True,connected=True,session='software-only',
            sample_ros_stamp_ns=stamp,complete_ros_stamp_ns=stamp,complete_sample_age_sec=0.,stop_proven=stopped))))
    node.create_timer(.15,publish_proof)
    policy_pub = node.create_publisher(String, '/delivery/navigation_policy',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def select_policy(pose,tree):
        from smart_delivery_core.navigation_policy import navigation_policy
        selected=navigation_policy(pose,tree,share)
        if selected is None:return tree
        path,payload=selected;policy_pub.publish(String(data=payload));return path
    node.configure_navigation_goal=select_policy
    buffer=Buffer();listener=TransformListener(buffer,node)
    station=StationNavigation(node,buffer,share)
    node.create_subscription(NavPath, '/plan', lambda m:paths.append((time.monotonic(),len(m.poses))), 1)
    def command(message):
        state['command'], state['received'] = message, time.monotonic()
        command_trace['/cmd_vel']=[message.linear.x,message.linear.y,message.angular.z]
        tree=phase['tree']
        command_peaks[tree]=max(command_peaks.get(tree,0.),math.hypot(message.linear.x,message.linear.y))
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    def sensors():
        now=time.monotonic(); dt=min(.1,now-state['last']); state['last']=now
        v=state['command'] if now-state['received']<.3 else Twist(); a=state['yaw']
        state['x']+=(math.cos(a)*v.linear.x-math.sin(a)*v.linear.y)*dt
        state['y']+=(math.sin(a)*v.linear.x+math.cos(a)*v.linear.y)*dt
        target=(1.5 if phase['stress'] else 1.)*v.angular.z if abs(v.angular.z)>.11 else 0.
        phase['rate']+=max(-.8*dt,min(.8*dt,target-phase['rate']))
        rate=phase['rate']
        state['yaw']+=rate*dt
        if not phase['stress'] or now-phase['feedback_at']>=.5:
            phase['feedback'],phase['feedback_at']=rate,now
        stamp=node.get_clock().now().to_msg()
        tr=TransformStamped(); tr.header.frame_id='odom'; tr.child_frame_id='base_footprint'
        tr.header.stamp=stamp; tr.transform.translation.x=state['x']; tr.transform.translation.y=state['y']
        tr.transform.rotation.z=math.sin(state['yaw']/2); tr.transform.rotation.w=math.cos(state['yaw']/2)
        correction=TransformStamped();correction.header.frame_id='map';correction.child_frame_id='odom'
        correction.header.stamp=stamp;correction.transform.rotation.w=1.
        correction.transform.translation.x=state['map_shift']
        frames.sendTransform([correction,tr])
        m=Odometry(); m.header.frame_id='odom'; m.child_frame_id='base_footprint'; m.header.stamp=stamp
        m.pose.pose.position.x=state['x']; m.pose.pose.position.y=state['y']
        m.pose.pose.orientation=tr.transform.rotation; m.twist.twist=deepcopy(v); m.twist.twist.angular.z=phase['feedback']
        odom.publish(m)
        laser=LaserScan(); laser.header.frame_id='base_footprint'; laser.header.stamp=stamp
        laser.angle_min=-math.pi; laser.angle_max=math.pi; laser.angle_increment=2*math.pi/720
        laser.range_min=.15; laser.range_max=12.; laser.ranges=[float('inf')]*721
        scan.publish(laser)
    node.create_timer(.025,sensors)
    def pump(predicate, seconds):
        end=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return True
        return bool(predicate())
    def resolve(future, seconds=10.):
        assert pump(future.done,seconds), ('future timeout',state['x'],state['y'],state['yaw'],
            state['command'].linear.x,state['command'].linear.y,state['command'].angular.z)
        return future.result()
    log=(root/'nav2.log').open('w')
    child=subprocess.Popen(['ros2','launch','smart_delivery_core','nav2_composed_bringup.launch.py',
        'map:='+str(root/'room.yaml'),'params_file:='+str(root/'parameters.yaml')],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    handle=None; results=[];observer=None;observer_log=None;recovery_report=None
    try:
        if '--recover' in sys.argv:
            observer_log=(root/'health.log').open('w')
            observer=subprocess.Popen(['ros2','run','smart_delivery_core','navigation_health','--ros-args',
                '-p','action_health_file:='+str(root/'action_fault.json')],
                stdout=observer_log,stderr=subprocess.STDOUT,start_new_session=True)
        service=node.create_client(GetState,'/bt_navigator/get_state')
        assert service.wait_for_service(timeout_sec=15.)
        end=time.monotonic()+30.
        while time.monotonic()<end:
            response=service.call_async(GetState.Request())
            if pump(response.done,1.5):
                if response.result().current_state.id==3:break
            else:
                response.cancel();service.remove_pending_request(response)
        else:raise AssertionError('Nav2 did not activate')
        client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        assert client.wait_for_server(timeout_sec=5.)
        def send(x,y,yaw,tree):
            phase['tree']=tree
            request=NavigateToPose.Goal(); request.behavior_tree=str(share/'behavior_trees'/tree)
            p=PoseStamped(); p.header.frame_id='map'; p.header.stamp=node.get_clock().now().to_msg()
            p.pose.position.x=x; p.pose.position.y=y
            p.pose.orientation.z=math.sin(yaw/2); p.pose.orientation.w=math.cos(yaw/2)
            request.pose=p
            if '--owned' in sys.argv:
                from smart_delivery_core.navigation_policy import navigation_policy
                selected = navigation_policy(p,request.behavior_tree,share)
                assert selected is not None
                request.behavior_tree,policy=selected
                policy_pub.publish(String(data=policy))
            answer=resolve(client.send_goal_async(request)); assert answer.accepted
            return answer

        def managed(x,y,yaw,tree):
            from types import SimpleNamespace
            phase['tree']=tree
            p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
            p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            station.start(p,label='software standby')
            assert pump(station.isTaskComplete,40.),(station.phase,state)
            return SimpleNamespace(status=4 if station.getResult()==TaskResult.SUCCEEDED else 6,
                result=SimpleNamespace(error_msg=station.error_details()[1],error_code=station.error_details()[0]))
        routes = [
            (.8,0.,0.,'smart_delivery_navigate.xml'),
            (.8,.45,0.,'station_y.xml'),
            (.8,.45,.125,'station_align.xml'),
            (.8,0.,.125,'station_exit_y.xml'),
            (0.,0.,0.,'smart_delivery_navigate.xml'),
            (.5,0.,0.,'smart_delivery_navigate.xml'),
            (.5,.4,0.,'station_y.xml'),
            (0.,0.,0.,'smart_delivery_navigate.xml'),
        ]
        if '--alignment-only' in sys.argv:routes=[]
        for index,(x,y,yaw,tree) in enumerate(routes):
            if tree=='station_align.xml':x,y=state['x'],state['y']
            start=time.monotonic()
            if tree=='smart_delivery_navigate.xml' and '--owned' in sys.argv:
                answer=managed(x,y,yaw,tree)
            else:
                handle=send(x,y,yaw,tree)
                answer=resolve(handle.get_result_async(),35.)
            result=dict(index=index,tree=tree,status=answer.status,error=answer.result.error_msg,
                code=answer.result.error_code,elapsed=time.monotonic()-start,plans=len(paths))
            results.append(result); print(json.dumps(result),flush=True)
            assert answer.status==4,result
            assert pump(lambda:math.hypot(state['command'].linear.x,state['command'].linear.y)<.01
                        and abs(state['command'].angular.z)<.01,3.)
        if '--alignment-stress' in sys.argv:
            phase['stress']=True
            for delta in (1.92,-1.92,.20,-.20):
                assert pump(lambda:abs(phase['rate'])<.03,3.)
                anchor=(state['x'],state['y']);target=state['yaw']+delta
                handle=send(anchor[0]+state['map_shift'],anchor[1],target,'station_align.xml')
                result=handle.get_result_async()
                assert pump(lambda:abs(phase['rate'])>.1,4.),dict(state=state,phase=phase,commands=command_trace,root=str(root))
                # Recorded map XY correction exceeded 5 cm, with no physical
                # translation. Latched XY + strict physical outer guard owns it.
                state['map_shift']+=.14
                answer=resolve(result,22.)
                assert answer.status==4,(delta,answer,state)
                native_finish_rate=phase['rate']
                # Native consumes held 2Hz feedback. Match the production
                # outer stop/settle gate before checking the final pose.
                assert pump(lambda:abs(phase['rate'])<=.03 and abs(state['command'].angular.z)<.01,3.)
                pump(lambda:False,.75)
                assert abs(math.atan2(math.sin(target-state['yaw']),math.cos(target-state['yaw'])))<=.06
                assert abs(phase['rate'])<=.03
                assert math.hypot(state['x']-anchor[0],state['y']-anchor[1])<.001
                print(json.dumps(dict(alignment_stress=delta,passed=True,native_finish_rate=native_finish_rate,settled_rate=phase['rate'])),flush=True)
            state['map_shift']=0.
            pump(lambda:False,.3)
            phase['stress']=False
            if '--alignment-only' in sys.argv:
                print(json.dumps(dict(passed=True,alignment_stress=True,domain=231,root=str(root))),flush=True)
                return
        handle=send(1.,0.,0.,'smart_delivery_navigate.xml')
        assert pump(lambda:math.hypot(state['command'].linear.x,state['command'].linear.y)>.05,5.)
        resolve(handle.cancel_goal_async()); assert resolve(handle.get_result_async()).status==5
        assert pump(lambda:math.hypot(state['command'].linear.x,state['command'].linear.y)<.01,3.)
        handle=send(state['x'],state['y']+.3,state['yaw'],'station_y.xml')
        assert resolve(handle.get_result_async(),20.).status==4
        assert command_peaks.get('smart_delivery_navigate.xml',0.)>.20,command_peaks
        assert command_peaks.get('station_y.xml',1.)<=.15001,command_peaks
        if '--owned' in sys.argv:
            assert pump(lambda:math.hypot(state['command'].linear.x,state['command'].linear.y)<.01,3.)
            anchor=(state['x'],state['y'],state['yaw']);count=len(paths)
            bad=NavigateToPose.Goal();bad.behavior_tree=str(share/'behavior_trees/smart_owned_navigate.xml')
            bad.pose.header.frame_id='map';bad.pose.header.stamp=node.get_clock().now().to_msg()
            bad.pose.pose.position.x=state['x']+.5;bad.pose.pose.position.y=state['y'];bad.pose.pose.orientation.w=1.
            handle=resolve(client.send_goal_async(bad))
            assert resolve(handle.get_result_async(),6.).status==6
            assert len(paths)==count and math.hypot(state['x']-anchor[0],state['y']-anchor[1])<.01
        if '--recover' in sys.argv:
            assert pump(lambda:health and health[-1]['state']=='ready',5.)
            proof['lease']=True
            assert pump(lambda:False,.4) is False
            from smart_delivery_core.navigation_transcript import NavigationTranscript
            from types import SimpleNamespace
            transcript=NavigationTranscript(root/'injected_transcript',action_health_file=root/'action_fault.json')
            transcript.output(SimpleNamespace(cmd=['component_container_isolated','__node:=nav2_container'],pid=999,
                from_stderr=True,text=b'[WARN] Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose\n'))
            transcript.close()
            assert pump(lambda:health and health[-1].get('action',{}).get('state')=='fault',4.)
            assert health[-1]['action']['delivery_owned_fault']
            proof['lease']=False;proof['enabled']=False;publish_proof()
            recover=node.create_client(Trigger,'/navigation/recover_actions')
            assert recover.wait_for_service(timeout_sec=2.)
            assert resolve(recover.call_async(Trigger.Request())).success
            pump(lambda:False,1.8)
            assert health[-1]['action']['state']=='waiting_stop',health[-1]
            proof['enabled']=True
            assert pump(lambda:health[-1].get('action',{}).get('recovered_generation',0)==1,35.),health[-1]
            assert resolve(node.create_client(GetState,'/amcl/get_state').call_async(GetState.Request())).current_state.id==3
            handle=send(state['x'],state['y']+.3,state['yaw'],'station_y.xml')
            assert resolve(handle.get_result_async(),20.).status==4
            if '--owned' in sys.argv:
                assert managed(state['x']+.4,state['y'],0.,'smart_delivery_navigate.xml').status==4
            else:
                handle=send(state['x']+.4,state['y'],0.,'smart_delivery_navigate.xml')
                assert resolve(handle.get_result_async(),20.).status==4
            recovery_report=dict(native_navigation_reset_startup=True,stale_stop_proof_blocked=True,
                localization_lifecycle_preserved=True,new_station_and_normal_goals_succeeded=True,
                injected_diagnostic_only=True,action=health[-1]['action'])
        (root/'result.json').write_text(json.dumps(dict(passed=True,results=results,
            after_cancel=True,wrong_policy_no_motion='--owned' in sys.argv,recovery=recovery_report),indent=2))
        print(json.dumps(dict(passed=True,domain=231,software_plant_only=True,root=str(root))),flush=True)
    finally:
        if handle is not None:
            try:resolve(handle.cancel_goal_async(),2.)
            except Exception:pass
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=25.)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL); child.wait(timeout=5.)
        log.close(); node.destroy_node()
        if observer is not None:
            observer.send_signal(signal.SIGINT)
            try:observer.wait(timeout=5.)
            except subprocess.TimeoutExpired:os.killpg(observer.pid,signal.SIGKILL);observer.wait(timeout=3.)
            observer_log.close()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
