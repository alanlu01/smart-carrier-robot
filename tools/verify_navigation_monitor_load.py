"""Localhost synthetic traffic, six active services, no motion or hardware."""
import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from bond.msg import Status
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy
from smart_delivery_core.navigation_health import TARGETS


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    rclpy.init();node=rclpy.create_node('codex_monitor_load_probe')
    def state(req,res):res.current_state.id=3;res.current_state.label='active';return res
    services=[node.create_service(GetState,'/'+name+'/get_state',state) for name in TARGETS]
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    bond=node.create_publisher(Status,'/bond',qos);tf=node.create_publisher(TFMessage,'/tf',qos)
    latest=[];node.create_subscription(String,'/navigation/pipeline_health',lambda m:latest.append(json.loads(m.data)),1)
    def feed():
        for i in range(11):
            m=Status();m.id=TARGETS[i%6];m.instance_id=str(i);m.active=True
            m.header.stamp=node.get_clock().now().to_msg();bond.publish(m)
        tr=TransformStamped();tr.header.frame_id='odom';tr.child_frame_id='base_footprint'
        tr.header.stamp=node.get_clock().now().to_msg();tr.transform.rotation.w=1.
        tf.publish(TFMessage(transforms=[tr]))
    node.create_timer(.05,feed)
    results=[]
    try:
        for enabled in (False,):
            latest.clear()
            child=subprocess.Popen(['python3','-B','-c','from smart_delivery_core.navigation_health import main;main()',
                '--ros-args','-p','detailed_transport_observer:='+str(enabled).lower()],start_new_session=True)
            def ticks():
                raw=Path(f'/proc/{child.pid}/stat').read_text();s=raw[raw.rfind(')')+2:].split()
                return int(s[11])+int(s[12])
            try:
                warm=time.monotonic()+3.
                while time.monotonic()<warm:rclpy.spin_once(node,timeout_sec=.01)
                start=time.monotonic();before=ticks()
                while time.monotonic()-start<6.:rclpy.spin_once(node,timeout_sec=.01)
                cpu=(ticks()-before)/os.sysconf('SC_CLK_TCK')/(time.monotonic()-start)*100
                assert latest and latest[-1]['state']=='ready'
                results.append(dict(detailed_transport_observer=enabled,cpu_one_core_percent=cpu,
                                    health_payload_bytes=len(json.dumps(latest[-1])),ready=True))
            finally:
                os.killpg(child.pid,signal.SIGINT);assert child.wait(5.)==0
        print(json.dumps(dict(synthetic_bond_hz=220,synthetic_tf_hz=20,results=results,hardware_commands=0)))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
