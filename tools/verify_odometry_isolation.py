"""Separate-process fault injection in localhost domain 231; no motor writers."""
import json
import os
import signal
import subprocess
import sys
import time
assert os.environ.get('ROS_DOMAIN_ID')=='231'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
assert os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML')=='1'
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from smart_delivery_core.odom_transport import LOCAL_ODOM_QOS


def main():
    source="from smart_delivery_core.mecanum_odom_real import main; main()"
    relay='''
import rclpy,time
from smart_delivery_core.odom_transport import OdomTransport
rclpy.init();node=OdomTransport()
send=node.tf.sendTransform
def slow(value):
    time.sleep(.8)
    send(value)
node.tf.sendTransform=slow
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
'''
    children=[]
    rclpy.init();node=rclpy.create_node('codex_odometry_isolation_probe')
    local=[];public=[];health=[]
    node.create_subscription(Odometry,'/chassis/odom_local',lambda m:local.append(time.monotonic()),LOCAL_ODOM_QOS)
    node.create_subscription(Odometry,'/odom',public.append,10)
    node.create_subscription(String,'/chassis/feedback_health',lambda m:health.append(json.loads(m.data)),LOCAL_ODOM_QOS)
    pub=node.create_publisher(String,'/stm32_data',10)
    try:
        for script in (source,relay):
            children.append(subprocess.Popen([sys.executable,'-B','-c',script],stdout=subprocess.PIPE,stderr=subprocess.PIPE))
        warmup=time.monotonic();started=None;sent_at=0.
        while time.monotonic()-warmup<12.:
            now=time.monotonic()
            if started is None and len(local)>=5 and pub.get_subscription_count():
                started=now;local.clear();public.clear();health.clear()
            if started is not None and now-started>=4.:
                break
            if now-sent_at>=.25:
                for letter in 'ABCD':
                    m=String();m.data=letter+'_RT: 0';pub.publish(m)
                sent_at=now
            rclpy.spin_once(node,timeout_sec=.005)
        assert started is not None and len(local)>45, len(local)
        gaps=[b-a for a,b in zip(local,local[1:])]
        assert max(gaps)<.25,max(gaps)
        assert not public  # every deliberately stalled snapshot expired
        assert health and health[-1]['state']=='healthy',health[-1:]
        print(json.dumps({'domain':231,'private_odom_samples':len(local),
            'max_private_gap_sec':round(max(gaps),4),'stale_public_samples':len(public),
            'wheel_health':health[-1]['state'],'fault_injected_tf_delay_sec':.8,'motor_commands':0,'goals':0}))
    finally:
        for child in children:child.send_signal(signal.SIGINT)
        for child in children:
            try:out,err=child.communicate(timeout=5.)
            except subprocess.TimeoutExpired:
                child.terminate();out,err=child.communicate(timeout=3.)
            if child.returncode not in (0,-signal.SIGINT):
                raise RuntimeError((out+err).decode(errors='replace')[-1500:])
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
