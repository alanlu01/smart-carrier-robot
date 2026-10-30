"""Real fusion callbacks in domain 231; only zero-speed safety publications."""
import json,math,os,time
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from hailo_vision.fusion_node import SensorFusionNode


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    rclpy.init();node=SensorFusionNode()
    try:
        scan=LaserScan();scan.header.stamp=node.get_clock().now().to_msg()
        scan.angle_min=0.;scan.angle_increment=2*math.pi/1800
        scan.range_min=.05;scan.range_max=10.;scan.ranges=[.4]*1800
        ns=scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec
        node.latest_semantic_received_at=time.monotonic();node.latest_semantic_stamp_ns=ns
        node.semantic_sequence=1
        node._process_semantic_data([dict(id=0,angle=0.,bbox_width=100.,score=.8)],scan)
        assert node.speed_multiplier==0.
        evidence=node.last_stop_association
        assert evidence['scan_stamp_ns']==ns and evidence['sync_skew_sec']==0.
        assert evidence['targets'][0]['held_distance_m']==.4
        assert evidence['targets'][0]['near_cluster_support']>0
        node.last_pipeline_report_at=0.
        node.semantic_watchdog_callback()
        print(json.dumps(dict(real_fusion_callbacks=True,stop_diagnostic=True,
                              tracks=len(node.target_ranges.tracks),goals=0,nonzero_motor_commands=0)))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
