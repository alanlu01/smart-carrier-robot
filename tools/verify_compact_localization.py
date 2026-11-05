"""Check real node initialization and diagnostic decimation, no executor/motion."""
import json,os,tempfile,time
from types import SimpleNamespace
import rclpy
from smart_delivery_core.localization_manager import LocalizationManager


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    with tempfile.TemporaryDirectory(prefix='codex_compact_state_') as directory:
        rclpy.init(args=['--ros-args','-p','auto_initialize:=false','-p','automatic_motion_recovery:=false',
                        '-p','pose_cache_path:='+directory+'/pose.json'])
        node=LocalizationManager();messages=[]
        try:
            node.state_publisher=SimpleNamespace(publish=lambda message:messages.append(json.loads(message.data)))
            node.last_summary_key=None;node.last_detail_at=-float('inf')
            node._publish_state()
            assert len(messages)==1 and messages[0]['detail_included']
            for _ in range(100):node._publish_state()
            assert len(messages)==1
            node.last_state_publish_at=time.monotonic()-1.1
            node._publish_state()
            assert len(messages)==2 and not messages[1]['detail_included']
            for key in ('sample_ros_stamp_ns','state','ready','dispatch_allowed','manual_intervention_kind',
                        'pipeline_wait_detail','verification_blockers','xy_std','yaw_std'):
                assert key in messages[1]
            node.recovery_step='diagnostic_transition'
            node._publish_state()
            assert len(messages)==3 and messages[-1]['detail_included']
            print(json.dumps(dict(summary_bytes=len(json.dumps(messages[1])),detail_bytes=len(json.dumps(messages[0])),
                unchanged_100_calls_suppressed=True,transitions_immediate=True,hardware_commands=0)))
        finally:node.pose_cache_writer.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
