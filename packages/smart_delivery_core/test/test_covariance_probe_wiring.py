"""Exercise the actual AMCL callback and zero-speed publication; no motors."""
from types import SimpleNamespace
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from test_localization_recovery import manager


def test_real_callback_stops_immediately_preserves_goal_and_requires_fresh_recovery(manager):
    n,c=manager
    n.state,n.ready,n.commanded_motion='LOCALIZED',True,True
    n.suspect_since=None
    n.ready_publisher=SimpleNamespace(publish=lambda _:None)
    n._persist_trusted_pose=lambda:None
    speeds=[]
    n.speed_limit_publisher=SimpleNamespace(publish=lambda msg:speeds.append(msg.data))
    tf=TransformStamped();tf.transform.rotation.w=1.
    n.tf_buffer=SimpleNamespace(lookup_transform=lambda *args:tf)
    n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(c.now*1e9)))
    def emit(at,xy):
        c.now=at
        n.latest_raw_scan_at=n.latest_scan_at=n.latest_odom_at=n.latest_map_score_at=at
        n.latest_odom_stamp_ns=n.latest_scan_stamp_ns=round(at*1e9)
        n.latest_raw_scan=LaserScan();n.latest_raw_scan.header.stamp.sec=int(at)
        n.latest_raw_scan.header.stamp.nanosec=round((at-int(at))*1e9)
        msg=PoseWithCovarianceStamped();msg.header.stamp=n.latest_raw_scan.header.stamp
        msg.pose.pose.position.x,msg.pose.pose.position.y=2.,5.
        msg.pose.pose.orientation.w=1.
        msg.pose.covariance[0]=msg.pose.covariance[7]=xy*xy
        msg.pose.covariance[35]=.01
        n._amcl_pose_callback(msg)
    emit(100.,.2);emit(100.1,1.2)
    assert n.covariance_probe.active and speeds[-1]==0.
    assert n.state=='LOCALIZED' and n.ready and not n.started_spins
    n._tick_state_machine()
    assert n.state=='LOCALIZED' and n.covariance_probe.active
    emit(100.3,.2);assert n.covariance_probe.active
    emit(100.5,.2);assert not n.covariance_probe.stopped
    emit(100.6,1.2);assert n.covariance_probe.active
    c.now=102.2
    n.latest_scan_at=n.latest_raw_scan_at=n.latest_odom_at=n.latest_map_score_at=c.now
    n._tick_state_machine()
    assert n.covariance_probe.failed and n.state=='SUSPECT' and not n.ready
