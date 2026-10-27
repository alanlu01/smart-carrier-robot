"""Refresh stopped recovery evidence without any real nodes or motion."""

from types import SimpleNamespace
import pytest
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

from test_localization_recovery import manager


def waiting(manager):
    node,clock=manager
    node.state='RECOVERING_GLOBAL'
    node._global_recovery_failed(clock.now,'test')
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock.now*1e9)))
    node.latest_scan=LaserScan()
    node.latest_scan.header.stamp.sec=100
    updates=[]
    def update(now):
        updates.append(now);node.latest_map_score_at=now;node.last_map_match_error=''
    node._update_map_match_score=update
    return node,clock,updates


def sample(node,at,xy=.20):
    message=PoseWithCovarianceStamped()
    message.header.stamp.sec=int(at)
    message.pose.pose.position.x,message.pose.pose.position.y=2.,5.
    message.pose.pose.orientation.w=1.
    message.pose.covariance[0]=message.pose.covariance[7]=xy**2
    message.pose.covariance[35]=.10**2
    node._amcl_pose_callback(message)


def test_second_wait_refreshes_map_and_requests_amcl_without_extending_deadline(manager):
    n,c,updates=waiting(manager)
    start=n.second_guard_started_at
    for at in (101.,102.,103.):
        c.now=at;n.latest_raw_scan_at=n.latest_scan_at=n.latest_odom_at=at
        n.latest_scan.header.stamp.sec=int(at)
        n._tick_state_machine()
    assert updates==[101.,102.,103.] and c.now in n.requests
    assert n.second_guard_started_at==start and n.recovery_step=='second_guard_wait'
    assert not n.started_spins and not n.ready
    c.now=108.;n._tick_state_machine()
    assert n.state=='MANUAL_REQUIRED' and not n.ready


def test_six_independent_healthy_samples_can_recover_in_place(manager):
    n,c,_=waiting(manager)
    for at in range(101,107):
        c.now=float(at);n.latest_raw_scan_at=n.latest_scan_at=n.latest_odom_at=c.now
        n.latest_scan.header.stamp.sec=at
        n._tick_state_machine();sample(n,at)
        assert n.ready is (at==106)
    assert n.stable_samples==6 and n.state=='LOCALIZED' and not n.started_spins


def test_repeated_amcl_or_scan_stamp_cannot_supply_six_samples(manager):
    n,c,updates=waiting(manager)
    c.now=101.;n.latest_scan_at=n.latest_raw_scan_at=n.latest_odom_at=c.now
    n.latest_scan.header.stamp.sec=101
    n._tick_state_machine();sample(n,101)
    assert n.stable_samples==1
    for at in (101.2,101.4,101.6):
        c.now=at;n._tick_state_machine();sample(n,101)
    assert n.stable_samples==0 and not n.ready and updates==[101.]


@pytest.mark.parametrize('fault',['covariance','low_map','stale_odom','inhibited','moving'])
def test_refresh_cannot_bypass_remaining_health_and_motion_checks(manager,fault):
    n,c,_=waiting(manager)
    for at in range(101,107):
        c.now=float(at);n.latest_scan_at=n.latest_raw_scan_at=n.latest_odom_at=c.now
        n.latest_scan.header.stamp.sec=at
        n._tick_state_machine()
        if fault=='low_map':n.latest_map_score=.1
        if fault=='stale_odom':n.latest_odom_at=90.
        if fault=='inhibited':n.motion_inhibited=True
        if fault=='moving':n.odom_motion=True
        sample(n,at,xy=.32 if fault=='covariance' else .20)
    assert not n.ready and not n.started_spins


@pytest.mark.parametrize('kind,expected',[('stale','地圖評分資料過期'),('low','吻合度偏低'),
                                      ('missing','地圖評分尚未取得'),('failed','地圖評分更新失敗')])
def test_failure_message_distinguishes_real_low_scores_from_unavailable_data(manager,kind,expected):
    n,c=manager
    if kind=='stale':n.latest_map_score_at=90.
    elif kind=='low':n.latest_map_score=.1
    elif kind=='missing':n.latest_map_score=None
    else:n.last_map_match_error='synthetic TF gap'
    text=n._recovery_failure_reason()
    assert expected in text
    if kind!='low':assert '吻合度偏低' not in text


def test_second_wait_preserves_existing_time_and_motion_limits(manager):
    n,c,_=waiting(manager)
    n._tick_state_machine()
    assert n.second_guard_wait==8. and n.stable_samples_required==6
    assert n.healthy_xy_std==.25 and n.healthy_yaw_std==.17
    assert not n.started_spins
