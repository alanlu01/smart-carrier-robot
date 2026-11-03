"""The last safety stage holds native near-field STOP through clear flicker."""
from types import SimpleNamespace
from geometry_msgs.msg import Twist
from hailo_vision import fusion_node as module
from test_fusion_diagnostics import fusion


def test_near_stop_immediately_zeros_and_holds_commands_after_clear(monkeypatch):
    n=fusion(monkeypatch,.1)
    n.collision_state_callback(SimpleNamespace(action_type=1,polygon_name='NearFootprintStop'))
    assert n.commands[-1].linear.x==0.
    m=Twist();m.linear.x=.4
    n.collision_state_callback(SimpleNamespace(action_type=0,polygon_name=''))
    n.cmd_vel_callback(m)
    assert n.commands[-1].linear.x==0.
    monkeypatch.setattr(module.time,'monotonic',lambda:104.)
    n.cmd_vel_callback(m)
    assert n.commands[-1].linear.x==.4


def test_only_native_near_stop_arms_hold_and_soft_motion_cap_is_vector_bounded(monkeypatch):
    n=fusion(monkeypatch,.1)
    n.collision_state_callback(SimpleNamespace(action_type=3,polygon_name='FootprintApproach'))
    assert not getattr(n,'near_stop_until',0.)
    n.localization_speed_limit.value=.4
    n.localization_speed_limit.last_at=103.
    # Use an explicit, fresh limit to isolate the geometric cap from TTL tests.
    n.localization_speed_limit.limit=lambda now:.4
    m=Twist();m.linear.x=m.linear.y=.5
    n.cmd_vel_callback(m)
    import math
    assert math.hypot(n.commands[-1].linear.x,n.commands[-1].linear.y)<=.200001
