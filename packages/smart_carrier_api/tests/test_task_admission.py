"""No permission heartbeat means no new claims only when explicitly enabled."""

import pytest

from smart_carrier_api.task_admission import TaskAdmissionGate


def permission(stamp=1000., accepting=True):
    return {'source': 'optional_idle', 'stamp_sec': stamp, 'accepting': accepting}


def test_disabled_gate_keeps_existing_claim_behavior_unchanged():
    gate = TaskAdmissionGate()
    assert gate.allows_claim(10.)
    gate.observe(permission(1000., False), 10., 1000.)
    assert gate.allows_claim(99999.)


def test_enabled_gate_requires_fresh_permission_and_blocks_after_manager_disappears():
    gate = TaskAdmissionGate(enabled=True)
    assert not gate.allows_claim(10.)
    gate.observe(permission(), 10., 1000.)
    assert gate.allows_claim(11.9)
    assert not gate.allows_claim(12.1)
    gate.observe(permission(1003., False), 13., 1003.)
    assert not gate.allows_claim(13.)


@pytest.mark.parametrize('payload', [
    {}, [], None, {'source': 'another_node', 'stamp_sec': 1000., 'accepting': True},
    {'source': 'optional_idle', 'stamp_sec': float('nan'), 'accepting': True},
    {'source': 'optional_idle', 'stamp_sec': 995., 'accepting': True},
    {'source': 'optional_idle', 'stamp_sec': 1001., 'accepting': True},
    {'source': 'optional_idle', 'stamp_sec': 1000., 'accepting': 'true'},
])
def test_bad_or_replayed_permission_never_opens_gate(payload):
    gate = TaskAdmissionGate(enabled=True)
    gate.observe(payload, 10., 1000.)
    assert not gate.allows_claim(10.)


def test_old_allow_cannot_override_new_home_stop():
    gate = TaskAdmissionGate(enabled=True)
    gate.observe(permission(1001., False), 11., 1001.)
    gate.observe(permission(1000., True), 11.1, 1001.1)
    assert not gate.allows_claim(11.1)


def test_bridge_stops_claim_submission_but_does_not_drop_pending_claim_record():
    from types import SimpleNamespace
    from smart_carrier_api.bridge_node import ApiBridgeNode
    node = object.__new__(ApiBridgeNode)
    node.task_admission = TaskAdmissionGate(enabled=True)
    node.store = SimpleNamespace(get_pending_claim=lambda: pytest.fail('must not retry a claim now'))
    node._submit = lambda *args, **kwargs: pytest.fail('no claim POST while home is latched')
    assert not node._try_submit_claim(10., 2.)
