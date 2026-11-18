"""ROS bridge admission wiring is tested in a real Jazzy Python environment."""

from types import SimpleNamespace

import pytest
from smart_carrier_api.bridge_node import ApiBridgeNode
from smart_carrier_api.task_admission import TaskAdmissionGate


def test_bridge_stops_claim_submission_but_does_not_drop_pending_claim_record():
    node = object.__new__(ApiBridgeNode)
    node.task_admission = TaskAdmissionGate(enabled=True)
    node.store = SimpleNamespace(
        get_pending_claim=lambda: pytest.fail('must not retry a claim now'))
    node._submit = lambda *args, **kwargs: pytest.fail('no claim POST while home is latched')
    assert not node._try_submit_claim(10., 2.)
