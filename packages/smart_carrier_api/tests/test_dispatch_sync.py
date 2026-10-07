"""Exercise startup reconciliation and ACK messages without network or ROS spin."""

import json
from types import SimpleNamespace

from smart_carrier_api.bridge_node import ApiBridgeNode
from smart_carrier_api.bridge_store import BridgeStore


def bridge(tmp_path):
    node = object.__new__(ApiBridgeNode)
    node.store = BridgeStore(tmp_path / 'bridge.sqlite3')
    node.claimed_tasks = []
    node.active_batch = None
    node.dispatch_request_id = None
    node.dispatch_empty_checked = False
    node.dispatch_last_publish_at = 0.0
    node.reconciled = True
    node.reconciliation_supported = True
    node.network_online = True
    node.collection_started_at = None
    node.last_batch_publish_at = 0.0
    node.messages, node.acks, node.syncs = [], [], []
    node.dispatch_sync_publisher = SimpleNamespace(publish=node.messages.append)
    node.result_ack_publisher = SimpleNamespace(publish=node.acks.append)
    node.result_sync_publisher = SimpleNamespace(publish=node.syncs.append)
    node.publish_claimed_orders = lambda: None
    node.last_cancel_publish_at, node.next_task_poll_at = {}, {}
    node.get_logger = lambda: SimpleNamespace(info=lambda _: None, warning=lambda _: None)
    return node


def test_new_nonce_cannot_reuse_old_empty_answer(tmp_path):
    node = bridge(tmp_path)
    node.dispatch_empty_checked = True
    node.on_dispatch_sync_request(SimpleNamespace(data='{"request_id":"fresh"}'))
    assert not node.reconciled
    assert not node.dispatch_empty_checked
    assert json.loads(node.messages[-1].data)['state'] == 'waiting'
    node.reconciled = node.dispatch_empty_checked = True
    node.publish_dispatch_sync(force=True)
    assert json.loads(node.messages[-1].data)['state'] == 'empty'
    # Retransmission of the same nonce must not reset an in-progress query.
    node.on_dispatch_sync_request(SimpleNamespace(data='{"request_id":"fresh"}'))
    assert node.dispatch_empty_checked
    node.store.close()


def test_offline_collecting_or_outbox_never_claim_empty(tmp_path):
    node = bridge(tmp_path)
    node.dispatch_request_id = 'r'
    node.dispatch_empty_checked = True
    node.network_online = False
    node.publish_dispatch_sync(force=True)
    assert json.loads(node.messages[-1].data)['state'] == 'waiting'
    node.network_online = True
    node.collection_started_at = 100
    node.publish_dispatch_sync(force=True)
    assert json.loads(node.messages[-1].data)['state'] == 'waiting'
    node.collection_started_at = None
    node.store.enqueue_result({'event_id': 'e', 'task_id': 't', 'status': 'done'})
    node.publish_dispatch_sync(force=True)
    assert json.loads(node.messages[-1].data)['state'] == 'waiting'
    node.store.close()


def test_local_durable_ack_is_not_cloud_confirmation(tmp_path):
    node = bridge(tmp_path)
    node.on_task_result(SimpleNamespace(data=json.dumps(
        {'event_id': 'e', 'task_id': 't', 'status': 'done'})))
    local = json.loads(node.acks[-1].data)
    assert local['ack_stage'] == 'local_durable' and not local['cloud_confirmed']
    node._ack_result({'event_id': 'e', 'task_id': 't', 'status': 'done'})
    cloud = json.loads(node.acks[-1].data)
    assert cloud['ack_stage'] == 'cloud_confirmed' and cloud['cloud_confirmed']
    node.store.close()


def test_reconciled_terminal_task_is_not_a_successful_cloud_result_post(tmp_path):
    node = bridge(tmp_path)
    item = {'event_id': 'e', 'task_id': 't', 'status': 'failed'}
    node.store.enqueue_result(item)
    node._ack_result(item, terminally_reconciled=True)
    ack = json.loads(node.acks[-1].data)
    assert ack['ack_stage'] == 'cloud_reconciled' and not ack['cloud_confirmed']
    node.store.close()
