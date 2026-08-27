from smart_carrier_api.bridge_store import BridgeStore


def test_store_recovers_active_task_and_result_outbox(tmp_path):
    path = tmp_path / "bridge.sqlite3"
    store = BridgeStore(path)
    store.set_active_task({"id": "task-1", "task_type": "borrow"})
    store.enqueue_result(
        {
            "event_id": "event-1",
            "task_id": "task-1",
            "status": "done",
            "note": "confirmed",
        }
    )
    store.close()

    recovered = BridgeStore(path)
    assert [task["id"] for task in recovered.get_claimed_tasks()] == ["task-1"]
    assert recovered.get_active_task() is None
    assert recovered.next_result(now=0)["event_id"] == "event-1"
    recovered.close()


def test_outbox_is_idempotent_and_uses_bounded_backoff(tmp_path):
    store = BridgeStore(tmp_path / "bridge.sqlite3")
    result = {
        "event_id": "event-1",
        "task_id": "task-1",
        "status": "done",
        "note": None,
    }
    store.enqueue_result(result)
    store.enqueue_result(result)
    item = store.next_result(now=10**12)
    assert item["attempts"] == 0
    assert store.mark_retry("event-1", 0, now=100.0) == 101.0
    assert store.next_result(now=100.5) is None
    assert store.next_result(now=101.0)["attempts"] == 1
    store.mark_delivered("event-1")
    assert not store.has_pending_results()
    store.close()


def test_pending_progress_keeps_only_latest_value(tmp_path):
    store = BridgeStore(tmp_path / "bridge.sqlite3")
    store.set_pending_progress({"task_id": "task-1", "progress_state": "navigating"})
    store.set_pending_progress({"task_id": "task-1", "progress_state": "arrived"})
    assert store.get_pending_progress()["progress_state"] == "arrived"
    store.set_pending_progress(None)
    assert store.get_pending_progress() is None
    store.close()


def test_claimed_queue_and_progress_for_multiple_tasks_survive_restart(tmp_path):
    path = tmp_path / "bridge.sqlite3"
    store = BridgeStore(path)
    store.set_claimed_tasks([{"id": "task-1"}, {"id": "task-2"}])
    store.set_pending_progress({"task_id": "task-1", "progress_state": "navigating"})
    store.set_pending_progress({"task_id": "task-2", "progress_state": "queued_on_robot"})
    store.close()

    recovered = BridgeStore(path)
    assert [task["id"] for task in recovered.get_claimed_tasks()] == ["task-1", "task-2"]
    assert recovered.get_pending_progress()["task_id"] == "task-1"
    recovered.clear_pending_progress("task-1")
    assert recovered.get_pending_progress()["task_id"] == "task-2"
    recovered.remove_claimed_task("task-1")
    assert [task["id"] for task in recovered.get_claimed_tasks()] == ["task-2"]
    recovered.close()
