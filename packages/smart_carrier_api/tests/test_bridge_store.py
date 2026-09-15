from smart_carrier_api.bridge_store import BridgeStore, is_terminal_result_error


def test_only_terminal_result_errors_are_locally_settled():
    assert is_terminal_result_error("done", 404)
    assert is_terminal_result_error("cancelled", 404)
    assert is_terminal_result_error("released", 409)
    assert not is_terminal_result_error("done", 409)
    assert not is_terminal_result_error("done", 500)
    assert not is_terminal_result_error("done", None)


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
    assert store.enqueue_result(result)
    assert not store.enqueue_result(result)
    item = store.next_result(now=10**12)
    assert item["attempts"] == 0
    assert store.mark_retry("event-1", 0, now=100.0) == 101.0
    assert store.next_result(now=100.5) is None
    assert store.next_result(now=101.0)["attempts"] == 1
    store.mark_delivered("event-1")
    assert not store.has_pending_results()
    store.close()


def test_settled_result_cannot_be_replayed_after_restart(tmp_path):
    path = tmp_path / "bridge.sqlite3"
    store = BridgeStore(path)
    store.set_active_task({"id": "task-1"})
    store.set_claimed_tasks([{"id": "task-1"}, {"id": "task-2"}])
    store.enqueue_progress(
        {"event_id": "p-1", "task_id": "task-1", "progress_state": "navigating"}
    )
    result = {
        "event_id": "event-1",
        "task_id": "task-1",
        "status": "done",
        "note": "confirmed",
    }
    assert store.enqueue_result(result)
    store.settle_result("event-1", "task-1")
    assert store.is_result_settled("event-1")
    assert not store.has_pending_results()
    assert not store.has_pending_progress("task-1")
    assert [task["id"] for task in store.get_claimed_tasks()] == ["task-2"]
    assert store.get_active_task() is None
    store.close()

    recovered = BridgeStore(path)
    assert not recovered.enqueue_result(result)
    assert recovered.is_result_settled("event-1")
    assert not recovered.has_pending_results()
    recovered.close()


def test_pending_progress_preserves_each_event_in_order(tmp_path):
    store = BridgeStore(tmp_path / "bridge.sqlite3")
    store.enqueue_progress(
        {"event_id": "p-1", "task_id": "task-1", "progress_state": "wrong_slot"}
    )
    store.enqueue_progress(
        {"event_id": "p-2", "task_id": "task-1", "progress_state": "verifying_action"}
    )
    assert store.get_pending_progress()["progress_state"] == "wrong_slot"
    store.mark_progress_delivered("p-1")
    assert store.get_pending_progress("task-1")["progress_state"] == "verifying_action"
    assert store.get_pending_progress("missing") is None
    store.set_pending_progress(None)
    assert store.get_pending_progress() is None
    store.close()


def test_claimed_queue_and_progress_for_multiple_tasks_survive_restart(tmp_path):
    path = tmp_path / "bridge.sqlite3"
    store = BridgeStore(path)
    store.set_claimed_tasks([{"id": "task-1"}, {"id": "task-2"}])
    store.enqueue_progress(
        {"event_id": "p-1", "task_id": "task-1", "progress_state": "navigating"}
    )
    store.enqueue_progress(
        {"event_id": "p-2", "task_id": "task-2", "progress_state": "queued_on_robot"}
    )
    store.close()

    recovered = BridgeStore(path)
    assert [task["id"] for task in recovered.get_claimed_tasks()] == ["task-1", "task-2"]
    assert recovered.get_pending_progress()["task_id"] == "task-1"
    recovered.clear_pending_progress("task-1")
    assert recovered.get_pending_progress()["task_id"] == "task-2"
    recovered.remove_claimed_task("task-1")
    assert [task["id"] for task in recovered.get_claimed_tasks()] == ["task-2"]
    recovered.close()


def test_pending_claim_survives_restart(tmp_path):
    path = tmp_path / "bridge.sqlite3"
    store = BridgeStore(path)
    store.set_pending_claim({"claim_request_id": "request-1", "slots": []})
    store.close()

    recovered = BridgeStore(path)
    assert recovered.get_pending_claim()["claim_request_id"] == "request-1"
    recovered.set_pending_claim(None)
    assert recovered.get_pending_claim() is None
    recovered.close()
