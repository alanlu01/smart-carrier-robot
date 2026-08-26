import json
from unittest.mock import MagicMock, patch

from smart_carrier_api.api_client import SmartCarrierApi


def test_heartbeat_sends_robot_token_and_payload():
    response = MagicMock()
    response.status = 200
    response.read.return_value = b'{"ok": true}'
    response.__enter__.return_value = response
    response.__exit__.return_value = False

    client = SmartCarrierApi("https://api.example.test/", "R1", "secret-token")
    with patch("smart_carrier_api.api_client.urlopen", return_value=response) as mocked:
        result = client.heartbeat({"mode": "idle", "slots": []})

    request = mocked.call_args.args[0]
    assert request.full_url == "https://api.example.test/api/v1/robots/R1/heartbeat"
    assert request.method == "POST"
    assert request.headers["Authorization"] == "Bearer secret-token"
    assert json.loads(request.data) == {"mode": "idle", "slots": []}
    assert result == {"ok": True}


def test_claim_task_accepts_no_content():
    response = MagicMock()
    response.status = 204
    response.__enter__.return_value = response
    response.__exit__.return_value = False

    client = SmartCarrierApi("https://api.example.test", "R1", "secret-token")
    slots = [
        {"slot": 1, "status": "ready", "charge": 96, "sensor_ok": True},
        {"slot": 2, "status": "low", "charge": None, "sensor_ok": True},
        {"slot": 3, "status": "empty", "charge": 0, "sensor_ok": True},
    ]
    with patch("smart_carrier_api.api_client.urlopen", return_value=response) as mocked:
        assert client.claim_task(slots) is None

    request = mocked.call_args.args[0]
    assert request.full_url == "https://api.example.test/api/v1/robots/R1/tasks/claim"
    assert json.loads(request.data) == {"slots": slots}


def test_release_task_sends_note_to_owned_task_endpoint():
    response = MagicMock()
    response.status = 200
    response.read.return_value = b'{"message": "ok"}'
    response.__enter__.return_value = response
    response.__exit__.return_value = False

    client = SmartCarrierApi("https://api.example.test", "R1", "secret-token")
    with patch("smart_carrier_api.api_client.urlopen", return_value=response) as mocked:
        client.release_task("task-1", "event-1", "inventory changed")

    request = mocked.call_args.args[0]
    assert request.full_url.endswith("/api/v1/robots/R1/tasks/task-1/release")
    assert json.loads(request.data) == {
        "event_id": "event-1",
        "note": "inventory changed",
    }


def test_get_task_reads_cancellation_state_from_owned_task_endpoint():
    response = MagicMock()
    response.status = 200
    response.read.return_value = b'{"id":"task-1","cancel_requested_at":"2026-08-25T01:00:00Z"}'
    response.__enter__.return_value = response
    response.__exit__.return_value = False

    client = SmartCarrierApi("https://api.example.test", "R1", "secret-token")
    with patch("smart_carrier_api.api_client.urlopen", return_value=response) as mocked:
        result = client.get_task("task-1")

    request = mocked.call_args.args[0]
    assert request.full_url.endswith("/api/v1/robots/R1/tasks/task-1")
    assert request.method == "GET"
    assert result["cancel_requested_at"] == "2026-08-25T01:00:00Z"


def test_report_cancelled_result_posts_cancelled_status():
    response = MagicMock()
    response.status = 200
    response.read.return_value = b'{"ok":true}'
    response.__enter__.return_value = response
    response.__exit__.return_value = False

    client = SmartCarrierApi("https://api.example.test", "R1", "secret-token")
    with patch("smart_carrier_api.api_client.urlopen", return_value=response) as mocked:
        client.report_result("task-1", "result-cancelled", "cancelled", "Nav2 stopped")

    request = mocked.call_args.args[0]
    assert request.full_url.endswith("/api/v1/robots/R1/tasks/task-1/result")
    assert json.loads(request.data) == {
        "event_id": "result-cancelled",
        "status": "cancelled",
        "note": "Nav2 stopped",
    }


def test_result_and_progress_include_event_ids():
    response = MagicMock()
    response.status = 200
    response.read.return_value = b'{"ok": true}'
    response.__enter__.return_value = response
    response.__exit__.return_value = False
    client = SmartCarrierApi("https://api.example.test", "R1", "secret-token")

    with patch("smart_carrier_api.api_client.urlopen", return_value=response) as mocked:
        client.report_result("task-1", "result-1", "done", "confirmed")
        client.report_progress(
            "task-1",
            {"event_id": "progress-1", "progress_state": "navigating"},
        )

    result_request = mocked.call_args_list[0].args[0]
    progress_request = mocked.call_args_list[1].args[0]
    assert json.loads(result_request.data)["event_id"] == "result-1"
    assert progress_request.full_url.endswith("/api/v1/robots/R1/tasks/task-1/progress")
