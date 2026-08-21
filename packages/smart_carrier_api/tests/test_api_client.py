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
        {"slot": 2, "status": "low", "charge": 45, "sensor_ok": True},
        {"slot": 3, "status": "empty", "charge": 0, "sensor_ok": True},
    ]
    with patch(
        "smart_carrier_api.api_client.urlopen", return_value=response
    ) as mocked:
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
    with patch(
        "smart_carrier_api.api_client.urlopen", return_value=response
    ) as mocked:
        client.release_task("task-1", "inventory changed")

    request = mocked.call_args.args[0]
    assert request.full_url.endswith("/api/v1/robots/R1/tasks/task-1/release")
    assert json.loads(request.data) == {"note": "inventory changed"}
