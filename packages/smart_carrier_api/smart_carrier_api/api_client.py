import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class ApiError(RuntimeError):
    def __init__(self, message: str, *, status_code: int | None = None):
        super().__init__(message)
        self.status_code = status_code


class SmartCarrierApi:
    def __init__(self, base_url: str, robot_id: str, token: str, timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.token = token
        self.timeout = timeout

    def _request(self, method: str, path: str, payload: dict[str, Any] | None = None):
        body = (
            json.dumps(payload, ensure_ascii=False).encode("utf-8") if payload is not None else None
        )
        request = Request(
            f"{self.base_url}{path}",
            data=body,
            method=method,
            headers={
                "Authorization": f"Bearer {self.token}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                if response.status == 204:
                    return None
                raw = response.read()
                return json.loads(raw) if raw else None
        except HTTPError as exc:
            if exc.code == 204:
                return None
            detail = exc.read().decode(errors="replace")
            raise ApiError(
                f"API returned {exc.code}: {detail}", status_code=exc.code
            ) from exc
        except URLError as exc:
            raise ApiError(f"API connection failed: {exc.reason}") from exc

    def heartbeat(self, payload: dict[str, Any]):
        return self._request("POST", f"/api/v1/robots/{self.robot_id}/heartbeat", payload)

    def claim_task(self, slots: list[dict[str, Any]], claim_request_id: str | None = None):
        payload: dict[str, Any] = {"slots": slots}
        if claim_request_id is not None:
            payload["claim_request_id"] = claim_request_id
        return self._request(
            "POST",
            f"/api/v1/robots/{self.robot_id}/tasks/claim",
            payload,
        )

    def list_tasks(self):
        return self._request("GET", f"/api/v1/robots/{self.robot_id}/tasks")

    def get_task(self, task_id: str):
        return self._request(
            "GET",
            f"/api/v1/robots/{self.robot_id}/tasks/{task_id}",
        )

    def release_task(self, task_id: str, event_id: str, note: str | None = None):
        return self._request(
            "POST",
            f"/api/v1/robots/{self.robot_id}/tasks/{task_id}/release",
            {"event_id": event_id, "note": note},
        )

    def report_result(
        self,
        task_id: str,
        event_id: str,
        status: str,
        note: str | None = None,
    ):
        return self._request(
            "POST",
            f"/api/v1/robots/{self.robot_id}/tasks/{task_id}/result",
            {"event_id": event_id, "status": status, "note": note},
        )

    def report_progress(self, task_id: str, progress: dict[str, Any]):
        return self._request(
            "POST",
            f"/api/v1/robots/{self.robot_id}/tasks/{task_id}/progress",
            progress,
        )
