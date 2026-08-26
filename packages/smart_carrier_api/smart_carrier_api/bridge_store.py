from __future__ import annotations

import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any

RETRY_DELAYS = (1.0, 2.0, 5.0, 10.0, 30.0)


class BridgeStore:
    """Durable active-task state and terminal-result outbox."""

    def __init__(self, path: str | Path | None = None):
        if path is None:
            state_root = Path(
                os.getenv(
                    "SMART_CARRIER_STATE_DIR",
                    Path.home() / ".local" / "state" / "smart-carrier",
                )
            )
            path = state_root / "api_bridge.sqlite3"
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.connection = sqlite3.connect(self.path)
        self.connection.row_factory = sqlite3.Row
        self.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS state (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            )
            """
        )
        self.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS result_outbox (
                event_id TEXT PRIMARY KEY,
                task_id TEXT NOT NULL,
                status TEXT NOT NULL,
                note TEXT,
                attempts INTEGER NOT NULL DEFAULT 0,
                next_attempt_at REAL NOT NULL DEFAULT 0,
                created_at REAL NOT NULL
            )
            """
        )
        self.connection.commit()

    def close(self) -> None:
        self.connection.close()

    def _set_json(self, key: str, value: dict[str, Any] | None) -> None:
        if value is None:
            self.connection.execute("DELETE FROM state WHERE key = ?", (key,))
        else:
            self.connection.execute(
                """
                INSERT INTO state(key, value) VALUES(?, ?)
                ON CONFLICT(key) DO UPDATE SET value = excluded.value
                """,
                (key, json.dumps(value, ensure_ascii=False)),
            )
        self.connection.commit()

    def _get_json(self, key: str) -> dict[str, Any] | None:
        row = self.connection.execute(
            "SELECT value FROM state WHERE key = ?", (key,)
        ).fetchone()
        return json.loads(row["value"]) if row else None

    def set_active_task(self, task: dict[str, Any] | None) -> None:
        self._set_json("active_task", task)

    def get_active_task(self) -> dict[str, Any] | None:
        return self._get_json("active_task")

    def set_pending_progress(self, progress: dict[str, Any] | None) -> None:
        self._set_json("pending_progress", progress)

    def get_pending_progress(self) -> dict[str, Any] | None:
        return self._get_json("pending_progress")

    def enqueue_result(self, result: dict[str, Any]) -> None:
        self.connection.execute(
            """
            INSERT OR IGNORE INTO result_outbox(
                event_id, task_id, status, note, attempts, next_attempt_at, created_at
            ) VALUES (?, ?, ?, ?, 0, 0, ?)
            """,
            (
                str(result["event_id"]),
                str(result["task_id"]),
                str(result["status"]),
                result.get("note"),
                time.time(),
            ),
        )
        self.connection.commit()

    def next_result(self, now: float | None = None) -> dict[str, Any] | None:
        now = time.time() if now is None else now
        row = self.connection.execute(
            """
            SELECT event_id, task_id, status, note, attempts, next_attempt_at
            FROM result_outbox
            WHERE next_attempt_at <= ?
            ORDER BY created_at
            LIMIT 1
            """,
            (now,),
        ).fetchone()
        return dict(row) if row else None

    def mark_retry(self, event_id: str, attempts: int, now: float | None = None) -> float:
        now = time.time() if now is None else now
        delay = RETRY_DELAYS[min(attempts, len(RETRY_DELAYS) - 1)]
        next_attempt = now + delay
        self.connection.execute(
            """
            UPDATE result_outbox
            SET attempts = ?, next_attempt_at = ?
            WHERE event_id = ?
            """,
            (attempts + 1, next_attempt, event_id),
        )
        self.connection.commit()
        return next_attempt

    def mark_delivered(self, event_id: str) -> None:
        self.connection.execute(
            "DELETE FROM result_outbox WHERE event_id = ?", (event_id,)
        )
        self.connection.commit()

    def has_pending_results(self) -> bool:
        row = self.connection.execute("SELECT 1 FROM result_outbox LIMIT 1").fetchone()
        return row is not None
