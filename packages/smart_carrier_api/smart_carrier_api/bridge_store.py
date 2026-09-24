from __future__ import annotations

import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any

RETRY_DELAYS = (1.0, 2.0, 5.0, 10.0, 30.0)


def is_terminal_result_error(result_status: str, status_code: int | None) -> bool:
    """Return whether retrying a terminal result can no longer change cloud state."""
    return status_code == 404 or (result_status == "released" and status_code == 409)


class BridgeStore:
    """Durable claimed-task queue, progress buffer, and terminal-result outbox."""

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
        self.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS progress_outbox (
                event_id TEXT PRIMARY KEY,
                task_id TEXT NOT NULL,
                payload TEXT NOT NULL,
                created_at REAL NOT NULL
            )
            """
        )
        self.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS settled_results (
                event_id TEXT PRIMARY KEY,
                task_id TEXT NOT NULL,
                settled_at REAL NOT NULL
            )
            """
        )
        self.connection.commit()
        self._migrate_legacy_progress()

    def close(self) -> None:
        self.connection.close()

    def _set_json(self, key: str, value: Any | None) -> None:
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

    def _get_json(self, key: str) -> Any | None:
        row = self.connection.execute("SELECT value FROM state WHERE key = ?", (key,)).fetchone()
        return json.loads(row["value"]) if row else None

    def set_active_task(self, task: dict[str, Any] | None) -> None:
        self._set_json("active_task", task)

    def get_active_task(self) -> dict[str, Any] | None:
        return self._get_json("active_task")

    def set_claimed_tasks(self, tasks: list[dict[str, Any]]) -> None:
        self._set_json("claimed_tasks", tasks if tasks else None)

    def get_claimed_tasks(self) -> list[dict[str, Any]]:
        tasks = self._get_json("claimed_tasks")
        if isinstance(tasks, list):
            return [task for task in tasks if isinstance(task, dict)]

        # Migrate the pre-queue single active task without losing an owned order.
        legacy = self.get_active_task()
        if isinstance(legacy, dict):
            self.set_claimed_tasks([legacy])
            self.set_active_task(None)
            return [legacy]
        return []

    def remove_claimed_task(self, task_id: str) -> None:
        self.set_claimed_tasks(
            [task for task in self.get_claimed_tasks() if str(task.get("id")) != str(task_id)]
        )

    def set_active_batch(self, batch: dict[str, Any] | None) -> None:
        self._set_json("active_batch", batch)

    def get_active_batch(self) -> dict[str, Any] | None:
        batch = self._get_json("active_batch")
        return batch if isinstance(batch, dict) else None

    def set_pending_claim(self, claim: dict[str, Any] | None) -> None:
        self._set_json("pending_claim", claim)

    def get_pending_claim(self) -> dict[str, Any] | None:
        claim = self._get_json("pending_claim")
        return claim if isinstance(claim, dict) else None

    def _get_pending_progresses(self) -> dict[str, dict[str, Any]]:
        progresses = self._get_json("pending_progresses")
        if isinstance(progresses, dict):
            return {
                str(task_id): progress
                for task_id, progress in progresses.items()
                if isinstance(progress, dict)
            }

        legacy = self._get_json("pending_progress")
        if isinstance(legacy, dict) and legacy.get("task_id") is not None:
            migrated = {str(legacy["task_id"]): legacy}
            self._set_json("pending_progresses", migrated)
            self._set_json("pending_progress", None)
            return migrated
        return {}

    def set_pending_progress(self, progress: dict[str, Any] | None) -> None:
        if progress is None:
            self.connection.execute("DELETE FROM progress_outbox")
            self.connection.commit()
            return
        self.enqueue_progress(progress)

    def get_pending_progress(self, task_id: str | None = None) -> dict[str, Any] | None:
        return self.next_progress(task_id)

    def clear_pending_progress(self, task_id: str) -> None:
        self.connection.execute(
            "DELETE FROM progress_outbox WHERE task_id = ?", (str(task_id),)
        )
        self.connection.commit()

    def _migrate_legacy_progress(self) -> None:
        progresses = self._get_pending_progresses()
        for progress in progresses.values():
            if progress.get("event_id") and progress.get("task_id"):
                self.enqueue_progress(progress)
        self._set_json("pending_progresses", None)
        self._set_json("pending_progress", None)

    def enqueue_progress(self, progress: dict[str, Any]) -> None:
        self.connection.execute(
            """
            INSERT OR IGNORE INTO progress_outbox(event_id, task_id, payload, created_at)
            VALUES (?, ?, ?, ?)
            """,
            (
                str(progress["event_id"]),
                str(progress["task_id"]),
                json.dumps(progress, ensure_ascii=False),
                time.time(),
            ),
        )
        self.connection.commit()

    def next_progress(self, task_id: str | None = None) -> dict[str, Any] | None:
        if task_id is None:
            row = self.connection.execute(
                "SELECT payload FROM progress_outbox ORDER BY created_at, rowid LIMIT 1"
            ).fetchone()
        else:
            row = self.connection.execute(
                """
                SELECT payload FROM progress_outbox
                WHERE task_id = ? ORDER BY created_at, rowid LIMIT 1
                """,
                (str(task_id),),
            ).fetchone()
        return json.loads(row["payload"]) if row else None

    def mark_progress_delivered(self, event_id: str) -> None:
        self.connection.execute(
            "DELETE FROM progress_outbox WHERE event_id = ?", (str(event_id),)
        )
        self.connection.commit()

    def has_pending_progress(self, task_id: str | None = None) -> bool:
        if task_id is None:
            row = self.connection.execute("SELECT 1 FROM progress_outbox LIMIT 1").fetchone()
        else:
            row = self.connection.execute(
                "SELECT 1 FROM progress_outbox WHERE task_id = ? LIMIT 1",
                (str(task_id),),
            ).fetchone()
        return row is not None

    def enqueue_result(self, result: dict[str, Any]) -> bool:
        cursor = self.connection.execute(
            """
            INSERT OR IGNORE INTO result_outbox(
                event_id, task_id, status, note, attempts, next_attempt_at, created_at
            )
            SELECT ?, ?, ?, ?, 0, 0, ?
            WHERE NOT EXISTS (
                SELECT 1 FROM settled_results WHERE event_id = ?
            )
            """,
            (
                str(result["event_id"]),
                str(result["task_id"]),
                str(result["status"]),
                result.get("note"),
                time.time(),
                str(result["event_id"]),
            ),
        )
        self.connection.commit()
        return cursor.rowcount > 0

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
        self.connection.execute("DELETE FROM result_outbox WHERE event_id = ?", (event_id,))
        self.connection.commit()

    def settle_result(self, event_id: str, task_id: str) -> None:
        """Atomically retire a result and all local state owned by its task."""
        event_id = str(event_id)
        task_id = str(task_id)
        claimed_tasks = [
            task
            for task in self.get_claimed_tasks()
            if str(task.get("id")) != task_id
        ]
        active_task = self.get_active_task()

        with self.connection:
            self.connection.execute(
                """
                INSERT INTO settled_results(event_id, task_id, settled_at)
                VALUES (?, ?, ?)
                ON CONFLICT(event_id) DO UPDATE SET
                    task_id = excluded.task_id,
                    settled_at = excluded.settled_at
                """,
                (event_id, task_id, time.time()),
            )
            self.connection.execute(
                "DELETE FROM result_outbox WHERE event_id = ?", (event_id,)
            )
            self.connection.execute(
                "DELETE FROM progress_outbox WHERE task_id = ?", (task_id,)
            )
            if claimed_tasks:
                self.connection.execute(
                    """
                    INSERT INTO state(key, value) VALUES('claimed_tasks', ?)
                    ON CONFLICT(key) DO UPDATE SET value = excluded.value
                    """,
                    (json.dumps(claimed_tasks, ensure_ascii=False),),
                )
            else:
                self.connection.execute("DELETE FROM state WHERE key = 'claimed_tasks'")
            if isinstance(active_task, dict) and str(
                active_task.get("id") or active_task.get("task_id")
            ) == task_id:
                self.connection.execute("DELETE FROM state WHERE key = 'active_task'")

            # Keep the idempotency ledger bounded while retaining ample restart history.
            self.connection.execute(
                """
                DELETE FROM settled_results
                WHERE event_id NOT IN (
                    SELECT event_id FROM settled_results
                    ORDER BY settled_at DESC LIMIT 256
                )
                """
            )

    def is_result_settled(self, event_id: str) -> bool:
        row = self.connection.execute(
            "SELECT 1 FROM settled_results WHERE event_id = ? LIMIT 1",
            (str(event_id),),
        ).fetchone()
        return row is not None

    def has_pending_results(self) -> bool:
        row = self.connection.execute("SELECT 1 FROM result_outbox LIMIT 1").fetchone()
        return row is not None

    def has_pending_result(self, task_id: str) -> bool:
        row = self.connection.execute(
            "SELECT 1 FROM result_outbox WHERE task_id = ? LIMIT 1",
            (str(task_id),),
        ).fetchone()
        return row is not None

    def has_result_event(self, event_id: str) -> bool:
        row = self.connection.execute(
            "SELECT 1 FROM result_outbox WHERE event_id = ? LIMIT 1",
            (str(event_id),),
        ).fetchone()
        return row is not None
