"""Small local idempotency store for at-least-once task delivery."""

from __future__ import annotations

import sqlite3
from pathlib import Path

from .protocol import TaskEnvelope, utc_now


class TaskStore:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.connection = sqlite3.connect(self.path)
        self.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS task_runs (
                task_id TEXT PRIMARY KEY,
                root_task_id TEXT NOT NULL,
                status TEXT NOT NULL,
                attempts INTEGER NOT NULL,
                updated_at TEXT NOT NULL,
                summary TEXT NOT NULL DEFAULT ''
            )
            """
        )
        self.connection.commit()

    def close(self) -> None:
        self.connection.close()

    def claim(self, task: TaskEnvelope) -> tuple[bool, int, str]:
        row = self.connection.execute(
            "SELECT status, attempts FROM task_runs WHERE task_id = ?", (task.task_id,)
        ).fetchone()
        if row and row[0] in {"completed", "rejected", "blocked"}:
            return False, int(row[1]), str(row[0])
        attempts = (int(row[1]) + 1) if row else 1
        self.connection.execute(
            """
            INSERT INTO task_runs(task_id, root_task_id, status, attempts, updated_at)
            VALUES(?, ?, 'running', ?, ?)
            ON CONFLICT(task_id) DO UPDATE SET
                status='running', attempts=excluded.attempts, updated_at=excluded.updated_at
            """,
            (task.task_id, task.root_task_id, attempts, utc_now()),
        )
        self.connection.commit()
        return True, attempts, "running"

    def finish(self, task_id: str, status: str, summary: str = "") -> None:
        self.connection.execute(
            "UPDATE task_runs SET status=?, summary=?, updated_at=? WHERE task_id=?",
            (status, summary[:20_000], utc_now(), task_id),
        )
        self.connection.commit()

    def status(self, task_id: str) -> str | None:
        row = self.connection.execute(
            "SELECT status FROM task_runs WHERE task_id = ?", (task_id,)
        ).fetchone()
        return str(row[0]) if row else None
