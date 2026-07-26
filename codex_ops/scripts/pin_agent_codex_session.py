#!/usr/bin/env python3
"""Safely point an agent Bridge at an existing local Codex session."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path


SESSION_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{5,255}$")


def find_rollout(codex_home: Path, session_id: str) -> Path:
    if not SESSION_ID_RE.fullmatch(session_id):
        raise RuntimeError(f"invalid Codex session id: {session_id!r}")
    matches = sorted((codex_home / "sessions").glob(f"**/*{session_id}*.jsonl"))
    if not matches:
        raise RuntimeError(f"no rollout found for Codex session {session_id}")
    if len(matches) != 1:
        rendered = ", ".join(str(path) for path in matches)
        raise RuntimeError(f"multiple rollouts found for Codex session {session_id}: {rendered}")
    return matches[0].resolve(strict=True)


def pin_session(
    session_file: Path,
    *,
    codex_home: Path,
    agent_id: str,
    session_id: str,
) -> tuple[Path, Path | None]:
    rollout = find_rollout(codex_home.resolve(strict=True), session_id)
    session_file = session_file.expanduser().resolve()
    session_file.parent.mkdir(parents=True, exist_ok=True)

    backup: Path | None = None
    if session_file.exists():
        current = json.loads(session_file.read_text(encoding="utf-8"))
        if (
            current.get("agent_id") == agent_id
            and current.get("thread_id") == session_id
        ):
            return rollout, None
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        backup = session_file.with_name(f"{session_file.name}.bak-{stamp}")
        shutil.copy2(session_file, backup)

    rendered = json.dumps(
        {"agent_id": agent_id, "thread_id": session_id},
        indent=2,
        ensure_ascii=False,
    ) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{session_file.name}.",
        dir=session_file.parent,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, session_file)
    finally:
        if temporary.exists():
            temporary.unlink()
    return rollout, backup


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-file", required=True, type=Path)
    parser.add_argument("--codex-home", required=True, type=Path)
    parser.add_argument("--agent-id", required=True)
    parser.add_argument("--session-id", required=True)
    args = parser.parse_args()

    rollout, backup = pin_session(
        args.session_file,
        codex_home=args.codex_home,
        agent_id=args.agent_id,
        session_id=args.session_id,
    )
    print(f"Pinned {args.agent_id} to Codex session {args.session_id}")
    print(f"Verified rollout: {rollout}")
    if backup:
        print(f"Previous Bridge session pointer backup: {backup}")
    else:
        print("Bridge session pointer already matched; no backup needed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
