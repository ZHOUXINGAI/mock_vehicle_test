#!/usr/bin/env python3
"""Safely toggle Codex execution in an installed agentd JSON configuration."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def update_config(
    path: Path,
    *,
    enabled: bool,
    require_agent: str,
    require_mode: str,
) -> Path | None:
    path = path.resolve(strict=True)
    original = path.stat()
    config: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    agent_id = str(config.get("agent_id", ""))
    mode = str(config.get("policy", {}).get("mode", ""))
    if agent_id != require_agent:
        raise RuntimeError(f"refusing agent mismatch: expected {require_agent}, got {agent_id}")
    if mode != require_mode:
        raise RuntimeError(f"refusing policy mode mismatch: expected {require_mode}, got {mode}")
    codex = config.get("codex")
    if not isinstance(codex, dict):
        raise RuntimeError("refusing configuration without codex object")
    if bool(codex.get("enabled")) == enabled:
        return None

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = path.with_name(f"{path.name}.bak-{stamp}")
    shutil.copy2(path, backup)
    codex["enabled"] = enabled
    rendered = json.dumps(config, indent=2, ensure_ascii=False) + "\n"

    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, original.st_mode)
        os.chown(temporary, original.st_uid, original.st_gid)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return backup


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--enabled", required=True, choices=["true", "false"])
    parser.add_argument("--require-agent", required=True)
    parser.add_argument("--require-mode", default="observe")
    args = parser.parse_args()

    desired = args.enabled == "true"
    backup = update_config(
        args.config,
        enabled=desired,
        require_agent=args.require_agent,
        require_mode=args.require_mode,
    )
    state = "enabled" if desired else "disabled"
    if backup:
        print(f"Codex execution {state}; backup: {backup}")
    else:
        print(f"Codex execution already {state}; no change made")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
