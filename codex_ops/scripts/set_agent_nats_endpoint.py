#!/usr/bin/env python3
"""Safely update the single NATS endpoint in an agent JSON configuration."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlparse


def update_endpoint(
    path: Path,
    *,
    endpoint: str,
    require_agent: str = "",
) -> Path | None:
    parsed = urlparse(endpoint)
    if parsed.scheme != "tls" or not parsed.hostname or parsed.port != 4222:
        raise RuntimeError("endpoint must be tls://<host>:4222")

    path = path.expanduser().resolve(strict=True)
    original = path.stat()
    config = json.loads(path.read_text(encoding="utf-8"))
    if require_agent and config.get("agent_id") != require_agent:
        raise RuntimeError(
            f"refusing agent mismatch: expected {require_agent}, "
            f"got {config.get('agent_id')}"
        )
    nats = config.get("nats")
    if not isinstance(nats, dict) or not isinstance(nats.get("servers"), list):
        raise RuntimeError("refusing configuration without nats.servers")
    if nats["servers"] == [endpoint]:
        return None

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = path.with_name(f"{path.name}.bak-{stamp}")
    shutil.copy2(path, backup)
    nats["servers"] = [endpoint]
    rendered = json.dumps(config, indent=2, ensure_ascii=False) + "\n"

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        dir=path.parent,
    )
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
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--require-agent", default="")
    args = parser.parse_args()

    backup = update_endpoint(
        args.config,
        endpoint=args.endpoint,
        require_agent=args.require_agent,
    )
    if backup:
        print(f"NATS endpoint updated; backup: {backup}")
    else:
        print("NATS endpoint already matched; no change made")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
