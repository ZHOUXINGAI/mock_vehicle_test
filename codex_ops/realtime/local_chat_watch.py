"""Follow an agent's private local event mirror as a terminal chat transcript."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from .console import format_event_as_chat


def render_event_line(line: str) -> str | None:
    try:
        event = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(event, dict):
        return None
    return format_event_as_chat(event)


def follow(path: Path) -> None:
    while not path.exists():
        time.sleep(0.2)
    with path.open(encoding="utf-8") as source:
        source.seek(0, 2)
        while True:
            line = source.readline()
            if not line:
                time.sleep(0.1)
                continue
            rendered = render_event_line(line)
            if rendered:
                print(rendered, flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", required=True, type=Path)
    args = parser.parse_args()
    follow(args.file.expanduser())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
