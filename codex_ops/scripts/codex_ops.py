#!/usr/bin/env python3
"""Small helper for rover/docking Codex coordination."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Any


OPS_ROOT = Path(__file__).resolve().parents[1]
VALID_AGENTS = {"rover", "docking"}
VALID_NOTE_TYPES = {
    "request",
    "result",
    "decision",
    "blocker",
    "observation",
    "handoff",
    "ack",
}
CST = timezone(timedelta(hours=8))


def now_cst() -> datetime:
    return datetime.now(CST)


def iso_now() -> str:
    return now_cst().isoformat(timespec="seconds")


def event_path(ts: datetime | None = None) -> Path:
    ts = ts or now_cst()
    path = OPS_ROOT / "events" / f"{ts.year:04d}" / f"{ts.date().isoformat()}.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def append_event(agent: str, event_type: str, summary: str, details: str = "", **extra: Any) -> None:
    record: dict[str, Any] = {
        "ts": iso_now(),
        "agent": agent,
        "type": event_type,
        "summary": summary,
    }
    if details:
        record["details"] = details
    record.update({k: v for k, v in extra.items() if v not in (None, "", [])})
    with event_path().open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")


def run(command: list[str], cwd: Path | None = None, timeout: float = 5.0) -> str:
    try:
        completed = subprocess.run(
            command,
            cwd=str(cwd) if cwd else None,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
    except Exception as exc:
        return f"ERROR: {exc}"
    output = completed.stdout.strip()
    error = completed.stderr.strip()
    if completed.returncode != 0 and error:
        return error
    return output


def git_summary(repo: str | None) -> dict[str, Any]:
    if not repo:
        return {}
    path = Path(repo).expanduser()
    if not path.exists():
        return {"repo": str(path), "error": "path does not exist"}
    root = run(["git", "rev-parse", "--show-toplevel"], cwd=path)
    if root.startswith("fatal:") or root.startswith("ERROR:"):
        return {"repo": str(path), "error": root}
    root_path = Path(root)
    status = run(["git", "status", "--porcelain"], cwd=root_path, timeout=10.0)
    lines = [line for line in status.splitlines() if line.strip()]
    return {
        "repo": str(root_path),
        "branch": run(["git", "branch", "--show-current"], cwd=root_path),
        "head": run(["git", "rev-parse", "--short", "HEAD"], cwd=root_path),
        "remote": run(["git", "remote", "get-url", "origin"], cwd=root_path),
        "dirty_count": len(lines),
        "dirty_sample": lines[:20],
    }


def require_agent(agent: str) -> None:
    if agent not in VALID_AGENTS:
        raise SystemExit(f"agent must be one of {sorted(VALID_AGENTS)}")


def slugify(text: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", text.strip().lower()).strip("-")
    return slug[:80] or "note"


def cmd_checkin(args: argparse.Namespace) -> int:
    require_agent(args.agent)
    status = {
        "agent": args.agent,
        "updated_at": iso_now(),
        "status": args.status,
        "task": args.task,
        "repo": args.repo,
        "repo_git": git_summary(args.repo),
        "needs": args.need or [],
        "notes": args.notes,
    }
    path = OPS_ROOT / "agents" / f"{args.agent}.json"
    path.write_text(json.dumps(status, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    append_event(args.agent, "checkin", f"{args.status}: {args.task}", args.notes, repo=args.repo)
    print(f"updated {path}")
    return 0


def cmd_event(args: argparse.Namespace) -> int:
    require_agent(args.agent)
    append_event(args.agent, args.type, args.summary, args.details)
    print(f"event appended to {event_path()}")
    return 0


def cmd_note(args: argparse.Namespace) -> int:
    require_agent(args.from_agent)
    require_agent(args.to_agent)
    if args.type not in VALID_NOTE_TYPES:
        raise SystemExit(f"type must be one of {sorted(VALID_NOTE_TYPES)}")
    stamp = now_cst().strftime("%Y%m%d_%H%M%S")
    slug = slugify(args.title)
    path = OPS_ROOT / "inbox" / args.to_agent / f"{stamp}_{args.from_agent}_{args.type}_{slug}.md"
    files = args.file or []
    related = "\n".join(f"- `{item}`" for item in files) if files else "- none"
    content = f"""# {args.title}

Status: open
Type: {args.type}
From: {args.from_agent}
To: {args.to_agent}
Created: {iso_now()}

## Summary

{args.summary or "none"}

## Related Files Or Commits

{related}

## Need From Peer

{args.need or "none"}

## Expected Validation

{args.validation or "none"}

## Safety Or Scope Limits

{args.safety or "none"}

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
{path}
```
"""
    path.write_text(content, encoding="utf-8")
    append_event(
        args.from_agent,
        args.type,
        args.title,
        args.summary,
        to=args.to_agent,
        note=str(path),
        files=files,
    )
    print(path)
    return 0


def load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"error": str(exc)}


def cmd_doctor(args: argparse.Namespace) -> int:
    git_root = run(["git", "rev-parse", "--show-toplevel"], cwd=OPS_ROOT)
    git_cwd = Path(git_root) if not git_root.startswith(("fatal:", "ERROR:")) else OPS_ROOT
    print(f"ops_root: {OPS_ROOT}")
    print(f"git_root: {git_root}")
    print(f"git_head: {run(['git', 'rev-parse', '--short', 'HEAD'], cwd=git_cwd)}")
    print(f"git_branch: {run(['git', 'branch', '--show-current'], cwd=git_cwd)}")
    print(f"git_remote: {run(['git', 'remote', 'get-url', 'origin'], cwd=git_cwd)}")
    print()
    for agent in sorted(VALID_AGENTS):
        status = load_json(OPS_ROOT / "agents" / f"{agent}.json")
        print(f"{agent}: {status.get('status')} updated={status.get('updated_at')} task={status.get('task')}")
    print()
    for agent in sorted(VALID_AGENTS):
        notes = sorted((OPS_ROOT / "inbox" / agent).glob("*.md"))
        print(f"inbox/{agent}: {len(notes)} note(s)")
        for note in notes[-10:]:
            print(f"  - {note.relative_to(OPS_ROOT)}")
    if args.agent:
        require_agent(args.agent)
        print()
        print(f"recommended read list for {args.agent}:")
        print("  - state/meeting_state.md")
        print("  - contracts/codex_sync_protocol.md")
        print(f"  - agents/{args.agent}.json")
        print(f"  - inbox/{args.agent}/")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("checkin", help="Update agent heartbeat/status.")
    p.add_argument("--agent", required=True, choices=sorted(VALID_AGENTS))
    p.add_argument("--repo", required=True)
    p.add_argument("--status", default="working")
    p.add_argument("--task", required=True)
    p.add_argument("--need", action="append")
    p.add_argument("--notes", default="")
    p.set_defaults(func=cmd_checkin)

    p = sub.add_parser("event", help="Append an event.")
    p.add_argument("--agent", required=True, choices=sorted(VALID_AGENTS))
    p.add_argument("--type", required=True)
    p.add_argument("--summary", required=True)
    p.add_argument("--details", default="")
    p.set_defaults(func=cmd_event)

    p = sub.add_parser("note", help="Write a peer inbox note.")
    p.add_argument("--from", dest="from_agent", required=True, choices=sorted(VALID_AGENTS))
    p.add_argument("--to", dest="to_agent", required=True, choices=sorted(VALID_AGENTS))
    p.add_argument("--type", required=True, choices=sorted(VALID_NOTE_TYPES))
    p.add_argument("--title", required=True)
    p.add_argument("--summary", default="")
    p.add_argument("--need", default="")
    p.add_argument("--validation", default="")
    p.add_argument("--safety", default="")
    p.add_argument("--file", action="append")
    p.set_defaults(func=cmd_note)

    p = sub.add_parser("doctor", help="Show current coordination status.")
    p.add_argument("--agent", choices=sorted(VALID_AGENTS))
    p.set_defaults(func=cmd_doctor)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
