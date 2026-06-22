#!/usr/bin/env bash

set -euo pipefail

QGC_SETTINGS_FILE="${QGC_SETTINGS_FILE:-$HOME/.config/QGroundControl.org/QGroundControl.ini}"
QGC_UDP_LISTEN_PORT="${QGC_UDP_LISTEN_PORT:-14550}"
QGC_UDP_TARGET_HOST="${QGC_UDP_TARGET_HOST:-127.0.0.1}"
QGC_UDP_TARGET_PORT="${QGC_UDP_TARGET_PORT:-14555}"
QGC_ALLOW_RUNNING="${QGC_ALLOW_RUNNING:-false}"

if pgrep -f '/QGroundControl( |$)' >/dev/null 2>&1; then
  if [ "$QGC_ALLOW_RUNNING" != "true" ]; then
    echo "QGroundControl is running. Close it before changing its settings." >&2
    echo "Current process:" >&2
    pgrep -af '/QGroundControl( |$)' >&2 || true
    exit 1
  fi
fi

mkdir -p "$(dirname "$QGC_SETTINGS_FILE")"

python3 - "$QGC_SETTINGS_FILE" "$QGC_UDP_LISTEN_PORT" "$QGC_UDP_TARGET_HOST" "$QGC_UDP_TARGET_PORT" <<'PY'
from __future__ import annotations

import datetime
import pathlib
import shutil
import sys

path = pathlib.Path(sys.argv[1]).expanduser()
udp_listen_port = sys.argv[2]
udp_target_host = sys.argv[3]
udp_target_port = sys.argv[4]

desired = [
    ("autoConnectUDP", "true"),
    ("autoConnectPixhawk", "false"),
    ("udpListenPort", udp_listen_port),
    ("udpTargetHostIP", udp_target_host),
    ("udpTargetHostPort", udp_target_port),
]

if path.exists():
    original = path.read_text(encoding="utf-8").splitlines(keepends=True)
else:
    original = []

lines = list(original)

def key_of(line: str) -> str | None:
    stripped = line.strip()
    if not stripped or stripped.startswith(("#", ";", "[")):
        return None
    if "=" not in line:
        return None
    return line.split("=", 1)[0].strip()

section_start = None
section_end = len(lines)
for index, line in enumerate(lines):
    if line.strip() == "[LinkManager]":
        section_start = index
        section_end = len(lines)
        for end_index in range(index + 1, len(lines)):
            if lines[end_index].lstrip().startswith("["):
                section_end = end_index
                break
        break

if section_start is None:
    if lines and lines[-1].strip():
        lines.append("\n")
    lines.append("[LinkManager]\n")
    for key, value in desired:
        lines.append(f"{key}={value}\n")
else:
    found: set[str] = set()
    for index in range(section_start + 1, section_end):
        existing_key = key_of(lines[index])
        if existing_key is None:
            continue
        for key, value in desired:
            if existing_key == key:
                lines[index] = f"{key}={value}\n"
                found.add(key)
                break

    insert_at = section_end
    additions = [f"{key}={value}\n" for key, value in desired if key not in found]
    if additions:
        lines[insert_at:insert_at] = additions

new_text = "".join(lines)
old_text = "".join(original)

if new_text != old_text:
    if path.exists():
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = path.with_name(f"{path.name}.bak.{stamp}")
        shutil.copy2(path, backup)
        print(f"Backed up QGC settings: {backup}")
    path.write_text(new_text, encoding="utf-8")
    print(f"Updated QGC settings: {path}")
else:
    print(f"QGC settings already correct: {path}")

print("QGC LinkManager:")
for key, value in desired:
    print(f"  {key}={value}")
PY
