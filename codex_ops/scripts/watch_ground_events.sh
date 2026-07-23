#!/usr/bin/env bash
set -euo pipefail

ops_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config="${1:-$ops_root/local/boss.json}"

printf 'Codex Ground Work Live\n'
printf 'Config: %s\n' "$config"
printf 'Readable task, command, file, tool, handoff and result activity.\n'
printf 'Orin2 remains silent until it is separately commissioned.\n\n'

exec "$ops_root/scripts/coordctl.sh" \
  --config "$config" watch --subject 'codex.event.>' --pretty
