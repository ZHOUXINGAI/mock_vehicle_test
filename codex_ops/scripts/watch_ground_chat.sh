#!/usr/bin/env bash
set -euo pipefail

ops_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config="${1:-$ops_root/local/boss.json}"

clear
printf 'Ground ↔ Orin Codex Chat (read-only mirror)\n'
printf 'Tasks and replies appear as chat panels; commands/tools appear as activity lines.\n'
printf 'This window does not execute or consume tasks. Ctrl-C only closes the mirror.\n\n'

exec "$ops_root/scripts/coordctl.sh" \
  --config "$config" watch --subject 'codex.event.>' --chat
