#!/usr/bin/env bash
set -euo pipefail

agent="${1:-}"
case "$agent" in
  orin1-carrier|orin2-mini) ;;
  *)
    echo "usage: $0 <orin1-carrier|orin2-mini>" >&2
    exit 2
    ;;
esac

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
config="$repo/codex_ops/local/$agent/visible-app-bridge.json"
if [[ ! -r "$config" ]]; then
  echo "Bridge config is not readable: $config" >&2
  exit 1
fi

clear
printf '%s Codex Chat (read-only mirror)\n' "$agent"
printf 'Ground tasks, Codex replies, commands and peer handoffs appear here.\n'
printf 'The Bridge remains the only task consumer. Ctrl-C only closes this mirror.\n\n'

exec "$repo/codex_ops/scripts/coordctl.sh" \
  --config "$config" watch --subject "codex.event.$agent" --chat
