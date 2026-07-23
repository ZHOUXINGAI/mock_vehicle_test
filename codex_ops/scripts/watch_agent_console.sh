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

service="codex-agentd-$agent.service"

printf 'Visible Codex console for %s\n' "$agent"
printf 'Service: %s\n' "$service"
printf 'Shows readable tasks, commands, file changes, tools, handoffs and results.\n'
printf 'Codex activity appears only when codex.enabled=true for this agent.\n'
printf 'This console is read-only; Ctrl-C stops viewing, not the worker.\n\n'

if journalctl -u "$service" -n 1 --no-pager >/dev/null 2>&1; then
  exec journalctl -u "$service" -n 50 -f -o cat
fi

echo "Journal access requires local interactive sudo authorization."
exec sudo journalctl -u "$service" -n 50 -f -o cat
