#!/bin/bash
set -euo pipefail

if [[ $EUID -ne 0 || $# -ne 1 ]]; then
  echo "usage: sudo $0 <orin1-carrier|orin2-mini>" >&2
  exit 2
fi

agent="$1"
case "$agent" in
  orin1-carrier)
    operator=jetson
    ;;
  orin2-mini)
    operator=seeed
    ;;
  *)
    echo "unknown agent: $agent" >&2
    exit 2
    ;;
esac

service="codex-agentd-$agent.service"
control="/usr/local/sbin/codex-agentd-$agent-control"
sudoers="/etc/sudoers.d/codex-agentd-$agent-control"

if ! /usr/bin/systemctl cat "$service" >/dev/null 2>&1; then
  echo "service is not installed: $service" >&2
  exit 1
fi
if ! command -v visudo >/dev/null 2>&1; then
  echo "visudo is required" >&2
  exit 1
fi

control_tmp="$(mktemp)"
sudoers_tmp="$(mktemp)"
cleanup() {
  rm -f -- "$control_tmp" "$sudoers_tmp"
}
trap cleanup EXIT

cat >"$control_tmp" <<EOF
#!/bin/bash
set -euo pipefail

readonly service="$service"
case "\${1:-}" in
  status)
    exec /usr/bin/systemctl status "\$service" --no-pager
    ;;
  restart)
    /usr/bin/systemctl restart "\$service"
    exec /usr/bin/systemctl is-active "\$service"
    ;;
  *)
    echo "usage: \$0 <status|restart>" >&2
    exit 2
    ;;
esac
EOF

cat >"$sudoers_tmp" <<EOF
$operator ALL=(root) NOPASSWD: $control status
$operator ALL=(root) NOPASSWD: $control restart
EOF

chmod 0755 "$control_tmp"
chmod 0440 "$sudoers_tmp"
visudo -cf "$sudoers_tmp"
install -o root -g root -m 0755 "$control_tmp" "$control"
install -o root -g root -m 0440 "$sudoers_tmp" "$sudoers"
visudo -cf "$sudoers"

echo "Installed restricted control: $control"
echo "Allowed passwordless actions for $operator: status, restart"
