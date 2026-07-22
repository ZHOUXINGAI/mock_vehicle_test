#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 || $# -gt 2 ]]; then
  echo "usage: sudo $0 [ground-ip-or-dns] [certificate-source-directory]" >&2
  exit 2
fi

ground_host="${1:-192.168.43.13}"
source_dir="${2:-/home/jetson}"
repo=/home/jetson/mock_vehicle_test
config=/etc/codex-agentd/orin1-carrier.json
cert_dir=/etc/codex-agentd/certs

ca="$source_dir/ca.crt"
cert="$source_dir/orin1-carrier.crt"
key="$source_dir/orin1-carrier.key"

for file in "$ca" "$cert" "$key"; do
  if [[ ! -f "$file" ]]; then
    echo "missing required certificate file: $file" >&2
    exit 1
  fi
done

openssl verify -CAfile "$ca" "$cert"
subject="$(openssl x509 -in "$cert" -noout -subject -nameopt RFC2253)"
if [[ "$subject" != "subject=CN=orin1-carrier" ]]; then
  echo "unexpected client certificate subject: $subject" >&2
  exit 1
fi

cert_pub="$(openssl x509 -in "$cert" -pubkey -noout \
  | openssl pkey -pubin -outform DER 2>/dev/null | sha256sum | cut -d' ' -f1)"
key_pub="$(openssl pkey -in "$key" -pubout -outform DER 2>/dev/null \
  | sha256sum | cut -d' ' -f1)"
if [[ "$cert_pub" != "$key_pub" ]]; then
  echo "orin1-carrier certificate and private key do not match" >&2
  exit 1
fi

"$repo/codex_ops/deploy/install_agentd.sh" orin1-carrier

install -d -m 0750 -o jetson -g jetson "$cert_dir"
install -m 0644 -o jetson -g jetson "$ca" "$cert_dir/ca.crt"
install -m 0644 -o jetson -g jetson "$cert" "$cert_dir/orin1-carrier.crt"
install -m 0600 -o jetson -g jetson "$key" "$cert_dir/orin1-carrier.key"

python3 - "$config" "$ground_host" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
ground_host = sys.argv[2]
config = json.loads(path.read_text(encoding="utf-8"))
config["nats"]["servers"] = [f"tls://{ground_host}:4222"]
config["policy"]["mode"] = "observe"
config["codex"]["enabled"] = False
path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
PY

systemctl enable --now codex-agentd-orin1-carrier.service
sleep 2
systemctl is-active --quiet codex-agentd-orin1-carrier.service

echo "Orin1 transport-only worker is active."
echo "Endpoint: tls://$ground_host:4222"
echo "Policy: observe; Codex execution: disabled"
echo "No vehicle or actuator process was started."
