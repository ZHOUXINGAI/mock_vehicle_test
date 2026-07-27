#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <new-server-ip-or-dns>" >&2
  exit 2
fi

endpoint="$1"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
certs="$root/certs"
umask 077

for required in ca.crt ca.key server.crt server.key; do
  if [[ ! -f "$certs/$required" ]]; then
    echo "missing required credential: $certs/$required" >&2
    exit 1
  fi
done

if [[ "$endpoint" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  san_type=IP
else
  san_type=DNS
fi

temporary="$(mktemp -d "$certs/.server-rotation.XXXXXX")"
cleanup() {
  case "$temporary" in
    "$certs"/.server-rotation.*) rm -rf -- "$temporary" ;;
    *) echo "refusing unexpected temporary path: $temporary" >&2 ;;
  esac
}
trap cleanup EXIT

cat >"$temporary/server.cnf" <<EOF
[req]
distinguished_name = dn
prompt = no
req_extensions = req_ext
[dn]
CN = codex-coordinator
[req_ext]
subjectAltName = @alt_names
[alt_names]
$san_type.1 = $endpoint
EOF

cat >"$temporary/server.ext" <<EOF
basicConstraints = CA:FALSE
keyUsage = critical,digitalSignature,keyEncipherment
extendedKeyUsage = serverAuth
subjectAltName = @alt_names
[alt_names]
$san_type.1 = $endpoint
EOF

openssl genrsa -out "$temporary/server.key" 3072
openssl req -new \
  -key "$temporary/server.key" \
  -out "$temporary/server.csr" \
  -config "$temporary/server.cnf"
openssl x509 -req -sha256 -days 825 \
  -in "$temporary/server.csr" \
  -CA "$certs/ca.crt" \
  -CAkey "$certs/ca.key" \
  -CAserial "$temporary/ca.srl" \
  -CAcreateserial \
  -out "$temporary/server.crt" \
  -extfile "$temporary/server.ext"

openssl verify -CAfile "$certs/ca.crt" "$temporary/server.crt"
openssl x509 -in "$temporary/server.crt" -noout -ext subjectAltName |
  grep -F "$san_type Address:$endpoint" >/dev/null 2>&1 ||
  openssl x509 -in "$temporary/server.crt" -noout -ext subjectAltName |
    grep -F "$san_type:$endpoint" >/dev/null

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
cp -p "$certs/server.crt" "$certs/server.crt.bak-$stamp"
cp -p "$certs/server.key" "$certs/server.key.bak-$stamp"
install -m 0600 "$temporary/server.key" "$certs/server.key"
install -m 0644 "$temporary/server.crt" "$certs/server.crt"

echo "Rotated only the NATS server certificate for $endpoint."
echo "CA and all client certificates were preserved."
echo "Backups: server.crt.bak-$stamp and server.key.bak-$stamp"
