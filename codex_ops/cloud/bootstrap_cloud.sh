#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <public-dns-or-ip>" >&2
  exit 2
fi

endpoint="$1"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
certs="$root/certs"
umask 077

if [[ -e "$certs/ca.key" ]]; then
  echo "refusing to overwrite existing cloud credentials under $root" >&2
  exit 1
fi

mkdir -p "$certs"

openssl genrsa -out "$certs/ca.key" 4096
openssl req -x509 -new -sha256 -days 3650 \
  -key "$certs/ca.key" -out "$certs/ca.crt" \
  -subj '/CN=Codex Coordination Local CA'

if [[ "$endpoint" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  san="IP.1 = $endpoint"
else
  san="DNS.1 = $endpoint"
fi

printf '%s\n' \
  '[req]' \
  'distinguished_name = dn' \
  'prompt = no' \
  'req_extensions = req_ext' \
  '[dn]' \
  'CN = codex-coordinator' \
  '[req_ext]' \
  'subjectAltName = @alt_names' \
  '[alt_names]' \
  "$san" > "$certs/server.cnf"

openssl req -new -newkey rsa:3072 -nodes \
  -keyout "$certs/server.key" -out "$certs/server.csr" \
  -config "$certs/server.cnf"
printf '%s\n' \
  'basicConstraints = CA:FALSE' \
  'keyUsage = critical,digitalSignature,keyEncipherment' \
  'extendedKeyUsage = serverAuth' \
  'subjectAltName = @alt_names' \
  '[alt_names]' \
  "$san" > "$certs/server.ext"
openssl x509 -req -sha256 -days 825 \
  -in "$certs/server.csr" -CA "$certs/ca.crt" -CAkey "$certs/ca.key" \
  -CAcreateserial -out "$certs/server.crt" -extfile "$certs/server.ext"

printf '%s\n' \
  'basicConstraints = CA:FALSE' \
  'keyUsage = critical,digitalSignature,keyEncipherment' \
  'extendedKeyUsage = clientAuth' > "$certs/client.ext"

for client in boss orin1-carrier orin2-mini; do
  openssl req -new -newkey rsa:3072 -nodes \
    -keyout "$certs/$client.key" -out "$certs/$client.csr" \
    -subj "/CN=$client"
  openssl x509 -req -sha256 -days 825 \
    -in "$certs/$client.csr" -CA "$certs/ca.crt" -CAkey "$certs/ca.key" \
    -CAcreateserial -out "$certs/$client.crt" -extfile "$certs/client.ext"
done

rm -f "$certs"/*.csr "$certs"/*.srl "$certs"/*.ext "$certs/server.cnf"
chmod 600 "$certs"/*.key
chmod 644 "$certs"/*.crt

echo "Generated TLS credentials. Keep ca.key only on the cloud host."
echo "Next: docker compose -f $root/docker-compose.yml up -d"
