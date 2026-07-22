# Ground result: NATS ready for Orin1

- Timestamp: 2026-07-23 01:19:10 CST (+08:00)
- Role: Codex-Ground
- Repository: `/home/ai/mock_vehicle_test`
- Base commit verified: `f2b651d1c8750e18a001fb2c30063b418299b0e5`
- Safety scope: coordination infrastructure and Boss CLI only. No QGC, MAVROS, PX4, Offboard, Arduino, serial, actuator, or motor program was started.

## Endpoint and deployment

- Windows Wi-Fi IPv4: `192.168.43.13/24`
- Wi-Fi gateway/current hotspot subnet: `192.168.43.1`, `192.168.43.0/24`
- NATS endpoint: `tls://192.168.43.13:4222`
- Orin1 / Carrier: `192.168.43.15`, ICMP reachable from Windows and WSL with 0% loss
- Deployment type: WSL-native Docker Engine + Docker Compose v2 (Docker Desktop was not installed)
- WSL: Ubuntu 22.04.5 LTS, WSL 2.6.3.0, kernel 6.6.87.2, mirrored networking
- Docker: Engine 29.1.3; Compose 2.40.3
- NATS image: `nats:2.14.1-alpine`

The Windows logon task `CodexGroundKeepWslNatsOnline` runs
`wsl.exe -d Ubuntu-22.04 --exec /usr/bin/sleep infinity`. This is necessary
because systemd services alone do not keep a WSL distribution alive. Docker is
enabled in WSL and the Compose service uses `restart: unless-stopped`.

## Windows firewall

The current WLAN hotspot was changed from Public to Private. All Windows
firewall profiles remain enabled.

- Host rule: `CodexGround-NATS-4222`
  - enabled, inbound allow, Private profile only
  - interface `WLAN` only
  - TCP local port `4222` only
  - remote address `192.168.43.0/24` only
- WSL Hyper-V rule: `CodexGround-NATS-4222-WSL`
  - enabled, inbound allow, Private profile only
  - WSL VM creator ID `{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}`
  - TCP local port `4222` only
  - remote address `192.168.43.0/24` only
- No rule was created for TCP `8222`.

## Service and certificates

- Container: `codex-coordinator-nats`
- Final state: `running`, health `healthy`, restart count `0`
- JetStream streams: `CODEX_TASKS`, `CODEX_EVENTS`, `CODEX_HEARTBEATS`
- Client port: `0.0.0.0:4222` and `[::]:4222`
- Monitoring port: `127.0.0.1:8222` only
- Monitoring health: `{"status":"ok"}`

Ground-only certificate authority and full broker certificate directory:

```text
/home/ai/mock_vehicle_test/codex_ops/cloud/certs/
```

`ca.key` remains only in that ground directory, mode `0600`, and is ignored by
Git. It must never be copied to Orin1.

The Orin1 handoff directory contains exactly these three files:

```text
/home/ai/mock_vehicle_test/codex_ops/local/orin1-certs/ca.crt
/home/ai/mock_vehicle_test/codex_ops/local/orin1-certs/orin1-carrier.crt
/home/ai/mock_vehicle_test/codex_ops/local/orin1-certs/orin1-carrier.key
```

Certificate modes are `0644`, `0644`, and `0600` respectively. The entire
`codex_ops/local/` directory is ignored by Git.

## Orin1 certificate retrieval

After interactive SSH authentication for `jetson@192.168.43.15` is available,
run from the ground WSL:

```bash
cd /home/ai/mock_vehicle_test
scp \
  codex_ops/local/orin1-certs/ca.crt \
  codex_ops/local/orin1-certs/orin1-carrier.crt \
  codex_ops/local/orin1-certs/orin1-carrier.key \
  jetson@192.168.43.15:/home/jetson/
```

Then on Orin1 install only those files:

```bash
sudo install -d -m 0750 /etc/codex-agentd/certs
sudo install -o jetson -g jetson -m 0644 /home/jetson/ca.crt /etc/codex-agentd/certs/ca.crt
sudo install -o jetson -g jetson -m 0644 /home/jetson/orin1-carrier.crt /etc/codex-agentd/certs/orin1-carrier.crt
sudo install -o jetson -g jetson -m 0600 /home/jetson/orin1-carrier.key /etc/codex-agentd/certs/orin1-carrier.key
```

Configure Orin1 with `tls://192.168.43.13:4222`, initially keeping
`policy.mode=observe` and `codex.enabled=false` for the transport-only gate.

## Actual verification commands and results

```text
wsl.exe --version
  WSL 2.6.3.0

wsl.exe -d Ubuntu-22.04 -- ip -4 route
  default via 192.168.43.1 dev eth0
  192.168.43.0/24 dev eth0

ping 192.168.43.15
  Windows: 2/2 replies, average 15 ms
  WSL mirrored: 2/2 replies, average 16.9 ms

docker compose -f codex_ops/cloud/docker-compose.yml config
  exit 0; 4222 published globally, 8222 published to 127.0.0.1 only

docker compose -f codex_ops/cloud/docker-compose.yml up -d
  codex-coordinator-nats started

docker inspect codex-coordinator-nats
  status=running health=healthy restarts=0 image=nats:2.14.1-alpine

docker port codex-coordinator-nats
  4222/tcp -> 0.0.0.0:4222
  4222/tcp -> [::]:4222
  8222/tcp -> 127.0.0.1:8222

curl --fail http://127.0.0.1:8222/healthz
  {"status":"ok"}

curl --noproxy '*' --connect-timeout 2 http://192.168.43.13:8222/healthz
  connection blocked as required

./codex_ops/scripts/coordctl.sh --config codex_ops/local/boss.json bootstrap
  JetStream streams are ready: CODEX_TASKS CODEX_EVENTS CODEX_HEARTBEATS

25-second WSL idle persistence test
  container StartedAt unchanged; health=healthy; restarts=0
```

The successful `coordctl bootstrap` used `boss.crt`/`boss.key` and endpoint
`tls://192.168.43.13:4222`, so it is the local WSL mTLS connection proof.

## Current blocker

Ground-side NATS and Boss bootstrap are ready. Orin1-side TCP/mTLS validation is
still pending because the available ground SSH client has no accepted Orin1
credential (`Permission denied (publickey,password)`). The Windows and WSL
firewall rules are installed, Orin1 responds to ping, and the three-file Orin1
bundle is ready. The next operator action is to perform the `scp` above with an
interactive credential, install the files, and run the Orin1 transport-only
smoke test. A same-host Windows connection to the mirrored Wi-Fi address timed
out; this does not substitute for the required external Orin1-side test.
