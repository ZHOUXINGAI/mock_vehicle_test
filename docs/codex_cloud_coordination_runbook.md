# Codex Cloud Coordination Runbook

Last updated: 2026-07-23 CST

This runbook deploys a persistent coordination channel between:

```text
Boss/operator -> Huawei ECS NATS JetStream <- Orin1/Carrier Codex
                                            <- Orin2/Mini Codex
Orin1/Carrier Codex <----------------------> Orin2/Mini Codex
```

The cloud service is a coordination control plane for software work. It is never
part of MAVLink, LR24, PX4, Offboard, arming, actuator, or robot motion control.
LR24 remains the vehicle runtime link.

## 1. What Is Already Implemented

- Persistent JetStream task, event, and heartbeat streams.
- Mutual TLS identity for `boss`, `orin1-carrier`, and `orin2-mini`.
- Per-agent publish/subscribe permissions.
- At-least-once delivery, explicit ACK/NAK, local SQLite idempotency, and retries.
- Persistent Codex session IDs so each worker continues its own thread.
- Structured peer requests: Orin1 can wake Orin2 and Orin2 can wake Orin1.
- A hard local safety gate that rejects all hardware-capability requests.
- `observe` mode for read-only deployment and `code` mode for repository edits.
- systemd services with private device access and no privilege escalation.

Local no-hardware validation completed on 2026-07-23:

```text
Boss parent task:       c89a3c97-caed-4628-98b7-d4b9e81cd893
Orin1 -> Orin2 child:   5003e99a-48c8-4c84-af77-00bf0e704a56
Result: Orin1 generated a structured peer request and Orin2 completed it
        without Boss relaying text and without connecting vehicle hardware.
```

## 2. Cloud Host Requirements

- One Huawei ECS with a stable public IP or DNS name.
- Docker Engine and the Docker Compose plugin.
- TCP `4222` allowed by the ECS security group from the networks used by the
  two Orins and the Boss client. Start with the narrowest practical source CIDR.
- TCP `8222` must not be exposed publicly; Compose binds it to loopback only.
- SSH access for initial installation and certificate distribution.

Do not expose Codex, SSH shells, ROS 2 DDS, or NATS monitoring through FRP. Both
Orins initiate outbound TLS connections to TCP `4222`.

## 3. Deploy NATS On Huawei ECS

On the ECS:

```bash
git clone git@github.com:ZHOUXINGAI/mock_vehicle_test.git
cd mock_vehicle_test
git pull --ff-only
./codex_ops/cloud/bootstrap_cloud.sh <ECS_PUBLIC_DNS_OR_IP>
docker compose -f codex_ops/cloud/docker-compose.yml config
docker compose -f codex_ops/cloud/docker-compose.yml up -d
docker compose -f codex_ops/cloud/docker-compose.yml ps
curl --fail http://127.0.0.1:8222/healthz
```

Keep `codex_ops/cloud/certs/ca.key` only on the ECS and back it up securely.
Never commit or send it to either Orin. Distribute only this matrix:

```text
Orin1: ca.crt, orin1-carrier.crt, orin1-carrier.key
Orin2: ca.crt, orin2-mini.crt, orin2-mini.key
Boss:  ca.crt, boss.crt, boss.key
```

Client private keys must be mode `0600`. If a device is lost, issue a new CA or
replace that certificate and remove the old identity from service.

## 4. Install Orin1 / Carrier

On Orin1:

```bash
cd /home/jetson/mock_vehicle_test
git pull --ff-only --autostash
codex --version
sudo ./codex_ops/deploy/install_agentd.sh orin1-carrier
```

Required Codex baseline is `codex-cli 0.145.0` with model `gpt-5.6-sol` and no
legacy profile. Upgrade if needed:

```bash
npm install -g @openai/codex@0.145.0
codex --version
```

Install the three Orin1 certificate files:

```bash
sudo install -o jetson -g jetson -m 0644 ca.crt /etc/codex-agentd/certs/ca.crt
sudo install -o jetson -g jetson -m 0644 orin1-carrier.crt /etc/codex-agentd/certs/orin1-carrier.crt
sudo install -o jetson -g jetson -m 0600 orin1-carrier.key /etc/codex-agentd/certs/orin1-carrier.key
sudoedit /etc/codex-agentd/orin1-carrier.json
```

Replace `REPLACE_WITH_HUAWEI_ECS_DNS_OR_IP`. Leave these values for the first
smoke test:

```json
"policy": { "mode": "observe" },
"codex": { "enabled": false }
```

Then start the transport-only worker:

```bash
sudo systemctl enable --now codex-agentd-orin1-carrier.service
systemctl status codex-agentd-orin1-carrier.service --no-pager
journalctl -u codex-agentd-orin1-carrier.service -n 100 --no-pager
```

## 5. Install Orin2 / Mini

On Orin2, use the same procedure with its own account and identity:

```bash
cd /home/seeed/mock_vehicle_test
git pull --ff-only --autostash
npm install -g @openai/codex@0.145.0
codex --version
sudo ./codex_ops/deploy/install_agentd.sh orin2-mini
sudo install -o seeed -g seeed -m 0644 ca.crt /etc/codex-agentd/certs/ca.crt
sudo install -o seeed -g seeed -m 0644 orin2-mini.crt /etc/codex-agentd/certs/orin2-mini.crt
sudo install -o seeed -g seeed -m 0600 orin2-mini.key /etc/codex-agentd/certs/orin2-mini.key
sudoedit /etc/codex-agentd/orin2-mini.json
sudo systemctl enable --now codex-agentd-orin2-mini.service
systemctl status codex-agentd-orin2-mini.service --no-pager
journalctl -u codex-agentd-orin2-mini.service -n 100 --no-pager
```

Replace the cloud endpoint and initially keep `observe` plus `enabled=false`.

## 6. Install The Boss Client And Bootstrap Streams

The Boss client may run on the ground computer or an operator machine. It is a
software-work dispatcher, not the docking planner. Create a Python environment:

```bash
cd /path/to/mock_vehicle_test
python3 -m venv codex_ops/local/venv
codex_ops/local/venv/bin/pip install -r codex_ops/realtime/requirements.txt
sudo install -d -m 0750 /etc/codex-agentd/certs
sudo install -m 0644 ca.crt /etc/codex-agentd/certs/ca.crt
sudo install -m 0644 boss.crt /etc/codex-agentd/certs/boss.crt
sudo install -m 0600 boss.key /etc/codex-agentd/certs/boss.key
cp codex_ops/realtime/config/boss.example.json codex_ops/local/boss.json
```

Edit `codex_ops/local/boss.json`, replace the ECS endpoint, then run:

```bash
./codex_ops/scripts/coordctl.sh --config codex_ops/local/boss.json bootstrap
./codex_ops/scripts/coordctl.sh --config codex_ops/local/boss.json \
  watch --subject 'codex.heartbeat.*'
```

## 7. Commission In Three Gates

Gate A, transport only. Both workers remain `codex.enabled=false`:

```bash
./codex_ops/scripts/coordctl.sh --config codex_ops/local/boss.json send \
  --to orin1-carrier --task-type analysis --repo mock_vehicle_test \
  --objective 'Transport smoke test only. Do not access hardware.' --wait 60

./codex_ops/scripts/coordctl.sh --config codex_ops/local/boss.json send \
  --to orin2-mini --task-type analysis --repo mock_vehicle_test \
  --objective 'Transport smoke test only. Do not access hardware.' --wait 60
```

Gate B, read-only Codex. On both Orins set `codex.enabled=true`, keep
`policy.mode=observe`, restart each service, and send an analysis task. Confirm
`accepted`, `progress`, then `completed` events and inspect each worker's
`codex_ops/runs/<agent>/<task-id>/` directory.

Gate C, repository work. Set `policy.mode=code` only after Gate B passes. This
allows edits in configured repository roots but still cannot grant motion,
arming, Offboard, serial, MAVLink, GPIO, Arduino, actuator, or sudo access.

```bash
sudo systemctl restart codex-agentd-orin1-carrier.service
sudo systemctl restart codex-agentd-orin2-mini.service
```

## 8. Normal Working Loop

1. Boss sends one objective to the owner.
2. Owner publishes `accepted` and progress events.
3. Owner does the local work in its allowed repository.
4. If peer work is required, owner returns a structured `peer_requests` entry.
5. `agentd` publishes the child task directly to the peer and records the parent,
   root, and child task IDs.
6. Peer works and reports its result without Boss copying chat text.
7. Code and formal architecture changes are still committed and pushed to
   GitHub. The task references the commit hash or file path.

NATS is for wake-up, task state, ACK, progress, and compact results. GitHub is
for source code, reviewable documents, and durable history. Large logs or images
belong in OBS or another artifact store, with only their URI and checksum sent
through NATS.

## 9. Stop, Roll Back, And Diagnose

Stop one worker without affecting any vehicle software:

```bash
sudo systemctl stop codex-agentd-orin1-carrier.service
sudo systemctl stop codex-agentd-orin2-mini.service
```

Cloud status:

```bash
docker compose -f codex_ops/cloud/docker-compose.yml logs --tail=200 nats
curl --fail http://127.0.0.1:8222/healthz
ss -ltnp | grep -E ':4222|:8222'
```

Orin status:

```bash
systemctl is-active codex-agentd-<agent>.service
journalctl -u codex-agentd-<agent>.service -f
openssl s_client -connect <ECS_HOST>:4222 -CAfile /etc/codex-agentd/certs/ca.crt \
  -cert /etc/codex-agentd/certs/<agent>.crt \
  -key /etc/codex-agentd/certs/<agent>.key </dev/null
```

Do not troubleshoot this service by starting MAVROS, QGC, PX4, or motor scripts.
