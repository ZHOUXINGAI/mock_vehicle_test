# Ground result: Orin1 persistent Bridge on the new private LAN

Date: 2026-07-27 (Asia/Shanghai)

## Network

- Ground Wi-Fi profile: `HUAWEI_F5BF`
- Ground Wi-Fi IPv4: `192.168.1.35/24`
- Orin1 / Carrier IPv4: `192.168.1.34/24`
- NATS endpoint: `tls://192.168.1.35:4222`
- Orin1 identity was verified by matching its ED25519 SSH host-key
  fingerprint with the former `192.168.43.15` host.

## NATS and firewall

- NATS container: healthy, `restart: unless-stopped`
- JetStream streams: `CODEX_TASKS`, `CODEX_EVENTS`, `CODEX_HEARTBEATS`
- Server certificate SAN: `IP Address:192.168.1.35`
- Existing CA and all client certificates were preserved.
- Windows Wi-Fi category: Private
- Windows firewall: inbound TCP 4222, Private profile only, remote
  `192.168.1.0/24`
- WSL mirrored-network Hyper-V firewall: inbound TCP 4222 only, remote
  `192.168.1.0/24`
- Monitoring port 8222 remains bound to `127.0.0.1`.

## Orin1 persistent service

- Service: `codex-agentd-orin1-carrier.service`
- State at installation verification: enabled / active
- Backend: local Codex app-server over stdio
- Policy: observe / read-only / approvals never
- Codex session:
  `019e3b9d-8417-76d2-bf88-4ec59aba48c4`
- NATS mTLS endpoint: `tls://192.168.1.35:4222`
- The service carries the Orin1 login-shell localhost proxy environment:
  HTTP/HTTPS `127.0.0.1:7897`, SOCKS5 `127.0.0.1:7897`.
- The read-only chat mirrors remain observers and do not consume tasks.
- The visible Bridge and interactive `codex resume` must not run concurrently
  with the systemd service.

## Verification

1. Ground-side mTLS bootstrap completed against the new endpoint.
2. Orin1 TCP connection to Ground port 4222 passed after both Windows and WSL
   Hyper-V firewall rules were moved to the new subnet.
3. Task `333d1a0a-1f9a-45f4-a674-b542e24e416d` passed on the new LAN through
   the visible Bridge.
4. The first systemd Codex task
   `942ad888-4f77-4ce3-be64-70e5982baf7b` failed because the unit lacked the
   login-shell localhost proxy variables. The failure was recorded and was
   not treated as a pass.
5. After adding the proxy environment, task
   `a78f47ff-ef8a-400b-af05-09a5f3813a87` completed through the systemd
   service and returned the original Codex session ID.
6. The temporary SSH ControlMaster was then explicitly closed.
7. With no SSH Master and no visible Bridge, task
   `51857a5b-4352-4ca2-b1ae-84b6c9fe0411` was accepted and completed through
   NATS. Its completion event returned session ID
   `019e3b9d-8417-76d2-bf88-4ec59aba48c4`.

## Current blockers and dependencies

- Orin2 / Mini remains uncommissioned and untouched.
- Orin1 Codex turns require its local proxy listener on `127.0.0.1:7897`.
- Ground coordination requires Windows to remain awake and the NATS container
  to remain running.
- No QGC, MAVROS, PX4, Offboard, Arduino, serial, MAVLink, motor, docking, or
  other vehicle/hardware program was started.
