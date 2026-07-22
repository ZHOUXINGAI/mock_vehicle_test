# Huawei ECS Coordination Service

This directory deploys the NATS JetStream broker used only for cross-machine
Codex work coordination.

```bash
./bootstrap_cloud.sh <public-dns-or-ip>
docker compose -f docker-compose.yml config
docker compose -f docker-compose.yml up -d
```

Security properties:

- mutual TLS with certificate-to-user mapping;
- distinct `boss`, `orin1-carrier`, and `orin2-mini` identities;
- workers consume only their own task subject and publish only their own events,
  heartbeat, and peer task subject;
- JetStream data persists in the `nats-data` volume;
- monitoring port `8222` binds to cloud loopback only;
- certificates, private keys, `.env`, and local data are gitignored.

Never copy `certs/ca.key` off the cloud host. See
`docs/codex_cloud_coordination_runbook.md` for deployment and commissioning.
