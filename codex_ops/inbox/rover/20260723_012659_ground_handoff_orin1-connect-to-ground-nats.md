# Ground handoff to Orin1 / Carrier: connect to ground NATS

时间：2026-07-23 01:26:59 CST (+08:00)

Orin1，请先拉取 `mock_vehicle_test` 的 `master`。地面端 Codex-Ground 已完成以下工作：

1. 地面 Wi-Fi IPv4 为 `192.168.43.13/24`，当前热点子网为
   `192.168.43.0/24`，Orin1 `192.168.43.15` 从 Windows 和 WSL 均可 ping
   通。
2. Ubuntu 22.04 WSL 已切换到 mirrored networking，直接持有
   `192.168.43.13`。
3. 地面端使用 WSL 原生 Docker Engine + Docker Compose 部署了
   `nats:2.14.1-alpine`，没有使用公网、FRP 或华为云。
4. NATS endpoint 为 `tls://192.168.43.13:4222`。
5. NATS 当前状态为 `running / healthy`，restart count 为 `0`；JetStream
   已初始化 `CODEX_TASKS`、`CODEX_EVENTS`、`CODEX_HEARTBEATS`。
6. TCP `4222` 发布到局域网；监控端口 `8222` 仅绑定
   `127.0.0.1:8222`。
7. Windows Defender 与 WSL Hyper-V 防火墙都只允许：Private WLAN、TCP
   `4222`、来源 `192.168.43.0/24`。没有开放 `8222`。
8. Boss CLI 已安装，使用 `boss.crt`/`boss.key` 对
   `tls://192.168.43.13:4222` 完成 mTLS bootstrap。
9. 已创建 Windows 登录计划任务 `CodexGroundKeepWslNatsOnline`，保证 WSL
   与 NATS 在地面电脑登录期间保持常驻；当前任务状态为 `Running`。
10. 没有启动 QGC、MAVROS、PX4、Offboard、Arduino、串口、执行器或任何电机程序。

详细地面验证记录在：

```text
codex_ops/inbox/rover/20260723_011910_ground_result_nats-ready-for-orin1.md
```

对应已推送提交：

```text
52c248b358ba0c9205df764c55c05aaa62e4398c
```

## Orin1 证书

证书不会通过 GitHub 提交。地面端已准备一个只含以下三个文件的目录：

```text
/home/ai/mock_vehicle_test/codex_ops/local/orin1-certs/ca.crt
/home/ai/mock_vehicle_test/codex_ops/local/orin1-certs/orin1-carrier.crt
/home/ai/mock_vehicle_test/codex_ops/local/orin1-certs/orin1-carrier.key
```

`ca.key` 只保留在地面电脑，绝不能复制到 Orin1，也不会提交 Git。

由于地面电脑目前没有可用的 Orin1 SSH 凭据，自动 `scp` 被
`Permission denied (publickey,password)` 阻止。因此证书尚未传到 Orin1。
管理员提供交互式 SSH 密码或公钥授权后，从地面 WSL 执行：

```bash
cd /home/ai/mock_vehicle_test
scp \
  codex_ops/local/orin1-certs/ca.crt \
  codex_ops/local/orin1-certs/orin1-carrier.crt \
  codex_ops/local/orin1-certs/orin1-carrier.key \
  jetson@192.168.43.15:/home/jetson/
```

## 请 Orin1 接下来执行

收到证书后，只安装上述三个文件：

```bash
sudo install -d -m 0750 /etc/codex-agentd/certs
sudo install -o jetson -g jetson -m 0644 /home/jetson/ca.crt \
  /etc/codex-agentd/certs/ca.crt
sudo install -o jetson -g jetson -m 0644 /home/jetson/orin1-carrier.crt \
  /etc/codex-agentd/certs/orin1-carrier.crt
sudo install -o jetson -g jetson -m 0600 /home/jetson/orin1-carrier.key \
  /etc/codex-agentd/certs/orin1-carrier.key
```

把 Orin1 agent 配置 endpoint 改为：

```text
tls://192.168.43.13:4222
```

第一次联调保持：

```json
"policy": { "mode": "observe" },
"codex": { "enabled": false }
```

然后只做 transport-only 验证：检查 Orin1 到 `192.168.43.13:4222` 的 TCP
连通性和 mTLS、启动 `codex-agentd-orin1-carrier.service`、确认 heartbeat。
不要为了排查 NATS 启动 QGC、MAVROS、PX4、Offboard、Arduino、串口或电机程序。

完成后请在 `codex_ops/inbox/` 与 `events/2026/2026-07-23.jsonl` 回写结果，
明确说明 TCP、mTLS、heartbeat 是否通过以及 blocker。
