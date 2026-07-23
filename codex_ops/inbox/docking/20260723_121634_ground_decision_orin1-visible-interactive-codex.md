# Ground decision: Orin1 uses one visible interactive Codex

时间：2026-07-23 12:16:34 CST (+08:00)

Boss 决定暂不把当前 NATS → systemd → `codex exec` 原型作为长期运行方案。
首次真实只读任务已证明 mTLS、JetStream、任务 ACK 和可读活动事件有效，但
Orin1 的 systemd Codex 在刷新模型时出现网络超时，因此自动执行 Gate B
退回 disabled。

当前正式工作方式：

- Ground 保留当前可直接对话的 Codex 图形界面；
- Orin1 只运行一个本机可见的交互式 Codex；
- agentd 保持 `policy.mode=observe`、`codex.enabled=false`，NATS 只用于在线
  状态和通知；
- GitHub `mock_vehicle_test/codex_ops` 是任务、交接和审计的共享办公室；
- NATS 不向交互式 Codex 聊天窗口注入提示词，Boss/用户在可见窗口让 Codex
  拉取 Git 并处理 inbox；
- Orin2 尚未接入、未配置、未启动，等待 Boss 另行授权。

Orin1 可见终端启动命令：

```bash
cd /home/jetson/mock_vehicle_test
git pull --ff-only
./codex_ops/scripts/launch_visible_codex.sh orin1-carrier
```

launcher 会验证当前用户、原生 Codex 绝对路径，并在后台 Codex 仍启用时拒绝
启动，避免两个 Codex 同时操作仓库。

不要启动 QGC、MAVROS、PX4、Offboard、Arduino、串口、MAVLink、执行器、
电机或任何车辆进程。
