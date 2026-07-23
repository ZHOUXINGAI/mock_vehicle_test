# Ground handoff to Orin1: visible Codex console

时间：2026-07-23 11:42:47 CST (+08:00)

Boss 要求后续 Codex 协同不能在后台静默运行。Ground 保持当前可直接对话的
Codex 窗口；Orin1/Carrier 使用一个可见 Terminal（也可以是 VS Code
Integrated Terminal）持续显示任务事件和 Codex 实际工作流。

本次更新实现：

- 修复 Ground `coordctl watch` 空闲约一秒即退出的问题；
- 新增 Ground 实时事件窗口脚本 `watch_ground_events.sh`；
- 新增 Orin 前台窗口脚本 `watch_agent_console.sh`；
- worker 保持 systemd 可靠消费，console 只读观察、不抢任务；
- 当 `codex.enabled=true` 时，console 会用中文显示任务目标、命令、文件修改、
  工具调用、Codex 阶段消息、peer 交接和最终结果；
- 原始 `codex exec --json` 只保存到 run artifact，不再作为主界面输出；
- 不显示模型隐藏思维，只显示可观察、可核验的工作动作；
- `policy.mode=observe` 与硬件拒绝策略不变。

Orin1 拉取后，在可见终端运行：

```bash
cd /home/jetson/mock_vehicle_test
git pull --ff-only
./codex_ops/scripts/watch_agent_console.sh orin1-carrier
```

如果本地用户没有 journal 读取权限，脚本会要求本机交互式 sudo；不得把
sudo 密码写入脚本、Git、memory 或日志。

要看到真实 Codex 工作而不只是 transport ACK，需要在 Orin1 本机明确进入
Gate B：保留 `policy.mode=observe`，把
`/etc/codex-agentd/orin1-carrier.json` 中 `codex.enabled` 改为 `true`，
推荐用带身份和模式校验、自动备份的脚本，然后交互式执行：

```bash
sudo python3 codex_ops/scripts/set_agent_codex_enabled.py \
  --config /etc/codex-agentd/orin1-carrier.json \
  --enabled true --require-agent orin1-carrier --require-mode observe
sudo systemctl restart codex-agentd-orin1-carrier.service
```

此后 Ground 发来的只读任务会启动独立的 automation Codex，并在上述 console
中实时显示。它不会自动插入已经打开的 VS Code Codex chat；VS Code 中应打开
Integrated Terminal 运行 console。

当前只部署和验证 Ground 与 Orin1；Orin2 尚未接入、未配置、未启动，等待 Boss
另行授权。

Ground 已运行显示测试任务
`0829397d-bd12-4528-81f9-ca163895a1d6`，Orin1 返回 accepted 和 completed，
且未启动 Codex 或任何硬件进程。

可读格式更新后的运输层复测任务为
`7b540d7c-bd94-4124-a0f9-fff292313ab8`。Ground 工作台显示具体下达目标，
Orin1 再次返回 accepted 和 completed；结果仍为
`agentd transport smoke test completed with Codex execution disabled`。

不要通过 console 或 NATS 请求 serial、MAVLink、PX4、MAVROS、QGC、Offboard、
Arduino、actuator、arming 或 motor 能力。
