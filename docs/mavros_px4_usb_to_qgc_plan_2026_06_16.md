# MAVROS 串口接 Pixhawk、UDP 转发 QGC 方案（2026-06-16）

目标链路：

```text
Orin Nano
  -> MAVROS 独占 Pixhawk USB 串口
  -> MAVROS ROS 2 topic/service 给控制器使用
  -> MAVROS gcs_url UDP 转发给 QGroundControl
  -> QGC 通过 UDP 14550 监控，不再抢 Pixhawk USB
```

当前 Pixhawk USB 设备：

```text
/dev/ttyACM0
/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00
```

Arduino CH340 是：

```text
/dev/ttyUSB0
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

## 安装状态

当前机器有 ROS 2 Humble，MAVROS 已安装完成。

安装脚本保留在仓库中：

```bash
cd /home/jetson/mock_vehicle_test
./scripts/install_mavros_humble.sh
```

已验证：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix mavros
ros2 pkg prefix mavros_msgs
```

验证结果：

```text
/opt/ros/humble
/opt/ros/humble
```

MAVROS 还需要 GeographicLib geoid 数据。`egm96-5` 已手动安装到：

```text
/usr/share/GeographicLib/geoids/egm96-5.pgm
```

## 启动 MAVROS 并转发到 QGC

先不要打开 QGC。先启动 MAVROS：

```bash
cd /home/jetson/mock_vehicle_test
./scripts/run_mavros_px4_usb_to_qgc.sh
```

户外测试建议用带日志保存的启动脚本：

```bash
cd /home/jetson/mock_vehicle_test
./scripts/run_mavros_px4_usb_to_qgc_logged.sh
```

日志会实时显示在终端，并保存到：

```text
results/mavros/<timestamp>/mavros.log
results/mavros/latest/mavros.log
```

默认参数：

```text
FCU: serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:115200
QGC: udp://:14555@127.0.0.1:14550
namespace: /mavros
```

MAVROS 成功连上后，再打开 QGroundControl：

```bash
cd /home/jetson/mock_vehicle_test
./tools/run-qgroundcontrol.sh
```

QGC 应该通过 UDP 14550 看到车。不要让 QGC 直接打开 Pixhawk USB 串口。

本仓库的 QGC 启动脚本会先写入 QGC 设置：

```text
[LinkManager]
autoConnectUDP=true
autoConnectPixhawk=false
udpListenPort=14550
udpTargetHostIP=127.0.0.1
udpTargetHostPort=14555
```

也可以单独执行：

```bash
cd /home/jetson/mock_vehicle_test
./scripts/configure_qgc_udp_only.sh
```

这个设置不是 QGC 的命令行参数，而是 QGC 的持久配置项。源码里 QGC v4.4.5 的命令行开关只有
`--clear-settings`、`--clear-cache`、`--logging`、`--fake-mobile`、`--log-output`
等，`autoConnectPixhawk` 在 `LinkManager` 设置组里。

2026-06-16 已做短启动验证：

```text
MAVROS opened Pixhawk USB successfully.
MAVROS opened QGC UDP endpoint udp://@127.0.0.1:14550.
MAVROS detected remote address 1.1.
MAVROS received PX4 heartbeat.
FCU: PX4 Autopilot.
```

后续将默认使用更明确的 MAVROS GCS URL：

```text
udp://:14555@127.0.0.1:14550
```

含义是 MAVROS 本地使用 UDP `14555`，发送到 QGC 的 UDP `14550`。QGC 只监听 UDP，
不再自动打开 Pixhawk USB。

短启动是用 `timeout` 自动停止的，没有保持 MAVROS 后台运行。

## MAVROS 版 smoke test

先确认：

- 车轮架空。
- 遥控器开机，并能切回手动/停止。
- QGC/Pixhawk 当前参数已经导出备份。
- MAVROS 已经连接 Pixhawk。
- QGC 通过 MAVROS UDP 转发能看到 Pixhawk。

默认测试命令：

```bash
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_offboard_smoke.sh
```

默认不主动切 Offboard，不主动 arm。脚本会持续发 stop，并等待 `/mavros/state` 显示：

```text
connected = true
mode      = OFFBOARD
armed     = true
```

安全接管保护默认开启：

```text
ABORT_ON_MODE_EXIT=true
ABORT_ON_DISARM=true
ABORT_ON_ARM_REJECTED=true
```

动作序列开始后，如果遥控器或 QGC 把 PX4 从 `OFFBOARD` 切到 `MANUAL`/其他模式，或者车辆
disarm，脚本会立即发布 stop、打印 abort 日志并退出，不再继续发送前进/后退 setpoint。
如果脚本请求 arm 被 PX4 拒绝，也会立即退出，不再停在等待 armed 的循环里。

手动接管测试应该车轮架空做：启动 smoke test 后，序列开始运动时用遥控器切到 `MANUAL`。
预期现象是 QGC 模式变成 `MANUAL`，脚本日志出现 `aborting smoke sequence`，轮子按手动输入或停止，
程序不再继续执行后续动作。如果 QGC 模式没有变，说明遥控器上的模式开关没有真正映射到 PX4 flight
mode，需要先回 QGC 校准/模式分配。

如果要让脚本主动请求 Offboard 和 arm，必须显式加：

```bash
MODE_CHANGE_ON_START=true ARM_ON_START=true \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_offboard_smoke.sh
```

只允许在车轮架空时这样做。

如果只是验证“能否进入 Offboard/Arm/Disarm”，可以用零速度入口测试：

```bash
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_offboard_entry_test.sh
```

这个脚本不会请求前进/后退/转向，只请求 `OFFBOARD`、`ARM`，然后 stop 并 `DISARM`。日志里重点看：

```text
OFFBOARD mode request response: mode_sent=true
state changed: connected=true mode=OFFBOARD armed=...
ARM request response: success=true
```

如果 `mode_sent=false`，说明 PX4 拒绝进入 Offboard；此时看 QGC Messages 的拒绝原因，以及
`/mavros/state` 当前 mode/armed 状态。QGC mission 相关提示如 `unexpected waypoint index`、
`mission download request ignored, already active` 是任务下载冲突噪声，不等于 Offboard 失败。

如果出现：

```text
OFFBOARD mode request response: mode_sent=true
ARM request response: success=false result=1
```

说明已经成功进入过 `OFFBOARD`，但 PX4 暂时拒绝 MAVROS 的 arm 请求。`result=1` 是
`TEMPORARILY_REJECTED`，应查看 QGC Messages 的 arming denied 原因。可以先测试“遥控器手动 arm
后再由脚本请求 Offboard”的路径。

## Smoke Test 动作

```text
stop/hold
forward 1.0 s
stop 1.0 s
backward 1.0 s
stop 1.0 s
left 0.5 s
stop 1.0 s
right 0.5 s
final stop
```

默认命令：

```text
LINEAR_SPEED_MPS=0.12
TURN_YAW_RATE_RADPS=0.25
```

户外第一次落地低速测试使用单独 wrapper：

```bash
CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_outdoor_offboard_task.sh
```

它默认 `TEST_SURFACE=ground`，速度更低：

```text
LINEAR_SPEED_MPS=0.05
TURN_YAW_RATE_RADPS=0.12
ARM_ON_START=false
REQUIRE_ARMED_BEFORE_MODE_CHANGE=true
MODE_CHANGE_ON_START=true
```

也就是说户外默认由遥控器手动 arm，脚本检测到 `armed=True` 后才请求进入 `OFFBOARD` 并发送低速动作。详细步骤见
`docs/outdoor_mavros_offboard_low_speed_test_2026_06_17.md`。

## 注意

MAVROS 是桥，不是控制器。后续飞机和小车都可以共用 MAVROS 作为通信层，但控制器仍然是我们写的
ROS 节点。动态时空走廊跟踪时，规划器输出 `p_d(t), v_d(t), a_d(t), yaw_d(t)`，控制器再通过
MAVROS setpoint topic/service 送给 PX4。
