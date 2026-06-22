# Offboard 最小真车任务设计（2026-06-16）

这份设计对应第三步：开始让 Orin Nano 通过 Pixhawk 给真车发最小动作命令。当前电池在充电，
所以本阶段先完成设计，不上电运行。

## 目标

不要一开始做复杂自动驾驶。第三步只验证：

1. Orin Nano 能稳定向 Pixhawk 发送控制命令。
2. Pixhawk 仍然保留 arming、mode、failsafe、遥控器接管能力。
3. Arduino/D24A/电机链路能按小命令动作。
4. 命令丢失或脚本退出后，车能停止。

必须先架空轮子测试，再落地低速测试。

## 当前硬件基线

沿用已固化的成功链路：

```text
Orin Nano -> Pixhawk 6C -> Arduino UNO -> D24A -> 四个电机
AT9S PRO -> R9DS -> Pixhawk 6C -> Arduino UNO -> D24A -> 四个电机
```

当前接线和方向映射以这里为准：

```text
docs/current_rover_success_baseline_2026_06_16.md
```

第三步开始前必须先导出 Pixhawk/QGC 参数：

```text
config/pixhawk/pixhawk6c_rover_success_2026_06_16.params
```

原因：Offboard 测试会碰 mode、arming、failsafe、actuator/output 配置。没有参数备份，
改错后不容易恢复。

## 为什么不能直接复用 3 米 SITL 脚本

现有 `src/mock_rover_offboard.py` 是 SITL 训练脚本，逻辑是：

- 等待本地位置有效。
- 切 Offboard。
- 发位置或速度目标。
- 前进 3 米，再回到起点。

真车第一次测试不适合直接用它，原因：

- 真实 Pixhawk 可能没有可靠本地位置/GPS/里程计。
- 3 米距离对第一次 Offboard 过大。
- 第一次要验证的是“命令链路和停止链路”，不是导航效果。
- 当前车没有确认闭环速度反馈，不应一上来做位置任务。

所以第三步需要一个新的硬件 smoke test 脚本，而不是直接跑 3 米任务。

## 第三步推荐路线

### 3A：伴随计算机短动作 smoke test

先做一个极小命令序列：

```text
hold 1.0 s
forward 1.0 s
stop 1.0 s
backward 1.0 s
stop 1.0 s
turn left 0.5 s
stop 1.0 s
turn right 0.5 s
stop and hold
```

默认参数要非常保守：

```text
command_rate_hz = 20
linear_command  = 0.10 到 0.20 的小量
turn_command    = 0.10 到 0.20 的小量
max_step_sec    = 1.0
require_manual_confirm = true
```

这里的 `linear_command` 和 `turn_command` 具体含义取决于最终选的 Pixhawk 控制入口：

- 如果走 PX4 速度/角速度 Offboard，就是 `vx` 和 `yaw_rate` 的小目标。
- 如果走 PX4 direct actuator，就是归一化的前进/转向或左右轮小输出。
- 如果走 MAVLink manual-control smoke test，就是模拟很小的摇杆量。

第一版脚本必须默认不自动 arm，或者至少要求显式参数才 arm。

### 3B：确认真正 PX4 Offboard 入口

第三步有两个可行入口，需要先用参数导出和 QGC 当前配置确认：

1. **优先安全入口：MAVLink manual-control smoke test**
   - Orin 模拟小摇杆命令给 Pixhawk。
   - 优点：沿用已经跑通的遥控器/接收机/Pixhawk 输出链路，不需要大改 actuator。
   - 缺点：严格说这更像“伴随计算机接管手动控制”，不是真正 PX4 Offboard mode。

2. **真正 Offboard 入口：PX4 Offboard setpoint**
   - Orin 通过 ROS 2 / uXRCE-DDS 发 Offboard setpoint。
   - 优点：和最终自主控制路线一致。
   - 风险：可能需要有效本地位置/速度估计，或需要重新配置 direct actuator 输出。

推荐顺序：

```text
先 manual-control smoke test 验证 Orin->Pixhawk->Arduino->D24A 链路
再 PX4 Offboard setpoint 验证真正 Offboard mode
```

这样不会在第一次真车测试时同时引入“新控制入口、新模式、新参数、新脚本”四个变量。

## 架空轮子测试条件

运行第三步前必须满足：

- 车轮离地，车体稳定。
- 遥控器开机，并确认可以切回手动/停止。
- QGC 已连接 Pixhawk。
- Pixhawk 参数已导出备份。
- Arduino 串口能看到中位输入和控制变化。
- D24A 电源最后接入。
- 脚本退出、Ctrl-C、通信断开时，电机停止。

架空测试只通过以下现象：

- forward 时四轮方向符合基线。
- backward 时四轮方向符合基线。
- left/right 时左右侧方向符合基线。
- stop 阶段电机停止。
- 停止遥控/切回手动后，Orin 命令不再继续推动车轮。

## 落地低速测试条件

只有架空测试通过后，才落地：

- 地面开阔，没有人站在车前后。
- 第一次 `linear_command <= 0.10`，动作时长不超过 `0.5 s`。
- 人手能立即断电或切回手动。
- 不做连续任务，只做单步动作。
- 每次只改一个变量：速度、时长、方向一次只改一项。

## 初版脚本行为设计

建议新增脚本：

```text
src/real_rover_offboard_smoke.py
scripts/run_real_rover_offboard_smoke.sh
```

当前已经按这个路径实现脚本。默认不会主动 arm，也不会主动切 Offboard；它会先持续发送
stop/hold，并等待 Pixhawk 已经进入 Offboard 且 armed 后才开始动作序列。若要让脚本主动
请求 Offboard/arm，必须显式设置 `MODE_CHANGE_ON_START=true` 和 `ARM_ON_START=true`。

脚本启动参数建议：

```text
PX4_NAMESPACE=/px4_1
COMMAND_RATE_HZ=20
LINEAR_COMMAND=0.12
TURN_COMMAND=0.10
FORWARD_SEC=1.0
BACKWARD_SEC=1.0
TURN_SEC=0.5
STOP_SEC=1.0
ARM_ON_START=false
MODE_CHANGE_ON_START=false
```

安全行为：

- 启动后先连续发送 stop/hold 命令 1 秒。
- 打印即将执行的动作序列。
- 未显式设置 `ARM_ON_START=true` 时，不主动 arm。
- 未确认 `CONFIRM_WHEELS_LIFTED=true`、`CONFIRM_RC_READY=true`、
  `CONFIRM_PARAM_BACKUP=true` 时，脚本拒绝运行。
- Ctrl-C 时发送至少 0.5 秒 stop/hold。
- 每个动作之间强制 stop。
- 任何异常都先发 stop，再退出。

架空测试启动命令模板：

```bash
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_offboard_smoke.sh
```

默认 `COMMAND_MODE=velocity`。如果真实 Pixhawk 没有可用本地速度/位置估计，可能无法执行
velocity Offboard；那时再评估 `COMMAND_MODE=direct_actuator`，但 direct actuator 要求
QGC/PX4 输出功能已经能把 actuator 0/1 路由到当前接 Arduino 的两个 PWM 输出。

当前 Jetson 环境验证结果：

- `src/real_rover_offboard_smoke.py` 通过 Python 语法编译检查。
- `scripts/run_real_rover_offboard_smoke.sh` 通过 bash 语法检查。
- 默认运行会在启动 ROS 前拒绝，要求先确认车轮架空、遥控器可接管、Pixhawk 参数已备份。
- 当前系统没有找到 `px4_msgs`，所以真正运行前还需要 source 或构建包含 `px4_msgs`
  的 ROS 2 工作区。
- 2026-06-16 Pixhawk 通过 USB 枚举为 `/dev/ttyACM0`，by-id 路径是
  `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00`。
- 已安装 `pymavlink`，并新增 MAVLink 版 Offboard smoke test：
  `src/real_rover_mavlink_offboard_smoke.py` 和
  `scripts/run_real_rover_mavlink_offboard_smoke.sh`。它通过 Pixhawk USB 发
  `SET_POSITION_TARGET_LOCAL_NED`，不依赖 `px4_msgs` 或 MicroXRCEAgent。

MAVLink 版架空测试启动模板：

```bash
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavlink_offboard_smoke.sh
```

默认仍然不自动切 Offboard、不自动 arm。如果要让脚本主动请求 Offboard 和 arm，必须显式加：

```bash
MODE_CHANGE_ON_START=true ARM_ON_START=true
```

这只允许在车轮架空、D24A 电源最后接入、遥控器可接管、QGC 参数已经备份后使用。

## 控制器应该怎么做

最终动态对接不是只规划一条空间轨迹，而是规划时空轨迹：

```text
期望位置 p_d(t)
期望速度 v_d(t)
期望加速度 a_d(t)
期望航向/姿态 yaw_d(t)
每个时刻允许的安全走廊 corridor(t)
```

控制器的职责不是“死追某个时间点”，而是在物理约束内尽量跟踪。如果发现按当前状态已经追不上，
应该减速、等待、扩大时间窗、重新规划或中止，而不是超出车辆/飞机极限硬追。

推荐分层：

```text
时空走廊/轨迹规划器
        ↓
轨迹跟踪器 / MPC / Pure Pursuit + 速度控制
        ↓
PX4 位置/速度/姿态控制器，或车端速度/转向控制器
        ↓
电机/舵机/电调/驱动板
```

对小车，早期可以这样演进：

1. 开环短动作：只验证链路，不谈精确跟踪。
2. 速度闭环：测实际速度，用 PID 跟踪 `v_d(t)` 和 `yaw_rate_d(t)`。
3. 路径跟踪：用 Pure Pursuit 或 Stanley 跟踪空间路径。
4. 时空跟踪：在路径跟踪外层加时间误差控制，必要时重规划。
5. MPC：把速度、加速度、转弯半径、障碍物、时间窗一起放进优化。

对飞机，通常不要自己直接控电机，而是：

1. 外层规划器给 `position/velocity/acceleration/yaw` setpoint。
2. PX4 内层做位置、速度、姿态、角速度控制。
3. 轨迹不可行时外层重规划或调整到达时间。

## 需要采集哪些数据

是的，需要先跑车/飞机收集数据。不是为了做一次漂亮实验，而是为了知道规划问题的约束边界。

小车至少要测：

- 最小可动命令和电机死区。
- 最大安全速度。
- 最大加速度和最大减速度。
- 刹停距离。
- 最大 yaw rate。
- 不同速度下的转弯半径。
- 命令到动作的延迟。
- 电池电压下降后速度变化。
- 左右轮不一致程度。
- 地面摩擦变化对速度/转弯的影响。

飞机至少要测：

- 最大水平速度。
- 最大上升/下降速度。
- 最大加速度和减速度。
- 最大 yaw rate / roll rate / pitch rate。
- 悬停油门或推重比余量。
- 指令到实际运动的延迟。
- 风扰下的位置误差。
- 负载变化后的控制余量。

第一阶段不需要精确动力学模型，先要经验包络：

```text
在 0.10 / 0.15 / 0.20 / 0.25 命令下，
车实际跑多快、多久开始动、多久停下来、会不会偏航。
```

这些数据会决定时空走廊里的速度上限、加速度上限、时间裕度和安全距离。

## 第三步之后的最近任务

1. 导出 Pixhawk/QGC 当前成功参数。
2. 确认 Orin 到 Pixhawk 的通信方式：ROS 2 uXRCE-DDS、MAVSDK，还是 MAVLink。
3. 写 `real_rover_offboard_smoke.py`，先只做架空轮子短动作。
4. 记录第一次架空测试结果。
5. 再决定是否进入真正 PX4 Offboard velocity/direct-actuator 测试。
