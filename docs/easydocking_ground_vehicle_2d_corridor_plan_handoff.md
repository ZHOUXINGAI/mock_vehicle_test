# Ground Vehicle 2D CorridorPlan Handoff

面向接手本项目的另一个 Codex：这份文档说明当前 `easydocking` 项目的方法核心、代码入口、PX4 仿真状态，以及如何把空中异构对接问题迁移成两个户外小车的二维平面实验。

当前推荐基线：

- Git commit: `6d9dd74 Optimize PX4 corridor docking planner`
- 已推送远端: `github.com:ZHOUXINGAI/easydocking.git`
- PX4 验证参考: `results/20260624_134445_px4_sih`
- 当前重点: 不再把项目理解成单纯 MPC 追踪问题，而是一个高层 `Docking Planner / CorridorPlan` 的时空轨迹规划问题。

## 1. 项目方法核心

### 1.1 原始空中问题

项目目标是异构无人机空中动态对接：

- `Carrier`: 四旋翼，速度上限约 `<= 9~10 m/s`，机动性强但追不上固定翼高速盘旋。
- `Mini`: 固定翼，速度下限约 `>= 8 m/s`，不能悬停，不能停，不能瞬移。
- 两机需要在飞行中完成 rendezvous 与对接。
- 不能让 Carrier 追着 Mini 当前点跑；必须先做全局几何规划，再做末端闭合。

这件事的核心不是“最后 0.1m 怎么追”，而是：

1. 从全局时空角度选择一个合适的 rendezvous 点。
2. 让两机各自走符合自身动力学约束的轨迹。
3. 在末端进入同一条切线走廊，再做精对接。

### 1.2 CorridorPlan 的思想

`CorridorPlan` 是项目核心创新点，可以理解为高层空管员：

- 它不直接控制电机。
- 它计算“谁什么时候到哪里、沿哪条走廊飞、以什么速度进入末端对接”。
- 低层控制器只负责跟踪这个时空计划。

当前算法在 `src/easydocking_control/src/docking_controller.cpp` 的 `computeCorridorPlan()` 中：

1. 已知 Mini 的轨道圆：
   - 圆心 `O = (mini_orbit_center_x, mini_orbit_center_y)`
   - 半径 `R = mini_orbit_radius`
   - 轨道速度 `mini_orbit_speed`
2. 已知 Carrier 当前二维位置 `C`。
3. 从 `C` 对 Mini 轨道圆求几何切线，得到两个候选切点 `T1/T2`。
4. 根据切线方向与 Mini 轨道运动方向的一致性打分，选出切点 `T`。
5. `T` 就是 rendezvous / tangent anchor。
6. Carrier 走一段圆弧，从当前位置平滑接到 `T`，弧线末端与 Mini 轨道圆相切。
7. Mini 在轨道上稳定盘旋，等到触发相位后从圆轨道切出，沿同一条切线走廊直行。
8. 两车/两机在切线走廊上保持同向，Carrier 必须始终在 Mini 前方。
9. 进入近距离后，由 gap controller / MPC 缩小相对误差。

一句话：`CorridorPlan = 全局几何切线 + 时间同步 + 末端同向走廊`。

## 2. 当前代码架构

### 2.1 包结构

主要目录：

- `src/easydocking_control/`
  - 控制器、PX4 bridge、仿真 bridge、launch 文件。
- `src/easydocking_msgs/`
  - 自定义消息，包括 `CorridorPlan`、`DockingStatus`、`DockingCommand`。
- `scripts/`
  - 一键实验脚本、自动 START gate、报告生成。
- `docs/`
  - 项目文档、复现实验说明、PPT、方法总结。
- `results/`
  - PX4 实验结果，默认被 `.gitignore` 忽略。
- `result_sim/`
  - mock 运动学实验结果，目前有很多本地历史数据，默认不要提交。

### 2.2 关键文件

#### `docking_controller.cpp`

路径：

```text
src/easydocking_control/src/docking_controller.cpp
```

职责：

- 主控制器。
- 状态机：`IDLE -> APPROACH -> COMPLETED/FAILED`，历史上也有 `TRACKING/DOCKING` 概念。
- `computeCorridorPlan()` 计算几何切线、Carrier 弧线、Mini 触发相位、计划时间。
- `approachPhaseControl()` 执行 CorridorPlan：
  - Phase 1: Carrier 沿弧线飞向切点 `T`。
  - Phase 2: Carrier 沿切线进入编队和 gap 闭合。
- 当前末端优化重点在 Phase 2 gap controller：
  - Carrier 速度上限约 `10 m/s`。
  - Mini 末端速度命令约 `8.55 m/s`。
  - 5m 内使用平滑收缩 gap 曲线。
  - front guard 保证 Carrier 不被 Mini 越过。

当前保留的验证版本特点：

- `results/20260624_134445_px4_sih`
- `dist < 20m` 后 Carrier 走过约 `123.4m`，满足“末端直线尽量 150m 内结束”的目标。
- `dist < 5m` 内 front violation 为 `0/121`，Carrier 全程在 Mini 前方。
- best distance 约 `0.332m`，尚未最终达到 `0.1m`，但轨迹形状和 front-consistency 更重要。

#### `docking_controller_node.cpp`

路径：

```text
src/easydocking_control/src/docking_controller_node.cpp
```

职责：

- ROS2 node wrapper。
- 订阅：
  - `/carrier/odom`
  - `/mini/odom`
  - `/docking/command`
  - `/docking/command_latched`
- 发布：
  - `/carrier/setpoint/velocity`
  - `/mini/setpoint/velocity`
  - `/carrier/setpoint/pose`
  - `/mini/setpoint/pose`
  - `/docking/relative_pose`
  - `/docking/status`
  - `/docking/controller_debug`
  - `/docking/corridor_plan`
- `CorridorPlan` publisher 使用 `transient_local`，这是必须的；晚启动的 bridge 也要收到最新计划。

#### `CorridorPlan.msg`

路径：

```text
src/easydocking_msgs/msg/CorridorPlan.msg
```

核心字段：

```text
rendezvous_x, rendezvous_y, rendezvous_z
tangent_dir_x, tangent_dir_y
corridor_length
ahead_distance
mini_arrival_time
carrier_start_x, carrier_start_y
carrier_target_x, carrier_target_y
mini_orbit_phase_trigger_rad
corridor_valid
plan_id
```

二维小车实验里，`z` 可以固定为 `0`，其余字段完全有用。

#### PX4 bridge

空中 PX4 相关文件：

- `src/easydocking_control/scripts/px4_fixed_wing_bridge.py`
  - Mini 固定翼 bridge。
  - 状态机复杂：takeoff/orbit/glide/terminal/capture。
  - 之前的瞬移问题根源在这里，尤其是 `COMPLETED` 时不能把 Mini target 设为 Carrier 位置。
- `src/easydocking_control/scripts/px4_offboard_bridge.py`
  - Carrier 四旋翼 bridge。
  - 负责 offboard/arm、位置/速度 setpoint 转 PX4。
- `src/easydocking_control/scripts/px4_odom_bridge.py`
  - PX4 local position/vehicle attitude 转 ROS `/carrier/odom`、`/mini/odom`。

小车实验不要直接套用 fixed-wing bridge；应该新写 ground bridge，把高层 CorridorPlan 复用，低层换成地面车控制。

### 2.3 实验脚本

PX4 当前主脚本：

```bash
EXPERIMENT_DURATION_SEC=190 START_RVIZ=false KEEP_VERBOSE_PX4_TEXT_LOGS=false \
  timeout 320 bash scripts/run_px4_sih_docking_experiment.sh
```

重要默认值集中在：

- `scripts/run_px4_sih_docking_experiment.sh`
- `src/easydocking_control/launch/docking.launch.py`

报告：

- `scripts/generate_report.py`
- 输出 `trajectory_xy.png`、`speed_profile.png`、`trajectory_xy_full.gif`、`summary.txt`、`classification.txt`。

## 3. 二维小车实验的等价问题

### 3.1 为什么小车实验是合理的

二维小车不是简单“降级版”，而是验证 CorridorPlan 的核心：

- 是否能从全局几何上选择正确 rendezvous 点。
- 是否能让一个“不能原地停/转向受限”的目标车先绕圈，再切线切出。
- 是否能让另一个车提前进入同向走廊，并始终保持在前。
- 是否能在末端把直线 docking 段控制在有限长度内。

空中问题中的高度、TECS、固定翼起飞等复杂因素先拿掉，保留核心时空规划。

### 3.2 车辆角色映射

建议仍沿用 `carrier` / `mini` 命名：

| 空中系统 | 小车系统 | 约束 |
|---|---|---|
| Mini 固定翼 | Mini 小车 | 不允许停止，保持最小速度，模拟固定翼不能失速 |
| Carrier 四旋翼 | Carrier 小车 | 更灵活，但速度上限不高，不能从后面硬追 |
| Mini 盘旋 | Mini 绕圆行驶 | 一圈稳定圆轨道后再切线驶出 |
| 切线 glide | 切线直线段 | 两车同向进入 docking corridor |
| 空中 docking | 地面近距离跟随/接触 | 末端 gap 闭合，可先用虚拟 docking 判据 |

### 3.3 推荐外场缩放参数

空中参数太大，小车建议缩放：

| 参数 | PX4 空中 | 小车建议初值 |
|---|---:|---:|
| Mini 轨道半径 | `80m` 左右 | 第一版 `4.5m`，后续 `5~8m` |
| Mini 速度 | `8~10m/s` | 第一版 `0.9m/s`，必须快于 Carrier |
| Carrier 速度上限 | `9~10m/s` | 第一版 `0.7m/s`，必须慢于 Mini |
| 末端 corridor 长度 | `~150m` | 第一版约 `8m`，限制在 `15m` 内 |
| docking 距离判据 | `0.1~0.3m` | `0.05~0.20m`，看定位精度 |
| front guard | `0.1~0.3m` | `0.15~0.40m` |

如果定位不是 RTK/UWB 级别，不要一开始追 `0.1m`；先把目标设成 `0.3~0.5m`，验证轨迹逻辑。

## 4. 小车硬件架构建议

用户的小车硬件：

- 每车一台 Orin Nano
- 每车一个 Pixhawk
- 每车一个 Arduino
- 电机驱动板
- 电机

### 4.1 推荐分层

最稳的分层如下：

```text
Orin Nano
  ├─ ROS2 / easydocking high-level planner
  ├─ localization fusion consumer
  ├─ ground vehicle bridge
  └─ logging / visualization / safety supervisor

Pixhawk
  ├─ IMU / GPS / compass / EKF
  ├─ RC/manual override
  ├─ failsafe / arming / geofence
  └─ optionally low-level rover controller

Arduino
  ├─ motor PWM / direction control
  ├─ encoder reading
  ├─ low-level speed PID
  └─ emergency stop input

Motor driver + motors
```

### 4.2 两种控制路线

#### 路线 A：Pixhawk 做 Rover autopilot

Orin 通过 MAVROS/MAVSDK/uXRCE-DDS 给 Pixhawk 发速度/航向/航点 setpoint，Pixhawk 输出 PWM 到电机驱动或 Arduino。

优点：

- Pixhawk failsafe 完整。
- 手动/自动切换自然。
- 更接近真实无人系统架构。

缺点：

- PX4/ArduPilot Rover offboard 接口需要额外适配。
- 对差速/阿克曼车辆的参数整定会花时间。

#### 路线 B：Orin + Arduino 直接控车，Pixhawk 做定位和安全

Orin 运行 ROS2，订阅 Pixhawk/GPS/IMU 位置，直接把 `/cmd_vel` 或左右轮速度发给 Arduino。Arduino 做电机 PID 和驱动。

优点：

- 最快跑通二维 CorridorPlan。
- 代码路径短，调试清楚。
- 适合先做老师能看懂的外场 demo。

缺点：

- 安全/failsafe 要自己写。
- 自动驾驶架构不如 Pixhawk autopilot 完整。

建议第一阶段用路线 B 快速验证算法；第二阶段再把低层切回 Pixhawk Rover autopilot。

## 5. 小车 ROS2 软件架构建议

### 5.1 保留现有核心

保留：

- `docking_controller.cpp`
- `docking_controller_node.cpp`
- `easydocking_msgs`
- `experiment_logger.py`
- `generate_report.py`

新增：

```text
src/easydocking_control/scripts/ground_vehicle_odom_bridge.py
src/easydocking_control/scripts/ground_vehicle_cmd_bridge.py
src/easydocking_control/launch/ground_docking.launch.py
config/ground_vehicle.yaml
scripts/run_ground_docking_experiment.sh
```

### 5.2 Topic 设计

保持 controller 的输入输出不变：

输入：

```text
/carrier/odom       nav_msgs/Odometry
/mini/odom          nav_msgs/Odometry
/docking/command    easydocking_msgs/DockingCommand
```

输出：

```text
/carrier/setpoint/velocity   geometry_msgs/TwistStamped
/carrier/setpoint/pose       geometry_msgs/PoseStamped
/mini/setpoint/velocity      geometry_msgs/TwistStamped
/mini/setpoint/pose          geometry_msgs/PoseStamped
/docking/corridor_plan       easydocking_msgs/CorridorPlan
/docking/status              easydocking_msgs/DockingStatus
```

小车 bridge 再把这些 setpoint 转为：

```text
/carrier/cmd_vel             geometry_msgs/Twist
/mini/cmd_vel                geometry_msgs/Twist
```

或者直接发给 Arduino：

```text
serial: left_wheel_target, right_wheel_target
```

### 5.3 小车控制模型

统一使用二维 unicycle / differential drive 模型：

```text
x_dot = v * cos(yaw)
y_dot = v * sin(yaw)
yaw_dot = omega
```

控制输入：

```text
v     线速度
omega 角速度
```

从全局速度向量 `(vx, vy)` 转小车命令：

```text
target_yaw = atan2(vy, vx)
heading_error = wrap(target_yaw - yaw)
v_cmd = clamp(norm(vx, vy) * cos(heading_error), v_min, v_max)
omega_cmd = clamp(k_yaw * heading_error, -omega_max, omega_max)
```

Mini 车要设置 `v_min > 0`，例如 `0.6m/s`，模拟固定翼不能停。

Carrier 可以允许低速，但 docking 阶段也尽量不要完全停，否则轨迹不好看。

## 6. 小车版 CorridorPlan 怎么做

### 6.1 保持几何不变

小车版仍然是：

1. Mini 先绕圆。
2. Carrier 从当前位置对 Mini 圆轨道求切线。
3. 选择与 Mini 运动方向一致的切点 `T`。
4. Carrier 走弧线到 `T`。
5. Mini 到达相位后从圆切出。
6. 两车沿同一切线直线行驶。
7. Carrier 保持在 Mini 前方。
8. 近距离缩小 gap。

### 6.2 去掉高度

二维小车中：

- `z = 0`
- `terminal_relative_position.z = 0`
- 所有 `rel_z` 和 z guard 暂时禁用或固定为 ready。

不要把空中的高度误差逻辑带到小车，否则会污染二维验证。

### 6.3 参数缩放

建议新增 ground 参数，不要直接改 PX4 默认：

```yaml
mini_orbit_center_x: 0.0
mini_orbit_center_y: 0.0
mini_orbit_radius: 4.5
mini_orbit_speed: 0.9

carrier_approach_speed_limit: 0.7
carrier_tracking_speed_limit: 0.7
carrier_docking_speed_limit: 0.7

terminal_relative_position: [0.0, 0.0, 0.0]
ground_mode_2d: true
```

### 6.4 Mini 小车行为

Mini 小车不建议直接听 controller 的 `/mini/setpoint/velocity`，而是像固定翼 bridge 一样，有自己的状态机：

1. `TAKEOFF` 等价为 `INIT_READY`
2. `ORBIT`
   - 以固定速度绕圆。
   - 必须先完成一整圈稳定轨道。
3. `GLIDE`
   - 收到有效 `/docking/corridor_plan`。
   - 到达 `mini_orbit_phase_trigger_rad` 或 `mini_arrival_time` 后切线驶出。
4. `TERMINAL`
   - 沿切线保持 `v ≈ 1.0m/s`。
   - 不追 Carrier 的当前位置。

小车版最重要的禁忌和固定翼一样：

> Mini 不能在末端突然把目标点切到 Carrier 当前坐标，否则就是“瞬移/假对接”的地面版。

### 6.5 Carrier 小车行为

Carrier 小车可以直接跟踪 controller 输出：

- 远距离：跟踪 `/carrier/setpoint/pose`，用 pure pursuit 或 Stanley。
- 近距离：跟踪 `/carrier/setpoint/velocity`，把全局速度向量转成 `v/omega`。

建议从 velocity tracking 开始，因为当前 controller 已经给出平滑的速度命令。

## 7. 定位建议

如果要在户外做二维 docking，定位是成败关键。

### 7.1 推荐方案

优先级：

1. 双 RTK GPS，最好每车 `u-blox F9P`，一个 base station。
2. RTK + IMU yaw 融合，Pixhawk EKF 输出局部 ENU。
3. 轮速编码器辅助短时平滑。
4. 如果场地小，也可以 UWB + IMU，但要先验证延迟和漂移。

### 7.2 不推荐一开始用普通 GPS

普通 GPS 米级误差，不适合验证 `0.1~0.3m` docking。可以用来验证“大轨迹形状”，但不能证明末端精度。

### 7.3 坐标系约定

统一使用本地 ENU：

```text
x: East / 场地右方向
y: North / 场地前方向
z: 0
yaw: 从 x 轴逆时针
```

两车必须用同一个 map/world frame。不要每车各自一个 local frame 后再硬拼。

## 8. Arduino 协议建议

Orin 到 Arduino 串口建议简单明确：

### 8.1 命令

```text
CMD <timestamp_ms> <v_mps> <omega_radps>\n
```

或者差速轮：

```text
WHEEL <timestamp_ms> <left_mps> <right_mps>\n
```

### 8.2 回传

```text
STATE <timestamp_ms> <left_mps> <right_mps> <battery_v> <estop>\n
```

### 8.3 安全

Arduino 必须实现：

- 超过 `300ms` 没收到命令自动刹车。
- E-stop 引脚最高优先级。
- 速度命令限幅。
- 加速度限幅。
- 电池低压保护。

## 9. 外场实验阶段计划

### Stage 0：桌面闭环

目标：

- 不接电机。
- 两个 ground bridge 使用 fake odom。
- 验证 controller、CorridorPlan、report 全链路。

验收：

- 能生成 `trajectory_xy.png`。
- Mini 圆轨道 + 切线直线。
- Carrier 弧线 + 切线直线。
- Carrier 全程在 Mini 前面。

### Stage 1：单车低速闭环

目标：

- 单车跟踪圆轨道和直线。
- 只验证 `ground_vehicle_cmd_bridge.py`。

验收：

- 小车能稳定绕 `R=5~8m` 圆。
- 切线驶出无急转折线。
- 速度曲线没有尖峰。

### Stage 2：双车无 docking 接触

目标：

- 两车都跑。
- Mini 先完成一圈。
- Carrier 根据 CorridorPlan 出发。

验收：

- 两车进入同一切线走廊。
- Carrier 在前。
- 最小距离可以先设 `0.5m`。

### Stage 3：近距离 docking

目标：

- 把最小距离压到 `0.2~0.3m`。
- 如果定位足够，再尝试 `0.1m`。

验收：

- `front violation = 0`
- Mini 不停车。
- terminal straight length 在缩放后合理，例如 `8~15m`。
- 没有突然大转角或目标点跳变。

### Stage 4：物理接触/机构

目标：

- 加软连接、磁吸、导向槽或机械 dock。

注意：

- 机械结构要先容忍 `0.2~0.3m` 误差。
- 不要一开始要求算法精度承担所有机械误差。

## 10. 给接手 Codex 的具体实现任务

### 10.1 新增 ground launch

新建：

```text
src/easydocking_control/launch/ground_docking.launch.py
```

启动节点：

- `docking_controller_node`
- `ground_vehicle_odom_bridge.py` for carrier
- `ground_vehicle_odom_bridge.py` for mini
- `ground_vehicle_cmd_bridge.py` for carrier
- `ground_vehicle_cmd_bridge.py` for mini
- `experiment_logger.py`
- 可选 `rviz_visualizer.py`

### 10.2 新增 ground config

新建：

```text
config/ground_vehicle.yaml
```

包含：

- 场地坐标系原点。
- Mini 圆心/半径/速度。
- Carrier 速度/加速度上限。
- 车体轴距或轮距。
- 串口设备名。
- 定位来源。
- safety timeout。

### 10.3 新增 odom bridge

新建：

```text
src/easydocking_control/scripts/ground_vehicle_odom_bridge.py
```

职责：

- 从 Pixhawk/MAVROS/MAVSDK/serial 读取位置、速度、yaw。
- 发布标准 `nav_msgs/Odometry` 到 `/carrier/odom` 或 `/mini/odom`。
- 保证 covariance 和 timestamp 合理。

### 10.4 新增 cmd bridge

新建：

```text
src/easydocking_control/scripts/ground_vehicle_cmd_bridge.py
```

职责：

- 订阅 `/carrier/setpoint/velocity` 或 `/mini/setpoint/velocity`。
- 订阅当前 odom。
- 全局速度向量转小车 `v/omega`。
- 串口发 Arduino。
- Mini 车在 `ORBIT/GLIDE` 状态下优先执行自己的 orbit/tangent tracker，不要末端追 Carrier 点。

### 10.5 修改 controller 的二维模式

建议加参数：

```text
ground_mode_2d: bool
```

开启后：

- 忽略 z 误差。
- `terminal_z_ready = true`。
- completion 不检查 `rel_z`。
- `terminal_relative_position.z = 0`。

不要破坏 PX4 空中默认参数；ground 走单独 launch/config。

## 11. 当前不要做的事

1. 不要把 `simple_dual_uav_sim.py` 当成主线继续改；小车要新建 ground bridge。
2. 不要把 Mini 的目标点切到 Carrier 当前点。
3. 不要为了 final distance 牺牲 front-consistency。
4. 不要让小车上来就跑高速。
5. 不要在普通 GPS 下宣称 `0.1m` 对接精度。
6. 不要把 PX4 fixed-wing bridge 的复杂起飞状态机搬到小车。

## 12. 推荐第一版小车 demo 指标

第一版不要追求太狠，建议目标：

- Mini 绕 `R=4.5m` 圆完整一圈。
- Mini 速度 `0.9m/s`，最低不低于 `0.6m/s`。
- Carrier 最大速度 `0.7m/s`，明确慢于 Mini。
- 两车进入同一切线走廊。
- Carrier 在 Mini 前方，全程无越位。
- terminal straight length `< 15m`，当前仿真约 `8.2m`。
- 全轨迹包络应小于 `30m x 30m`，当前仿真约 `12.3m x 11.3m`。
- 最小距离 `< 0.5m`。
- 有完整 `trajectory_xy.png`、`speed_profile.png`、CSV log。

第二版再把最小距离压到 `< 0.2m`。


### 12.1 当前默认 Stage-0 路线

`scripts/run_ground_2d_corridor_sim.py` 的当前默认小车场景用于有限场地第一版验证：

```text
Mini orbit center: (0.0, 0.0) m
Mini orbit radius: 4.5 m
Mini speed: 0.9 m/s
Mini required orbit: 1 full lap
Turn direction: ccw
Mini planning phase: 315 deg
Carrier start: (-7.0, -6.0) m
Carrier max speed: 0.7 m/s
Carrier max acceleration: 0.30 m/s^2
Pass distance: 0.5 m
```

仿真参考 `results/ground_2d/20260625_003145_ground_2d`：

```text
Tangent point: (-1.553, -4.224) m
Tangent direction: (0.939, -0.345)
Carrier arc length: 6.178 m
Carrier arc duration: 17.652 s
Mini arrival delay after plan: 25.724 s
First pass time: 64.60 s
Terminal path until first pass: 8.214 m
Bounding box until first pass: 12.23 m x 11.22 m
Front violations: 0/334
Minimum terminal distance: 0.498 m
```

这条路线故意让 Carrier 比 Mini 慢，验证 planner 不是靠追，而是靠提前占位和切线走廊调度。

## 13. 一句话给老师讲

我们不是简单让后车追前车，而是提出了一个面向异构平台的时空走廊规划器：

> 对受约束目标的周期轨道求可达切线 rendezvous 点，将两车/两机调度进同一条末端切线走廊，再在短距离内用反馈控制闭合 gap。

这个方法的价值在于：

- 它先解决“什么时候、在哪里、沿什么方向接近”的全局规划问题。
- 它尊重异构平台约束：固定翼/目标车不能停，Carrier/接收车不能盲追。
- 它把高层 planner 和低层 tracker 解耦，方便从 PX4 空中仿真迁移到地面小车。
