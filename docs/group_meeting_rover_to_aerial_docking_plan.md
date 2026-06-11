# 从小车验证到双无人机空中对接的长期实验路线

## 1. 最终目标

本项目最终目标是实现 **双无人机空中对接**：

```text
固定翼无人机
    -> 在空中完成末端引导
    -> 降落/对接到大型四旋翼平台上
```

当前仿真部分已经有较大进展，主观估计约完成 70%。下一阶段的核心任务不是继续只在仿真中堆功能，而是逐步把控制链路、安全流程、传感器和算法迁移到实物系统中。

最终系统可以抽象为：

```text
机载电脑 / 视觉计算单元
    -> PX4 / Pixhawk 飞控
    -> 电调 / 舵机 / 执行机构
    -> 飞行器运动
```

其中最危险、最需要提前熟悉的是 **offboard 控制、模式切换、failsafe、安全边界和外场调试流程**。

## 2. 为什么先做小车实验

我之前主要飞过航点任务和航线任务，对 PX4 offboard 实物飞行经验不足。offboard 飞行一旦控制链路、坐标系、failsafe 或模式切换出问题，风险会直接体现在真实飞机上。

所以当前选择先做小车，是为了建立一个低风险的实物训练平台：

```text
小车平台
    -> 低速、低空、低风险
    -> 可以反复断电、架空、手动干预
    -> 能验证 companion computer + PX4/MCU + 电机执行链路
    -> 能提前练习 ROS2、MAVLink、offboard、日志、安全策略
```

小车不是最终目标，而是把空中对接系统拆成更安全的地面版本：

```text
双无人机对接问题
    -> 先转换成双小车 mock docking 问题
    -> 验证相对定位、状态机、控制闭环、末端视觉和安全边界
    -> 再逐步迁移回无人机
```

这样可以在不冒飞行风险的情况下，把控制逻辑和工程流程提前跑通。

## 3. 无人机系统和小车系统的对应关系

| 无人机系统 | 小车实验阶段对应物 | 作用 |
| --- | --- | --- |
| 机载电脑 / Origin Nano / 鲁班猫 | 鲁班猫 / Origin Nano | ROS2、视觉、导航、任务逻辑、日志 |
| Pixhawk 飞控 | Pixhawk Rover / Arduino / ESP32 底盘控制层 | 模式、安全、PWM/执行器控制、底层实时控制 |
| 电调 ESC | D153B/TB6612 / 有刷电调 | 把低功率控制信号变成电机大电流 |
| 电机 / 舵机 | MG513X 直流减速电机 / 编码器 | 实际执行机构 |
| QGC / MAVLink / PX4 参数 | QGC / MAVLink / PX4 Rover | 模式管理、调参、failsafe、geofence |
| 空中对接末端视觉 | 小车二维码/AprilTag/深度相机识别 | 消除末端稳态误差 |

关键认识：

```text
鲁班猫不应该直接 bit-bang 控电机
更成熟的方式是：
鲁班猫/Origin Nano -> Pixhawk/MCU -> 电机驱动 -> 电机
```

这和无人机中“机载电脑不直接控电调，Pixhawk 负责实时控制和安全”的架构是一致的。

## 4. 当前已经完成的工作

### 4.1 Arduino + 遥控器 + 小车验证

已经跑通：

```text
遥控器 / 接收机
    -> Arduino
    -> D153B/TB6612 电机驱动板
    -> MG513X 左右轮电机
```

验证内容：

- 左右轮能前进、后退、差速转向。
- D153B 可以驱动 MG513X 两线直流减速电机。
- MG513X 是直流减速电机，不是三相无刷电机；无人机三相无刷电调不能直接使用。

### 4.2 鲁班猫 + Arduino + 小车验证

已经跑通：

```text
鲁班猫
    -> USB 串口
    -> Arduino
    -> D153B
    -> 左右轮电机
```

仓库中对应代码：

```text
arduino/d153b_serial_bridge/d153b_serial_bridge.ino
scripts/lubancat_arduino_serial_drive.py
```

鲁班猫可以通过命令控制小车：

```bash
python3 scripts/lubancat_arduino_serial_drive.py --mode forward
python3 scripts/lubancat_arduino_serial_drive.py --mode backward
python3 scripts/lubancat_arduino_serial_drive.py --mode left
python3 scripts/lubancat_arduino_serial_drive.py --mode right
```

这一步的意义是：鲁班猫已经开始作为 companion computer 参与小车控制，后续可以逐步替换成 ROS2 节点、MAVLink 节点或更高层任务逻辑。

### 4.3 硬件供电经验

这几天排查出几个关键电源问题：

- Arduino 不要同时接 USB 5V 和外部稳压模块 5V。
- GND 必须共地，尤其是 Arduino GND 和 D153B GND 要直接连接。
- 不同 5V 降压模块输出不要直接并联。
- D153B 电机电源应由 12V 电池直接进入 J3/VM，不要经过 1A 降压模块。
- 电机大电流线和信号线要分清，杜邦线接触不可靠，需要后续固定或换连接器。

推荐电源结构：

```text
12V 锂电池
    -> 总开关 / 保险丝
    -> 12V 直出给 D153B 电机驱动板
    -> 5.1V 5A 降压给鲁班猫
    -> 5V BEC 给 Pixhawk / 接收机 / 小舵机
```

原则：

```text
共 GND
不并 5V
电机电源直供
计算机和飞控独立稳压
```

### 4.4 Pixhawk + Arduino + D153B 翻译层准备

已经新增 Pixhawk 过渡方案：

```text
Pixhawk 6C
    -> PWM 输出
    -> Arduino 翻译层
    -> D153B
    -> MG513X 电机
```

仓库中对应代码和文档：

```text
arduino/d153b_pixhawk_pwm_bridge/d153b_pixhawk_pwm_bridge.ino
docs/pixhawk6c_arduino_d153b_bridge.md
```

这个方案的原因是：

```text
Pixhawk 输出：1000-2000us 舵机 PWM
D153B 输入：PWMA/PWMB + AIN/BIN 方向控制
```

两者不能直接相连，所以 Arduino 暂时作为翻译层。

## 5. 长期路线规划

### 阶段 1：稳定小车基础硬件

目标：

- 固定当前小车底盘、电机、电源和线束。
- 保证左右轮基本运动可靠。
- 建立明确的上电、断电、急停流程。

验收标准：

- 前进、后退、左转、右转稳定。
- 小车架空和落地测试一致。
- 电源线、地线、信号线可靠固定。

### 阶段 2：鲁班猫作为 companion computer 控制小车

目标：

- 鲁班猫通过 USB 串口控制 Arduino。
- Arduino 做底层电机控制和超时停车。
- 鲁班猫逐步从命令行脚本升级为 ROS2 节点。

意义：

```text
先练 companion computer -> 底层控制器 的工程链路
为后续 Origin Nano / Pixhawk / offboard 打基础
```

### 阶段 3：Pixhawk 6C 接入小车控制链

目标：

```text
遥控器 / QGC / PX4
    -> Pixhawk 6C
    -> Arduino 翻译层
    -> D153B
    -> 电机
```

重点验证：

- Pixhawk MAIN PWM 输出。
- QGC 参数配置。
- Manual 模式控制。
- failsafe、解锁、模式切换。
- RC 接管链路。

这一步是从普通 Arduino 小车向 PX4 Rover 体系迁移的关键。

### 阶段 4：PX4 Rover + ROS2/offboard 入门

目标：

- 鲁班猫连接 Pixhawk。
- 读取 Pixhawk 状态。
- 从只读状态开始，不直接控制。
- 再逐步发送简单 offboard setpoint。

初始任务：

```text
小车前进 1-3 米
停止
返回
超出安全边界自动停车
```

这一阶段主要练：

- ROS2 节点。
- MAVLink / px4_msgs。
- offboard 模式进入和退出。
- 安全超时。
- 日志记录。

### 阶段 5：外场 GPS / geofence / return 测试

目标：

- 在室外低速环境下测试定位、安全边界和恢复流程。
- 熟悉真实外场调试纪律。

重点：

- GPS 定位。
- geofence。
- 手动接管。
- 低电压保护。
- 通信中断保护。
- 日志复盘。

这一步是之后上飞机前必须具备的安全训练。

### 阶段 6：双小车 mock docking

目标：

用两台地面车模拟双无人机对接：

```text
小车 A：模拟大型四旋翼平台
小车 B：模拟固定翼接近目标
```

计划：

- 一台车使用鲁班猫。
- 一台车使用 Origin Nano。
- 参考 docking 仓库中的控制和状态机算法。
- 在地面实现“相对接近、队形保持、末端对准、停止/对接”的 mock 流程。

意义：

```text
先在地面验证双机协同算法
再迁移到空中
```

### 阶段 7：末端视觉对接

目标：

解决末端稳态误差。

方案：

- 使用二维码 / AprilTag 作为末端视觉标志。
- 在 Origin Nano 上接深度相机或普通相机。
- 输出相对位姿。
- 小车阶段先验证视觉闭环。

迁移方向：

```text
地面二维码对准
    -> 空中末端视觉引导
    -> 固定翼对准四旋翼平台
```

### 阶段 8：一车 + 一固定翼子系统验证

目标：

在进入双飞机前，先做混合验证：

- 地面平台模拟大型四旋翼。
- 固定翼相关子系统验证末端识别、状态机、通信和安全逻辑。

重点不是直接飞，而是逐步把固定翼端的感知和决策链路接入。

### 阶段 9：大型四旋翼 + 固定翼空中对接

最终目标：

```text
大型四旋翼稳定飞行 / 承载平台
固定翼进入末端引导
视觉定位消除误差
PX4/offboard 管理控制过程
完成固定翼在大型四旋翼上的降落/对接
```

这一阶段只有在以下条件满足后才推进：

- 小车 offboard 流程稳定。
- 双小车 mock docking 稳定。
- 外场 geofence/failsafe 流程成熟。
- 末端视觉识别稳定。
- 手动接管和中止策略明确。

## 6. 从小车迁移到 PX4 双无人机的关键映射

| 小车阶段验证内容 | 无人机阶段迁移目标 |
| --- | --- |
| 串口命令控制小车 | companion computer 发高层控制指令 |
| Arduino 底层超时停车 | 飞控 failsafe / offboard timeout |
| Pixhawk Rover 输出 PWM | Pixhawk 飞机执行器输出 |
| 小车 geofence | 无人机地理围栏 |
| 双小车相对接近 | 双机相对导航 |
| 二维码/AprilTag 末端对准 | 空中末端视觉引导 |
| 小车手动接管 | 飞行器 RC 接管 / mode switch |
| 小车日志复盘 | 飞行日志和事故复盘 |

核心思想：

```text
先在小车上验证流程
再在 PX4 Rover 中验证 PX4 模式和 offboard
最后迁移到真实飞机
```

## 7. 当前下一步计划

近期优先级：

1. 固定小车现有线束，避免杜邦线接触不良。
2. 完成一电池多路供电方案：
   - 12V 给 D153B。
   - 5.1V/5A 给鲁班猫。
   - 独立 5V BEC 给 Pixhawk/接收机。
3. 上传并测试 Pixhawk -> Arduino -> D153B 翻译层。
4. 在 QGC 中配置 Pixhawk Rover 手动控制。
5. 只做架空测试，确认 PWM 输出方向。
6. 落地低速测试。
7. 再接入鲁班猫，做 ROS2/offboard 最小实验。

## 8. 总结

当前小车实验的定位不是“做一台玩具车”，而是为双无人机空中对接建立一套低风险实物验证流程。

小车阶段要解决的是：

- companion computer 如何参与控制；
- PX4/offboard 如何安全进入和退出；
- 电源、地线、执行器如何可靠工作；
- 手动接管、地理围栏、failsafe 如何设计；
- 双机协同和末端视觉如何从仿真落到实物。

最终迁移目标是：

```text
小车 mock docking
    -> PX4 Rover offboard
    -> 双小车协同
    -> 末端视觉对接
    -> 固定翼 + 大型四旋翼空中对接
```

这条路线的核心价值是：用低风险平台提前暴露工程问题，把高风险飞行实验拆成可控、可复现、可逐步验证的阶段。
