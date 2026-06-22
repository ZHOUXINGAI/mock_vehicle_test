# 当前四驱车成功基线（2026-06-16）

这份文档只记录现在已经跑通的状态，目的是以后改坏了可以按这里恢复。

## 已确认跑通的两条主链路

1. `Orin Nano -> Pixhawk 6C -> Arduino UNO -> D24A -> 四个直流电机`
2. `AT9S PRO 遥控器 -> R9DS 接收机 -> Pixhawk 6C -> Arduino UNO -> D24A -> 四个直流电机`

当前主控链路以 Pixhawk 输出 PWM 给 Arduino 为准。Arduino 只做 PWM 读取、混控和 D24A
方向/占空比输出，不在 Arduino 里做自主导航。

## 当前主 Arduino 程序

主程序：

```text
arduino/d24a_pixhawk_pwm_bridge/d24a_pixhawk_pwm_bridge.ino
```

关键行为：

- 当前遥控器通道固定为：`CH2` 控制前进/后退，`CH4` 控制左右转向。
- PX4 中对应关系固定为：`RC_MAP_PITCH=2`、`RC_MAP_YAW=4`。
- 按当前实测输出路径，PX4 输出功能必须交叉分配：
  `PWM_MAIN_FUNC1=405` (`RC Yaw`)、`PWM_MAIN_FUNC2=403` (`RC Pitch`)。
- 这组交叉分配是为了让当前硬件路径最终表现为：`CH2` 控制 Arduino
  前进/后退输入，`CH4` 控制 Arduino 左/右转向输入。
- 中位是 `1500 us`。
- 输入丢失、超出有效范围或启动初期未收到有效 PWM 时，Arduino 停止所有电机。
- 当前电机最大 PWM 被限制在 `140`，适合继续低速安全测试。
- 串口波特率 `115200`，会打印 `thr_us`、`steer_us`、`left`、`right` 等调试信息。

备用/历史测试程序：

- `arduino/d24a_serial_bridge/d24a_serial_bridge.ino`：Orin/电脑通过 USB 串口直接控制 Arduino 和 D24A。

## Pixhawk 到 Arduino 接线

```text
Pixhawk PWM output GND      -> Arduino GND
Pixhawk PWM +5V             -> 不接 Arduino 5V
```

要求：

- Pixhawk、Arduino、D24A 必须共地。
- 当前不要把 Pixhawk PWM 口的 `+5V` 接到 Arduino `5V`。
- 当前两路信号功能按实测恢复为：`PWM_MAIN_FUNC1=405`、
  `PWM_MAIN_FUNC2=403`。不要仅按 output 编号推断前后/转向功能。
- 测试时先确认 Pixhawk 两路输出中位约为 `1500 us`，再给 D24A 上电。

## D24A 到 Arduino 接线

```text
D24A PWMA -> Arduino D3
D24A AIN1 -> Arduino D4
D24A AIN2 -> Arduino D7

D24A PWMB -> Arduino D5
D24A BIN1 -> Arduino D8
D24A BIN2 -> Arduino D12

D24A PWMC -> Arduino D6
D24A CIN1 -> Arduino D10
D24A CIN2 -> Arduino D11

D24A PWMD -> Arduino D9
D24A DIN1 -> Arduino A0
D24A DIN2 -> Arduino A1

D24A STBY -> Arduino A2
D24A GND  -> Arduino GND
```

供电基线：

- D24A 电机电源使用当前已验证的 12V 电池/电源输入。
- Arduino 可以由 Orin Nano/电脑 USB 供电，或者使用稳定 5V 逻辑电源。
- 不要同时用 D24A 的 5V 输出和 USB 给 Arduino 供电。
- Orin Nano、Pixhawk、Arduino、D24A 的信号链路必须有共同地线。

## 当前电机方向基线

原始 D24A 通道到实际轮子的方向：

```text
A forward  = 右前轮后退
A backward = 右前轮前进

B forward  = 左前轮后退
B backward = 左前轮前进

C forward  = 左后轮前进
C backward = 左后轮后退

D forward  = 右后轮前进
D backward = 右后轮后退
```

因此当前高层动作映射固定为：

```text
forward  = A:-pwm  B:-pwm  C:+pwm  D:+pwm
backward = A:+pwm  B:+pwm  C:-pwm  D:-pwm
left     = A:-pwm  B:+pwm  C:-pwm  D:+pwm
right    = A:+pwm  B:-pwm  C:+pwm  D:-pwm
```

不要随意反接电机线或改这组映射。以后如果单个轮子方向不对，先用
`docs/d24a_current_motor_mapping.md` 重新做轮子悬空校准，再改代码。

## QGC / Pixhawk 当前记录

已经确认：

- 遥控器到接收机再进 Pixhawk 的手动链路已能通过 Pixhawk 控制 Arduino 和 D24A。
- Pixhawk 给 Arduino 的两路输出按 `1500 us` 中位、两边偏移控制。
- 当前遥控器 `CH2` 是前进/后退输入，`CH4` 是左右转向输入。
- 当前 PX4 输出功能必须使用实测交叉分配：`MAIN1=RC Yaw`、
  `MAIN2=RC Pitch`。这才会让最终车辆表现为 `CH2` 前进/后退、`CH4` 转向。

## 当前 PX4 输出配置（2026-06-21 02:40 CST）

当前飞控固件为 PX4 v1.17.0 rover，差分车架已保存：

```text
SYS_AUTOSTART=50000
SYS_AUTOCONFIG=0
MAV_TYPE=10
CA_AIRFRAME=6
CA_R_REV=3
COM_RC_IN_MODE=3
```

接线不变时，遥控器到电机的恢复基线是 Pixhawk 两路 PWM 直通 Arduino：

```text
RC_MAP_PITCH=2
RC_MAP_YAW=4
PWM_MAIN_FUNC1=405   # RC Yaw passthrough, required by observed output path
PWM_MAIN_FUNC2=403   # RC Pitch passthrough, required by observed output path
PWM_MAIN_FUNC6=0
PWM_MAIN_FUNC7=0
PWM_MAIN_MIN1=1000
PWM_MAIN_MAX1=2000
PWM_MAIN_MIN2=1000
PWM_MAIN_MAX2=2000
PWM_MAIN_DIS1=1500
PWM_MAIN_DIS2=1500
PWM_MAIN_DIS6=1500
PWM_MAIN_DIS7=1500
PWM_MAIN_FAIL1=1500
PWM_MAIN_FAIL2=1500
PWM_MAIN_FAIL6=1500
PWM_MAIN_FAIL7=1500
```

`MAIN6/MAIN7` 必须保持 disabled。旧配置里这两路曾被设成 `Motor 1`，在本车的
Arduino/D24A 桥上会增加右前/右后轮异常转动的风险。

## Differential Rover Offboard 测试入口

真正按 PX4 differential rover 控制器做 Offboard 时，PX4 输出会临时切到：

```text
PWM_MAIN_FUNC1=101   # Motor 1
PWM_MAIN_FUNC2=102   # Motor 2
```

这要求 Arduino 使用：

```text
arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino
```

不要把 differential Offboard 输出接到旧的 throttle/steering Arduino mixer
里二次混控。测试脚本见：

```text
docs/differential_rover_offboard_tests_2026_06_21.md
```

尚未固化到文件的内容：

- Pixhawk/QGC 完整参数还没有导出为 `.params`。
- 具体 flight mode、arm/safety 开关、actuator/function 分配还需要截图或导出确认。
- 下一步不要靠记忆改 QGC 参数，应先导出当前参数作为可恢复备份。

建议下一步导出到：

```text
config/pixhawk/pixhawk6c_rover_success_2026_06_16.params
```

## 恢复这套成功状态的顺序

1. 上传 `arduino/d24a_pixhawk_pwm_bridge/d24a_pixhawk_pwm_bridge.ino` 到 Arduino UNO。
2. 按本文恢复 D24A 到 Arduino 的所有方向脚、PWM 脚和 `STBY` 脚。
3. 恢复 Pixhawk 到 Arduino 的两路 PWM 信号和共地；功能分配按
   `PWM_MAIN_FUNC1=405`、`PWM_MAIN_FUNC2=403`，不要改回 `403/405`。
4. 先不要给 D24A 电机电源上电，打开 Arduino 串口 `115200` 看 PWM 输入是否在中位附近。
5. 在 QGC 里确认 Pixhawk 输出中位约 `1500 us`，拨杆/摇杆变化方向正确。
6. 抬起车轮或架空车体。
7. 最后给 D24A 上 12V 电机电源。
8. 先小幅推杆，确认前进、后退、左转、右转方向都和本文一致。

## 当前不要动的东西

- 不要改 D24A 到 Arduino 的引脚表。
- 不要改四个轮子的高层方向映射。
- 不要提高 `MAX_DRIVE_PWM`，除非车轮悬空测试和急停都重新确认。
- 不要绕过 Pixhawk 的手动安全链路直接做 Orin 自主控制。
- 不要在未导出参数前大改 QGC actuator、mixer、RC mapping。

## 下一步

先导出 Pixhawk/QGC 当前成功参数，再进入最小自主控制测试。自主控制前仍然要求：

- 手动遥控链路可随时接管。
- 车轮先悬空。
- 软件限速。
- 有明确断电/停止手段。
