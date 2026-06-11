# Pixhawk 6C + Lubancat + Arduino + D153B 明日接线单

目标链路：

```text
ET16S 遥控器
  -> RF209S 接收机
  -> Pixhawk 6C
  -> MAIN PWM 输出
  -> Arduino UNO 翻译层
  -> D153B / TB6612 驱动板
  -> 左右有刷电机

Lubancat 4
  -> USB 连 Pixhawk（QGC / MAVLink / 后续 ROS2）
```

这一步里，Lubancat 先不要直接控电机。
明天先把 `Pixhawk -> Arduino -> D153B -> 电机` 跑通，Lubancat 只做上位机。

## 1. 先记住三条硬规则

1. Pixhawk 的 PWM 口只能出控制信号，不能直接带电机。
2. D153B 的 `VM` 是电机电源输入，接 12V 电池。
3. Arduino 不要同时吃 USB 5V 和外部 5V。

## 2. 电源怎么分

推荐先这样分，最稳：

```text
12V 电池
  -> 直接给 D153B 的 VM/GND
  -> 经过 5V BEC / 降压模块，给 Pixhawk / 接收机
  -> Lubancat 单独供电

Arduino UNO
  -> 先只用 USB 供电
```

说明：

- D153B 电机电流不要走小电流降压模块。
- D153B 的电机电源直接吃 12V 主电池。
- Pixhawk 走它自己的电源模块或稳定 5V。
- Lubancat 继续单独供电，别跟电机电源硬绑在一起。

## 3. D153B -> 电机

先按这个接：

```text
左电机两根粗线  -> D153B AO1 / AO2
右电机两根粗线  -> D153B BO1 / BO2
```

如果后面方向反了，不急着重接硬件，优先改 Arduino 里的方向反相常量。

## 4. D153B -> Arduino UNO

按已经验证过的桥接程序接：

| D153B | Arduino UNO |
| --- | --- |
| PWMA | D3 |
| AIN2 | D4 |
| AIN1 | D5 |
| STBY | D7 |
| BIN1 | D8 |
| BIN2 | D9 |
| PWMB | D10 |
| GND1 | Arduino GND |

先不要接：

```text
D153B 5V/VCC -> Arduino 5V/VCC
```

原因：明天首测 Arduino 先只用 USB 供电。D153B 手册里的确有用板载 5V
给 Arduino 供电的接法，但不能和 USB 5V 同时并在一起。

说明：

- `PWMA/PWMB` 是速度 PWM。
- `AIN1/AIN2/BIN1/BIN2` 是方向。
- `STBY` 是驱动使能，高电平才工作。
- `GND1` 接 Arduino GND。

## 5. D153B -> 12V 主电池

```text
12V 电池正极 -> D153B J3 / VM+
12V 电池负极 -> D153B J3 / GND
```

如果板子带电源开关：

- 初次接线完成后，先保持 `OFF`
- 所有信号线确认完，再拨到 `ON`

## 6. Pixhawk 6C -> Arduino UNO

这里不是接 D153B，而是先接 Arduino 输入脚：

| Pixhawk 6C | Arduino UNO | 作用 |
| --- | --- | --- |
| MAIN OUT 1 信号 | D2 | 左轮命令输入 |
| MAIN OUT 2 信号 | D6 | 右轮命令输入 |
| MAIN OUT GND | Arduino GND | 信号地 |

这一步先只接：

- 信号
- 地

不要接 Pixhawk PWM 排针上的 `+5V` 到 Arduino `5V`。

## 7. 接收机 -> Pixhawk

这一段按飞控常规接：

```text
RF209S -> Pixhawk RC IN
```

如果你现在已经有一根接收机到飞控的 RC 输入线，就沿用那根。

明天这一步的目标不是折腾接收机协议，而是让 Pixhawk 能在 QGC 里看到遥杆。

## 8. Lubancat -> Pixhawk

明天先简单接：

```text
Lubancat USB -> Pixhawk USB
```

这样你能先做：

- QGC 连飞控
- 看模式 / 遥控 / PWM 输出
- 后续再做 MAVLink / ROS2 / offboard

先不要让 Lubancat 直接参与电机控制闭环。

### Ubuntu 里确认 Pixhawk USB 串口

Pixhawk 6C 通过 USB 接到 Lubancat 后，正常会出现：

```text
/dev/ttyACM0
```

更稳定的路径是：

```text
/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00
```

检查命令：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
ls -l /dev/serial/by-id/
```

如果能看到类似下面的内容，就说明 USB 串口已经识别：

```text
/dev/ttyACM0
usb-Auterion_PX4_FMU_v6C.x_0-if00 -> ../../ttyACM0
```

如果提示权限不够，把当前用户加入 `dialout` 组，然后重登或重启：

```bash
sudo usermod -aG dialout cat
```

## 9. 全部接线总览

```text
12V 电池 +
  -> D153B VM+

12V 电池 -
  -> D153B GND

左电机
  -> D153B AO1 / AO2

右电机
  -> D153B BO1 / BO2

D153B PWMA -> Arduino D3
D153B AIN2 -> Arduino D4
D153B AIN1 -> Arduino D5
D153B STBY -> Arduino D7
D153B BIN1 -> Arduino D8
D153B BIN2 -> Arduino D9
D153B PWMB -> Arduino D10
D153B GND1 -> Arduino GND

Pixhawk MAIN1 signal -> Arduino D2
Pixhawk MAIN2 signal -> Arduino D6
Pixhawk MAIN GND     -> Arduino GND

RF209S -> Pixhawk RC IN

Lubancat USB -> Pixhawk USB
Arduino USB  -> 电脑或 Lubancat（只负责供电/刷程序/串口看日志）
```

## 10. 必须共地的地方

至少这三处地必须共地：

```text
D153B GND
Arduino GND
Pixhawk PWM GND
```

如果 Pixhawk 是独立供电，也没关系，但 PWM 信号地一定要跟 Arduino 共地。

## 11. 明天的上电顺序

按这个顺序，最安全：

1. 先别接 12V 电机电源。
2. Arduino USB 接上，上传桥接程序。
3. Pixhawk 上 USB 或电源，QGC 连上。
4. 接好 `Pixhawk MAIN1/MAIN2/GND -> Arduino D2/D6/GND`。
5. 小车架空，轮子离地。
6. 确认遥控器油门在中位。
7. 最后再给 D153B 接 12V，并打开开关。
8. 在 QGC 里先做低风险输出测试。

## 12. 明天别做的事

- 不要让车直接落地首测。
- 不要让 Lubancat 直接出 GPIO 控 D153B。
- 不要把 Arduino 同时接 USB 和外部 5V。
- 不要把 Pixhawk 的 `+5V` 直接乱并到 Arduino 或 D153B 逻辑电源。
- 不要先接满所有外设再查错。

## 13. 对应 Arduino 程序

明天用这个程序：

`arduino/d153b_pixhawk_pwm_bridge/d153b_pixhawk_pwm_bridge.ino`

它的逻辑是：

- Pixhawk `1000-2000us` PWM 输入
- 中位 `1500us` 停车
- 丢失 PWM 就双轮停车

## 14. 明天的实际目标

只做三件事：

1. QGC 里确认遥控输入正常。
2. 架空时确认左轮和右轮都能被 Pixhawk 正确驱动。
3. 确认松杆回中后，两个轮子都停。

这三件做完，才算明天成功。
