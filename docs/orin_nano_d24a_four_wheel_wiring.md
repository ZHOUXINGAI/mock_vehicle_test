# Orin Nano + Arduino + D24A 四驱小车接线文档

本文档用于第二台四驱小车：

```text
Jetson Orin Nano -> Arduino UNO -> D24A 四路 TB6612 电机驱动板 -> 4 个 MG513X 有刷直流电机
```

目标是先跑通开环四轮控制，再接 Pixhawk / PX4 / offboard。

## 1. 安全规则

- 改线之前必须断开 12V 电池。
- 第一次测试必须把车架起来，轮子离地。
- 12V 电机电源最后再接。
- Arduino USB 可以先接，用来上传程序和收串口命令。
- D24A 和 Arduino 必须共地。
- 不要同时用 D24A 的 5V 输出和 USB 给 Arduino 供电。
- 发现电机驱动板、电机、电源线明显发热，马上断电。
- TB6612 手册给出的量级是单路约 `1.2A` 持续、`3.2A` 峰值，电机电源不要超过 `15V`。

## 2. 推荐架构

第一版不要让 Orin Nano 直接用 GPIO/PWM 控四个电机，先用 Arduino 做实时翻译层：

```text
Orin Nano USB -> Arduino UNO -> D24A four-channel TB6612 -> four DC motors
```

原因：

- Orin Nano 跑 Linux，GPIO 软件 PWM 控四路带载电机不够稳。
- Arduino 负责产生稳定 PWM 和方向控制。
- Orin Nano 只通过 USB 串口发命令，后续接 PX4 / ROS2 更清楚。
- 之前两轮车已经验证过 `电脑/鲁班猫 -> Arduino -> 电机驱动板` 这条路线可用。

## 3. 电源接线

```text
12V 电池正极 -> D24A 主电源输入 12V/VIN/+
12V 电池负极 -> D24A 主电源输入 GND/-

Orin Nano      -> 自己单独稳定供电
Arduino UNO    -> Orin Nano USB 供电
D24A GND       -> Arduino GND
```

注意：

- 不要用 D24A 的 5V 输出给 Orin Nano 供电。
- Arduino 已经插 USB 时，不要再从 D24A 5V 接 Arduino 5V。
- D24A 有两个蓝色大电源端子时，优先使用丝印/手册标明的 `12V 输入 / VIN 输入` 那个。
- 如果另一个端子写的是“输入电源并联输出”，它本质是把输入电源并出来给别的设备用，不是必须接。

## 4. D24A 控制线接 Arduino UNO

Arduino `D0/D1` 留给 USB 串口，不接电机驱动板。

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

说明：

- `PWMx` 控制速度。
- `xIN1/xIN2` 控制方向。
- `STBY` 是驱动板总使能，拉高才允许电机转。
- Arduino 的 `A0/A1/A2` 在这里当普通数字 IO 用。

## 5. 四个电机输出接线

先按这个临时顺序接，后面用脚本标定真实方向：

```text
D24A Motor_A / AOUT1-AOUT2 -> 左前轮
D24A Motor_B / BOUT1-BOUT2 -> 右前轮
D24A Motor_C / COUT1-COUT2 -> 左后轮
D24A Motor_D / DOUT1-DOUT2 -> 右后轮
```

如果某个轮子方向反了，先不用急着换线，标定后可以在代码里改映射。

## 6. MG513X 六线电机说明

MG513X 是带编码器的有刷直流减速电机。第一阶段开环测试只用两根电机动力线。

```text
电机线+ / 电机线- -> D24A 对应通道的 OUT1/OUT2
编码器 5V        -> 暂时不接
编码器 GND       -> 暂时不接
编码器 A 相      -> 暂时不接
编码器 B 相      -> 暂时不接
```

禁止：

- 不要把编码器 5V/GND/A/B 接到 D24A 的电机输出端。
- 不要把电机动力线接到 Orin Nano 或 Arduino IO。
- 不要把 12V 接到 Arduino 5V 或 Orin Nano 40pin。

## 7. Arduino 程序

Upload this Arduino sketch first:

```text
arduino/d24a_serial_bridge/d24a_serial_bridge.ino
```

这个程序接受 Orin Nano 发来的串口命令：

```text
AF / AB -> A 通道正/反
BF / BB -> B 通道正/反
CF / CB -> C 通道正/反
DF / DB -> D 通道正/反
S       -> 停止
```

## 8. Orin Nano 端逐路标定

确认：

- 车已经架空。
- Arduino 已插 Orin Nano USB。
- D24A GND 已接 Arduino GND。
- 12V 电池接到 D24A 主电源输入。
- D24A 开关打开。

然后逐条运行：

```bash
python3 scripts/d24a_raw_motor_test.py a_forward  --pwm 80 --duration 1 --armed
python3 scripts/d24a_raw_motor_test.py a_backward --pwm 80 --duration 1 --armed
python3 scripts/d24a_raw_motor_test.py b_forward  --pwm 80 --duration 1 --armed
python3 scripts/d24a_raw_motor_test.py b_backward --pwm 80 --duration 1 --armed
python3 scripts/d24a_raw_motor_test.py c_forward  --pwm 80 --duration 1 --armed
python3 scripts/d24a_raw_motor_test.py c_backward --pwm 80 --duration 1 --armed
python3 scripts/d24a_raw_motor_test.py d_forward  --pwm 80 --duration 1 --armed
python3 scripts/d24a_raw_motor_test.py d_backward --pwm 80 --duration 1 --armed
```

每条命令只应该有一个轮子转。把结果记成下面这样：

```text
A forward  = 哪个轮子，向前/向后
A backward = 哪个轮子，向前/向后
B forward  = 哪个轮子，向前/向后
B backward = 哪个轮子，向前/向后
C forward  = 哪个轮子，向前/向后
C backward = 哪个轮子，向前/向后
D forward  = 哪个轮子，向前/向后
D backward = 哪个轮子，向前/向后
```

拿到这 8 行后，再写高层动作：

```text
前进：四个轮子都向前
后退：四个轮子都向后
左转：左侧轮后退，右侧轮前进
右转：左侧轮前进，右侧轮后退
```

## 9. 上电前检查表

```text
[ ] 12V 电池暂时断开
[ ] Arduino D0/D1 没接任何控制线
[ ] D24A GND 接 Arduino GND
[ ] D24A STBY 接 Arduino A2
[ ] 四个电机只接动力线，编码器线暂时不接
[ ] Orin Nano 单独稳定供电
[ ] Arduino 只通过 Orin Nano USB 供电
[ ] 车轮离地
[ ] 上传 d24a_serial_bridge.ino
[ ] 最后再接 12V 电池
```
