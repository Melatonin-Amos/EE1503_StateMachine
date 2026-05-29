# 矩形泡沫赛道状态机设计笔记

本文档用于交接 `main_state_machine.ino` 的设计思路、关键参数和后续调参入口。代码目标是在蓝色泡沫矩形赛道内沿两侧中线行驶：顺时针完成一圈后停车，人工调转方向并重启后，程序不变地逆时针完成一圈。

## 硬件和坐标约定

小车为标准麦克纳姆轮排布：俯视 O 型、触地 X 型。电机指令约定如下：

```cpp
mecanum.driveAllMotor(100, 100, 100, 100);      // 前进
mecanum.driveAllMotor(-100, -100, -100, -100);  // 后退
mecanum.driveAllMotor(100, -100, -100, 100);    // 右平移
mecanum.driveAllMotor(-100, 100, 100, -100);    // 左平移
mecanum.driveAllMotor(100, -100, 100, -100);    // 原地右转
mecanum.driveAllMotor(-100, 100, -100, 100);    // 原地左转
```

雷达角度在代码中固定为：

```cpp
static const int FRONT_DEG = 180;
static const int REAR_DEG = 0;
static const int LEFT_DEG = 90;
static const int RIGHT_DEG = 270;
```

几何假设：

```cpp
static const float TRACK_WIDTH_M = 0.45f;
static const float CAR_WIDTH_M = 0.23f;
static const float LIDAR_TO_FRONT_BUMPER_M = 0.08f;
```

雷达 180 度方向对应车头，雷达到车头约 8 cm；0 度方向对应车尾，雷达到车尾约 20 cm。轨道宽约 45 cm，车宽约 23 cm，因此理论单侧车身余量约 11 cm，代码用雷达到两侧墙的中线目标 `SIDE_TARGET_M = 0.225m` 来控制居中。

## 状态机

核心状态定义在 `TrackState`：

```cpp
STATE_ACQUIRE_STRAIGHT
STATE_STRAIGHT
STATE_CORNER_DECIDE
STATE_TURN_RIGHT
STATE_TURN_LEFT
STATE_ALIGN_AFTER_TURN
STATE_FINISH_STRAIGHT
STATE_STOPPED
```

状态流：

1. `ACQUIRE_STRAIGHT`：上电后先根据侧墙和偏航找直道中线。
2. `STRAIGHT`：沿两侧中线前进，累计本条直道的进度。
3. `CORNER_DECIDE`：前方接近墙时，判断开口在左侧还是右侧；第一次转弯会锁定方向。
4. `TURN_RIGHT` / `TURN_LEFT`：执行四分之一转弯。
5. `ALIGN_AFTER_TURN`：出弯后微调横向居中和车头方向。
6. 四次转弯后进入 `FINISH_STRAIGHT`：按已记录的对边长度估算补足一圈，或遇到终点前墙时停车。
7. `STOPPED`：停止电机，等待人工拿起、调头、关机重启。

方向无关性来自第一次弯道检测：第一次识别右开口就锁定右转，第一次识别左开口就锁定左转。因此同一份程序可用于顺时针和逆时针。

## 直行速度控制

当前只对直行巡航做了提速，电机输出限幅仍保持 `-100..100`，没有改到底层 125。

```cpp
// Motion tuning, logical PWM range is -100..100
static const int STRAIGHT_FAST_PWM = 78;
static const int STRAIGHT_SLOW_PWM = 28;
```

旧值为：

```cpp
static const int STRAIGHT_FAST_PWM = 52;
```

所以之前体感不到全速是正常的：`STATE_STRAIGHT` 和 `STATE_FINISH_STRAIGHT` 的基础前进量并不是 100，而是 `STRAIGHT_FAST_PWM`。此外还有两个主动降速条件：

```cpp
if (statCloserThan(features.frontWide, FRONT_SLOW_M)) {
  float ratio = (features.frontWide.near - FRONT_TURN_TRIGGER_M) / (FRONT_SLOW_M - FRONT_TURN_TRIGGER_M);
  ratio = clampFloat(ratio, 0.0f, 1.0f);
  forward = STRAIGHT_SLOW_PWM + (int)((STRAIGHT_FAST_PWM - STRAIGHT_SLOW_PWM) * ratio);
}

if (fabs(yawError(features)) > 0.14f) {
  forward = min(forward, STRAIGHT_SLOW_PWM);
}
```

含义：

- 前方宽扇区距离小于 `FRONT_SLOW_M = 0.70m` 时，直行速度会从 78 线性降到 28。
- 前方宽扇区距离小于 `FRONT_TURN_TRIGGER_M = 0.42m` 并确认后，进入转弯判断。
- 偏航误差过大时，直行速度被限制到 28，优先把车头修正回来。

调参建议：

- 如果直道仍慢，优先把 `STRAIGHT_FAST_PWM` 从 78 试到 85 或 90。
- 如果入弯太冲，不要先降 `STRAIGHT_FAST_PWM`，优先增大 `FRONT_SLOW_M`，例如 0.70 -> 0.78。
- 如果直行容易左右摆，先降低 `STRAIGHT_FAST_PWM` 或 `YAW_LIMIT_STRAIGHT_PWM`，不要直接改转弯状态。

## 混控和限幅

所有运动最终进入 `driveVector(forward, strafeRight, rotateRight)`：

```cpp
int lf = forward + strafeRight + rotateRight;
int rf = forward - strafeRight - rotateRight;
int lr = forward - strafeRight + rotateRight;
int rr = forward + strafeRight - rotateRight;
```

若任一轮超过 100，会按比例缩放到 100：

```cpp
if (maxAbs > 100) {
  lf = (lf * 100) / maxAbs;
  rf = (rf * 100) / maxAbs;
  lr = (lr * 100) / maxAbs;
  rr = (rr * 100) / maxAbs;
}
```

底层 `LocalMecanumDriver.cpp` 也会把单轮占空限制到 `0..100`：

```cpp
int duty = constrain(abs(speed), 0, 100);
```

因此当前版本即使某处误给了大于 100 的值，也不会真正输出超过 100。

## 转弯策略

本轮没有改转弯参数。转弯仍由以下量控制：

```cpp
static const int TURN_ROTATE_FAST_PWM = 46;
static const int TURN_ROTATE_SLOW_PWM = 38;
static const int TURN_ROTATE_FINE_PWM = 38;
static const int TURN_EXIT_FORWARD_PWM = 8;
```

转弯退出条件在 `turnExitReady()`：等待最小转弯时间、前方重新变空、侧墙可用，再检查偏航误差。这个策略偏稳，转弯体感慢主要来自旋转 PWM 和最小时间，而不是直行 `forward`。

如果后续要专门提速转弯，建议单独开一轮改动，优先小步提高 `TURN_ROTATE_FAST_PWM`，并保留 `TURN_ROTATE_FINE_PWM`，避免出弯过冲。

## 串口观察字段

调试输出格式在 `printDebug()`。试车时重点看：

```text
STATE      当前状态
turn       锁定的转向 RIGHT/LEFT
n          已完成转弯次数
F/FW       正前方/前方宽扇区距离
L/R        左右侧墙距离
LF/RF      左前/右前斜向距离
CE         横向居中误差
YE         偏航误差
cmd        forward, strafe, rotate
leg        当前直道累计进度
finish     终点补足进度
```

这次提速后，直道空旷且姿态正常时，`cmd` 的第一项应接近 `78`。如果经常只有 `28`，通常说明前方已经进入减速区，或 `YE` 超过了 `0.14`。
