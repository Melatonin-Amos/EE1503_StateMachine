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

## 本轮提速摘要

当前版本在不改底层 `-100..100` 输出限幅的前提下做全面提速：

- 直道巡航 `STRAIGHT_FAST_PWM` 提到 100，慢速档从 28 提到 36。
- 入弯减速区从 `0.70m` 提前到 `0.84m`，给满速直道留制动距离。
- 第一次转弯仍识别并锁定方向；第二到第四个弯已知方向后直接进入转弯状态，不再额外停留判向。
- 第一次弯道如果强判向没有及时确认，750ms 后使用弱判向兜底，避免在弯口长时间慢走。
- 已锁定方向后，接近弯口会加入轻微预转 `PRE_TURN_ROTATE_PWM = 12`。
- 转弯状态带弧形前进：安全距离内给 `TURN_ARC_FORWARD_PWM = 16`，出弯清空后给 `TURN_EXIT_FORWARD_PWM = 18`。
- 为抑制转弯过头，加入制动阶段 `TURN_ROTATE_BRAKE_PWM = 22`，精细旋转速度从 38 降到 30；偏航进入 0.300m 窗口时切换到制动速度。
- 出弯偏航已在 `ALIGN_YAW_OK_M = 0.100m` 以内时直接进直行，跳过 `STATE_ALIGN_AFTER_TURN`，消除反向回正停顿；`ALIGN_MIN_MS` 缩短为 80ms 作为兜底最短等待。

## 直行速度控制

当前直行和转弯都已提速，但电机输出限幅仍保持 `-100..100`，没有改到底层 125。

```cpp
// Motion tuning, logical PWM range is -100..100
static const int STRAIGHT_FAST_PWM = 100;
static const int STRAIGHT_SLOW_PWM = 36;
```

历史值为：

```cpp
static const int STRAIGHT_FAST_PWM = 52;
// 中间版本曾小步提速为 78，再到 90
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

- 前方宽扇区距离小于 `FRONT_SLOW_M = 0.84m` 时，直行速度会从 100 线性降到 36。
- 前方宽扇区距离小于 `FRONT_TURN_TRIGGER_M = 0.44m` 并确认后，进入转弯。
- 偏航误差过大时，直行速度被限制到 36，优先把车头修正回来。
- 已锁定转弯方向后，进入减速区会叠加轻微预转，减少到弯口后的原地转动时间。

调参建议：

- 现在直道已经试到 100；如果入弯仍稳，可以保持这个值。
- 如果入弯太冲，不要先降 `STRAIGHT_FAST_PWM`，优先增大 `FRONT_SLOW_M`，例如 0.84 -> 0.90。
- 如果预转导致擦内侧墙，先降低 `PRE_TURN_ROTATE_PWM`，例如 12 -> 8。
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

注意：直道 `forward=100` 时，如果同时叠加横移和偏航修正，混控会按比例缩放四个轮子的实际输出。因此串口里的 `cmd=100,...,...` 表示期望向量，实际单轮会受 100 限幅保护。转弯参数有意留了余量，例如 `forward=8 + rotate=54 + strafe<=18` 时最大单轮约 80，通常不会触发缩放。

## 转弯策略

当前转弯参数：

```cpp
static const int PRE_TURN_ROTATE_PWM    = 12;
static const int CORNER_APPROACH_FORWARD_PWM = 12;
static const int TURN_ROTATE_FAST_PWM   = 54;
static const int TURN_ROTATE_SLOW_PWM   = 42;
static const int TURN_ROTATE_FINE_PWM   = 30;   // 历史值 38
static const int TURN_ROTATE_BRAKE_PWM  = 22;   // 新增
static const int TURN_ARC_FORWARD_PWM   = 16;   // 历史值 8
static const int TURN_EXIT_FORWARD_PWM  = 18;   // 历史值 10
```

转弯退出条件在 `turnExitReady()`：等待最小转弯时间、前方重新变空、侧墙可用，再检查偏航误差。当前最小传感器退出时间为 `TURN_SENSOR_MIN_MS = 760ms`，比旧值 1050ms 更短。

转弯流程现在分三段：

1. 入弯前：已锁定方向时，`STATE_STRAIGHT` 在减速区叠加小预转。
2. 弯中：`STATE_TURN_RIGHT/LEFT` 分快速→慢速→精细→制动四档旋转速度，并在前方不危险时带前进分量走弧形，减少纯原地旋转时间。
3. 出弯：偏航已在 `ALIGN_YAW_OK_M = 0.100m` 以内则直接进直行，否则进 `ALIGN_AFTER_TURN` 做短对齐兜底。

第一次弯道仍需要识别方向。强判向连续确认优先；若 `CORNER_DECIDE_MAX_MS = 750ms` 后仍未强确认，会使用 `fallbackCornerDirection()` 的弱分数兜底。方向一旦锁定，后续弯道不再进入完整判向流程。

如果后续还要提速转弯，优先观察是否过冲：

- 过冲或出弯蛇形：降低 `TURN_ROTATE_BRAKE_PWM`（例如 22 → 18），或扩大制动窗口（`* 2.0f` → `* 2.5f`）。
- 弯中贴近内/外墙：调整 `TURN_ARC_FORWARD_PWM`（例如 16 → 12）。
- 出弯后直行开头跑偏：收紧 `ALIGN_YAW_OK_M`（例如 0.100 → 0.080）。

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

这次提速后，直道空旷且姿态正常时，`cmd` 的第一项应接近 `100`。如果经常只有 `36`，通常说明前方已经进入减速区，或 `YE` 超过了 `0.14`。转弯时应能看到 `cmd` 同时带有 forward 和 rotate，例如 `16,*,54` 附近，而不是长期 `0,*,46`。

## 转弯优化

本节记录针对「转弯略过头」「转弯速度偏低」「出弯停顿过长」三个问题的综合修改。

### 1. 抗过冲：转弯制动阶段

**问题**：转弯以 fine 速（原 38 PWM）旋转时，退出需要 2 次连续确认（各约 100ms），期间惯性导致持续过冲。

**修改**：新增 `TURN_ROTATE_BRAKE_PWM = 22`，并把 `TURN_ROTATE_FINE_PWM` 从 38 降到 30。在 `updateQuarterTurn()` 的旋转量决策中加入制动条件：

```cpp
static const int TURN_ROTATE_FINE_PWM  = 30;   // 原值 38
static const int TURN_ROTATE_BRAKE_PWM = 22;   // 新增
```

```cpp
if (elapsed >= TURN_OPEN_LOOP_MIN_MS &&
    yawDataUsable(features) &&
    fabs(yawError(features)) < TURN_EXIT_YAW_OK_M * 2.0f) {
  rotateMagnitude = TURN_ROTATE_BRAKE_PWM;
}
```

当偏航误差进入退出阈值的 2 倍窗口（`0.300m`）时，旋转从 fine（30）降到 brake（22），最后一段接近目标角度时转速已很低，过冲大幅减少。`TURN_MAX_MS` 兜底仍覆盖在最后，防止卡在制动区。

### 2. 提速：加大弧形前进分量

**问题**：`TURN_ARC_FORWARD_PWM = 8` 和 `TURN_EXIT_FORWARD_PWM = 10` 太小，弯中几乎纯原地旋转。

**修改**：

```cpp
static const int TURN_ARC_FORWARD_PWM  = 16;   // 原值 8
static const int TURN_EXIT_FORWARD_PWM = 18;   // 原值 10
```

弧形运动下（forward=16, rotate=54），最大单轮约 70，叠加 strafe ≤ 18 后约 88，不触发 `driveVector` 的比例缩放。

### 3. 出弯直接进直行（跳过 ALIGN）

**问题**：`STATE_ALIGN_AFTER_TURN` 用 `YAW_LIMIT_ALIGN_PWM = 26` 做比例旋转修正，微量过冲时会反向回转，产生可见停顿。制动阶段生效后出弯偏航已很小，这个修正不再必要。

**修改一**：`ALIGN_MIN_MS` 从 220 降到 80，并新增专用阈值 `ALIGN_YAW_OK_M = 0.100f`（介于直行用的 `YAW_OK_M = 0.075m` 和出弯退出用的 `TURN_EXIT_YAW_OK_M = 0.150m` 之间），用于 `updateAlignAfterTurn()` 的退出判断：

```cpp
static const unsigned long ALIGN_MIN_MS   = 80;      // 原值 220
static const float         ALIGN_YAW_OK_M = 0.100f;  // 新增
```

```cpp
bool ready = sideWallsUsable(features) &&
             frontIsClear(features) &&
             yawDataUsable(features) &&
             fabs(sideCenterError(features)) < CENTER_OK_M &&
             fabs(yawError(features)) < ALIGN_YAW_OK_M;
```

`YAW_OK_M = 0.075m` 保持不变，仍用于上电初始对齐（`STATE_ACQUIRE_STRAIGHT`）。

**修改二**：在 `updateQuarterTurn()` 的退出块中，判断是否跳过 ALIGN：

```cpp
bool skipAlign = frontIsClear(features) &&
                 sideWallsUsable(features) &&
                 yawDataUsable(features) &&
                 fabs(yawError(features)) < ALIGN_YAW_OK_M;
if (skipAlign) {
  enterState(finishPendingAfterAlign ? STATE_FINISH_STRAIGHT : STATE_STRAIGHT);
} else {
  enterState(STATE_ALIGN_AFTER_TURN);
}
```

出弯偏航已在 0.100m 以内时，直接进 `STATE_STRAIGHT`，完全消除反向回正和额外等待。`STATE_STRAIGHT` 自带偏航修正（`YAW_LIMIT_STRAIGHT_PWM = 17`），残余偏航在直行段平滑收敛。出弯偏航 > 0.100m 时（制动未完全生效等异常情况），仍正常进入 `STATE_ALIGN_AFTER_TURN` 兜底。

### 转弯阶段时序与 cmd 预期

| 阶段 | 时间范围 | 旋转速度 | 前进分量 | 触发条件 |
|------|---------|---------|---------|---------|
| 快速旋转 | 0–460ms | 54 | 16（前方 > 0.33m 时） | 入弯即生效 |
| 慢速旋转 | 460ms–640ms | 42 | 16 | `TURN_FAST_MS` 超时 |
| 精细旋转 | 640ms+ | 30 | 16 或 18 | 前方清空且过 `TURN_OPEN_LOOP_MIN_MS` |
| 制动旋转 | 640ms+ | 22 | 16 或 18 | 偏航进入 0.300m 窗口 |
| 超时兜底 | 1850ms+ | 42 | 依前方状态 | `TURN_MAX_MS` |

串口中典型序列：`16,*,54` → `16,*,42` → `16,*,30` → `16,*,22` → 跳出。

### 调参建议

- **仍有过冲**：降低 `TURN_ROTATE_BRAKE_PWM`（22 → 18），或扩大制动窗口（`* 2.0f` → `* 2.5f`）。
- **制动触发太早、弯中变慢**：把制动触发条件改为 `elapsed >= TURN_SENSOR_MIN_MS`（760ms 起）。
- **弯中贴近内侧墙**：降低 `TURN_ARC_FORWARD_PWM`（16 → 12）。
- **弯中贴近外侧墙**：`insideClearanceStrafe()` 已有侧向推开；若仍不够，降低 `TURN_ARC_FORWARD_PWM`。
- **跳过 ALIGN 后直行开头跑偏**：收紧 `ALIGN_YAW_OK_M`（0.100 → 0.080）。
- **仍偶发进入 ALIGN 停顿**：放宽 `ALIGN_YAW_OK_M`（0.100 → 0.120）。
