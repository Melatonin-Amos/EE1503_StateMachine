# 8字形泡沫赛道状态机设计笔记

本文档记录 `main_state_machine.ino`（8字形版本）的设计思路、状态流、关键参数和调参建议。

## 赛道与路径

赛道在原矩形赛道基础上，增加了一条贯穿中间的横道，整体形成"8"字形。

**路径规则（单次运行）：**

```
半圈 A（右转×4）：横道起点 → T字口右转 → 角落右转 → 角落右转 → T字口右转 → 回到横道
半圈 B（左转×4）：横道继续前行 → T字口左转 → 角落左转 → 角落左转 → T字口左转 → 回到起点
完成两个半圈，也就是 8 次四分之一转弯后停车。
```

停车后人工拿起小车、调转方向、关机重启，再用同一份程序跑反方向的一圈。程序内部始终按“先右转半圈、再左转半圈”的相对路线运行。

**T字口触发：** T 字口不再只依赖“前方近墙”。直行段使用 `cornerTriggerReady()`，包含两种触发：

- `frontIsCloseForCorner()`：前方宽扇区看到近墙。
- `plannedTurnSideOpen()`：预定转向一侧的前斜/宽侧扇区明显变空，同时车尾侧或对侧仍能看到赛道，表示车头已经进入 T 口。

这样即使 T 口前方泡沫墙被几何关系遮住、雷达点稀疏，也能根据计划转向侧的开口进入转弯节点。

## 硬件和坐标约定

与矩形版本相同：

```cpp
static const int FRONT_DEG = 180;   // 车头
static const int REAR_DEG  = 0;     // 车尾
static const int LEFT_DEG  = 90;
static const int RIGHT_DEG = 270;
```

```cpp
static const float TRACK_WIDTH_M        = 0.45f;
static const float CAR_WIDTH_M          = 0.23f;
static const float LIDAR_TO_FRONT_BUMPER_M = 0.08f;
```

麦克纳姆轮混控：`driveVector(forward, strafeRight, rotateRight)`，任一轮超过 100 时按比例缩放。

## 状态机

```cpp
STATE_ACQUIRE_STRAIGHT   // 上电初始对齐
STATE_STRAIGHT           // 直行
STATE_CORNER_DECIDE      // 首次弯道方向识别
STATE_TURN_RIGHT         // 右转四分之一圈
STATE_TURN_LEFT          // 左转四分之一圈
STATE_ALIGN_AFTER_TURN   // 出弯后微调（兜底）
STATE_STOPPED            // 紧急停车
```

与矩形版本相比，移除了 `STATE_FINISH_STRAIGHT`（不再用 PWM 积分估算终点距离），加入半圈计数、方向翻转和满一圈停车逻辑。

### 状态流

```
上电
  └→ ACQUIRE_STRAIGHT（初始对齐）
       └→ STRAIGHT（直行）
            ├→ CORNER_DECIDE（仅第一次转弯，识别方向）
            └→ TURN_RIGHT / TURN_LEFT（每次到达弯口）
                 ├→ STRAIGHT（出弯偏航 < ALIGN_YAW_OK_M，跳过对齐）
                 └→ ALIGN_AFTER_TURN（出弯偏航超标时兜底）
                      └→ STRAIGHT

每完成 4 次同向转弯（一个半圈）：
  如果 halfLaps < 2：lockedTurnDirection 翻转（RIGHT↔LEFT），继续 STRAIGHT
  如果 halfLaps == 2：进入 STOPPED
```

### 半圈计数与方向翻转

核心计数变量：

```cpp
static int completedTurns = 0;   // 当前半圈内已完成的转弯数，满 4 后清零
static int halfLaps        = 0;  // 已完成的半圈总数
```

每次转弯完成时（`readyConfirm >= TURN_READY_CONFIRM_SCANS`）：

```cpp
completedTurns++;

if (completedTurns >= TURNS_PER_HALF_LAP) {
    onHalfLapComplete(features);   // halfLaps++，满 2 半圈停车，否则翻转方向
} else {
    enterState(skipAlign ? STATE_STRAIGHT : STATE_ALIGN_AFTER_TURN);
}
```

`onHalfLapComplete()` 内：

```cpp
halfLaps++;
completedTurns = 0;
if (halfLaps >= HALF_LAPS_PER_RUN) {
    enterState(STATE_STOPPED);
    return;
}
lockedTurnDirection = (lockedTurnDirection == TURN_RIGHT) ? TURN_LEFT : TURN_RIGHT;
enterState(skipAlign ? STATE_STRAIGHT : STATE_ALIGN_AFTER_TURN);
```

### 方向识别

`lockedTurnDirection` 在 `setup()` 中由 `INITIAL_TURN_DIRECTION = TURN_RIGHT` 预设，上电即已锁定。  
每次到弯口直接进 `STATE_TURN_RIGHT` 或 `STATE_TURN_LEFT`，完全跳过 `STATE_CORNER_DECIDE`。  
第一个半圈结束时翻转方向，第二个半圈结束时停车。

**为何预设而不靠识别：** 8字赛道的起点是横道中段，首次到T字口时横道两侧都有开口，`detectedCornerDirection()` 得分差可能不足 `margin = 0.45`，导致在 `CORNER_DECIDE` 里慢行直至 750ms 超时，再等 `fallbackCornerDirection` 兜底，合计可能停顿超过 1 秒。路线已知（始终右转开始），预设最可靠。

若需要换成左转开始，只需改：

```cpp
static const TurnDirection INITIAL_TURN_DIRECTION = TURN_LEFT;
```

## 转弯策略

当前转弯参数：

```cpp
static const int TURN_ROTATE_FAST_PWM   = 64;
static const int TURN_ROTATE_SLOW_PWM   = 50;
static const int TURN_ROTATE_FINE_PWM   = 36;
static const int TURN_ROTATE_BRAKE_PWM  = 24;   // 制动阶段，抑制过冲
static const int TURN_ARC_FORWARD_PWM   = 24;   // 弧形前进分量
static const int TURN_EXIT_FORWARD_PWM  = 30;
```

转弯分四个旋转阶段（快速→慢速→精细→制动），出弯后根据偏航决定是否跳过 `ALIGN_AFTER_TURN`：

```cpp
bool skipAlign = frontIsClear(features) &&
                 sideWallsUsable(features) &&
                 yawDataUsable(features) &&
                 fabs(yawError(features)) < ALIGN_YAW_OK_M;  // 0.120m
```

如果出弯后两侧墙信号短时间较弱，但前方已经清空且赛道仍可见，`turnExitReady()` 会在 `TURN_SOFT_MAX_MS` 后放行到直行段，避免在 T 口附近原地犹豫。

## 直行速度控制

```cpp
static const int STRAIGHT_FAST_PWM = 100;
static const int STRAIGHT_SLOW_PWM = 48;
```

前方进入 `FRONT_SLOW_M = 0.86m` 区域时线性减速；偏航误差 > `STRAIGHT_YAW_SLOW_M = 0.200m` 时限速至 72。
已锁定转向后进入减速区，或 `plannedTurnSideOpen()` 识别到 T 口开口时，叠加小预转 `PRE_TURN_ROTATE_PWM = 15`，并把前进限制到 `STRAIGHT_SLOW_PWM`，减少直冲和纯原地旋转时间。

## 混控和限幅

```cpp
int lf = forward + strafeRight + rotateRight;
int rf = forward - strafeRight - rotateRight;
int lr = forward - strafeRight + rotateRight;
int rr = forward + strafeRight - rotateRight;
// 任一轮超 100 则四轮等比缩放
```

底层 `LocalMecanumDriver.cpp` 单轮限幅到 `0..100`，双重保护。

## 串口观察字段

调试格式：

```
#<scan> <STATE> ms=<elapsed> turn=<dir> half=<halfLaps> n=<completedTurns> ready=<confirm>
T=<front-trigger> TO=<turn-side-open> F=<front> FW=<frontWide> L=<left> R=<right> LF=<leftFront> RF=<rightFront>
CE=<sideCenterError> YE=<yawError> cmd=<fwd>,<strafe>,<rot>
```

重点字段：
- `turn` — 当前半圈的转弯方向，每半圈翻转一次
- `half` — 已完成半圈数，正常应每 4 次转弯加 1
- `n` — 当前半圈内已完成转弯数，0~3，满 4 后清零
- `T` — 前方近墙触发，`1` 表示 `frontIsCloseForCorner()` 成立
- `TO` — 计划转向侧开口触发，`1` 表示 `plannedTurnSideOpen()` 成立
- `cmd` — 转弯时应看到 `24,*,64` → `24,*,50` → `24/30,*,36` → `24/30,*,24`

半圈完成时会额外输出：

```
HALF_LAP_DONE total_halves=<n> next_turn=<LEFT/RIGHT>
FULL_LAP_DONE total_halves=2 stopping
```

## 调参建议

与矩形版本相同的调参逻辑，额外注意：

- **T字口仍直行穿过**：看串口 `T/TO`。若 `T=0, TO=0`，可降低 `plannedTurnSideOpen()` 的 `plannedScore > 0.95f` 或适当降低 `DIAGONAL_OPEN_M`；若 `TO=1` 但没有进转弯，检查是否还没过 `MIN_STRAIGHT_BEFORE_CORNER_MS` 或确认次数不足。
- **T字口误触发过早**：提高 `plannedScore > 0.95f`，或把 `(stillInCorridor || plannedScore > 1.35f)` 中的 1.35 提高。
- **半圈方向翻转后立即跑偏**：翻转后方向变了但姿态可能略有偏，`ALIGN_YAW_OK_M`（0.120 → 0.090）收紧可减少直行开头摆动。
- **横道两侧都开，侧墙信号弱**：横道行进中若双侧墙距都超过 `SIDE_VALID_MAX_M = 0.64m`，`sideWallsUsable` 返回 false，偏航修正会暂时失效。若横道较宽，可适当放宽 `SIDE_VALID_MAX_M`。
