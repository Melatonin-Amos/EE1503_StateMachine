# 8字形泡沫赛道状态机设计笔记

本文档记录 `main_state_machine.ino`（8字形版本）的设计思路、状态流、关键参数和调参建议。

## 赛道与路径

赛道在原矩形赛道基础上，增加了一条贯穿中间的横道，整体形成"8"字形。

**路径规则（无限循环）：**

```
半圈 A（右转×4）：横道起点 → T字口右转 → 角落右转 → 角落右转 → T字口右转 → 回到横道
半圈 B（左转×4）：横道继续前行 → T字口左转 → 角落左转 → 角落左转 → T字口左转 → 回到起点
然后切换方向，进入下一个半圈 A，如此往复。
```

**T字口与普通角落的统一处理：** 两类路口在 LiDAR 看来特征相同——前方有墙、一侧开口。因此矩形版本的转弯检测逻辑（`frontIsCloseForCorner` + `detectedCornerDirection`）对 T 字口同样有效，无需专门区分。

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

与矩形版本相比，移除了 `STATE_FINISH_STRAIGHT`（不再需要估算终点距离和停车），加入半圈计数和方向翻转逻辑。

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
  lockedTurnDirection 翻转（RIGHT↔LEFT）
  completedTurns 清零
  继续 STRAIGHT
```

### 半圈计数与方向翻转

核心计数变量：

```cpp
static int completedTurns = 0;   // 当前半圈内已完成的转弯数，满 4 后清零
static int halfLaps        = 0;  // 已完成的半圈总数
```

每次转弯完成时（`readyConfirm >= READY_CONFIRM_SCANS`）：

```cpp
completedTurns++;

if (completedTurns >= TURNS_PER_HALF_LAP) {
    onHalfLapComplete(features);   // 翻转方向，halfLaps++，completedTurns=0
} else {
    enterState(skipAlign ? STATE_STRAIGHT : STATE_ALIGN_AFTER_TURN);
}
```

`onHalfLapComplete()` 内：

```cpp
halfLaps++;
completedTurns = 0;
lockedTurnDirection = (lockedTurnDirection == TURN_RIGHT) ? TURN_LEFT : TURN_RIGHT;
enterState(skipAlign ? STATE_STRAIGHT : STATE_ALIGN_AFTER_TURN);
```

### 方向识别

`lockedTurnDirection` 在 `setup()` 中由 `INITIAL_TURN_DIRECTION = TURN_RIGHT` 预设，上电即已锁定。  
每次到弯口直接进 `STATE_TURN_RIGHT` 或 `STATE_TURN_LEFT`，完全跳过 `STATE_CORNER_DECIDE`。  
每个半圈结束时翻转方向，下一个半圈仍直接按锁定方向转弯。

**为何预设而不靠识别：** 8字赛道的起点是横道中段，首次到T字口时横道两侧都有开口，`detectedCornerDirection()` 得分差可能不足 `margin = 0.45`，导致在 `CORNER_DECIDE` 里慢行直至 750ms 超时，再等 `fallbackCornerDirection` 兜底，合计可能停顿超过 1 秒。路线已知（始终右转开始），预设最可靠。

若需要换成左转开始，只需改：

```cpp
static const TurnDirection INITIAL_TURN_DIRECTION = TURN_LEFT;
```

## 转弯策略

与矩形版本完全相同，所有参数不变：

```cpp
static const int TURN_ROTATE_FAST_PWM   = 54;
static const int TURN_ROTATE_SLOW_PWM   = 42;
static const int TURN_ROTATE_FINE_PWM   = 30;
static const int TURN_ROTATE_BRAKE_PWM  = 22;   // 制动阶段，抑制过冲
static const int TURN_ARC_FORWARD_PWM   = 16;   // 弧形前进分量
static const int TURN_EXIT_FORWARD_PWM  = 18;
```

转弯分四个旋转阶段（快速→慢速→精细→制动），出弯后根据偏航决定是否跳过 `ALIGN_AFTER_TURN`：

```cpp
bool skipAlign = frontIsClear(features) &&
                 sideWallsUsable(features) &&
                 yawDataUsable(features) &&
                 fabs(yawError(features)) < ALIGN_YAW_OK_M;  // 0.100m
```

## 直行速度控制

```cpp
static const int STRAIGHT_FAST_PWM = 100;
static const int STRAIGHT_SLOW_PWM = 36;
```

前方进入 `FRONT_SLOW_M = 0.84m` 区域时线性减速；偏航误差 > 0.14m 时限速至 36。  
已锁定转向后进入减速区，叠加小预转 `PRE_TURN_ROTATE_PWM = 12` 以减少到弯口后的纯原地旋转时间。

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
F=<front> FW=<frontWide> L=<left> R=<right> LF=<leftFront> RF=<rightFront>
CE=<sideCenterError> YE=<yawError> cmd=<fwd>,<strafe>,<rot>
```

重点字段：
- `turn` — 当前半圈的转弯方向，每半圈翻转一次
- `half` — 已完成半圈数，正常应每 4 次转弯加 1
- `n` — 当前半圈内已完成转弯数，0~3，满 4 后清零
- `cmd` — 转弯时应看到 `16,*,54` → `16,*,42` → `16,*,30` → `16,*,22`

半圈完成时会额外输出：

```
HALF_LAP_DONE total_halves=<n> next_turn=<LEFT/RIGHT>
```

## 调参建议

与矩形版本相同的调参逻辑，额外注意：

- **T字口识别失败**（在 T 字口直行穿过）：通常是 `frontIsCloseForCorner` 未触发，前方开口被识别为无墙。此时可适当增大 `FRONT_TURN_TRIGGER_M`（0.44 → 0.48），使触发距离更远。
- **半圈方向翻转后立即跑偏**：翻转后方向变了但姿态可能略有偏，`ALIGN_YAW_OK_M`（0.100 → 0.080）收紧可减少直行开头摆动。
- **横道两侧都开，侧墙信号弱**：横道行进中若双侧墙距都超过 `SIDE_VALID_MAX_M = 0.64m`，`sideWallsUsable` 返回 false，偏航修正会暂时失效。若横道较宽，可适当放宽 `SIDE_VALID_MAX_M`。
