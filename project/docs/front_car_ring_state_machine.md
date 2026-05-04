# Front_Car 圆环状态机拆解

本文用于后续把原工程圆环逻辑移植到当前工程 `project/code/image.cpp`。

## 原工程地址

- 圆环核心：`/home/loren/下载/Front_Car/code/Ring.c`
- 圆环状态定义：`/home/loren/下载/Front_Car/code/Ring.h`
- 原工程视觉主流程：`/home/loren/下载/Front_Car/code/PTS_Deal.c`
- 角点、直线、丢线判定：`/home/loren/下载/Front_Car/code/point_judge.c`
- 图像几何工具：`/home/loren/下载/Front_Car/code/image_processing.c`

当前工程主链：

- `/home/loren/桌面/LS2K0300_Library/LS2K300_Library/Seekfree_LS2K0300_Opensource_Library/project/code/image.cpp`

## 总体结论

原工程圆环不是“检测到圆环后设置一个标志”，而是一个多阶段边线接管状态机。

正常流程是：

```text
普通追边
-> 投影、平滑、重采样
-> find_corner() 得到角点、直线、丢线状态
-> Ring_Process() 判断是否进入圆环
-> 圆环状态机改写 left_boundary/right_boundary
-> MidPts_Deal() 统一生成中线
```

圆环状态机的核心动作是：

- 主动清空某一侧边线的 `now_step` 或 `original_step`
- 用 `seek_up()` 从指定位置向上找前探种子
- 用 `FLeftPts_Deal_Original()` / `FRightPts_Deal_Original()` 搜前探边线
- 用 `seek_corner()` 在前探边线或对侧边线中找关键角点
- 用 `add_three_point_bezier()` 或 `addline()` 人工构造边线
- 把构造结果写回 `left_boundary.original_pts` 或 `right_boundary.original_pts`
- 再调用 `LeftPts_Deal_Now()` / `RightPts_Deal_Now()` 重新投影、平滑、重采样

因此它有“自动切换/接管边线”，但不是人工按键切换。它是在状态机内部通过改写边线数组，让后面的 `MidPts_Deal()` 自然选择被保留下来的那一侧或被重建的一侧。

## 核心数据结构

### RingState

定义位置：`Ring.h`

```c
typedef struct {
    ring_state state;
    ring_kind kind;
    uint8 phase_counter;
    float sum_zangle;
    uint8 last_high;
    uint32 time_counter;
    uint8 flag;
} RingState;
```

字段含义：

- `state`：圆环当前阶段，`Ring_state1` 到 `Ring_state9`。
- `kind`：圆环方向，`no_ring`、`left_ring`、`right_ring`。
- `phase_counter`：阶段稳定计数器，多数状态要求连续若干帧满足条件再跳转。
- `sum_zangle`：从 `Ring_state3` 开始累计陀螺仪 Z 轴角速度积分，用于判断是否完成足够转向。
- `last_high`：出环补线时记录过渡高度。
- `time_counter`：用于计算 `sum_zangle` 的时间差。
- `flag`：预留/辅助标志，当前圆环主逻辑里作用不强。

### RingContext

定义位置：`Ring.h`

```c
typedef struct {
    bool is_left;
    struct {
        BoundaryData *current;
        BoundaryData *opposite;
        FBoundaryData *Fcurrent;
        FBoundaryData *Fopposite;
    } bounds;
} RingContext;
```

`InitRingContext(kind)` 会根据圆环方向绑定四类边线：

| 圆环方向 | current | opposite | Fcurrent | Fopposite |
| --- | --- | --- | --- | --- |
| `left_ring` | `left_boundary` | `right_boundary` | `Fleft_boundary` | `Fright_boundary` |
| `right_ring` | `right_boundary` | `left_boundary` | `Fright_boundary` | `Fleft_boundary` |

理解关键：

- `current` 是圆环所在侧。
- `opposite` 是另一侧，也是很多阶段中被构造、被保留、被用于生成中线的一侧。
- `Fcurrent` / `Fopposite` 是前探边线缓存，不是普通主边线。

## 入口判定

入口函数：`Ring_Process()`

未进入圆环时，`ring_ctrl.kind == no_ring`。触发条件：

```c
if (left_boundary.Lp_state && right_boundary.is_straight && cross_ctrl.kind == no_cross)
{
    ring_ctrl.kind = left_ring;
}
else if (right_boundary.Lp_state && left_boundary.is_straight && cross_ctrl.kind == no_cross)
{
    ring_ctrl.kind = right_ring;
}
```

含义：

- 左边有明显角点，右边是长直线，判断为左环。
- 右边有明显角点，左边是长直线，判断为右环。
- 十字状态机不能正在工作。

进入后立即调用 `InitRingContext()`：

- 绑定 `current/opposite/Fcurrent/Fopposite`
- 重置 `state = Ring_state1`
- 清零 `phase_counter`
- 清零 `sum_zangle`
- 记录当前时间

## 公共跳转函数

函数：`Ring_switch()`

```c
if (ring_ctrl.phase_counter >= 3)
{
    ring_ctrl.state++;
    ring_ctrl.phase_counter = 0;
    Ring_Process();
}
```

特点：

- 多数阶段不是满足一次条件就跳转，而是 `phase_counter >= 3`。
- 跳转后马上递归调用 `Ring_Process()`，让新状态在同一帧继续执行。
- 代码里有些地方用 `> PHASE_COUNT_THRESHOLD`，有些地方用 `>= 3`，语义上都是“连续稳定几帧”。

## 状态总览

| 状态 | 原工程注释含义 | 主要目的 |
| --- | --- | --- |
| `Ring_state1` | 检测环岛入口 | 进入后压掉当前侧边线，等待当前侧丢线稳定 |
| `Ring_state2` | 确认进入环岛 | 继续压掉当前侧边线，等待当前侧重新出现 |
| `Ring_state3` | 环内处理阶段 1 | 前探当前侧，构造 opposite 边线 |
| `Ring_state4` | 环内处理阶段 2 | 继续环内补线，等待转角和对侧边线足够 |
| `Ring_state5` | 出环处理 | 从 opposite 角点出发，重建出环边线 |
| `Ring_state6` | 出环收尾阶段 1 | 继续补 opposite，压掉 current |
| `Ring_state7` | 出环收尾阶段 2 | 等待 opposite 恢复后继续过渡 |
| `Ring_state8` | 准备完全退出 | 继续压掉 current，等待进入最终确认 |
| `Ring_state9` | 完全退出确认 | 检查累计角度和双边恢复，清除圆环状态 |

## Ring_state1：入口后等待当前侧丢线

代码位置：`Ring_Process()` 的 `case Ring_state1`

判定：

```c
if (ring_ctx.bounds.current->is_lost)
    ring_ctrl.phase_counter++;
else if (ring_ctrl.phase_counter > 0)
    ring_ctrl.phase_counter--;
```

强制操作：

```c
ring_ctx.bounds.current->now_step = 0;
```

跳转：

```c
if (ring_ctrl.phase_counter > PHASE_COUNT_THRESHOLD)
{
    ring_ctrl.state = Ring_state2;
    ring_ctrl.phase_counter = 0;
}
```

含义：

- 进入圆环后，不再信任圆环所在侧的当前边线。
- 即使底层追边搜到了 `current`，也把 `current->now_step` 清零。
- 等待 `current->is_lost` 连续稳定，确认当前侧确实进入环岛缺口。

对当前工程的启发：

- 左环时不要继续用左边线直接补中线。
- 右环时不要继续用右边线直接补中线。
- 这一步本质是状态机主动“切掉当前侧”。

## Ring_state2：确认进入，等待当前侧重新出现

代码位置：`Ring_Process()` 的 `case Ring_state2`

判定：

```c
if (!ring_ctx.bounds.current->is_lost)
    ring_ctrl.phase_counter++;
```

强制操作：

```c
ring_ctx.bounds.current->now_step = 0;
```

跳转：

```c
if (ring_ctrl.phase_counter >= PHASE_COUNT_THRESHOLD)
{
    ring_ctrl.state = Ring_state3;
    ring_ctrl.phase_counter = 0;
}
```

含义：

- 当前侧经历了丢线后又能搜到线，认为车辆已经进入环岛内部的可处理区域。
- 但仍然不允许它参与中线，因为 `current->now_step` 继续被清零。

对当前工程的启发：

- 不能只依赖“角点触发”就直接补线。
- 原工程要求当前侧先丢再恢复，作为进入环岛的时间顺序确认。

## Ring_state3：环内补线阶段 1

代码位置：`Ring_evolve()`

`Ring_state3` 和 `Ring_state4` 都走 `Ring_evolve()`，区别在跳转条件。

### 进入 Ring_evolve 的公共操作

每次先调用：

```c
Ring_switch();
```

如果上一帧已经积累够 `phase_counter`，会直接进入下一阶段。

### Ring_state3 的阶段推进条件

如果当前侧边线较短：

```c
else if (ring_ctx.bounds.current->now_step < Step_Max / 2)
{
    if (ring_ctrl.state == Ring_state3)
    {
        if (ring_ctx.bounds.opposite->original_step < MIN_POINTS)
            ring_ctrl.phase_counter++;
        if (ring_ctrl.phase_counter > 3)
        {
            Ring_switch();
            return;
        }
    }
    ...
}
```

含义：

- `current` 短，说明当前侧仍不可靠。
- `opposite->original_step < MIN_POINTS` 时增加计数，说明对侧也不够长，准备进入下一段环内策略。

### 构造补线起点 start

```c
start[1] = Start_high;
if (ring_ctx.bounds.current->original_step > 0)
{
    temp_x = current->original_pts[0][0] + (left_ring ? width_base : -width_base);
    start[0] = clamp(temp_x);
}
else if (ring_ctx.bounds.opposite->original_step > Step_Max / 3)
{
    start = opposite->original_pts[0];
}
else
{
    temp_x = mid_position + (left_ring ? width_base / 2 : -width_base / 2);
    start[0] = clamp(temp_x);
}
```

优先级：

1. 当前侧原图边线存在：从当前侧近端点横向偏移一个车道宽。
2. 当前侧不存在，但对侧足够长：从对侧近端点开始。
3. 两边都不可靠：从图像中线附近偏移半个车道宽。

### 构造前探种子 point

```c
if (current->original_step > 0)
{
    id = Arry_Filter_2(current->original_pts, current->original_step);
    point[0] = current->original_pts[id][0] + (left_ring ? 5 : -5);
    point[1] = current->original_pts[id][1] - 3;
}
else
{
    point[0] = left_ring ? 10 : 178;
    point[1] = Start_high;
}
```

含义：

- 如果当前侧还有点，就从当前侧的一个筛选点附近向上前探。
- 如果没有点，使用图像侧边附近的兜底点。

注意：

- 原工程图像宽度不是当前工程的 `160`，因此 `10`、`178`、`94` 这类硬编码不能直接照搬。

### 前探当前侧 Fcurrent

```c
seek_up(point, left_ring ? left_pts : Right_pts);
(left_ring ? FLeftPts_Deal_Original : FRightPts_Deal_Original)();
Fcurrent->original_step = PtsPerspective(Fcurrent->original_pts, Fcurrent->now_pts, ...);
```

含义：

- 从 `point` 向上找前探种子。
- 沿当前侧规则追一条前探边线。
- 把前探边线投影到俯视图。

### 找前探角点

```c
corner_id = seek_corner(Fcurrent->original_pts, Fcurrent->original_step, ring_ctx.is_left + 2);
turn_point = Fcurrent->original_pts[corner_id];
```

`seek_corner()` 的模式：

- 左环：`is_left == true`，模式 `3`，偏向左前方角点。
- 右环：`is_left == false`，模式 `2`，偏向右前方角点。

### 用贝塞尔生成 opposite 边线

```c
point[0] = (start[0] + turn_point[0]) / 2;
point[1] = min(turn_point[1] * 1.3, Start_high);

opposite->original_step =
    add_three_point_bezier(start, point, turn_point, opposite->original_pts, 0, 1);
```

含义：

- `start` 是近端补线起点。
- `turn_point` 是前探角点。
- 中间控制点把曲线拉成圆环过渡形状。
- 结果写入 `opposite->original_pts`，也就是直接改写对侧主边线。

### 拼接前探边线

```c
opposite->original_step = Arry_rollback(
    Fcurrent->original_pts,
    opposite->original_pts,
    opposite->original_step,
    corner_id,
    MT9V03X_W
);
```

含义：

- 把 `Fcurrent` 的一段点拼回 `opposite`。
- 这是圆环逻辑里比较强的“数据层接管”：前探当前侧的点，被用于构造对侧边线。

### 刷新 opposite 的俯视点

```c
(left_ring ? RightPts_Deal_Now : LeftPts_Deal_Now)();
```

含义：

- 左环时，刚刚改写的是右边线，所以刷新 `RightPts_Deal_Now()`。
- 右环时，刚刚改写的是左边线，所以刷新 `LeftPts_Deal_Now()`。

## Ring_state4：环内补线阶段 2

代码位置：同样在 `Ring_evolve()`

关键判定：

```c
if (opposite->now_step > Step_Max / 2 && state == Ring_state4)
{
    phase_counter++;
    if (phase_counter > 3 && fabs(sum_zangle) > 45)
    {
        Ring_switch();
        return;
    }
}
```

含义：

- 到 `Ring_state4` 后，主要看 `opposite` 是否已经足够长。
- 同时要求累计转角超过 `45` 度。
- 满足后跳到 `Ring_state5`，也就是出环处理。

否则：

- 如果 `current->now_step < Step_Max / 2`，继续执行 `Ring_state3` 里的前探、贝塞尔补线逻辑。
- 如果 `current` 已经足够长，则清空 `opposite->now_step = 0`，避免对侧错误参与中线。

对当前工程的启发：

- 环内阶段不是一次补线完成，而是持续多帧根据当前可见边线重建。
- `sum_zangle` 是重要的时间维度确认，避免仅靠图像误判提前出环。

## Ring_state5：出环处理

代码位置：`Ring_Out()`

### 找 opposite 上的出环角点

```c
corner_id = seek_corner(opposite->original_pts, opposite->now_step_original, ring_ctx.is_left);
```

模式：

- 左环：`seek_corner(..., 1)`，偏向左侧极值。
- 右环：`seek_corner(..., 0)`，偏向右侧极值。

这个 `corner_id` 是出环阶段的关键断点。

### 分支 1：角点位于可用范围内

条件：

```c
corner_id < opposite->original_step * 2 / 3 &&
corner_id < opposite->now_step_original - 3
```

含义：

- 角点不能太靠后。
- 角点后面还要有足够点用于接线。

阶段推进：

```c
if (opposite->original_pts[corner_id][1] > Mini_high - 10 && corner_id > 3)
    phase_counter++;
if (phase_counter > 3)
{
    Ring_switch();
    return;
}
```

如果角点足够靠近近端区域，连续稳定后进入 `Ring_state6`。

### 从 opposite 角点附近前探 Fopposite

```c
point[0] = opposite->original_pts[corner_id][0] + (left_ring ? -5 : 5);
point[1] = opposite->original_pts[corner_id][1] - 5;

seek_up(point, left_ring ? Right_pts : left_pts);
(left_ring ? FRightPts_Deal_Original : FLeftPts_Deal_Original)();
```

含义：

- 出环时开始围绕 `opposite` 做前探。
- 左环时前探右边线，右环时前探左边线。

### 根据几何比例找 Fopposite 拼接点

```c
h1 = abs(opposite[corner_id].y - opposite[0].y);
w1 = abs(opposite[corner_id].x - opposite[0].x);
h2 = abs(Fopposite[0].y - opposite[corner_id].y);
w2 = w1 * h2 / h1;
MX = opposite[corner_id].x + (left_ring ? -w2 : w2);
```

含义：

- 用当前 opposite 近端到角点的横纵比例，估计前探线应该接到哪里。
- `MX` 是在 Fopposite 中寻找拼接点的参考 x。

然后遍历 Fopposite：

- 左环：找 `x <= MX` 的点。
- 右环：找 `x >= MX` 的点。

### 用贝塞尔重建 opposite 出环边线

```c
turn_point = Fopposite->original_pts[Fopposite->original_step / 2];
point = Fopposite->original_pts[index];

opposite->original_step =
    add_three_point_bezier(opposite->original_pts[corner_id],
                           point,
                           turn_point,
                           opposite->original_pts,
                           corner_id,
                           1) + corner_id;
```

含义：

- 保留 `corner_id` 之前的 opposite。
- 从 `corner_id` 开始，用贝塞尔曲线把边线拉向 Fopposite。

再用：

```c
opposite->original_step = Arry_roll(...);
```

把 Fopposite 剩余段拼进 opposite。

最后：

```c
(left_ring ? RightPts_Deal_Now : LeftPts_Deal_Now)();
```

刷新被改写的 opposite。

### 分支 2：角点不在可用范围内

操作：

```c
if (opposite->original_pts[corner_id][1] > Mini_high - 10 &&
    sum_zangle > 200 &&
    corner_id > 3)
    phase_counter++;

if (opposite->is_lost && sum_zangle > 200)
    phase_counter++;

opposite->original_step = corner_id;
```

含义：

- 如果几何条件不够好，就根据累计角度和丢线状态推进。
- 同时把 opposite 截断到 `corner_id`，避免后面不可信的点参与中线。

## Ring_state6：出环收尾阶段 1

代码位置：`Ring_end()`

一开始：

```c
Ring_switch();
```

快速推进条件：

```c
if (opposite->is_lost && state == Ring_state6)
    phase_counter = 3;
```

含义：

- 如果出环收尾阶段 1 里 opposite 丢线，直接准备跳到下一阶段。

### 选择补线起点 start

如果 `opposite` 有足够点且当前是 `Ring_state6`：

```c
corner_id = seek_corner(opposite->original_pts, opposite->original_step, ring_ctx.is_left);
start.x = opposite->original_pts[corner_id].x + (left_ring ? 3 : -3);
start.y = opposite->original_pts[corner_id].y - 3;
```

否则：

```c
start.x = mid_position + (left_ring ? width_base / 2 : -width_base / 2);
start.y = Start_high;
```

含义：

- 优先从 opposite 当前角点附近继续补。
- 如果 opposite 不可靠，就从中线附近的默认位置补。

### 前探 Fopposite

```c
seek_up(start, left_ring ? Right_pts : left_pts);
(left_ring ? FRightPts_Deal_Original : FLeftPts_Deal_Original)();
```

### 选择 Fopposite 拼接点

左环：

```c
找第一个 Fopposite.x < 94 的点
```

右环：

```c
找第一个 Fopposite.x > 94 的点
```

注意：

- `94` 是原工程图像中心附近的硬编码。
- 当前工程 `image_width = 160`，中心应按 `image_width / 2` 或运行时中心计算。

### 用直线拼接 opposite

```c
opposite->original_step = addline(start, point, opposite->original_pts, 0, 1);
opposite->original_step = Arry_roll(Fopposite->original_pts,
                                    opposite->original_pts,
                                    ...);
```

含义：

- 收尾阶段不再大量使用贝塞尔，而是用直线加前探段把边线拉回正常车道。

最后：

```c
(left_ring ? RightPts_Deal_Now : LeftPts_Deal_Now)();
current->original_step = 0;
```

继续清空 current 原图边线，避免当前侧参与。

## Ring_state7：出环收尾阶段 2

代码位置：同样在 `Ring_end()`

推进条件：

```c
if (!opposite->is_lost && state == Ring_state7)
{
    phase_counter++;
}

if (phase_counter >= 3)
{
    Ring_switch();
    return;
}
```

含义：

- 等待 opposite 重新稳定。
- 稳定后跳到 `Ring_state8`。

实际操作：

- 仍然执行 `Ring_end()` 的前探 Fopposite、直线补线、`Arry_roll()` 拼接。
- 仍然刷新 opposite。
- 仍然清空 current 的 `original_step`。

对当前工程的启发：

- 出环后不要立刻切回普通双边。
- 需要一段过渡补线期，让中线连续。

## Ring_state8：准备完全退出

代码位置：`Ring_Process()` 的 `case Ring_state8`

推进条件：

```c
if (current->now_step < 3)
    phase_counter++;
else
    phase_counter--;
```

操作：

```c
Ring_switch();

if (opposite->now_step > Step_Max / 2)
{
    if (left_ring)
    {
        if (original_right[1] < Start_high - 5)
            Ring_end();
    }
    else
    {
        if (original_left[1] < Start_high - 5)
            Ring_end();
    }
}

current->now_step = 0;
```

含义：

- 继续压掉 current。
- 如果 opposite 足够长，并且底部起始点已经上移到一定位置，则调用 `Ring_end()` 继续做补线过渡。
- `phase_counter` 达标后进入 `Ring_state9`。

注意：

- `Ring_end()` 里有 `if (state == Ring_state8) return;` 的保护，所以 `Ring_state8` 对补线的调用比较保守。

## Ring_state9：完全退出确认

代码位置：`Ring_Process()` 的 `case Ring_state9`

判定：

```c
dx = opposite->now_pts[opposite->now_step / 2].x -
     current->now_pts[current->now_step / 2].x;
dy = opposite->now_pts[opposite->now_step / 2].y -
     current->now_pts[current->now_step / 2].y;
distance = sqrt(dx * dx + dy * dy);

if (current->now_step > Step_Max / 2 && distance < 60)
    phase_counter++;
```

操作：

```c
current->now_step = 0;
```

退出条件：

```c
if (fabs(sum_zangle) > ANGLE_THRESHOLD_LARGE &&
    phase_counter > PHASE_COUNT_THRESHOLD)
{
    kind = no_ring;
    phase_counter = 0;
    sum_zangle = 0;
}
```

含义：

- 等当前侧也恢复到足够长。
- 两侧中段距离小于 `60`，认为车道宽度恢复正常。
- 累计转角超过 `300` 度，认为完成环岛过程。
- 最后清除圆环状态，交还给普通追边。

对当前工程的启发：

- 完全退出不能只看图像恢复，还要看转角或时间/距离一类状态。
- 如果当前没有稳定陀螺积分，可以先用图像恢复 + 帧计数替代，但最终建议接入 `yaw` 或 `Zangle_acc`。

## 原工程是否有手动切换边线

没有人工按键式切边。

但有状态机自动切边/接管边线：

- `Ring_state1/2/8/9` 反复执行 `current->now_step = 0`
- `Ring_end()` 末尾执行 `current->original_step = 0`
- `Ring_evolve()`、`Ring_Out()`、`Ring_end()` 会主动重写 `opposite->original_pts`
- 重写后再调用 `LeftPts_Deal_Now()` 或 `RightPts_Deal_Now()`

这会影响 `MidPts_Deal()` 的选边逻辑。

`MidPts_Deal()` 的原则：

- 哪侧短、丢线、不可靠，就选另一侧。
- 如果两侧都可用，根据点数选择一侧偏移生成中线。

因此圆环状态机通过“清空 current、重建 opposite”间接控制中线来源。

## 和当前 image.cpp 的概念映射

| Front_Car 原工程 | 当前工程候选对应 |
| --- | --- |
| `left_boundary.original_pts` | `g_front_left_original_points` |
| `right_boundary.original_pts` | `g_front_right_original_points` |
| `left_boundary.original_step` | `g_front_left_original_point_count` |
| `right_boundary.original_step` | `g_front_right_original_point_count` |
| `left_boundary.now_pts` | `g_front_left_ipm_points` |
| `right_boundary.now_pts` | `g_front_right_ipm_points` |
| `left_boundary.now_step` | `g_front_left_ipm_point_count` |
| `right_boundary.now_step` | `g_front_right_ipm_point_count` |
| `Lp_state` | `g_front_left_corner.valid` / `g_front_right_corner.valid`，建议加稳定计数 |
| `is_straight` | 当前工程还缺，需要参考 `IPM_image.cpp::is_boundary_long_straight()` |
| `is_lost` | `g_front_line_lost` 的单侧状态 |
| `Fleft_boundary/Fright_boundary` | `g_front_left_probe_original_points` / `g_front_right_probe_original_points` |
| `LeftPts_Deal_Now()` / `RightPts_Deal_Now()` | `refresh_front_boundary_features()` 或拆分后的单侧刷新 |
| `MidPts_Deal()` | `choose_track_side()` + `offset_from_left/right_boundary()` |
| `seek_up()` | 当前工程的 `find_front_probe_seed()` / 前探搜索逻辑 |
| `findline_*_adaptive()` | 当前工程的 `follow_left_boundary()` / `follow_right_boundary()` |

## 移植时不能直接照搬的点

### 图像宽度硬编码

原工程里有：

- `94`
- `178`
- `188`
- `MT9V03X_W`

当前工程图像范围：

- `x: 0~159`
- `y: 0~119`

所以这些值必须改成：

- `image_width / 2`
- `image_width - 1`
- 根据运行时 `g_front_last_center_x`
- 根据 `valid_l_bound/valid_r_bound`

### 点集含义不同

原工程的 `now_pts` 是投影后重采样点。

当前工程也是 `g_front_*_ipm_points`，但投影、平滑、重采样策略已经改过，不能假设相同 index 对应相同距离。

### 当前工程爬线遇到丢线就停

这会导致上方断开的拐点根本不在点集里。

因此圆环移植不能依赖“四分裂模板直接找到上拐点”。应该先：

```text
稳定通用角点
-> 用角点/断点做前探
-> 补出上方断线
-> 再做圆环状态分类和边线接管
```

## 建议的当前工程最小实现顺序

### 第一步：只做圆环检测 HUD

先不要接管中线。

需要状态：

```cpp
enum FrontRingKind {
    FrontRingNone = 0,
    FrontRingLeft,
    FrontRingRight,
};

enum FrontRingState {
    FrontRingState1 = 0,
    FrontRingState2,
    FrontRingState3,
    FrontRingState4,
    FrontRingState5,
    FrontRingState6,
    FrontRingState7,
    FrontRingState8,
    FrontRingState9,
};
```

需要 HUD：

- `RING:N/L/R`
- `RST:1~9`
- `RPC:phase_counter`
- `LS/RG`：左右角点稳定状态
- `LST/RST`：左右长直线状态

### 第二步：补长直线判定

参考当前工程已有的 `IPM_image.cpp::is_boundary_long_straight()`。

没有长直线判定时，不建议触发圆环，因为原工程入口强依赖：

```text
一侧角点 + 另一侧长直线
```

### 第三步：复刻入口条件

当前工程最小入口：

```text
left stable corner && right long straight && not cross -> left ring
right stable corner && left long straight && not cross -> right ring
```

### 第四步：只实现 State1/State2 的“压当前侧”

先验证：

- 左环时清空左侧是否会让中线稳定转向右侧
- 右环时清空右侧是否会让中线稳定转向左侧

不要马上写贝塞尔补线。

### 第五步：实现前探补线

用当前工程已有前探链：

- `get_front_probe_connection_context()`
- `find_front_probe_seed()`
- `follow_boundary_from_single_seed()`
- `splice_front_probe_points_into_original()`

先做到能从角点/断点向上补出一段线。

### 第六步：再实现 Ring_evolve / Ring_Out 的边线重建

把原工程的：

- `add_three_point_bezier()`
- `Arry_rollback()`
- `Arry_roll()`
- `addline()`

转成当前工程 `uint16` 点集版本。

## 一句话总结

原工程圆环靠“状态机改写边线”完成，不是靠单帧识别。移植到当前工程时，应该先补齐稳定角点和长直线判定，再逐步引入边线接管；不要一开始就完整照搬 `Ring.c` 的 9 状态补线。
