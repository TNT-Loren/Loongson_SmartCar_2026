# Loongson SmartCar 2026

本仓库为第二十一届智能车竞赛龙芯赛道工程代码，当前版本已经完成从摄像头采集、图像巡线、IPM 中线生成、目标航向发布，到调度器闭环控车、TCP 图传调试的主链路联通。

这份 README 按当前工程实际进展整理，重点说明"现在已经做到什么""核心代码在哪""下一步还缺什么"。

## 当前进展

目前已经接通的能力：

- UVC 摄像头采集与曝光控制
- 图像巡线与边线提取（八邻域边界追踪）
- IPM 逆透视点集生成与等距重采样
- 双边界起点搜索增强：底行失败时向上最多试探 7 行
- 单可靠边法向偏移中线（`build_ipm_offset_midline`）
- 基于 IPM 中线的 Pure Pursuit 预瞄目标计算
- 场景分类与速度策略切换（含新增 ObstacleAvoid 绕行场景）
- 5ms 调度器驱动 IMU、编码器、PID（含角度环抗积分饱和）与电机输出
- TCP 图传与调试叠加显示（原始视角显示可靠边+场景，IPM 视角显示点集）
- 键盘触发调试模式、可靠边切换、临时绕行触发

目前仍处于预留、半成品或未正式接入主流程的部分：

- `project/code/track_element.cpp`
  赛道元素状态机文件目前基本为注释草稿，未作为当前主控制链路的一部分
- `project/model/`
  模型文件和 TFLM/NCNN 依赖已预留，但 AI 推理还未形成当前稳定主流程
- `project/code/VSTAB_module/`
  视频稳像相关代码已放入仓库，但不是当前控车主路径

## 工程结构

```text
Loongson_SmartCar_2026/
├─ project/
│  ├─ user/
│  │  ├─ main.cpp              # 程序入口，初始化硬件、视觉、调度器
│  │  ├─ build.sh              # 一键编译并上传到板端
│  │  └─ CMakeLists.txt        # 龙芯交叉编译配置
│  ├─ code/
│  │  ├─ image_test.cpp        # 当前视觉主流程
│  │  ├─ car_control.cpp       # 视觉结果转目标航向
│  │  ├─ scheduler.cpp         # 5ms/10ms 控制调度核心
│  │  ├─ speed_strategy.cpp    # 场景速度策略
│  │  ├─ tcp.cpp               # 图传与上位机通信
│  │  ├─ zgc_draw_tool.cpp     # 调试绘图工具
│  │  └─ Control/
│  │     ├─ encoder.cpp        # 编码器速度获取
│  │     ├─ imu.cpp            # IMU 航向估计
│  │     ├─ pid.cpp            # 角度环/速度环 PID（含抗积分饱和）
│  │     └─ motor.cpp          # 电机 PWM 输出
│  ├─ docs/
│  │  └─ front_car_ring_state_machine.md
│  └─ model/
│     └─ loong_cnn_model_simple.tflite
└─ libraries/
   ├─ zf_common/
   ├─ zf_device/
   ├─ zf_driver/
   └─ zf_components/
```

## 主控制链路

当前运行主线如下：

1. `project/user/main.cpp`
   初始化 IMU、编码器、蜂鸣器、电机、TCP 图传和 UVC 摄像头。
2. 启动前先执行一次 `image_test()`
   先产出一帧视觉结果，避免控制器带着空目标起步。
3. 主循环中持续执行 `image_test()`
   视觉线程持续更新边线、中线、调试图和场景信息。
4. `project/code/car_control.cpp`
   将视觉结果转换为 `vision_target_yaw`，并发布 `g_track_info`。
5. `project/code/scheduler.cpp`
   每 5ms 更新 IMU、编码器、蜂鸣器与绕行倒计时；每 10ms 运行角度环、速度环和电机输出。

可以把数据流理解为：

```text
摄像头采集
  -> image_test() 提取边线/中线
  -> car_control.cpp 计算目标航向和场景
  -> scheduler.cpp 计算 steer / 左右轮目标速度
  -> PID 输出 PWM
  -> motor.cpp 驱动电机
```

## 视觉部分现状

当前视觉主流程以 `project/code/image_test.cpp` 为核心，已经不是早期单纯靠原图中线偏差的版本，而是以 IPM 点集和预瞄控制为主。

### 边界搜索

`get_start_point()` 从图像底行（y=118）开始，寻找左右边界起点。若当前行无法同时确认左右起点，则逐行向上重试，最多搜索 7 行。任意一行左右起点均成立则立即返回，避免因底行局部噪点/阴影导致整帧作废。

### IPM 中线生成

1. 八邻域边界追踪 → `Left_Line` / `Right_Line`
2. 丢线统计 → `update_ipm_auto_reliable_edge_selection()` 打分选出可靠边
3. `transform_lines_to_ipm()` 将可靠边投影到 IPM 空间，等距重采样（3px 间距）
4. `build_ipm_offset_midline()` 对可靠边的每个点，取前后 2 点计算切线，向赛道内侧偏移 `k_ipm_half_road_width=15` 得到中线点
5. 圆环状态机（`Island_Detect`）可强制干预可靠边选择，覆盖 Auto 结果

### 原图旧方案处理

- 原图左右拟合黄线（`build_test_midline`）已废弃，不再在原始视角图传中绘制
- TCP 图传原始视角现显示：丢线状态栏、IPM 实际可靠边（LEFT/RIGHT）、赛道场景（Straight/Gentle/Sharp/Circle/LOST）
- IPM 视角（`g_debug_show_ipm_lines`）可查看 IPM 白底图 + 边线 + 中线 + 预瞄点 + 调试标志

### 环岛状态机

`Island_Detect()` 用 `island_state`（0~6）管理左环/右环全过程：

| 状态 | 含义 | 触发方式 |
|------|------|---------|
| 0 | 正常行驶 | 双边丢线 < 20 |
| 1 | 入环识别 | 单边丢线 20~70 + 对侧连续 + hightest < 30 + 下拐点 |
| 2 | 过了入口 | continuity-单调差异 > 5 |
| 3 | 环内 | Monotonicity_Change > 40（或 fallback） |
| 4 | 深环 | yaw 累计 > 45° + 最短白列 < 40 |
| 5 | 出环 | yaw 累计 > 260° + 对侧下拐点 |
| 6 | 退出后 | 上下拐点延长边界，等丢线恢复正常 → 状态 0 |

## 控制与速度策略现状

### 1. 调度器

`project/code/scheduler.cpp` 是当前控制核心：

- 5ms 周期：编码器更新、IMU 更新、蜂鸣器任务、绕行计时任务
- 10ms 周期：读取 `vision_target_yaw` → 计算 base_speed → 角度环计算 steer → 左右轮速度环输出 PWM

`k_max_steer_ratio` 当前为 `0.67`，即转向修正上限 = `base_speed × 0.67`。

PWM 软件限幅：左轮 `[-50, 50]`，右轮 `[-50, 50]`。

### 2. 场景类型

`project/code/speed_strategy.hpp` 中的 `TrackScene` 枚举：

- `Straight` — 直道
- `GentleCurve` — 缓弯
- `SharpCurve` — 急弯
- `ObstacleAvoid` — 绕行（新增）
- `Circle` — 环岛
- `LostLine` — 丢线

### 3. 速度策略

`project/code/speed_strategy.cpp` 中当前基础速度参数：

| 场景 | 速度值 |
|------|--------|
| 直道 | 130 |
| 缓弯 | 100 |
| 急弯 | 90 |
| 绕行 | 40 |
| 环岛 | 95 |
| 丢线 | 95 |

加减速步长：加速 `8`，减速 `10`。

> **已知 bug**：`calc_base_speed` 函数返回的是未平滑的 `target` 而非平滑后的 `static base_speed`，导致场景切换时速度直接跳变，加减速平滑逻辑实际上不生效。

### 4. PID 结构与抗积分饱和

**角度环** — `PositionalPID`（kp=3.5, ki=0.3, kd=0.2, output_limit=100, integral_limit=30）：

- 新增 `calc(target, current, dt, dynamic_output_limit)` 重载
- 调度器传入 `steer_limit = base_speed × k_max_steer_ratio` 作为动态限幅
- 当输出饱和且误差方向仍在增大输出时，回退本次积分累积，防止弯内积分堆积导致出弯拖尾
- 死区内输出为 0，误差角度使用 `wrap180f` 处理 ±180° 跨越

**速度环** — `IncrementalPID`（kp=0.6, ki=0.3, kd=0, output_limit=60）

### 5. 目标航向生成

`project/code/car_control.cpp`：

- Pure Pursuit 预瞄（`calc_preview_target_yaw`）：IPM 中线 → 控制侧近端补全 + 远端切向外推 → 按预瞄距离取点 → `atan2` 算角度误差
- IPM 不足时退回 Legacy 预瞄（基于原图中线行偏差）
- 丢线兜底：`vision_target_yaw = yaw`（保持当前航向）

## 调试方式

### 图传调试

原始视角（`D` 键切换）顶部状态栏：

- 三条丢线指示条（左 L / 右 R / 双 B，红色=丢线，绿色=正常）
- 十字/环岛/状态指示块
- IPM 实际可靠边（`selected_ipm_reliable_edge_mode()` 结果）：黄色文字
- 当前赛道场景（`Control_Ipm_Debug_Scene`）：青色文字

IPM 视角显示 IPM 白底图 + 左右边线 + 中线 + 预瞄点 + 调试标志。

### 键盘调试

`project/user/main.cpp` 中按键：

| 按键 | 功能 |
|------|------|
| `A` | 切换测试中线模式 + IPM 可靠边模式（ForceLeft/ForceRight/Auto 循环） |
| `B` | 预留（当前提示已切回 image_test） |
| `D` | 切换普通图传 / IPM 边线图 |
| `S` | 打印 IPM 中线点集快照 |
| `Q` | 触发左绕行 |
| `E` | 触发右绕行 |

若程序异常退出导致终端输入异常：

```bash
stty sane
```

## 编译与部署

### 常规编译

```bash
cd build
cmake ../user
make -j$(nproc)
```

### 一键编译上传

```bash
cd project/user
./build.sh
```

`build.sh` 流程：清理 → cmake → make -j$(nproc) → scp 上传到板端。

脚本写死的目标板信息：

- 目标 IP：`192.168.31.21`
- 用户：`root`
- 目标路径：`/home/root/`

如果板端地址变化需先修改脚本。

## 当前已知注意点

- `project/本项目代码的一些说明.txt` 中部分描述偏旧，阅读时以当前代码实现为准
- `track_element.cpp` 保留了元素状态机设计思路但未落地成主流程
- **`calc_base_speed` 函数存在 bug**：返回的是未平滑的 `target` 而非经过加减速步长限幅的 `base_speed`，平滑逻辑不生效
- `build_test_midline` 及原图黄色拟合测试中线已废弃，原始视角不再绘制；可靠边选择通过顶部文字直接显示
- 更换 IPM 变换矩阵（Mat2）后，需同步修改：`k_ipm_half_road_width`、`k_vehicle_x/y`、`lookahead_dist`、`k_ipm_sample_distance`、`k_ipm_break_distance` 等 IPM 像素坐标系下所有参数
- `main.cpp` 中图传目标 IP 为 `192.168.31.20:8086`，与 `build.sh` 的板端 IP（`192.168.31.21`）不同，属正常情况
- 工程里同时保留了旧版思路和新版 IPM 控制思路，后续迭代要明确"谁是主链路"，避免调参对象混淆

## 下一步建议

1. **修复 `calc_base_speed` 返回值 bug**，让加减速平滑真正生效
2. **落地元素识别状态机**，重点补全十字、环岛、特殊弯道的稳定进入/退出条件
3. **清理视觉旧链路分支**，去掉已废弃的逻辑，减少调参混淆
4. **固化调参文档**，把速度、预瞄距离、可靠边切换阈值、丢线兜底条件统一整理
5. **明确 AI 模型路线**，是继续以传统视觉为主，还是把 `model/` 中的推理能力真正接入

## 相关文件

| 文件 | 说明 |
|------|------|
| `project/user/main.cpp` | 程序入口 |
| `project/code/image_test.cpp` | 视觉主流程（4447 行） |
| `project/code/car_control.cpp` | 控制目标生成（Pure Pursuit + Legacy） |
| `project/code/scheduler.cpp` | 5ms/10ms 控制调度核心 |
| `project/code/speed_strategy.cpp` | 场景速度策略 |
| `project/code/Control/Control_Source/pid.cpp` | PID 控制（含抗积分饱和） |
| `project/code/tcp.cpp` | 图传与上位机通信 |
| `project/user/build.sh` | 一键编译上传脚本 |
