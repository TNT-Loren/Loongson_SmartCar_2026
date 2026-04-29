🚗 第二十一届智能车竞赛龙芯赛道 — 工程框架导读

为了方便大家协同开发，这里梳理了当前车模代码的整体架构（截至 2026-04-28）。

---

## 1. 核心设计思想：中央调度器

系统采用 **中央调度器模式**，全车只有一个主定时器 `master_timer`，每 5ms 跳动一次。所有传感器读取、PID 计算都严格按照它的节拍执行，每次计算真实的间隔时间 `dt`，保证 Linux 系统下控制时序的绝对稳定。

```
master_scheduler_callback (5ms)
  ├── encoder_update_task() / imu_update_task()   ← 每 5ms
  ├── 控制任务 (10ms)
  │     ├── pid_angle.calc()       → 角度环
  │     ├── calc_base_speed()      → 速度策略
  │     ├── 阿克曼差速分配
  │     ├── pid_left/right.calc()  → 速度环
  │     └── motor_set_speed()      → PWM 输出
  └── 打印心跳 (1s)
```

独立线程：

```
tcp_background_thread (nice 19, 50Hz)
  ├── seekfree_assistant_camera_send()  → TCP 图传
  └── usleep(20ms)
```

---

## 2. 目录结构与模块分工

### `user/main.cpp` — 程序入口
初始化所有硬件（电机、IMU、编码器、TCP 网络、摄像头），启动中央调度器后主线程进入低频监控状态。

### `code/scheduler.cpp + scheduler.hpp` — 核心调度器
全车最核心的文件。5ms 高频任务读取编码器/IMU，10ms 控制任务执行串级 PID + 差速分配 + 电机驱动。

### `code/IPM_image.cpp + IPM_image.hpp` — 逆透视图像处理（当前视觉主链路）
完整的赛道视觉管线：

| 步骤 | 说明 |
|------|------|
| 摄像头采集 | UVC 免驱摄像头 320×240 |
| 降采样 | `cv::resize` → 160×120（节约算力） |
| Otsu 阈值 | 在原图上计算（ROI 从 image_height/3 开始） |
| IPM 逆透视 | 四点透视标定矩阵 Mat1，逐像素反向采样映射 |
| 二值化 | 仅对有效区内像素做阈值二值化 |
| 形态学滤波 | 3×3 邻域去孤立白点/填黑洞 |
| 画有效区框 | 左右黑边 + 底边封口，为八邻域提供虚拟边界 |
| 起始点搜索 | 底边上一行找最长白段 → 左右黑跳变起始点 |
| 八邻域边界跟踪 | `search_l_r()` 左右同时沿黑白交界向上爬 |
| 边界线输出 | `left_edge_line[120]` / `right_edge_line[120]` / `mid_line[120]` |

**关键假设**：相机高度/俯仰/安装位置固定，Mat1 矩阵通过四点透视标定一次即不变。

### `code/car_control.cpp + car_control.hpp` — 中线滤波与偏差计算
- `fit_midline()`：梯度限制滤波，防止中线跳变
- `HDPJ_lvbo()`：五点滑动平均滤波
- `Cal_Weigth1()` / `Cal_Weigth2()`：基于加权平均的中线偏差计算（近距/远距两套权重），输出归一化偏差值

### `code/speed_strategy.cpp + speed_strategy.hpp` — 速度策略
- `TrackInfo` 结构体：赛道场景枚举（Straight / GentleCurve / SharpCurve / LostLine）+ 偏差 + 曲率
- `calc_base_speed()`：根据场景切换目标车速，含非对称加减速平滑 + 大偏差安全降速

### `code/tcp.cpp + tcp.hpp` — 上位机通信
基于 TCP 的逐飞助手协议，负责：
- **图传**：发送图像 + 边界线叠加（左边界/中线/右边界）
- **虚拟示波器**：最多 4 通道浮点波形
- **在线调参**：8 路滑块参数实时下发

### `code/zgc_draw_tool.cpp + zgc_draw_tool.hpp` — 调试绘图库
独立的 RGB565 彩色调试绘图工具库，提供：

| 函数 | 用途 |
|------|------|
| `dbg_point()` | 单像素落点 |
| `dbg_cross()` | 十字标记（搜线起始点/拐点） |
| `dbg_rect()` | 空心正方形框（候选元素区域） |
| `dbg_fill_rect()` | 实心矩形填充 |
| `dbg_line()` | Bresenham 整数直线 |
| `dbg_circle()` | 空心圆（圆环元素标注） |
| `dbg_trace_points()` | 离散点轨迹（八邻域搜线路径可视化） |

全部纯整数运算，坐标越界静默跳过，兼容 SCC8660 彩色图传。

### `code/Control/` — 底层控制组件
| 文件 | 功能 |
|------|------|
| `imu.cpp/hpp` | 陀螺仪数据读取，梯形积分计算偏航角 Yaw，开机静止零偏校准 |
| `encoder.cpp/hpp` | 编码器脉冲采集，RC 低通滤波计算车轮真实速度 |
| `pid.cpp/hpp` | 增量式 PID（速度内环）+ 位置式 PID（角度外环） |
| `motor.cpp/hpp` | 电机 PWM 驱动接口 |

### `code/image.cpp + image.hpp` — 旧版搜线模块（已停用）
差分比和搜线 + 圆环搜索，当前主链路已迁移至 IPM 管线，保留作为参考。

### `code/brushless.cpp + brushless.hpp` — 无刷电机/负压控制

### `code/image_test.cpp + image_test.hpp` — 图像模块独立测试

### `code/VSTAB_module/` — 视频稳像模块

### `project/model/` — AI 模型存放目录（预留给后续 NCNN/TFLite 部署）

---

## 3. 数据流向：从图像到车轮

```
摄像头 320×240
  ↓ cv::resize
160×120 灰度原图
  ↓ Otsu 阈值 + IPM 逆透视
ipm_image_array
  ↓ 二值化 → 形态学滤波 → 画框 → 起始点搜索
bin_image + start_point_l/r
  ↓ 八邻域边界跟踪 search_l_r()
left_edge_line / right_edge_line / mid_line
  ↓ fit_midline() + HDPJ_lvbo() 滤波
  ↓ Cal_Weigth1/2() 加权偏差
vision_target_yaw ← 角度环 pid_angle
  ↓
阿克曼差速分配 → 速度环 pid_left/right → PWM → 电机
```

---

## 4. TCP 调试可视化

```
fill_debug_image()
  ├── memcpy(bin_image)              → 灰底二值图
  ├── dbg_cross(start_point)         → 绿色十字标起始点
  ├── dbg_trace_points(points_l)     → 红色描左搜线轨迹
  ├── dbg_trace_points(points_r)     → 蓝色描右搜线轨迹
  └── dbg_rect(角点) / dbg_circle(圆环) → 彩色标记元素候选
        ↓
seekfree_assistant_camera_send()     → TCP 上位机实时显示
```

---

## 5. 队友协作指南

### 调试视觉
- 将 tcp.cpp 图传源切到 `debug_image`（RGB565/SCC8660），可看到搜线轨迹和候选区域
- 用 `zgc_draw_tool` 库在关键位置画框画圆，不用改算法代码

### 调参
- 连上逐飞助手，在 scheduler.cpp 的 10ms 任务中打开 `get_online_param()` 注释，将滑块值赋给目标变量

### 添加新模块
- 在 `code/` 下新建 `.cpp/.hpp`，提供 `xx_update_task()` 接口
- 在 `scheduler.cpp` 的 `master_scheduler_callback` 中找合适时间片调用
- **注意**：回调函数在定时器线程中运行，禁止死循环或阻塞延时

### 添加视觉元素识别（十字/圆环/坡道）
- 在 `IPM_image.cpp` 中新增检测函数，利用 `mid_line` / `left_edge_line` / `right_edge_line` 做特征判断
- 检测结果写入 `g_track_info`（TrackInfo 结构体），speed_strategy 自动响应

---

## 6. 打包脚本

终端输入 `python3 pack_code.py` 即可将工程下所有文件打包为 `project_code.txt`，方便投喂大模型网页版。
