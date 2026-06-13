# Loongson SmartCar 2026

本仓库为第二十一届智能车竞赛龙芯赛道工程代码，当前版本已经完成从摄像头采集、图像巡线、IPM 中线生成、目标航向发布，到调度器闭环控车、TCP 图传调试的主链路联通。

这份 README 按当前工程实际进展整理，重点说明“现在已经做到什么”“核心代码在哪”“下一步还缺什么”。

## 当前进展

目前已经接通的能力：

- UVC 摄像头采集与曝光控制
- 图像巡线与边线提取
- IPM 逆透视点集生成
- 基于 IPM 中线的预瞄目标计算
- 场景分类与速度策略切换
- 5ms 调度器驱动 IMU、编码器、PID 与电机输出
- TCP 图传与调试叠加显示
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
│  │     ├─ pid.cpp            # 角度环/速度环 PID
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

现阶段已确认在代码里存在的关键能力：

- 原图左右边线提取
- 边线转换为 IPM 左右点集和行坐标
- 双边中线与单可靠边偏移中线两套结果
- 自动/强制可靠边模式切换
- 基于连续点集的中线平滑与重采样
- IPM 中线路径形状分类
  包括 `Straight`、`GentleCurve`、`SharpCurve`、`SCurve`、`Invalid`
- 调试图叠加显示
  可显示二值图、IPM 图、边线、中线、预瞄点和状态文本
- 临时绕行机制
  可通过按键触发左绕/右绕，暂时用单边线偏移中线参与控制

补充说明：

- 环岛相关标志位仍在图像标志结构中保留，并且在控制层会参与 `Circle` 场景判断
- 十字/元素状态机并未形成一个完整独立模块接管控制，当前更多是由图像标志和 IPM 场景结果共同参与判断

## 控制与速度策略现状

### 1. 调度器

`project/code/scheduler.cpp` 是当前控制核心：

- 5ms 周期：
  编码器更新、IMU 更新、蜂鸣器任务、绕行计时任务
- 10ms 周期：
  读取视觉发布的 `vision_target_yaw`
  根据 `g_track_info` 计算基础速度
  角度环输出转向修正量
  左右轮速度环输出 PWM

当前 PWM 在软件层被限制为：

- 左轮 `[-50, 50]`
- 右轮 `[-50, 50]`

### 2. 场景类型

控制层当前使用的场景枚举在 `project/code/speed_strategy.hpp`：

- `Straight`
- `GentleCurve`
- `SharpCurve`
- `Circle`
- `LostLine`

### 3. 速度策略

`project/code/speed_strategy.cpp` 中当前基础速度参数为：

- 直道：`130`
- 缓弯：`100`
- 急弯：`90`
- 环岛：`95`
- 丢线：`95`

并且做了非对称加减速平滑：

- 加速步长：`8`
- 减速步长：`10`

### 4. 目标航向生成

`project/code/car_control.cpp` 当前已经接入：

- 基于 IPM 中线点集的预瞄目标计算
- 丢线时双边中线兜底
- IPM 结果不足时退回旧版预瞄偏差方案

也就是说，当前控制链路不是“图像偏差直接打舵机/电机”，而是：

- 先求目标航向 `vision_target_yaw`
- 再由角度环闭环跟踪真实 `yaw`

## 调试方式

### 图传调试

`project/code/tcp.cpp` 与 `scheduler.cpp` 中后台线程负责图传发送，当前支持将调试图通过 TCP 发给上位机。

### 键盘调试

`project/user/main.cpp` 中已经接入简单键盘调试：

- `A`
  切换测试中线模式，并切换 IPM 可靠边模式
- `D`
  切换调试视图
- `S`
  打印 IPM 中线点集快照
- `Q`
  触发左绕行
- `E`
  触发右绕行

如果程序异常退出导致终端输入异常，可手动执行：

```bash
stty sane
```

## 编译与部署

当前工程使用龙芯交叉编译环境，配置集中在：

- `project/user/CMakeLists.txt`
- `project/user/cross.cmake`
- `project/user/build.sh`

### 常规编译

```bash
mkdir -p project/out
cd project/out
cmake ../user
make -j$(nproc)
```

### 一键编译上传

```bash
cd project/user
./build.sh
```

`build.sh` 当前会执行：

1. 清理 `project/out`
2. 重新 `cmake`
3. `make -j$(nproc)`
4. 通过 `scp` 上传生成文件到板端

脚本中当前写死的目标板信息为：

- 目标 IP：`192.168.31.21`
- 用户：`root`
- 目标路径：`/home/root/`

如果板端地址变化，需要先修改脚本。

## 当前已知注意点

- 根目录 `README.md` 之外，`project/本项目代码的一些说明.txt` 里还有不少历史说明，但其中部分描述偏旧，阅读时要以当前代码实现为准
- `track_element.cpp` 虽然保留了元素状态机设计思路，但目前未真正落地成主流程
- 工程里同时保留了旧版思路和新版 IPM 控制思路，后续继续迭代时要明确“谁是主链路”，避免调参对象混淆
- `main.cpp` 里的图传目标 IP 当前为 `192.168.31.20:8086`，与 `build.sh` 的板端 IP 不是同一个地址，这属于正常情况，但调试时要确认上位机地址是否正确

## 下一步建议

如果按比赛可用性继续推进，建议优先做下面几件事：

1. 把元素识别状态机真正落地
   重点补全十字、环岛、特殊弯道的稳定进入/退出条件
2. 整理视觉主链路和旧逻辑
   去掉已经不再使用的分支，减少调参混淆
3. 固化调参文档
   把速度、预瞄距离、可靠边切换阈值、丢线兜底条件统一整理出来
4. 明确 AI 模型路线
   是继续以传统视觉为主，还是把 `model/` 中的推理能力真正接入

## 相关文件

- 入口文件：`project/user/main.cpp`
- 视觉主流程：`project/code/image_test.cpp`
- 控制目标生成：`project/code/car_control.cpp`
- 调度器：`project/code/scheduler.cpp`
- 速度策略：`project/code/speed_strategy.cpp`
- 构建脚本：`project/user/build.sh`

