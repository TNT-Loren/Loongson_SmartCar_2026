🚗 第二十一届智能车龙芯赛道 - 基础工程框架导读
为了方便大家协同开发，这里梳理了咱们目前第一版车模代码的整体架构。

./user/main.cpp          # 主函数，初始化各模块，启动调度器
./code/                  # 核心控制与通信代码
├── scheduler.hpp/cpp    # 中央调度器，5ms 定时器 + 10ms 控制循环
├── tcp.hpp/cpp          # TCP 通信模块，与上位机交互
├── Control/             # 控制算法
│   ├── Control_Header/  # 头文件
│   │   ├── pid.hpp      # PID 控制器类声明
│   │   ├── motor.hpp    # 电机驱动接口
│   │   ├── imu.hpp      # IMU 数据处理类
│   │   └── encoder.hpp  # 编码器读取接口
│   └── Control_Source/  # 实现文件
│       ├── pid.cpp      # 增量式/位置式 PID 实现
│       ├── motor.cpp    # 电机驱动实现
│       ├── imu.cpp      # YawTracker 实现，陀螺仪积分
│       └── encoder.cpp  # 编码器速度计算
./model/                 # 神经网络模型（未使用）
    └── loong_cnn_model_simple.h

1. 核心设计思想：中央大脑调度
我们的系统并不是各个模块“各跑各的”，而是采用了一个**“中央调度器 (Scheduler)”模式。
大家可以把 scheduler.cpp 想象成车辆的“心脏起搏器”。全车只有一个**主定时器 (master_timer)，每隔 5ms 跳动一次。所有的传感器读取、PID 计算都严格按照它的节拍来进行，并且我们每次都会计算真实的间隔时间 (dt)，这保证了在 Linux 系统下控制时序的绝对稳定。

2. 目录结构与模块分工
user/main.cpp (程序入口)：
程序的起点。负责把所有硬件（电机、IMU、编码器、TCP 网络）初始化一遍，然后启动“中央调度器”，之后主线程就进入低频的休眠和打印监控状态。

code/scheduler.cpp (核心调度与运动学解算)：
这是全车最核心的文件，车辆的运动逻辑都在这里！

高频任务 (5ms)：获取最新、最真实的编码器速度和 IMU 偏航角 (Yaw)。

控制任务 (10ms)：执行串级 PID 控制（外环角度、内环速度），并进行阿克曼差速分配，最后输出 PWM 驱动电机。

code/tcp.cpp (上位机通信与在线调参)：
这里开了一个独立的后台异步线程（并且主动降低了优先级，绝不抢占控制算力），专门用来和“逐飞助手”通信。负责把车身数据画成波形，并实时接收滑块的调参指令。

code/Control/ (底层控制组件库)：

imu.cpp：处理陀螺仪数据，内置梯形积分算出车身偏航角 (Yaw)，开机自带校准静止零偏的功能。

encoder.cpp：读取编码器脉冲，经过 RC 低通滤波后算出车轮的真实物理速度。

pid.cpp：封装了增量式 PID（给速度内环用）和位置式 PID（给角度外环用）。

model/：存放我们后续要部署的 AI 神经网络模型（如 TFLite 文件）。

3. 小车是怎么动起来的？(数据流向图)
我们的运动控制是一个经典的双闭环（串级）+ 差速运动学模型，以 10ms 为一个控制周期，数据流向如下：

读反馈：从 encoder 拿到左右轮当前速度，从 imu 拿到当前车头角度 (yaw)。

算中环 (角度环)：我们想跑直线（开机时 target_yaw = 0）。将 0 度与当前 yaw 的偏差送入位置式 PID (pid_angle)，算出一个用于纠偏的转向修正量 (steer)。

算差速 (运动学)：

左轮目标速度 = 基础直线速度 + steer

右轮目标速度 = 基础直线速度 - steer

算内环 (速度环)：将计算出的左右轮“目标速度”与“当前真实速度”分别送入各自的增量式 PID (pid_left, pid_right)。

控电机：速度环 PID 计算出最终需要输出的 PWM 占空比，经过安全限幅后打给电机，车子完成一次姿态调整。

4. 队友协作指南
如果要调参：直接连上逐飞助手，在 scheduler.cpp 的 10ms 任务中，把 get_online_param 的注释打开，将对应滑块的值赋给目标变量（如 target_speed 或 PID 参数）即可实时看效果。

如果要加新的传感器（比如电磁、摄像头）：

在 Control 文件夹下新建对应的 .cpp/.hpp，并提供类似 xx_update_task() 的更新接口。

在 scheduler.cpp 的 master_scheduler_callback 中，找一个合适的时间片（比如 10ms 或是再分频出 20ms）调用你的任务接口。

注意：回调函数是在定时器中断/独立高优先级线程中运行的，千万不要在里面写死循环 (while) 或死延时 (delay)，否则全车直接瘫痪！


./user/main.cpp          # 主函数，初始化各模块，启动调度器
./code/                  # 核心控制与通信代码
├── scheduler.hpp/cpp    # 中央调度器，5ms 定时器 + 10ms 控制循环
├── tcp.hpp/cpp          # TCP 通信模块，与上位机交互
├── Control/             # 控制算法
│   ├── Control_Header/  # 头文件
│   │   ├── pid.hpp      # PID 控制器类声明
│   │   ├── motor.hpp    # 电机驱动接口
│   │   ├── imu.hpp      # IMU 数据处理类
│   │   └── encoder.hpp  # 编码器读取接口
│   └── Control_Source/  # 实现文件
│       ├── pid.cpp      # 增量式/位置式 PID 实现
│       ├── motor.cpp    # 电机驱动实现
│       ├── imu.cpp      # YawTracker 实现，陀螺仪积分
│       └── encoder.cpp  # 编码器速度计算
./model/                 # 神经网络模型（未使用）
    └── loong_cnn_model_simple.h

最后提供了一个手动打包的Python脚本，在其终端内输入：python3 pack_code.py即可将工程下所有文件打包成一个txt文件(project_code.txt)，方便直接投喂大模型网页版