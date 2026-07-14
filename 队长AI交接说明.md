# 龙芯 IPS200 实体按键菜单——队长 AI 交接说明

## 结论先行

这份代码来自队员个人分支 `codex/menu-key-port`，基于队长仓库提交 `0ae2005`（`合并陆的tcp和代码`）移植，最终提交为 `11dcb7f`（`发车后停用IPS200菜单`）。个人分支没有 push 到队长仓库。

交接包中的“两个按键文件”是两个模块。为避免只拿 `.cpp` 后缺少声明，已把头文件一起带出：

- `common_menu.cpp/.hpp`：菜单树、菜单项、最多 64 项的静态存储，不使用堆分配。
- `common_MYmenu.cpp/.hpp`：IPS200 显示、5 个实体按键、速度/PID 参数菜单及发车安全门。

不要只复制这 4 个文件就直接编译；还必须按下文接入 `pid.hpp`、`speed_strategy.hpp`、`main.cpp` 和 `scheduler.cpp`。如果队长主分支已前进，请基于新代码逐项合并，不要整文件覆盖。

## 当前行为

- 上电默认 `STOP`，调度器强制左右 PWM 为 0。
- 菜单只用于发车前调参；只有 `STOP` 状态允许修改速度和 PID。
- 在 `Smart Car -> start car` 上按 KEY5 后：IPS200 填黑一次，发布原子 `RUN` 状态，随后 `Menu_Task()` 永久返回，不再扫描 GPIO，也不再写 framebuffer。
- 发车是本次程序运行内的单向操作，不能再从菜单切回 `STOP`；只有重启程序或整车重新上电才恢复菜单。
- IPS200 驱动没有背光关闭/反初始化接口，所以只能做到“黑屏 + CPU 零刷新”，物理背光可能仍亮。
- 停车期间，调度器持续清空左右速度 PID、角度 PID 和速度爬升状态，避免启动时沿用旧积分或旧 PWM。

## 实体按键映射

按键按下为低电平，使用下降沿检测和 40 ms 防抖；KEY5 使用霍尔/磁铁 IO `ZF_GPIO_HALL_DETECTION`。

| 按键 | 浏览状态 | 数值编辑状态 |
|---|---|---|
| KEY1 | 进入文件夹 | 无操作 |
| KEY2 | 返回父文件夹 | 无操作 |
| KEY3 | 下一项 | 增加数值 |
| KEY4 | 上一项 | 减少数值 |
| KEY5 | 布尔项直接切换；数值项进入编辑 | 退出编辑 |

`start car` 是布尔项：在该项按 KEY5 会执行一次性发车锁存。

## 菜单参数

- `Speed`：直线、缓弯、急弯、绕行、圆环、丢线速度，以及加速/减速步长。
- `Speed PID`：左右速度环 Kp/Ki/Kd。
- `Angle PID`：角度环 Kp/Ki/Kd、输出限幅、积分限幅、死区。
- 速度每次调整 5，速度步长每次调整 1，PID 每次调整 0.1；代码内同时有限值范围。
- IPS200 最多约 20 Hz 刷新，按下发车后不再刷新。

## 接入队长工程时必须做的改动

1. 把本包 4 个文件放入：
   `project/code/Control/Menu/`
2. 检查 `project/user/CMakeLists.txt` 是否同时包含：
   - `include_directories(... ../code/Control/Menu ...)`
   - `aux_source_directory(../code/Control/Menu DIR_SRCS)`
   基础提交 `0ae2005` 已有这两项，新分支仍需核对。
3. 在 `project/code/Control/Control_Header/pid.hpp` 暴露停车菜单所需参数地址：
   - 速度 PID：`kp_ptr()`、`ki_ptr()`、`kd_ptr()`。
   - 角度 PID：除上述 3 个外，再加 `output_limit_ptr()`、`integral_limit_ptr()`、`deadband_ptr()`。
4. 在 `project/code/speed_strategy.hpp` 声明菜单使用的全局量：
   - `k_speed_up_step`
   - `k_speed_down_step`
   - `k_large_alpha_slowdown_deg`
5. 在 `project/user/main.cpp`：
   - `#include "common_MYmenu.hpp"`
   - 外设初始化后调用 `Menu_Init()`。
   - 主循环每轮调用 `Menu_Task()`。
   - `sigint_handler()` 和 `cleanup()` 开头调用 `Menu_Force_Stop()`。
6. 在 `project/code/scheduler.cpp`：
   - 包含 `common_MYmenu.hpp`。
   - 每次进入 10 ms 控制段后，先检查 `Menu_Car_Enabled()`。
   - `STOP` 时令目标速度、左右 PWM、`base_start_speed` 为 0，目标角设为当前 yaw，清空 `pid_left/pid_right/pid_angle`，执行 `motor_set_speed(0, 0)` 后立即返回。
7. 参数指针必须只在停车状态编辑。当前实现通过原子 `g_car_enabled` 在菜单线程和调度器线程间传递启停状态；不要改回普通全局 `bool`。

## 现场配置（不是菜单本体，但合并时别误覆盖）

- 用户电脑/逐飞助手：`192.168.31.30:8086`。
- 龙芯板：`192.168.31.21`。
- 队长要求保留：`TCP_IMAGE_SOURCE_VS = 0`、`VS_ENABLE_GUIDELINES = 0`。
- 当前 `TCP_IMAGE_SOURCE_VS` 已被改成真正的布尔开关：`0=巡线图传`，`1=VS 图传`。上游原来的枚举写法若直接把宏改成 0 会发生常量冲突并触发 `#error`。
- 上述 IP/图传配置与菜单逻辑独立；若队长现场网络不同，应单独确认，不要因为移植菜单而静默改掉。

## 验证状态

- `menu_structure_test.ps1`、`tcp_camera_structure_test.ps1`、`vs_runtime_structure_test.ps1` 均通过。
- 在虚拟机 `~/work2/Loongson_SmartCar_2026-worktree/project/out` 执行 `cmake ../user && make -j2`，结果为 `[100%] Built target project`。
- 最终产物为 64 位 LoongArch ELF，SHA-256：`a76fce3bacbc29ad649fb91a2efcd207af7e6bd76451c2de70ed0d25865629eb`。
- 7 月 13 日板子 `192.168.31.21` 不在线，因此尚未上传、尚未上板验证按键电平和实际显示。
- 仍有队长原代码的已知警告：`g_obstacle_avoid_ticks_left` 是 `std::atomic<uint8_t>`，却写入 400，实际截断为 144；与本菜单无关，交接时没有顺手修改。

## 建议队长 AI 的合并顺序

1. 先比较队长当前 HEAD 与基础提交 `0ae2005`，确认 PID、速度策略、调度器接口是否变化。
2. 放入两个菜单模块，再按上面的 4 个接入文件逐项适配。
3. 先跑结构测试和交叉编译，不要直接上板。
4. 上板后车轮架空，确认上电 `STOP` 时 PWM 恒为 0。
5. 依次验证 KEY1~KEY5、数值上下限和防抖。
6. 最后触发 `start car`，确认屏幕填黑、按键永久失效、车辆进入 `RUN`；再次验证只能靠重启恢复菜单。

## 本包代码文件校验值（SHA-256）

- `common_menu.cpp`：`85A187B36255AF4381845221D72260D750D19F96070D5D0E8A69343D1918CFC7`
- `common_menu.hpp`：`AE986193F7D6745379F9F9AC239E314594F5FC028DA847B8009021655E9AADFC`
- `common_MYmenu.cpp`：`5CC5ADA3033321D0F2349C696677D4AFCA0E35205AB5602F92943B291BD13488`
- `common_MYmenu.hpp`：`06D9BB3E5CC86F6887E32EC522A6E2E024FB42A14A2FE423A17540F9ECB8B729`

