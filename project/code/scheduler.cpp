#include "scheduler.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>  // 用于 usleep 和 nice
#include <algorithm> // 必须包含这个才能用 std::clamp

#define base_speed 30.0f

// 【全车唯一的主定时器】
zf_driver_pit master_timer;

// 定义一个原子标志，初始值为0，表示不需要打印// std::atomic<uint8_t> 表示这是一个原子的 8 位无符号整数
std::atomic<uint8_t> need_print(0);

// 确保 last_time 和 tick_5ms 是 static 的，防止每次回调被重新初始化
static uint32_t tick_5ms = 0;
static auto last_time = std::chrono::high_resolution_clock::now(); // 调度器专属的全局时间戳
static float dt_sum_10ms = 0.0f;

//===================================下面设置为全局变量，为了方便 TCP 线程访问和调试
float target_speed = 0.0f;
float target_yaw = 0.0f;
float pwm_l = 0.0f;
float pwm_r = 0.0f;

//====================================================================================================================
void master_scheduler_callback()
{
    // ------------------ 1. 获取真实 dt ------------------
    auto now = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    // ------------------ 2. 极端异常保护 ------------------
    if (dt <= 0.0001f)
        return;
    if (dt > 0.2f)
    {
        dt = 0.005f;        // 强行截断，防止积分暴走
        dt_sum_10ms = 0.0f; // 清空累加器，丢弃脏数据
    }

    // 累积控制周期时间
    dt_sum_10ms += dt;

    // ------------------ 3. 基础高频采样 (5ms) ------------------
    encoder_update_task(dt);
    imu_update_task(dt);

    tick_5ms++; // 时间轴向前推移

    // ------------------ 4. 核心控制任务 (10ms) ------------------
    if (tick_5ms % 2 == 0)
    {
        // 提取真实的控制周期并清零累加器
        float control_dt = dt_sum_10ms;
        dt_sum_10ms = 0.0f;
        // ==========================================================
        // 【第一阶段：读取滑块，现在全权交给角度环】
        // ==========================================================
        //  target_yaw = get_online_param(0); // 滑块0：基础直行速度 (建议先给 150)
        // float angle_kp = get_online_param(1);   // 滑块1：角度环 P
        // float angle_ki = get_online_param(2);   // 滑块2：角度环 I
        // float angle_kd = get_online_param(3);   // 滑块3：角度环 D

        // pid_angle.set_pid(angle_kp, angle_ki, angle_kd);
        // ==========================================================
        // 【第二阶段：运行中环 (角度环 - 位置式 PID)】
        // 目标：最终的绝对 target_yaw
        // 如果后面接入视觉循迹，建议在这里使用：
        // target_yaw = wrap_to_180(yaw + vision_delta_yaw);
        // 其中 vision_delta_yaw 由 image_test() 基于 mid_line 计算得到，
        // 表示“相对当前车头，还需要补多少角度”，而不是直接把图像偏差当 target_yaw。
        // 反馈：全车的真实 yaw 角
        // 输出：差速转向修正量 (steer)
        // ==========================================================
             float steer = pid_angle.calc(target_yaw, yaw, control_dt);
                    // // ==========================================================
                    // // 【第三阶段：核心纽带 —— 差速分配 (阿克曼/差速模型)】
                    // ==========================================================
                    float target_speed_l = base_speed + steer;
                    float target_speed_r = base_speed - steer;

        //===============================================================速度环内环
        // /////////////////// 调速度环PID用
        // target_speed = get_online_param(0);
        // kp = get_online_param(1);
        // ki = get_online_param(2);
        // 刷新 PID 参数
        // pid_left.set_pid(kp, ki, 0.0f);
        // pid_right.set_pid(kp, ki, 0.0f);
        // 极度精准的闭环计算
        //============================================
        pwm_l = pid_left.calc(target_speed_l, speed1, control_dt);
        pwm_r = pid_right.calc(target_speed_r, speed2, control_dt);
        // 软件限幅
        pwm_l = std::clamp(pwm_l, -50.0f, 50.0f);
        pwm_r = std::clamp(pwm_r, -50.0f, 50.0f);
        // 驱动电机
        motor_set_speed((int)pwm_l, (int)pwm_r);
        //================================================
    }

    // ------------------ 5. 低频心跳/打印任务 (1秒) ------------------
    // 坚决不重置 tick_5ms，利用取模保证时间轴的绝对连贯！
    if (tick_5ms % 200 == 0)
    {
        need_print.store(1);
    }
}
//===============================================================================================================
// 【后台通信线程】：不受 5ms 定时器约束，随便阻塞
// ====================================================
void tcp_background_thread()
{
    // nice(19) 将此线程优先级降到最低，绝不抢占控制算力
    nice(19);
    while (true)
    {
       // tcp_update_task(); // 执行网络收发与调参读取
        // 简单图传方案：主循环持续刷新 bin_image，这里只负责按固定频率发送当前图像。
        // 这样改动最小，但发送时没有加锁，偶发情况下可能抓到一半更新中的图像。
        // seekfree_assistant_camera_send();
        usleep(20000); // 休眠约 30ms (33Hz)，足够看波形
    }
}
    
// 调度器初始化
void scheduler_init()
{
    // 初始化时间戳
    last_time = std::chrono::high_resolution_clock::now();
    // 启动全车唯一的心脏，5ms 跳动一次
    master_timer.init_ms(5, master_scheduler_callback);
    // printf("中央调度器已启动 (基准周期: 5ms)\n");
    // 启动 TCP 后台线程并将其分离(detach)
    std::thread comm_thread(tcp_background_thread);
    comm_thread.detach();
    printf("系统启动：5ms控制心脏 + 30ms异步通信线程 已就绪！\n");
}
