#include "scheduler.hpp"
#include "beep.hpp"
#include "common_MYmenu.hpp"

#include <iostream>
#include <chrono>
#include <cmath>
#include <thread>
#include <unistd.h>  // 用于 usleep 和 nice
#include <algorithm> // 必须包含这个才能用 std::clamp

extern TrackInfo g_track_info;
// false ture
namespace
{
constexpr bool k_use_menu_start_gate = false;//是否用菜单
constexpr bool k_speed_pid_tuning_mode = false;
constexpr bool k_angle_pid_tuning_mode = false;
constexpr float k_speed_tuning_target_max = 250.0f;
constexpr float k_speed_tuning_gain_max = 20.0f;
constexpr float k_angle_tuning_target_limit_deg = 180.0f;
constexpr float k_angle_tuning_steer_limit = 30.0f;
static_assert(!(k_speed_pid_tuning_mode && k_angle_pid_tuning_mode),
              "Only one PID tuning mode can be enabled");
}

//#define base_speed 120.0f

// 【全车唯一的主定时器】
zf_driver_pit master_timer;
// 定义一个原子标志，初始值为0，表示不需要打印// std::atomic<uint8_t> 表示这是一个原子的 8 位无符号整数
std::atomic<uint8_t> need_print(0);

// 确保 last_time 和 tick_5ms 是 static 的，防止每次回调被重新初始化
static uint32_t tick_5ms = 0;
static auto last_time = std::chrono::steady_clock::now(); // 调度器专属的单调时间戳
static float dt_sum_10ms = 0.0f;


//===================================下面设置为全局变量，为了方便 TCP 线程访问和调试
float target_speed = 0.0f;
float target_yaw = 0.0f;
float pwm_l = 0.0f;
float pwm_r = 0.0f;

namespace
{
std::atomic<float> g_telemetry_left_speed{0.0f};
std::atomic<float> g_telemetry_right_speed{0.0f};
std::atomic<float> g_telemetry_left_pwm{0.0f};
std::atomic<float> g_telemetry_right_pwm{0.0f};

void publish_wheel_control_telemetry()
{
    g_telemetry_left_speed.store(speed1, std::memory_order_relaxed);
    g_telemetry_right_speed.store(speed2, std::memory_order_relaxed);
    g_telemetry_left_pwm.store(pwm_l, std::memory_order_relaxed);
    g_telemetry_right_pwm.store(pwm_r, std::memory_order_relaxed);
}
}

WheelControlTelemetry wheel_control_telemetry_snapshot()
{
    WheelControlTelemetry telemetry;
    telemetry.left_speed = g_telemetry_left_speed.load(std::memory_order_relaxed);
    telemetry.right_speed = g_telemetry_right_speed.load(std::memory_order_relaxed);
    telemetry.left_pwm = g_telemetry_left_pwm.load(std::memory_order_relaxed);
    telemetry.right_pwm = g_telemetry_right_pwm.load(std::memory_order_relaxed);
    return telemetry;
}

//====================================================================================================================
void master_scheduler_callback()
{
    // 5 ms PIT 回调以单调时钟测量真实周期；dt 和后面的 control_dt 单位均为秒。
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    // 极短周期没有有效采样意义，直接忽略且不推进控制时间轴。
    if (dt <= 0.0001f)
        return;
    if (dt > 0.2f)
    {
        // 长周期的数据已失真：丢弃累计脉冲和 10 ms 时间，不伪造 dt，也不突变当前 PWM。
        // 角度环恢复后的第一个有效周期禁用 D，防止跨越长间隔产生微分冲击。
        dt_sum_10ms = 0.0f;
        encoder_clear_counts();
        pid_angle.suppress_derivative_once();
        return;
    }

    // 两次约 5 ms 的真实周期累加为一次速度/角度闭环使用的 control_dt。
    dt_sum_10ms += dt;

    // 5 ms 高频任务：编码器先按真实 dt 换算轮速，IMU 按真实 dt 积分 yaw。
    encoder_update_task(dt);
    imu_update_task(dt);
    Beep_Task_5ms();
    // 绕行倒计时挂在 5ms 主节拍上，持续时间在 image_test.cpp 中统一配置。
    obstacle_avoid_5ms_task();

    tick_5ms++; // 仅正常周期推进逻辑节拍，异常周期不会扰乱 10 ms 分频。

    // 10 ms 主闭环：控制目标 -> 左右轮目标速度 -> 速度 PID -> PWM。
    if (tick_5ms % 2 == 0)
    {
        // 使用两次采样的真实累计时间，随后立即清零等待下一个控制周期。
        float control_dt = dt_sum_10ms;
        dt_sum_10ms = 0.0f;

        // 发车安全门优先级最高。STOP 时持续清空全部闭环状态，禁止残留积分和 PWM。、
        if (k_use_menu_start_gate && !Menu_Car_Enabled())
        {
            if (!Menu_Car_Enabled())
            {
                target_speed = 0.0f;
                target_yaw = yaw;
                pwm_l = 0.0f;
                pwm_r = 0.0f;
                base_start_speed = 0.0f;

                pid_left.clear();
                pid_right.clear();
                pid_angle.clear();
                motor_set_speed(0, 0);
                publish_wheel_control_telemetry();
                return;
            }
        }


        // 三种模式最终都只发布左右轮目标速度，后面的速度内环保持完全一致。
        float target_speed_l = 0.0f;
        float target_speed_r = 0.0f;
        if constexpr (k_speed_pid_tuning_mode)
        {
            // 速度环调参：绕过视觉和角度环，两轮使用相同目标；滑块0/1/2=速度/Kp/Ki。
            target_speed = std::clamp(get_online_param(0), 0.0f, k_speed_tuning_target_max);
            const float speed_kp = std::clamp(get_online_param(1), 0.0f, k_speed_tuning_gain_max);
            const float speed_ki = std::clamp(get_online_param(2), 0.0f, k_speed_tuning_gain_max);
            pid_left.set_pid(speed_kp, speed_ki, 0.0f);
            pid_right.set_pid(speed_kp, speed_ki, 0.0f);

            target_yaw = yaw;
            target_speed_l = target_speed;
            target_speed_r = target_speed;
        }
        else if constexpr (k_angle_pid_tuning_mode)
        {
            // 角度环调参：滑块0/1/2/3/4=绝对yaw/基础速度/Kp/Ki/Kd。
            // 固定 steer 限幅且允许负轮速，基础速度为 0 时可用 +steer/-steer 原地转向。
            static float last_tuning_target_yaw = 0.0f;
            static bool first_tuning_cycle = true;
            target_yaw = std::clamp(get_online_param(0),
                                    -k_angle_tuning_target_limit_deg,
                                    k_angle_tuning_target_limit_deg);
            target_speed = std::clamp(get_online_param(1), 0.0f, k_speed_tuning_target_max);
            const float angle_kp = std::clamp(get_online_param(2), 0.0f, k_speed_tuning_gain_max);
            const float angle_ki = std::clamp(get_online_param(3), 0.0f, k_speed_tuning_gain_max);
            const float angle_kd = std::clamp(get_online_param(4), 0.0f, k_speed_tuning_gain_max);
            const bool target_changed = std::fabs(target_yaw - last_tuning_target_yaw) > 1e-4f;
            pid_angle.set_pid(angle_kp, angle_ki, angle_kd);

            // 目标阶跃只屏蔽首周期 D；P、I 仍正常计算，后续 D 用于抑制 yaw 摆动。
            if (first_tuning_cycle || target_changed)
            {
                pid_angle.suppress_derivative_once();
            }
            first_tuning_cycle = false;
            const float steer = pid_angle.calc(target_yaw,
                                               yaw,
                                               control_dt,
                                               k_angle_tuning_steer_limit);
            target_speed_l = target_speed + steer;
            target_speed_r = target_speed - steer;
            last_tuning_target_yaw = target_yaw;
        }
        else
        {
            // 正式巡线模式只短暂持锁复制视觉快照，PID 和电机 I/O 均在锁外执行。
            float local_vision_target_yaw = yaw;
            TrackInfo local_track_info;
            {
                std::lock_guard<std::mutex> lock(g_vision_result_mutex);
                local_vision_target_yaw = vision_target_yaw;
                local_track_info = g_track_info;
            }

            // 速度策略输出基础轮速；角度 PID 输出同单位的单轮速度偏移 steer。
            const float base_speed = calc_base_speed(local_track_info);
            target_speed = base_speed;
            target_yaw = local_vision_target_yaw;
            // 正式模式按基础速度动态限制转向权限，避免低速/弯道时差速过度。
            constexpr float k_max_steer_ratio = 0.60f;
            const float steer_limit = base_speed * k_max_steer_ratio;
            const float steer = pid_angle.calc(local_vision_target_yaw, yaw, control_dt, steer_limit);
            // 比赛模式不允许内轮倒转；正 steer 表示左轮加速、右轮减速，车辆向右转。
            target_speed_l = std::max(0.0f, base_speed + steer);
            target_speed_r = std::max(0.0f, base_speed - steer);
        }

        // 左右速度内环把轮速误差转换为 PWM 百分比，最终限幅与 PID 内部限幅使用同一常量。
        pwm_l = pid_left.calc(target_speed_l, speed1, control_dt);
        pwm_r = pid_right.calc(target_speed_r, speed2, control_dt);
        pwm_l = std::clamp(pwm_l, -k_speed_pwm_limit_percent, k_speed_pwm_limit_percent);
        pwm_r = std::clamp(pwm_r, -k_speed_pwm_limit_percent, k_speed_pwm_limit_percent);
        motor_set_speed((int)pwm_l, (int)pwm_r);
        publish_wheel_control_telemetry();
    }

    // 约 1 秒发布一次低频状态标志；tick 不清零，保持调度时间轴连续。
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
   nice(19);// nice(19) 将此线程优先级降到最低，绝不抢占控制算力
    constexpr uint32_t k_min_send_interval_us = 16666; // 60fps
    constexpr uint32_t k_max_send_interval_us = 500000;
    uint32_t send_interval_us = k_min_send_interval_us;
    // 自适应退避的图传发送循环
    while (true)
    {
        tcp_update_task(); // 上报速度曲线并接收在线调参滑块
        tcp_camera_snapshot_debug_image();
        seekfree_assistant_camera_send();
        if (tcp_camera_send_failed())           // 发送失败
        {
            send_interval_us = std::min<uint32_t>(send_interval_us + 100000, k_max_send_interval_us); // 间隔 +100ms，上限封顶 500ms，防止网络拥塞时疯狂重试
        }
        else
        {
            send_interval_us = (send_interval_us > k_min_send_interval_us + 20000)
                               ? (send_interval_us - 20000)
                               : k_min_send_interval_us;
        }
        usleep(send_interval_us);
    }
}

// 调度器初始化
void scheduler_init()
{
    // 初始化时间戳
    last_time = std::chrono::steady_clock::now();
    // 启动全车唯一的心脏，5ms 跳动一次
    master_timer.init_ms(5, master_scheduler_callback);
    // printf("中央调度器已启动 (基准周期: 5ms)\n");
    // 启动 TCP 后台线程并将其分离(detach)
    std::thread comm_thread(tcp_background_thread);
    comm_thread.detach();
    printf("系统启动：5ms控制心脏 + 30ms异步通信线程 已就绪！\n");
}
