#include "pid.hpp"
#include <cmath> // 用于 std::abs

IncrementalPID pid_left(0.6f, 0.2f, 0.0f, 60.0f);
IncrementalPID pid_right(0.6f, 0.2f, 0.0f, 60.0f);
PositionalPID pid_angle(3.5f, 0.3f, 0.2f, 100.0f, 30.0f);

/*
当你写下 IncrementalPID speed_pid; 的那一瞬间，
这个函数会自动运行。它负责把所有误差、输出都清零，
确保你的 PID 刚诞生时是干净的，不会带着上一次运行的残余数据。
 */
// 在 pid.cpp 中添加这段代码
IncrementalPID::IncrementalPID(float p, float i, float d, float max_out)
{
    kp = p;
    ki = i;
    kd = d;
    output_limit = max_out;
    deadband = 0.1f; // 默认给一个微小的死区

    // 必须调用一下清除函数，把 error_1, error_2 和 output 初始化为 0
    clear();
}
void IncrementalPID::set_pid(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void IncrementalPID::clear()
{
    error_1 = 0.0f;
    error_2 = 0.0f;
    output = 0.0f;
}

float IncrementalPID::calc(float target, float current, float dt)
{
    float error = target - current;
    if (std::abs(error) < deadband)
        return output;

    float delta_out = kp * (error - error_1) +
                      (ki * dt) * error +
                      (kd / dt) * (error - 2.0f * error_1 + error_2);

    output += delta_out;
    // 限幅
    if (output > output_limit)
        output = output_limit;
    if (output < -output_limit)
        output = -output_limit;

    error_2 = error_1;
    error_1 = error;
    return output;
}
// =========================================================================================
//  位置式 PID (用于中环：角度环 / 外环：位置环)
// ==========================================

PositionalPID::PositionalPID()
{
    kp = 0;
    ki = 0;
    kd = 0;
    output_limit = 100.0f;
    integral_limit = 30.0f;
    deadband = 0.2f; // 角度死区可以稍微大一点点，比如 0.2 度
    clear();
}

PositionalPID::PositionalPID(float p, float i, float d, float max_out, float max_integral)
{
    kp = p;
    ki = i;
    kd = d;
    output_limit = max_out;
    integral_limit = max_integral;
    deadband = 0.2f;
    clear();
}

void PositionalPID::set_pid(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PositionalPID::clear()
{
    last_error = 0.0f;
    integral = 0.0f; // 清空积分池
}

static float wrap180f(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a <= -180.0f)
        a += 360.0f;
    return a;
}

float PositionalPID::calc(float target, float current, float dt)
{
   // float error = target - current;
   float error = wrap180f(target - current);
   // 很好的死区处理：进入死区就直接输出 0，并且刷新 last_error 防止微分暴走
   if (std::abs(error) < deadband)
   {
       // 必须刷新 last_error，防止冲出死区时微分项（D）暴走
       last_error = error;
       // 角度环进入死区，意味着车身已正，必须输出 0 差速让其直行
       return 0.0f;
   }

    integral += (error * dt);
    integral = std::clamp(integral, -integral_limit, integral_limit);

    // 【优化2】：底层除零与极小值保护
    float safe_dt = (dt > 0.0001f) ? dt : 0.005f;
    float derivative = (error - last_error) / safe_dt;

    float output = (kp * error) +
                   (ki * integral) +
                   (kd * derivative);

    output = std::clamp(output, -output_limit, output_limit);
    last_error = error;

    return output;
}