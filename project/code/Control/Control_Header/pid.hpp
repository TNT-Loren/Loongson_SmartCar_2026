#ifndef __PID_HPP__
#define __PID_HPP__

#include "zf_common_headfile.hpp"

//============================================================增量式PID（速度环内环）
class IncrementalPID
{
private:
    float kp, ki, kd;
    float error_1, error_2; // e(k-1), e(k-2)
    float output;           // 当前绝对输出
    float output_limit;     // 限幅
    float deadband;         // 死区

public:
    // 1. 默认构造函数
    IncrementalPID();
    // 2. 带参数的构造函数（对应你 pid.cpp 顶部的初始化）
    IncrementalPID(float p, float i, float d, float max_out);

    void set_pid(float p, float i, float d);
    void clear();

    // 核心计算：必须包含 dt 参数
    float calc(float target, float current, float dt);
};
// ==========================================
//  位置式 PID (用于中环：角度环 / 外环：位置环)
// ==========================================
class PositionalPID
{
private:
    float kp, ki, kd;
    float last_error;
    float integral;
    float output_limit;
    float integral_limit;
    float deadband;

public:
    PositionalPID();
    PositionalPID(float p, float i, float d, float max_out, float max_integral);

    void set_pid(float p, float i, float d);
    void clear();

    float calc(float target, float current, float dt);

    //用于观测积分项健康度的接口
    float get_integral() const { return integral; }
};

// ==========================================
//  全局对象声明
// ==========================================
extern IncrementalPID pid_left;
extern IncrementalPID pid_right;
// 给中环（角度锁尾）用的 PID 对象
extern PositionalPID pid_angle;




#endif