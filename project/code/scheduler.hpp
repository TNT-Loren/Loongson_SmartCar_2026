#ifndef __SCHEDULER_HPP__
#define __SCHEDULER_HPP__

#include "zf_common_headfile.hpp"
#include <atomic>


// 声明：告诉外部有一个主定时器和打印标志位
extern zf_driver_pit master_timer;
extern std::atomic<uint8_t> need_print;

extern float target_yaw; // 声明一个全局变量，用于存储在线调试的目标速度
extern float target_speed; // 声明一个全局变量，用于存储在线调试的目标速度

extern float pwm_l; // 声明一个全局变量，用于存储左轮 PWM 输出
extern float pwm_r; // 声明一个全局变量，用于存储右轮 PWM 输出

// 暴露出调度器初始化接口
void scheduler_init();

#endif