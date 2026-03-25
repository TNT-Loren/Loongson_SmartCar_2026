#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_common_headfile.hpp"

// =================================================================================
// 宏定义：硬件路径与限幅
// =================================================================================
#define PWM_1_PATH ZF_PWM_MOTOR_1
#define DIR_1_PATH ZF_GPIO_MOTOR_1
#define PWM_2_PATH ZF_PWM_MOTOR_2
#define DIR_2_PATH ZF_GPIO_MOTOR_2

#define MOTOR_MAX_DUTY_PERCENT (30) // 限制最大占空比为 30%，保护电机

// =================================================================================
// 外部全局变量声明 (供 PID 和 调度器使用)
// =================================================================================

// 驱动对象声明
extern zf_driver_gpio drv8701e_dir_1;
extern zf_driver_gpio drv8701e_dir_2;
extern zf_driver_pwm drv8701e_pwm_1;
extern zf_driver_pwm drv8701e_pwm_2;

// PWM 信息结构体
extern struct pwm_info drv8701e_pwm_1_info;
extern struct pwm_info drv8701e_pwm_2_info;

// =================================================================================
// 宏定义：快速获取底层 PWM 最大计数值
// =================================================================================
#define MOTOR1_PWM_DUTY_MAX (drv8701e_pwm_1_info.duty_max)
#define MOTOR2_PWM_DUTY_MAX (drv8701e_pwm_2_info.duty_max)

// =================================================================================
// 接口函数声明
// =================================================================================
/**
 * @brief 初始化电机硬件 (获取底层计数值)
 */
void motor_init();

/**
 * @brief 设置电机占空比
 * @param duty_val_left  左轮占空比 (-100 到 100)
 * @param duty_val_right 右轮占空比 (-100 到 100)
 */
void motor_set_speed(int duty_val_left, int duty_val_right);

/**
 * @brief 电机急停 (占空比直接清零)
 */
void motor_stop();

#endif
