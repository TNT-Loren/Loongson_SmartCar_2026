
#include "zf_common_headfile.hpp"
#include "motor.hpp"
// 定义全局结构体变量（参考示例）
struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;

// 定义全局驱动对象（参考示例）
zf_driver_gpio drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm drv8701e_pwm_2(PWM_2_PATH);

// 定义全局控制变量（参考示例）
int8 duty = 0;
bool dir = true;

// 函数实现
void motor_init()
{
    // drv8701e_dir_1.set_path(DIR_1_PATH, O_RDWR);
    // drv8701e_dir_2.set_path(DIR_2_PATH, O_RDWR);
        // 获取PWM信息（参考示例main中的代码）
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);
}

// 
void motor_set_speed(int duty_val_left, int duty_val_right)
{
    // 逻辑保持不变
    if (duty_val_left >= 0)
    {
        drv8701e_dir_1.set_level(1);
        drv8701e_pwm_1.set_duty(duty_val_left * (MOTOR1_PWM_DUTY_MAX / 100));
    }
    else
    {
        drv8701e_dir_1.set_level(0);
        drv8701e_pwm_1.set_duty(-duty_val_left * (MOTOR1_PWM_DUTY_MAX / 100));
    }

    if (duty_val_right >= 0)
    {
        drv8701e_dir_2.set_level(1);
        drv8701e_pwm_2.set_duty(duty_val_right * (MOTOR2_PWM_DUTY_MAX / 100));
    }
    else
    {
        drv8701e_dir_2.set_level(0);
        drv8701e_pwm_2.set_duty(-duty_val_right * (MOTOR2_PWM_DUTY_MAX / 100));
    }
}

void motor_stop()
{
    // 停止电机（参考示例cleanup函数）
    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);
}
