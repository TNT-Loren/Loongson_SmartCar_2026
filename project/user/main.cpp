#include "zf_common_headfile.hpp"
#include <iostream>
#include "scheduler.hpp" // 引入中央调度器
// 测试  3.25
/*
    速度是160的时候已经很快了0.72 0.16    0.6   0.2
    角度环：3.5 0.3    0.2
*/
//===================================================
void cleanup();
void sigint_handler(int signum);

int main(int, char **)
{
    imu_init();
    Encoder_Init();
    motor_init();
    // if (tcp_debug_init("192.168.31.20", 8086))
    // {
    //     //tcp_bind_variables(&target_yaw, &yaw);
    //    //  tcp_bind_variables(&speed1, &speed2);
    // }

    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    // 启动中央大脑！全车所有模块开始按时间片同步运转
    scheduler_init();
    while (1)
    {
        if (need_print.load() == 1)
        {
            need_print.store(0);
            std::cout << "pwm_l: " << pwm_l << " pwm_r: " << pwm_r << std::endl;
        }
        system_delay_ms(20);
    }
}
//================================================================
void sigint_handler(int signum)
{
    master_timer.stop();
    printf("收到Ctrl+C,程序即将退出\n");
    motor_stop();
    exit(0);
}

void cleanup()
{
    // 采用中央调度器，全车只有一个定时器
    master_timer.stop();

    printf("程序异常退出，执行清理操作\n");
    motor_stop();
}


