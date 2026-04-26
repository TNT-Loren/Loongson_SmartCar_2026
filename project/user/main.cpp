#include "zf_common_headfile.hpp"
#include "main.hpp"
#include <iostream>
#include "scheduler.hpp" // 引入中央调度器

#include <termios.h>
#include <unistd.h>



//键盘输入相关的全局变量和函数声明 （方便调试）
static struct termios g_old_tio;
static bool g_keyboard_ready = false;
// stty sane   // 恢复终端默认设置的命令，程序退出时会调用
void keyboard_init_simple();
void keyboard_restore_simple();
void keyboard_poll_simple();
// 测试  3.25
/*
    速度是160的时候已经很快了0.72 0.16    0.6   0.2
    角度环：3.5 0.3    0.2
*/

//int test=80;
float test1, test2, test3;
int key_mode = 0; 
//===================================================
void cleanup();
void sigint_handler(int signum);

int main(int, char **)
{
    // esc_init();
    // esc_set_speed_percent(0);
    imu_init();
    Encoder_Init();
    motor_init();
    init_ipm_valid_region(); // 预先计算逆透视有效区域边界，供后续处理使用
        // if (tcp_debug_init("192.168.31.20", 8086))
        // {
        //    //tcp_bind_variables(&target_yaw, &yaw);
        //    //tcp_bind_variables(&speed1, &speed2);
        // }

        if (!(tcp_image_transmission_init("192.168.31.20", 8086)))
    {
        return -1;
    }

    if (uvc_dev.init(UVC_PATH) < 0)
    {
        std::cout << "  1 " << std::endl;
        return -1; // 摄像头初始化失败，直接退出程序
    }
    

    atexit(cleanup);
    signal(SIGINT, sigint_handler);
    keyboard_init_simple();// 初始化简单键盘输入，供调试用

    // 启动中央大脑！全车所有模块开始按时间片同步运转
    scheduler_init();
    uvc_dev.set_auto_exposure(1); // 关闭自动曝光，进入手动模式，才能设置曝光值
    uvc_dev.set_exposure_value(100); // 设置初始曝光值
    while (1)
    {

        //keyboard_poll_simple();// 轮询键盘输入，供调试用
       // esc_set_speed_percent(test1);
        //image_test();
        image_process();
        // motor_set_speed(0, 0);
        if (need_print.load() == 1)
        {
            static int count = 0;
            if(++count==5)
            {
                count = 0;
            }
            // std::cout << "test1: " << test1 << "test2: " << test2 << "test3: " << test3 << std::endl;
            // need_print.store(0);
            std::cout << " "
                      << static_cast<int>(start_point_l[0]) << ","
                      << static_cast<int>(start_point_l[1]) << " ,"
                      << static_cast<int>(start_point_r[0]) << " ,"
                      << static_cast<int>(start_point_r[1]) << std::endl;
            // std::cout << "pwm_l: " << pwm_l << " pwm_r: " << pwm_r << std::endl;
            //  std::cout << "speed1: " << speed1 << " speed2: " << speed2 << "  yaw: " << yaw <<  std::endl;
             need_print.store(0);
        }
        system_delay_ms(10);
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

    keyboard_restore_simple();// 恢复键盘设置，防止程序退出后终端异常
    // 采用中央调度器，全车只有一个定时器
    master_timer.stop();

    printf("程序异常退出，执行清理操作\n");
    motor_stop();
}

//================================================================// 键盘输入相关的函数实现
void keyboard_init_simple()
{
    if (!isatty(STDIN_FILENO))
    {
        std::cout << "stdin 不是终端，键盘控制不可用" << std::endl;
        return;
    }

    if (tcgetattr(STDIN_FILENO, &g_old_tio) == -1)
    {
        perror("tcgetattr");
        return;
    }

    struct termios new_tio = g_old_tio;

    // 关闭规范模式和回显
    // 保留 ISIG，这样 Ctrl+C 还能正常触发 SIGINT
    new_tio.c_lflag &= ~(ICANON | ECHO);

    // 非阻塞读：没有字符时立刻返回
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) == -1)
    {
        perror("tcsetattr");
        return;
    }

    g_keyboard_ready = true;
}

// 按键处理函数
//     这个函数就是最核心的“简单按一个键做简单操作”：
void keyboard_poll_simple()
{
    if (!g_keyboard_ready)
    {
        return;
    }

    char ch = 0;

    // 用 while 把这一轮积压的按键都读掉
    while (read(STDIN_FILENO, &ch, 1) > 0)
    {
        switch (ch)
        {
        case 'a':
        case 'A':
            test1 += 10.0f;
            std::cout << "test1: " << test1 << std::endl;
            break;

        case 'b':
        case 'B':
            key_mode = (key_mode + 1) % 3; // 切换模式
             std::cout << "切换到模式 " << key_mode << std::endl;
            break;

        // case 'r':
        // case 'R':
        //     test = 0.0f;
        //     std::cout << "test reset -> " << test << std::endl;
        //     break;

        // case 's':
        // case 'S':
        //     motor_stop();
        //     std::cout << "motor stop" << std::endl;
        //     break;

        default:
            break;
        }
    }
}

void keyboard_restore_simple()
{
    if (!g_keyboard_ready)
    {
        return;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
    g_keyboard_ready = false;
}
