#include "zf_common_headfile.hpp"
#include "main.hpp"
#include <iostream>
#include "scheduler.hpp" // 引入中央调度器
#include "beep.hpp"

#include <cmath>
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
float test1, test2, test3, test4;
int key_mode = 0;
//===================================================
void cleanup();
void sigint_handler(int signum);

const char *track_scene_name(TrackScene scene)
{
    switch (scene)
    {
    case TrackScene::Straight:
        return "Straight";
    case TrackScene::GentleCurve:
        return "GentleCurve";
    case TrackScene::SharpCurve:
        return "SharpCurve";
    case TrackScene::Circle:
        return "Circle";
    case TrackScene::LostLine:
        return "LostLine";
    default:
        return "Unknown";
    }
}

int main(int, char **)
{
    // esc_init();
    //esc_set_speed_percent(0);
     imu_init();
     Encoder_Init();
     Beep_Init();
     motor_init();
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

    uvc_dev.set_auto_exposure(1); // 关闭自动曝光，进入手动模式，才能设置曝光值
    uvc_dev.set_exposure_value(100); // 设置初始曝光值
    {
        std::lock_guard<std::mutex> lock(g_vision_result_mutex);
        vision_target_yaw = yaw;
    }
    image_test(); // 启动控制调度前先准备一帧视觉结果，避免电机用默认视觉目标起步

    // 启动中央大脑！全车所有模块开始按时间片同步运转
    scheduler_init();
    //test_front_point_patch_tools();

    while (1)
    {

        keyboard_poll_simple();// 轮询键盘输入，供调试用
       // esc_set_speed_percent(test1);
        fps_timer_start();
        image_test();
        fps_timer_end();
        // motor_set_speed(30, 30);
        if (need_print.load() == 1)
        {
            TrackInfo track_info;
            float local_target_yaw = 0.0f;
            {
                std::lock_guard<std::mutex> lock(g_vision_result_mutex);
                track_info = g_track_info;
                local_target_yaw = vision_target_yaw;
            }

            std::cout << "  scene: " << track_scene_name(track_info.scene)
            <<"   fps: " << g_fps_value
                    //   << "  target_yaw: " << local_target_yaw
                    //   << "  abs_deviation: " << std::fabs(track_info.deviation)
                      << std::endl;
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
    Beep(Off);
    motor_stop();
    exit(0);
}

void cleanup()
{

    keyboard_restore_simple();// 恢复键盘设置，防止程序退出后终端异常
    // 采用中央调度器，全车只有一个定时器
    master_timer.stop();

    printf("程序异常退出，执行清理操作\n");
    Beep(Off);
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
            cycle_test_midline_mode();
            std::cout << "test midline mode -> "
                      << test_midline_mode_name(g_test_midline_mode)
                      << std::endl;
            break;

        case 'b':
        case 'B':
            std::cout << "当前版本已切回 image_test，巡线模式按键暂不使用" << std::endl;
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
