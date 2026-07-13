#include "zf_common_headfile.hpp"
#include "main.hpp"
#include <iostream>
#include "scheduler.hpp" // 引入中央调度器
#include "tcp.hpp"

// 提供 VSInference 类：色块检测 + NCNN推理 + 跟踪状态机 + 彩色图传输出
#include "../code/vs_inference.hpp"

#include <algorithm>
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

bool g_vs_ready = false;              // VS是否初始化成功（单摄像头时可能失败，降级跳过）

//===================================================
void cleanup();
void sigint_handler(int signum);
void print_ipm_mid_points_snapshot();

float wrap_delta_yaw_for_print(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle <= -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

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
    case TrackScene::ObstacleAvoid:
        return "ObstacleAvoid";
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
    //  esc_init();
    //  esc_set_speed_percent(0);
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
    // VS/NCNN 初始化
    //  g_vs_ready = g_vs.init_smartcar_defaults(&uvc_dev);
    //  if (!g_vs_ready)
    //  {
    //      printf("vs init failed, AI/图传降级跳过\r\n");
    //  }
    //  ===== VS添加结束 =====

    atexit(cleanup);
    signal(SIGINT, sigint_handler);
    keyboard_init_simple();// 初始化简单键盘输入，供调试用

    uvc_dev.set_auto_exposure(1); // 关闭自动曝光，进入手动模式，才能设置曝光值
    uvc_dev.set_exposure_value(100); // 设置初始曝光值
    {
        std::lock_guard<std::mutex> lock(g_vision_result_mutex);
        vision_target_yaw = yaw;
    }
    if (image_test()) // 启动控制调度前先准备一帧视觉结果，避免电机用默认视觉目标起步
    {
        update_control_target();
    }
    else
    {
        publish_lost_line_result();
    }

    // 启动中央大脑！全车所有模块开始按时间片同步运转
    scheduler_init();
    Set_Beepfreq(3);
    // test_front_point_patch_tools();
    std::string result;

    while (1)
    {
        //Set_Beeptime(5000);      // 响 500ms
        keyboard_poll_simple(); // 轮询键盘输入，供调试用
                                // esc_set_speed_percent(test1);
        fps_timer_start();
        if (image_test())
        {
            update_control_target();
        }
        else
        {
            publish_lost_line_result();
        }
        fps_timer_end();
        // ===== VS添加：AI视觉推理 — 复用巡线已抓帧，彩色给AI，不额外等帧 =====
        // if (g_vs_ready)
        // {
        //     cv::Mat color_frame = uvc_dev.get_frame_mjpg();
        //     if (!g_vs.tick_bgr(color_frame))
        //     {
        //         printf("vs.tick error\r\n");
        //     }
        //     else
        //     {
        //         tcp_camera_update_vs_image();
        //     }
        //     if (g_vs.consume_new_result(result))
        //     {
        //         // AI结果映射绕行：武器→左绕  物资→右绕  载具→直行压过
        //         if (result == "武器")
        //         {
        //             trigger_obstacle_avoid(ObstacleAvoidDirection::Left);
        //         }
        //         else if (result == "物资")
        //         {
        //             trigger_obstacle_avoid(ObstacleAvoidDirection::Right);
        //         }
        //         // 载具/交通工具 → 直行，不触发绕行
        //     }
        // }
        // ===== VS添加结束 =====
        if (need_print.load() == 1)
        {
            // TrackInfo track_info;
            // float local_yaw = 0.0f;
            // float local_target_yaw = 0.0f;
            // {
            //     std::lock_guard<std::mutex> lock(g_vision_result_mutex);
            //     track_info = g_track_info;
            //     local_yaw = yaw;
            //     local_target_yaw = vision_target_yaw;
            // }
            // const float relative_yaw_error = wrap_delta_yaw_for_print(local_target_yaw - local_yaw);

            // std::cout << "  scene: " << track_scene_name(track_info.scene)
            //           << "   fps: " << g_fps_value
            //           << "  rel_yaw_error: " << relative_yaw_error
            //           << "  deviation: " << track_info.deviation
            //           << std::endl;
            // for(uint8 i=0;i<MT9V03X_H;i++)
            // {
            //     std::cout << static_cast<int>(Mid_Line[i]) << " ,";
            // }
            // std::cout << "  " << std::endl;
            std::cout << "fps: " << g_fps_value << std::endl;

            // if (g_vs_ready)
            // {
            //     printf("VS FPS: %d\r\n", g_vs.take_fps_count());
            // }
            // std::cout << static_cast<int>(Threshold) << std::endl;

            need_print.store(0);
        }
        system_delay_ms(5);
    }
}

void print_ipm_mid_points_snapshot()// 供调试用的函数：打印一帧 IPM 中线点集的快照到控制台
{
    uint16 mid_points[MT9V03X_H][2] = {};
    uint16 mid_count = 0;
    uint8 current_hightest = 0;
    {
        std::lock_guard<std::mutex> lock(g_image_mutex);
        mid_count = std::min<uint16>(Ipm_Mid_Point_Count, MT9V03X_H);
        current_hightest = hightest;
        for (uint16 i = 0; i < mid_count; ++i)
        {
            mid_points[i][0] = Ipm_Mid_Points[i][0];
            mid_points[i][1] = Ipm_Mid_Points[i][1];
        }
    }

    const IpmMidlineSceneResult scene_result =
        classify_ipm_midline_scene(mid_points, mid_count, current_hightest);

    std::cout << "\n[IPM_MID_POINTS] count=" << mid_count
              << " hightest=" << static_cast<int>(current_hightest)
              << " scene=" << ipm_midline_scene_name(scene_result.scene)
              << " x_span=" << scene_result.x_span
              << " y_span=" << scene_result.y_span
              << " ratio=" << scene_result.x_y_ratio
              << " turns=" << scene_result.turn_count
              << " straightness=" << scene_result.straightness
              << " mean_curv=" << scene_result.mean_abs_curv
              << " max_curv=" << scene_result.max_abs_curv
              << std::endl;
    std::cout << "scores straight=" << scene_result.straight_score
              << " gentle=" << scene_result.gentle_score
              << " s=" << scene_result.s_score
              << " sharp=" << scene_result.sharp_score
              << std::endl;
    for (uint16 i = 0; i < mid_count; ++i)
    {
        std::cout << "(" << mid_points[i][0] << "," << mid_points[i][1] << ")";
        if ((i + 1) % 12 == 0 || i + 1 == mid_count)
        {
            std::cout << std::endl;
        }
        else
        {
            std::cout << " ";
        }
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
            cycle_ipm_reliable_edge_mode();
            std::cout << "test midline mode -> "
                      << test_midline_mode_name(g_test_midline_mode)
                      << ", ipm reliable edge -> "
                      << reliable_edge_mode_name(g_ipm_reliable_edge_mode)
                      << std::endl;
            break;

        case 'b':
        case 'B':
            std::cout << "当前版本已切回 image_test，巡线模式按键暂不使用" << std::endl;
            break;

        case 'd':
        case 'D':
            cycle_debug_view_mode();
            std::cout << "debug view -> "
                      << debug_view_mode_name()
                      << std::endl;
            break;

        case 's':
        case 'S':
            print_ipm_mid_points_snapshot();
            break;

        case 'q':
        case 'Q':
            // 手动模拟“左绕行”：用左边线作为定时临时中线参考。
            trigger_obstacle_avoid(ObstacleAvoidDirection::Left);
            std::cout << "obstacle avoid -> L, offset="
                      << g_ipm_midline_offset_px.load()
                      << std::endl;
            break;

        case 'e':
        case 'E':
            // 手动模拟“右绕行”：用右边线作为定时临时中线参考。
            trigger_obstacle_avoid(ObstacleAvoidDirection::Right);
            std::cout << "obstacle avoid -> R, offset="
                      << g_ipm_midline_offset_px.load()
                      << std::endl;
            break;

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
