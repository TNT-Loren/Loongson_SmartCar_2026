#include "speed_strategy.hpp"
#include <algorithm>
#include <cmath>

TrackInfo g_track_info;

namespace
{
    // 基础目标车速配置
    // constexpr float k_speed_straight = 150.0f;
    // constexpr float k_speed_curve = 125.0f;
    // constexpr float k_speed_sharp = 110.0f;
    // constexpr float k_speed_lost = 90.0f;

    // constexpr float k_speed_straight = 130.0f;
    // constexpr float k_speed_curve = 115.0f;
    // constexpr float k_speed_sharp = 105.0f;
    // constexpr float k_speed_lost = 90.0f;

    constexpr float k_speed_straight = 120.0f;
    constexpr float k_speed_curve = 90.0f;
    constexpr float k_speed_sharp = 80.0f;
    constexpr float k_speed_circle = 80.0f;
    constexpr float k_speed_lost = 80.0f;

    // constexpr float k_speed_straight = 90.0f;
    // constexpr float k_speed_curve = 80.0f;
    // constexpr float k_speed_sharp = 75.0f;
    // constexpr float k_speed_circle = 75.0f;
    // constexpr float k_speed_lost = 50.0f;

    // 加减速步长限制（非对称平滑）
    constexpr float k_speed_up_step = 8.0f;
    constexpr float k_speed_down_step = 10.0f;
    // deviation 现在是 pure pursuit 的 alpha 角度，不是历史归一化横偏。
    constexpr float k_large_alpha_slowdown_deg = 25.0f;// 角度误差过大时的强制降速幅度（不考虑场景，仅按角度误差单因素调整速度）
}

float calc_base_speed(const TrackInfo &info)
{
    static float base_speed = 120.0f; // 初始起步速度

    float target = k_speed_straight;

    // 1. 识别基础场景定目标速度
    if (info.scene == TrackScene::LostLine)
    {
        target = k_speed_lost;
        Set_Beepfreq(3);
        //  test3 = 0;
    }
    else if (info.scene == TrackScene::SharpCurve)
    {
       target = k_speed_sharp;
        // test3 = 1;
    }
    else if (info.scene == TrackScene::Circle)
    {
        target = k_speed_circle;
    }
        
    else if (info.scene == TrackScene::GentleCurve)
    {
    target = k_speed_curve;
  //  test3 = 2;
    }
     else
    {
        target = k_speed_straight;
       // test3 = 3;
    }
        

    // // 2. 安全兜底：pure pursuit 角度误差过大时，无论直道弯道均强制降速
    // if (std::fabs(info.deviation) > k_large_alpha_slowdown_deg)
    //     target -= 8.0f;

    // 3. 动态平滑限幅（入弯急刹，出弯缓加）
    float delta = target - base_speed;
    delta = std::clamp(delta, -k_speed_down_step, k_speed_up_step);
    base_speed += delta;

    return base_speed;
}
