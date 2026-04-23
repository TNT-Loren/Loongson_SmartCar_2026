#include "speed_strategy.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    // 基础目标车速配置
    constexpr float k_speed_straight = 110.0f;
    constexpr float k_speed_curve = 90.0f;
    constexpr float k_speed_sharp = 80.0f;
    constexpr float k_speed_lost = 75.0f;

    // 加减速步长限制（非对称平滑）
    constexpr float k_speed_up_step = 2.0f;
    constexpr float k_speed_down_step = 3.0f;
}

float calc_base_speed(const TrackInfo &info)
{
    static float base_speed = 100.0f; // 初始起步速度

    float target = k_speed_straight;

    // 1. 识别基础场景定目标速度
    if (info.scene == TrackScene::LostLine)
    {
        target = k_speed_lost;
        test3 = 0;
    }
    else if (info.scene == TrackScene::SharpCurve)
    {
       target = k_speed_sharp;
         test3 = 1;
    }
        
    else if (info.scene == TrackScene::GentleCurve)
    {
    target = k_speed_curve;
    test3 = 2;
    }
     else
    {
        target = k_speed_straight;
        test3 = 3;
    }
        

    // 2. 安全兜底：当横向偏差过大时，无论直道弯道均强制降速
    if (std::fabs(info.deviation) > 0.35f)
        target -= 8.0f;

    // 3. 动态平滑限幅（入弯急刹，出弯缓加）
    float delta = target - base_speed;
    delta = std::clamp(delta, -k_speed_down_step, k_speed_up_step);
    base_speed += delta;

    return base_speed;
}