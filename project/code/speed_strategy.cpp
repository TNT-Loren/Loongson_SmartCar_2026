#include "speed_strategy.hpp"
#include "smartcar_params.hpp"

#include <algorithm>
#include <cmath>

TrackInfo g_track_info;

// 30 0.62 m / s 180 RPM
// 40 0.82 m / s 240 RPM 
// 60 1.23 m / s 360 RPM
// 80 1.64 m / s 480 RPM 
// 100 2.05 m / s 600 RPM 
// 120 2.46 m / s 720 RPM
// 140 2.87 m / s 840 RPM
// 146 约 3.0 m / s 876 RPM

// 这些是菜单可修改的运行时副本；启动默认值统一来自 smartcar_params.hpp。
float k_speed_straight = smartcar::params::speed::straight_target;
float k_speed_curve = smartcar::params::speed::gentle_curve_target;
float k_speed_sharp = smartcar::params::speed::sharp_curve_target;
float k_speed_obstacle_avoid = smartcar::params::speed::obstacle_avoid_target;
float k_speed_circle = smartcar::params::speed::circle_target;
float k_speed_lost = smartcar::params::speed::lost_line_target;
float k_speed_up_step = smartcar::params::speed::acceleration_step;
float k_speed_down_step = smartcar::params::speed::deceleration_step;
float k_large_alpha_slowdown_deg = smartcar::params::speed::large_alpha_slowdown_threshold_deg;
float base_start_speed = smartcar::params::speed::initial_target;

float calc_base_speed(const TrackInfo &info)
{
    float target = k_speed_straight;

    if (info.scene == TrackScene::LostLine)
    {
        target = k_speed_lost;
        Set_Beepfreq(3);
    }
    else if (info.scene == TrackScene::SharpCurve)
    {
        target = k_speed_sharp;
    }
    else if (info.scene == TrackScene::ObstacleAvoid)
    {
        target = k_speed_obstacle_avoid;
    }
    else if (info.scene == TrackScene::Circle)
    {
        target = k_speed_circle;
    }
    else if (info.scene == TrackScene::GentleCurve)
    {
        target = k_speed_curve;
    }

    // 非对称平滑：入弯快速降速，出弯逐步加速。
    float delta = target - base_start_speed;
    delta = std::clamp(delta, -k_speed_down_step, k_speed_up_step);
    base_start_speed += delta;
    return base_start_speed;
}
