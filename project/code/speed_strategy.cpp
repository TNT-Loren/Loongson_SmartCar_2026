#include "speed_strategy.hpp"
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

  //  基础目标车速配置
    //  float k_speed_straight = 130.0f;//aaa
    //  float k_speed_curve = 105.0f;
    //  float k_speed_sharp = 95.0f;
    //  float k_speed_obstacle_avoid = 40.0f;//绕行
    //  float k_speed_circle = 105.0f;
    //  float k_speed_lost = 95.0f;

     float k_speed_straight = 170.0f; //716
     float k_speed_curve = 150.0f;
     float k_speed_sharp = 140.0f;
     float k_speed_obstacle_avoid = 40.0f; // 绕行
     float k_speed_circle = 140.0f;
     float k_speed_lost = 160.0f;
    //  float k_speed_straight = 170.0f*1.3f; //
    //  float k_speed_curve = 140.0f * 1.3f;
    //  float k_speed_sharp = 135.0f * 1.3f;
    //  float k_speed_obstacle_avoid = 40.0f; // 绕行
    //  float k_speed_circle = 137.0f * 1.3f;
    //  float k_speed_lost = 130.0f * 1.3f;

     // float k_speed_straight = 100.0f; // 713
     // float k_speed_curve = 80.0f;
     // float k_speed_sharp = 80.0f;
     // float k_speed_obstacle_avoid = 40.0f; // 绕行
     // float k_speed_circle = 80.0f;
     // float k_speed_lost = 80.0f;

     // 加减速步长限制（非对称平滑）
     float k_speed_up_step = 8.0f;
     float k_speed_down_step = 10.0f;
     // deviation 现在是 pure pursuit 的 alpha 角度，不是历史归一化横偏。
     float k_large_alpha_slowdown_deg = 25.0f; // 角度误差过大时的强制降速幅度（不考虑场景，仅按角度误差单因素调整速度）

     float base_start_speed = 130.0f;
     float calc_base_speed(const TrackInfo &info)
     {

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
         float delta = target - base_start_speed;
         delta = std::clamp(delta, -k_speed_down_step, k_speed_up_step);
         base_start_speed += delta;

         return base_start_speed;
     }
