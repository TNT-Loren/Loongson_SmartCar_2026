#pragma once

#include <chrono>
#include <cstdint>

// inline constexpr float straight_target = 115.0f;
// inline constexpr float gentle_curve_target = 100.0f;
// inline constexpr float sharp_curve_target = 95.0f;
// inline constexpr float obstacle_avoid_target = 70.f;
// inline constexpr float circle_target = 100.0f;
// inline constexpr float lost_line_target = 110.0f;
// inline constexpr PurePursuitSetting straight_pursuit{60.2f, 35.0f};
// inline constexpr PurePursuitSetting gentle_curve_pursuit{52.0f, 40.0f};
// inline constexpr PurePursuitSetting sharp_curve_pursuit{50.0f, 45.0f};
// inline constexpr PurePursuitSetting obstacle_avoid_pursuit{35.0f, 90.0f};
// inline constexpr PurePursuitSetting lost_line_pursuit{65.0f, 15.0f};
// inline constexpr PurePursuitSetting circle_entry_pursuit{50.0f, 40.0f};
// inline constexpr PurePursuitSetting circle_inside_pursuit{45.0f, 34.0f};
// inline constexpr PurePursuitSetting circle_exit_pursuit{45.0f, 30.0f};

// 智能车可标定参数的唯一默认值入口；修改后需重新编译。
// 保持分组顺序：speed -> control -> obstacle -> vision。
namespace smartcar::params
{
namespace speed
{
// 各赛道场景的默认目标轮速。运行时会复制到可由菜单修改的变量中。
inline constexpr float straight_target = 115.0f;
inline constexpr float gentle_curve_target = 100.0f;
inline constexpr float sharp_curve_target = 95.0f;
inline constexpr float obstacle_avoid_target = 70.f;
inline constexpr float circle_target = 100.0f;
inline constexpr float lost_line_target = 110.0f ;



inline constexpr float initial_target = 100.0f;//起步速度
inline constexpr float acceleration_step = 20.0f;//升
inline constexpr float deceleration_step = 25.0f;
inline constexpr float large_alpha_slowdown_threshold_deg = 25.0f;

static_assert(straight_target >= 0.0f && gentle_curve_target >= 0.0f &&
              sharp_curve_target >= 0.0f && obstacle_avoid_target >= 0.0f &&
              circle_target >= 0.0f && lost_line_target >= 0.0f,
              "Speed targets must be non-negative");
static_assert(acceleration_step > 0.0f && deceleration_step > 0.0f,
              "Speed smoothing steps must be positive");
}

namespace control
{
// 普通视觉修正与大幅反向换向的 steer 最大变化率，单位：steer/s。
inline constexpr float k_vision_steer_normal_rate_per_second = 2000.0f;
inline constexpr float k_vision_steer_reversal_rate_per_second = 2600.0f;
// 反向请求幅度达到该值后，才使用更快的换向变化率。
inline constexpr float k_vision_steer_reversal_threshold = 30.0f;

static_assert(k_vision_steer_normal_rate_per_second > 0.0f &&
              k_vision_steer_reversal_rate_per_second > 0.0f,
              "Vision steer rate limits must be positive");
static_assert(k_vision_steer_reversal_threshold >= 0.0f,
              "Vision steer reversal threshold must be non-negative");
}

namespace obstacle
{
// 减速及等待分类阶段。
//状态一：
inline constexpr float decelerate_target_speed = -200.0f;//制动目标速度
inline constexpr float decelerate_exit_speed = 10.0f;//制动完成阈值
inline constexpr float warning_hold_speed = 20.0f;//低速行驶
inline constexpr auto warning_result_timeout = std::chrono::milliseconds{2000};//等待分类超时
inline constexpr auto decelerate_timeout = std::chrono::milliseconds{2000};//减速超时

// 定角转向阶段。负 yaw 为左转，正 yaw 为右转。
//状态二：
inline constexpr float turn_target_speed = 30.0f;//转向目标速度
// 目标是实际需要打出的状态二角度，不再用“大目标角-大容差”模拟小转角。
inline constexpr float left_turn_relative_yaw_deg = -6.0f;//左转相对 yaw 角度
inline constexpr float right_turn_relative_yaw_deg = 6.0f;//右转相对 yaw 角度
inline constexpr float turn_yaw_tolerance_deg = 1.0f;//转向 yaw 容差角度
// 保持两轮都尽量向前，避免 turn_target_speed=30 时变成单轮原地转。
inline constexpr float turn_steer_limit = 22.0f;//转向差速限制
// 状态二按进入时的弯向派生目标角，避免为三类场景增加独立调参项。
inline constexpr float same_direction_turn_scale = 1.5f;
inline constexpr float opposite_direction_turn_scale = 2.0f / 3.0f;
inline constexpr auto turn_timeout = std::chrono::milliseconds{1000};//转向超时

// 沿边绕行及安全超时。
inline constexpr bool use_edge_forward_preview = true;
inline constexpr float edge_lookahead_distance_px = 13.0f;//边线前瞻距离
inline constexpr float edge_midline_offset_px = -5.0f;//无
inline constexpr auto edge_follow_duration = std::chrono::milliseconds{1000};
inline constexpr auto reentry_cooldown = std::chrono::milliseconds{500}; // 绕行结束后再次接受请求前的冷却时间。
inline constexpr auto total_timeout = std::chrono::milliseconds{3000};

static_assert(decelerate_exit_speed >= 0.0f,
              "Obstacle deceleration exit speed must be non-negative");
static_assert(left_turn_relative_yaw_deg < 0.0f &&
              right_turn_relative_yaw_deg > 0.0f,
              "Obstacle turn yaw signs must match their directions");
static_assert(turn_yaw_tolerance_deg >= 0.0f &&
              turn_yaw_tolerance_deg <= 180.0f,
              "Obstacle yaw tolerance must be in [0, 180]");
static_assert(-left_turn_relative_yaw_deg > turn_yaw_tolerance_deg &&
                  right_turn_relative_yaw_deg > turn_yaw_tolerance_deg,
              "Obstacle turn target must be larger than its yaw tolerance");
static_assert(turn_target_speed > turn_steer_limit,
              "Obstacle turn steer limit must keep both wheel targets forward");
static_assert(same_direction_turn_scale > 0.0f &&
                  opposite_direction_turn_scale > 0.0f,
              "Obstacle turn direction scales must be positive");
static_assert(edge_lookahead_distance_px > 0.0f,
              "Obstacle edge lookahead must be positive");
}

namespace vision
{
namespace tracking
{
inline constexpr std::uint8_t max_lost_frame_count = 5;
inline constexpr int lost_line_count_threshold = 30;
inline constexpr std::uint8_t search_top_stop_row = 0;
inline constexpr std::uint8_t start_black_confirm_count = 2;

static_assert(max_lost_frame_count > 0, "Lost-frame limit must be positive");
static_assert(lost_line_count_threshold > 0, "Lost-line threshold must be positive");
static_assert(start_black_confirm_count > 0, "Black-point confirmation must be positive");
}

namespace circle
{
inline constexpr int left_short_column_fallback_x = 60;
inline constexpr int right_short_column_fallback_x = 99;
inline constexpr int short_column_fallback_y = 20;
inline constexpr int state2_no_monotonicity_timeout_frames = 20;
inline constexpr int entry_confirm_frames = 3;
inline constexpr auto state1_to_state2_cooldown =
    std::chrono::milliseconds{50};//由状态一到状态二的冷却时间

inline constexpr auto state1_entry_detected_timeout =
    std::chrono::milliseconds{4000};
inline constexpr auto state2_wait_upper_arc_timeout =
    std::chrono::milliseconds{3000};
inline constexpr auto state3_find_arc_timeout =
    std::chrono::milliseconds{3000};
inline constexpr auto state4_exit_trend_timeout =
    std::chrono::milliseconds{5000};
inline constexpr auto state5_repair_exit_line_timeout =
    std::chrono::milliseconds{3000};
inline constexpr auto state6_recover_track_timeout =
    std::chrono::milliseconds{3000};
inline constexpr auto normal_exit_reentry_cooldown =
    std::chrono::milliseconds{2000};

static_assert(entry_confirm_frames > 0, "Circle entry confirmation must be positive");
static_assert(state2_no_monotonicity_timeout_frames > 0,
              "Circle state-2 fallback timeout must be positive");
}

namespace ipm
{
// 原图坐标到 IPM 坐标的单应性矩阵。
inline constexpr double image_to_ipm_matrix[3][3] = {
    {3.88086826550718, 6.83393912629936, -221.200998709152},
    {0.0710985800665834, 13.1611692370406, -252.156090767037},
    {0.00156260615530949, 0.0848223384740814, 1.0},
};

// 视觉侧 IPM 边线、中线构建。
inline constexpr float edge_resample_distance_px = 3.9f;
inline constexpr float break_distance_px = 18.0f;
inline constexpr int midline_tangent_span_points = 2;
inline constexpr float default_midline_offset_px = 22.5f;
inline constexpr float front_anchor_x = 79.0f;
inline constexpr float front_anchor_y = 119.0f;
inline constexpr float front_anchor_max_gap_px = 36.0f;

// 控制侧对 IPM 中线的补全、重采样及车身参考点。
inline constexpr float control_resample_distance_px = 3.0f;
inline constexpr std::uint16_t control_max_points = 35;
inline constexpr float vehicle_reference_x = 79.0f;
inline constexpr float vehicle_reference_y = 138.0f;

struct PurePursuitSetting
{
    float lookahead_distance_px;
    float max_alpha_deg;
};

// inline constexpr PurePursuitSetting straight_pursuit{40.2f, 35.0f};
// inline constexpr PurePursuitSetting gentle_curve_pursuit{37.0f, 40.0f};
// inline constexpr PurePursuitSetting sharp_curve_pursuit{34.0f, 45.0f};
// inline constexpr PurePursuitSetting obstacle_avoid_pursuit{35.0f, 90.0f};
// inline constexpr PurePursuitSetting lost_line_pursuit{57.0f, 20.0f};
// inline constexpr PurePursuitSetting circle_entry_pursuit{44.0f, 40.0f};
// inline constexpr PurePursuitSetting circle_inside_pursuit{44.0f, 34.0f};
// inline constexpr PurePursuitSetting circle_exit_pursuit{44.0f, 30.0f};
inline constexpr PurePursuitSetting straight_pursuit{60.2f-15.0f, 35.0f};
inline constexpr PurePursuitSetting gentle_curve_pursuit{52.0 - 15.0f, 40.0f};
inline constexpr PurePursuitSetting sharp_curve_pursuit{50.0 - 15.0f, 45.0f};
inline constexpr PurePursuitSetting obstacle_avoid_pursuit{35.0 - 15.0f, 90.0f};
inline constexpr PurePursuitSetting lost_line_pursuit{65.0 - 15.0f, 15.0f};
inline constexpr PurePursuitSetting circle_entry_pursuit{50.0f, 40.0f};
inline constexpr PurePursuitSetting circle_inside_pursuit{45.0f, 34.0f};
inline constexpr PurePursuitSetting circle_exit_pursuit{45.0f, 30.0f};

static_assert(edge_resample_distance_px > 0.0f &&
              control_resample_distance_px > 0.0f,
              "IPM resample distances must be positive");
static_assert(break_distance_px > edge_resample_distance_px,
              "IPM break distance must exceed the resample distance");
static_assert(midline_tangent_span_points > 0, "IPM tangent span must be positive");
static_assert(control_max_points > 1, "IPM control path needs at least two points");

}
}
}
