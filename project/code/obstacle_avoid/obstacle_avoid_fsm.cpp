#include "obstacle_avoid/obstacle_avoid_fsm.hpp"

#include "event_timer.hpp"
#include "smartcar_params.hpp"

#include <cmath>
#include <mutex>

namespace obstacle_param = smartcar::params::obstacle;

namespace
{
// 本地别名只用于保持状态机语义清晰；唯一调参入口为 smartcar_params.hpp。
// 状态 1A 使用反向速度目标快速制动；最终 PWM 仍受速度环的 +/-55% 上限保护。
constexpr float k_avoid_decelerate_target_speed = obstacle_param::decelerate_target_speed;
// 状态 1B 在等待 AI 最终分类时保持正向低速，避免 -90 目标让车辆继续倒车。
constexpr float k_avoid_warning_hold_speed = obstacle_param::warning_hold_speed;
// 左右轮的有符号速度都不超过该值时，锁存“制动完成”，不再回到 -90。
constexpr float k_avoid_decelerate_exit_speed = obstacle_param::decelerate_exit_speed;
// 红色预警发生后等待最终分类，超时则取消本次绕行。
constexpr auto k_avoid_warning_result_timeout = obstacle_param::warning_result_timeout;
// 方向已确认但 2 秒内仍未降到阈值，按减速超时取消本次绕行。
constexpr auto k_avoid_decelerate_timeout = obstacle_param::decelerate_timeout;

// 状态 2 定角转向时的基础轮速；角度 PID 的 steer 在此基础上形成左右差速。
constexpr float k_avoid_turn_target_speed = obstacle_param::turn_target_speed;
// 状态 2 左绕相对角：进入状态 2 时，以当时 yaw 为零点锁存 -60 度目标。
constexpr float k_avoid_turn_relative_yaw_left = obstacle_param::left_turn_relative_yaw_deg;
// 状态 2 右绕相对角：进入状态 2 时，以当时 yaw 为零点锁存 +60 度目标。
constexpr float k_avoid_turn_relative_yaw_right = obstacle_param::right_turn_relative_yaw_deg;
// 状态 2 正常退出容差：最短 yaw 角差进入配置容差时进入状态 3。
constexpr float k_avoid_turn_yaw_tolerance = obstacle_param::turn_yaw_tolerance_deg;
// 状态 2 角度 PID 输出上限，单位与单轮目标速度相同，不是角度上限。
constexpr float k_avoid_turn_steer_limit = obstacle_param::turn_steer_limit;
// 状态 2 最长等待时间：1 秒内未达到目标角则按转向超时取消本次绕行。
constexpr auto k_avoid_turn_timeout = obstacle_param::turn_timeout;

// 状态 3 沿边线控制持续 1 秒；从进入 FollowEdge 状态的时刻单独开始计时。
constexpr auto k_avoid_edge_follow_duration = obstacle_param::edge_follow_duration;
// 绕行完成或异常退出后的冷却时间；冷却期间拒绝新的 AI/键盘绕行请求。
constexpr auto k_avoid_reentry_cooldown = obstacle_param::reentry_cooldown;
// 正式绕行总超时保护：从进入状态 2 开始计时，不让预警等待消耗后续时间。
constexpr auto k_avoid_total_timeout = obstacle_param::total_timeout;
// 状态 3 从近车端沿连续 IPM 边线累计的预瞄弧长，单位：IPM 像素。
// 该距离同时计入 x/y 变化，不是固定上移的 y 行数。
constexpr float k_obstacle_avoid_edge_lookahead_distance = obstacle_param::edge_lookahead_distance_px;
// 状态 3 从指定可靠边向法线方向生成临时中线时使用的 IPM 像素偏移；
// car_control.cpp 启用直接边线弧长预瞄时，最终 ALP 不使用这条偏移中线。
constexpr float k_avoid_edge_midline_offset_px = obstacle_param::edge_midline_offset_px;

std::mutex g_obstacle_avoid_mutex;
ObstacleAvoidState g_state = ObstacleAvoidState::Idle;
ObstacleAvoidDirection g_direction = ObstacleAvoidDirection::None;
ObstacleAvoidExitReason g_last_exit_reason = ObstacleAvoidExitReason::None;
float g_turn_target_yaw = 0.0f;
bool g_waiting_for_result = false;
bool g_decelerate_brake_completed = false;
std::uint32_t g_transition_sequence = 0;
MonotonicEventTimer g_state_timer;
MonotonicEventTimer g_total_timer;

float wrap_to_180(float angle)
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

bool is_active_state(ObstacleAvoidState state)
{
    return state == ObstacleAvoidState::Decelerate ||
           state == ObstacleAvoidState::TurnToRelativeYaw ||
           state == ObstacleAvoidState::FollowEdge ||
           state == ObstacleAvoidState::ReturnToCenter;
}

void enter_state(ObstacleAvoidState state, MonotonicEventTimer::TimePoint now)
{
    g_state = state;
    g_state_timer.start(now);
    ++g_transition_sequence;
}

void enter_cooldown(ObstacleAvoidExitReason reason,
                    MonotonicEventTimer::TimePoint now)
{
    g_last_exit_reason = reason;
    g_direction = ObstacleAvoidDirection::None;
    g_turn_target_yaw = 0.0f;
    g_waiting_for_result = false;
    g_decelerate_brake_completed = false;
    g_total_timer.reset();
    enter_state(ObstacleAvoidState::Cooldown, now);
}

void start_decelerate(ObstacleAvoidDirection direction,
                      bool waiting_for_result,
                      MonotonicEventTimer::TimePoint now)
{
    g_direction = direction;
    g_last_exit_reason = ObstacleAvoidExitReason::None;
    g_turn_target_yaw = 0.0f;
    g_waiting_for_result = waiting_for_result;
    g_decelerate_brake_completed = false;
    g_total_timer.reset();
    enter_state(ObstacleAvoidState::Decelerate, now);
}

void enter_turn_to_relative_yaw(float current_yaw,
                                MonotonicEventTimer::TimePoint now)
{
    const float relative_yaw =
        g_direction == ObstacleAvoidDirection::Left
            ? k_avoid_turn_relative_yaw_left
            : k_avoid_turn_relative_yaw_right;
    g_turn_target_yaw = wrap_to_180(current_yaw + relative_yaw);
    // 预警阶段有独立超时；正式绕行的 3 秒从此处重新起算。
    g_total_timer.start(now);
    enter_state(ObstacleAvoidState::TurnToRelativeYaw, now);
}

ObstacleAvoidControl make_control(float current_yaw)
{
    ObstacleAvoidControl control;
    control.state = g_state;
    control.direction = g_direction;
    control.last_exit_reason = g_last_exit_reason;
    control.target_yaw = current_yaw;
    control.transition_sequence = g_transition_sequence;

    if (g_state == ObstacleAvoidState::Decelerate)
    {
        control.target_speed = g_decelerate_brake_completed
                                   ? k_avoid_warning_hold_speed
                                   : k_avoid_decelerate_target_speed;
    }
    else if (g_state == ObstacleAvoidState::TurnToRelativeYaw)
    {
        control.target_speed = k_avoid_turn_target_speed;
        control.target_yaw = g_turn_target_yaw;
        control.steer_limit = k_avoid_turn_steer_limit;
    }

    return control;
}
}

bool obstacle_avoid_request(ObstacleAvoidDirection direction)
{
    if (direction == ObstacleAvoidDirection::None)
    {
        return false;
    }

    const auto now = MonotonicEventTimer::Clock::now();
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    if (g_state != ObstacleAvoidState::Idle)
    {
        return false;
    }

    start_decelerate(direction, false, now);
    return true;
}

bool obstacle_avoid_warning_request()
{
    const auto now = MonotonicEventTimer::Clock::now();
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    if (g_state != ObstacleAvoidState::Idle)
    {
        return false;
    }

    start_decelerate(ObstacleAvoidDirection::None, true, now);
    return true;
}

bool obstacle_avoid_confirm(ObstacleAvoidDirection direction)
{
    if (direction == ObstacleAvoidDirection::None)
    {
        return false;
    }

    const auto now = MonotonicEventTimer::Clock::now();
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    if (g_state == ObstacleAvoidState::Decelerate && g_waiting_for_result)
    {
        // 只锁存方向，yaw 必须等到制动完成、真正进入状态 2 时再锁存。
        g_direction = direction;
        g_waiting_for_result = false;
        return true;
    }
    if (g_state == ObstacleAvoidState::Idle)
    {
        // 最终分类比预警更可靠；预警漏掉时仍保留原来的直接绕行能力。
        start_decelerate(direction, false, now);
        return true;
    }
    if (g_state == ObstacleAvoidState::Cooldown &&
        g_last_exit_reason == ObstacleAvoidExitReason::WarningTimeout)
    {
        // 最终分类可能恰好在预警等待超时后的短暂冷却期到达。
        // 这不是重复绕行请求，而是同一目标的权威分类结果。
        start_decelerate(direction, false, now);
        return true;
    }
    return false;
}

bool obstacle_avoid_cancel_warning()
{
    const auto now = MonotonicEventTimer::Clock::now();
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    if (g_state != ObstacleAvoidState::Decelerate || !g_waiting_for_result)
    {
        return false;
    }

    enter_cooldown(ObstacleAvoidExitReason::ClassificationRejected, now);
    return true;
}

ObstacleAvoidControl obstacle_avoid_update(float current_yaw,
                                           float left_speed,
                                           float right_speed)
{
    const auto now = MonotonicEventTimer::Clock::now();
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);

    if (is_active_state(g_state) && g_total_timer.expired(k_avoid_total_timeout, now))
    {
        enter_cooldown(ObstacleAvoidExitReason::TotalTimeout, now);
        return make_control(current_yaw);
    }

    switch (g_state)
    {
    case ObstacleAvoidState::Decelerate:
        if (!g_decelerate_brake_completed &&
            left_speed <= k_avoid_decelerate_exit_speed &&
            right_speed <= k_avoid_decelerate_exit_speed)
        {
            // 用有符号速度判断并永久锁存，避免某一轮已反转后 fabs(speed) 反而让制动无法结束。
            g_decelerate_brake_completed = true;
            if (g_direction == ObstacleAvoidDirection::None)
            {
                // 状态号不变，但控制目标从 -90 跳到 +30；递增序号让调度器清除制动 PID 历史。
                ++g_transition_sequence;
            }
        }

        if (g_decelerate_brake_completed &&
            g_direction != ObstacleAvoidDirection::None)
        {
            enter_turn_to_relative_yaw(current_yaw, now);
        }
        else if (g_waiting_for_result &&
                 g_state_timer.expired(k_avoid_warning_result_timeout, now))
        {
            enter_cooldown(ObstacleAvoidExitReason::WarningTimeout, now);
        }
        else if (!g_waiting_for_result &&
                 !g_decelerate_brake_completed &&
                 g_state_timer.expired(k_avoid_decelerate_timeout, now))
        {
            enter_cooldown(ObstacleAvoidExitReason::DecelerateTimeout, now);
        }
        break;

    case ObstacleAvoidState::TurnToRelativeYaw:
        // 第一版只按单次航向误差判断完成，不增加帧数或行驶距离条件。
        if (std::fabs(wrap_to_180(g_turn_target_yaw - current_yaw)) <=
            k_avoid_turn_yaw_tolerance)
        {
            enter_state(ObstacleAvoidState::FollowEdge, now);
        }
        else if (g_state_timer.expired(k_avoid_turn_timeout, now))
        {
            enter_cooldown(ObstacleAvoidExitReason::TurnTimeout, now);
        }
        break;

    case ObstacleAvoidState::FollowEdge:
        // 状态 3 的固定 1 秒从进入本状态时开始，不包含减速和转向时间。
        if (g_state_timer.expired(k_avoid_edge_follow_duration, now))
        {
            enter_state(ObstacleAvoidState::ReturnToCenter, now);
        }
        break;

    case ObstacleAvoidState::ReturnToCenter:
        // 第一版只负责释放绕行边线，让视觉立即恢复普通中线；后续再增加平滑回中线条件。
        enter_cooldown(ObstacleAvoidExitReason::Completed, now);
        break;

    case ObstacleAvoidState::Cooldown:
        if (g_state_timer.expired(k_avoid_reentry_cooldown, now))
        {
            g_state = ObstacleAvoidState::Idle;
            g_state_timer.reset();
            ++g_transition_sequence;
        }
        break;

    case ObstacleAvoidState::Idle:
    default:
        break;
    }

    return make_control(current_yaw);
}

ObstacleAvoidControl obstacle_avoid_snapshot()
{
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    return make_control(0.0f);
}

bool obstacle_avoid_active()
{
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    return is_active_state(g_state);
}

bool obstacle_avoid_follow_edge_active()
{
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    return g_state == ObstacleAvoidState::FollowEdge;
}

ObstacleAvoidDirection current_obstacle_avoid_direction()
{
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    return g_direction;
}

const char *obstacle_avoid_direction_name(ObstacleAvoidDirection direction)
{
    switch (direction)
    {
    case ObstacleAvoidDirection::Left:
        return "L";
    case ObstacleAvoidDirection::Right:
        return "R";
    default:
        return "-";
    }
}

const char *obstacle_avoid_state_name(ObstacleAvoidState state)
{
    switch (state)
    {
    case ObstacleAvoidState::Idle:
        return "Idle";
    case ObstacleAvoidState::Decelerate:
        return "Decelerate";
    case ObstacleAvoidState::TurnToRelativeYaw:
        return "TurnToRelativeYaw";
    case ObstacleAvoidState::FollowEdge:
        return "FollowEdge";
    case ObstacleAvoidState::ReturnToCenter:
        return "ReturnToCenter";
    case ObstacleAvoidState::Cooldown:
        return "Cooldown";
    default:
        return "Unknown";
    }
}

const char *obstacle_avoid_exit_reason_name(ObstacleAvoidExitReason reason)
{
    switch (reason)
    {
    case ObstacleAvoidExitReason::Completed:
        return "Completed";
    case ObstacleAvoidExitReason::WarningTimeout:
        return "WarningTimeout";
    case ObstacleAvoidExitReason::ClassificationRejected:
        return "ClassificationRejected";
    case ObstacleAvoidExitReason::DecelerateTimeout:
        return "DecelerateTimeout";
    case ObstacleAvoidExitReason::TurnTimeout:
        return "TurnTimeout";
    case ObstacleAvoidExitReason::TotalTimeout:
        return "TotalTimeout";
    default:
        return "None";
    }
}

float obstacle_avoid_edge_midline_offset_px()
{
    return k_avoid_edge_midline_offset_px;
}

float obstacle_avoid_edge_lookahead_distance()
{
    return k_obstacle_avoid_edge_lookahead_distance;
}
