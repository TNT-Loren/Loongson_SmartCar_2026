#ifndef OBSTACLE_AVOID_FSM_HPP
#define OBSTACLE_AVOID_FSM_HPP

#include <cstdint>

enum class ObstacleAvoidDirection : std::uint8_t
{
    None = 0,
    Left = 1,  // 使用左边线绕行，状态 2 的相对 yaw 为负。
    Right = 2  // 使用右边线绕行，状态 2 的相对 yaw 为正。
};

enum class ObstacleAvoidState : std::uint8_t
{
    Idle = 0,
    Decelerate = 1,
    TurnToRelativeYaw = 2,
    FollowEdge = 3,
    ReturnToCenter = 4,
    Cooldown = 5
};

enum class ObstacleAvoidExitReason : std::uint8_t
{
    None = 0,
    Completed,
    WarningTimeout,
    ClassificationRejected,
    DecelerateTimeout,
    TurnTimeout,
    TotalTimeout
};

struct ObstacleAvoidControl
{
    ObstacleAvoidState state = ObstacleAvoidState::Idle;
    ObstacleAvoidDirection direction = ObstacleAvoidDirection::None;
    ObstacleAvoidExitReason last_exit_reason = ObstacleAvoidExitReason::None;
    float target_speed = 0.0f;
    float target_yaw = 0.0f;
    float steer_limit = 0.0f;
    std::uint32_t transition_sequence = 0;
};

// 已知左右方向的直接请求，供 Q/E 键盘测试及漏掉预警时的最终结果兜底。
bool obstacle_avoid_request(ObstacleAvoidDirection direction);

// AI 红色预警只知道需要减速，尚未知道绕行方向。
bool obstacle_avoid_warning_request();
// 最终分类为武器/物资时锁存方向；如果预警漏掉且当前空闲，则直接启动绕行。
bool obstacle_avoid_confirm(ObstacleAvoidDirection direction);
// 最终分类为载具/错误时，只取消正在等待分类的状态1。
bool obstacle_avoid_cancel_warning();

// 由 10 ms 控制周期调用，推进状态并返回当前状态需要的控制目标。
ObstacleAvoidControl obstacle_avoid_update(float current_yaw,
                                           float left_speed,
                                           float right_speed);

ObstacleAvoidControl obstacle_avoid_snapshot();
bool obstacle_avoid_active();
bool obstacle_avoid_follow_edge_active();
ObstacleAvoidDirection current_obstacle_avoid_direction();
const char *obstacle_avoid_direction_name(ObstacleAvoidDirection direction);
const char *obstacle_avoid_state_name(ObstacleAvoidState state);
const char *obstacle_avoid_exit_reason_name(ObstacleAvoidExitReason reason);

// 状态 3 沿用原有的可靠边法向偏移，普通巡线偏移仍由 image_test.cpp 管理。
float obstacle_avoid_edge_midline_offset_px();

#endif
