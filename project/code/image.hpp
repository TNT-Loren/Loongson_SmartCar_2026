#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__

#include "image_ring.hpp"
#include "image_size.hpp"
#include "zf_common_headfile.hpp"

static constexpr uint16 k_front_migration_max_points = image_height * 4;

struct FrontCornerPoint
{
    bool valid = false;
    uint16 x = 0;
    uint16 y = 0;
    uint16 index = 0;
    float angle_deg = 0.0f;
};

enum FrontDebugDisplayMode : uint8
{
    FrontDebugDisplayOriginal = 0,
    FrontDebugDisplayIpm = 1,
};

enum FrontCrossKind : uint8
{
    FrontCrossNone = 0,
    FrontCrossMid = 1,
};

enum FrontCrossState : uint8
{
    FrontCrossState1 = 0,
    FrontCrossState2 = 1,
};

enum FrontTrackSideControlMode : uint8
{
    FrontTrackSideControlAuto = 0,
    FrontTrackSideControlLeft = 1,
    FrontTrackSideControlRight = 2,
    FrontTrackSideControlNone = 3,
};

extern float vision_target_yaw; // 当前视觉模块给出的目标航向
extern uint8 g_front_debug_display_mode; // 调试图显示模式：0=原图调试视图，1=IPM 视图
extern uint8 g_front_cross_kind; // 实验性十字状态：0=未进入，1=十字处理中
extern uint8 g_front_cross_state; // 实验性十字阶段：0=state1，1=state2

extern uint8 g_front_mid_line[image_height];           // Front_Car 管线产出的逐行中线 x 坐标（IPM 空间）
extern uint8 g_front_mid_line_valid[image_height];     // 对应行中线是否有效，1=有效
extern uint16 g_front_left_ipm_points[k_front_migration_max_points][2];  // 左边边界点集（IPM 空间，平滑+重采样后）
extern uint16 g_front_right_ipm_points[k_front_migration_max_points][2]; // 右边边界点集（IPM 空间，平滑+重采样后）
extern uint16 g_front_mid_ipm_points[k_front_migration_max_points][2];   // 中线点集（IPM 空间，平滑+重采样后）
extern uint16 g_front_left_ipm_point_count;   // 左边 IPM 边界点数量
extern uint16 g_front_right_ipm_point_count;  // 右边 IPM 边界点数量
extern uint16 g_front_mid_ipm_point_count;    // 中线 IPM 点数量
extern uint8 g_front_midline_ready;           // 中线是否就绪，1=有效行数达标，可供控制模块使用
extern uint8 g_front_track_side_mode;         // 巡线基准边：0=两边都无效, 1=基于左边补中线, 2=基于右边补中线
extern uint8 g_front_track_side_control_mode; // 巡线控制权：0=自动选择, 1=强制左边, 2=强制右边, 3=强制无中线
extern uint8 g_front_line_lost;               // 丢线状态：0=都不丢线, 1=左边丢线, 2=右边丢线, 3=双边都丢线a
extern uint8 g_front_left_long_straight_flag; // 左边界长直线几何特征// 1=存在长直线特征，0=无明显长直线特征
extern uint8 g_front_right_long_straight_flag; // 右边界长直线几何特征
extern uint8 g_front_left_ring_yaw_recording_flag; // 左圆环累计角是否正在记录
extern float g_front_left_ring_enter_unbounded_yaw; // 左圆环进入时记录的连续 yaw
extern float g_front_left_ring_progress_yaw; // 左圆环累计角，左转时递增
extern FrontCornerPoint g_front_left_upper_corner;   // 左上拐点（原图空间），valid 时有效
extern FrontCornerPoint g_front_right_upper_corner;  // 右上拐点（原图空间）
extern FrontCornerPoint g_front_left_lower_corner;   // 左下拐点（原图空间）
extern FrontCornerPoint g_front_right_lower_corner;  // 右下拐点（原图空间）

extern uint8_t mode_state; // 0是普通，1是十字，2，做圆环，3右圆环
extern uint8_t left_ring_process_state;

void image_process(void);
void process_front_car_migration(void);
void draw_front_car_migration_overlay(uint16 (*img)[image_width], bool big_endian = true);
void fill_front_car_migration_debug_image(uint16 (*img)[image_width], bool big_endian = true);
void update_vision_control_test(void);
void toggle_front_debug_display_mode(void);
const char *get_front_debug_display_mode_name(void);
void set_front_track_side_control_mode(uint8 mode);
uint8 get_front_track_side_control_mode(void);
const char *get_front_track_side_control_mode_name(void);
//=============================================================
void start_front_left_ring_yaw_tracking(void);
void stop_front_left_ring_yaw_tracking(void);
void update_front_left_ring_yaw_tracking(void);
bool front_left_ring_exit_yaw_ready(void);

#endif
