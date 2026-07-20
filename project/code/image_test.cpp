#include "image_test.hpp"
#include "car_control.hpp"
#include "event_timer.hpp"
#include "scheduler.hpp"
#include "zgc_draw_tool.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>

zf_device_uvc uvc_dev;
std::mutex g_image_mutex;// 保护图像数据的互斥锁
std::mutex g_vision_result_mutex;// 保护图像数据和视觉结果的互斥锁

uint8 *rgay_image = nullptr;
uint8 image_copy[image_height][image_width] = {{0}};
uint16 debug_image[image_height][image_width] = {{0}};
bool g_debug_show_binary_image = true;
bool g_debug_show_ipm_lines = false;
uint8 left_edge_line[image_height] = {0};
uint8 right_edge_line[image_height] = {0};
uint8 mid_line[image_height] = {0};
uint8 boundary_y_line[image_height] = {0};

uint8 complete = 0; //后续可与斑马线配合完赛

extern float yaw;
extern float test1, test2, test3;

namespace
{
    enum class DebugViewMode : uint8_t
    {
        Composite,
        Normal,
        Ipm
    };

    DebugViewMode g_debug_view_mode = DebugViewMode::Composite;

    // constexpr 编译时常量 不可修改值 比define更安全
    constexpr uint8 k_max_lost_frame_count = 5; // 摄像头连续丢帧的最大允许次数
    constexpr uint8 k_default_left = 1;         // 默认左边界位置
    constexpr uint8 k_default_right = image_width - 2;
    constexpr uint8 k_default_mid = image_width / 2;
    constexpr int k_lost_line_count_threshold = 30;//丢线行数阈值
    constexpr uint8 k_search_top_stop_row = 0;//八邻域搜到顶行后停止，避免沿顶边横向爬线
    constexpr uint8 k_start_black_confirm_count = 2;//起点边界确认需要的连续黑点数
    constexpr int k_left_circle_short_col_fallback_x = 60;//状态三兜底点
    constexpr int k_right_circle_short_col_fallback_x = (MT9V03X_W - 1) - k_left_circle_short_col_fallback_x;
    constexpr int k_circle_short_col_fallback_y = 20; // 状态三兜底点

    constexpr int k_circle_state2_no_monotonicity_timeout_frames = 20;//状态二到状态三的兜底次数
    constexpr int k_circle_entry_confirm_frames = 3; // 左/右圆环入口均需连续命中的图像帧数。

    // 圆环各状态事件的超时参数集中在此，实车标定时可独立修改。
    constexpr auto k_circle_state1_entry_detected_timeout = std::chrono::milliseconds(4000);
    constexpr auto k_circle_state2_wait_upper_arc_timeout = std::chrono::milliseconds(3000);
    constexpr auto k_circle_state3_find_arc_timeout = std::chrono::milliseconds(3000);
    constexpr auto k_circle_state4_exit_trend_timeout = std::chrono::milliseconds(5000);
    constexpr auto k_circle_state5_repair_exit_line_timeout = std::chrono::milliseconds(3000);
    constexpr auto k_circle_state6_recover_track_timeout = std::chrono::milliseconds(3000);
    constexpr auto k_circle_normal_exit_reentry_cooldown = std::chrono::milliseconds(2000);
    // 正常巡线时，从可靠边法向偏移半个 IPM 路宽得到赛道中线。
    constexpr float k_ipm_default_midline_offset_px = 22.5f;

    enum class IpmReliableEdge : uint8_t
    {
        Left,
        Right
    };

    IpmReliableEdge g_ipm_auto_reliable_edge = IpmReliableEdge::Left;
    IpmReliableEdge g_ipm_auto_reliable_edge_candidate = IpmReliableEdge::Left;
    int g_ipm_auto_reliable_edge_candidate_frames = 0;
    int g_left_circle_entry_candidate_frames = 0;
    int g_right_circle_entry_candidate_frames = 0;
    int g_left_circle_state2_no_monotonicity_frames = 0;
    int g_right_circle_state2_no_monotonicity_frames = 0;
    int g_left_circle_exit_mono_row = 0;//圆环出口单调点行号
    int g_left_circle_exit_mono_col = 0;
    int g_right_circle_exit_mono_row = 0;
    int g_right_circle_exit_mono_col = 0;
    int g_prefill_left_lost_time = MT9V03X_H;
    int g_prefill_right_lost_time = MT9V03X_H;
    float g_prefill_left_lost_ratio = 1.0f;
    float g_prefill_right_lost_ratio = 1.0f;
    MonotonicEventTimer g_circle_state_event_timer;
    MonotonicEventTimer g_circle_normal_exit_cooldown_timer;

    struct HuCarCompat
    {
        float circle_intergrate_yaw = 0.0f;
    };

    struct HuMcxCompat
    {
        bool start_finish = false;
    };

HuCarCompat car; // 用于圆环 yaw 累计角度。
HuMcxCompat mcx;//用于判断是否已经完成起跑/停车逻辑
float g_circle_unbounded_yaw_zero = 0.0f;// 记录进入圆环阶段时的无界 yaw 零点。

// 安全读取二值图像素：越界坐标按白色处理，避免八邻域搜线访问数组外。
uint8 safe_image_pixel(uint8 (*image)[MT9V03X_W], int x, int y)
{
    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H)
    {
        return IMG_WHITE;
    }
    return image[y][x];
}

// 记录进入圆环阶段时的 yaw 零点，供圆环状态机按累计转角切状态。
void reset_circle_integrate_yaw()
{
    g_circle_unbounded_yaw_zero = yaw_tracker.get_unbounded_yaw();
    car.circle_intergrate_yaw = 0.0f;
}

// 更新圆环累计 yaw：当前无界 yaw 减去圆环零点。
void update_circle_integrate_yaw()
{
    car.circle_intergrate_yaw = yaw_tracker.get_unbounded_yaw() - g_circle_unbounded_yaw_zero;
}

}

//==========================================================================
int my_abs(int value);
int16 limit_a_b(int16 x, int a, int b);
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);
void turn_to_bin(void);
void image_filter(uint8 (*bin_image)[MT9V03X_W]);//简单腐蚀
void image_draw_rectan(uint8 (*image)[MT9V03X_W]);//画边框
void Draw_Line(int startX, int startY, int endX, int endY);//画线
void Left_Add_Line(int x1, int y1, int x2, int y2);//俩点连线
void Right_Add_Line(int x1, int y1, int x2, int y2);
void Lengthen_Left_Boundry(int start, int end);//边界线延长（固定向下延展）
void Lengthen_Right_Boundry(int start, int end);
// void Get_Longest_Line(void); // 旧版最长白列起点参考，已停用但保留实现
uint8_t get_lost_line(void);
uint8 get_start_point(uint8 start_row);
void search_l_r(uint16 break_flag, uint8 (*image)[MT9V03X_W], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest);//八邻域
void get_down_turning_point(void);
void get_up_turning_point(void);
int shortest_White_Column(uint8 x, uint8 y);
int shortest_White_Column(uint8 x, uint8 y, uint8 start_row, uint8 end_row);
int shortest_White_Column(uint8 x, uint8 y, uint8 start_row, uint8 end_row, bool reverse_scan);
float Get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy);
void get_left(uint16 total_L);
void get_right(uint16 total_R);
bool Get_K_b(uint8 x1, uint8 y1, uint8 x2, uint8 y2, float *slope_rate, float *intercept);
void get_turning_point(void);
void circle_get_turning_point(void); // break_flag 在函数内没有清零，如果以后启用要修。
void validate_and_count_turn_points(void);
int Find_Left_Down_Point(int start, int end);
int Find_Left_Up_Point(int start, int end);
int Find_Right_Down_Point(int start, int end);
int Find_Right_Up_Point(int start, int end);
uint8_t Continuity_Change_Left(int start, int end); // 连续性检验，连续返回0，不连续返回断线出行数
uint8_t Continuity_Change_Right(int start, int end);
int Monotonicity_Change_Left(int start, int end);//单调性检验，返回突变点个数
int Monotonicity_Change_Right(int start, int end);
void get_right_cusp_point(void);//角点，暂未使用
void get_left_cusp_point(void);
void Bu_right(void);//根据左边还有赛道宽补右边线
void Bu_left(void);
void Island_Detect(void);
void Cross_fill(void);
void get_turn_point(void);
void fit_midline(void);
void HDPJ_lvbo(void);
void Image_Process(void);
void update_track_lines(void);
void build_debug_image(bool show_binary);


//找八邻域的最高点 即白线最高点
uint8 hightest = 0;
//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5//此参数可根据自己的需求调节
#define threshold_min   255*1//此参数可根据自己的需求调节

uint8 bin_image[MT9V03X_H][MT9V03X_W];  //120 x 160
uint8 Left_Line[MT9V03X_H]; //左边线数组
uint8 Right_Line[MT9V03X_H];//右边线数组
uint8 Mid_Line[MT9V03X_H];
uint8 End_Mid_Line[MT9V03X_H];
uint8 Test_Mid_Line[MT9V03X_H];
int16 Ipm_Left_Line[MT9V03X_H];
int16 Ipm_Right_Line[MT9V03X_H];
uint16 Ipm_Left_Points[MT9V03X_H][2];// IPM 左边界点集 [i][0]=X, [i][1]=Y
uint16 Ipm_Right_Points[MT9V03X_H][2];
uint16 Ipm_Mid_Points[MT9V03X_H][2];
uint16 Ipm_Bilateral_Mid_Points[MT9V03X_H][2];
uint16 Ipm_Left_Point_Count = 0;
uint16 Ipm_Right_Point_Count = 0;
uint16 Ipm_Mid_Point_Count = 0;
uint16 Ipm_Bilateral_Mid_Point_Count = 0;
uint8 Road_Wide[MT9V03X_H]; //赛道宽度
uint8 bin_image_ipm[image_h][image_w];  //
uint8 sobel_image[MT9V03X_H][MT9V03X_W];
uint8_t White_Column[MT9V03X_H];//旧版白列缓存，当前有效列宽统计使用 White_col
std::atomic<float> g_ipm_midline_offset_px{k_ipm_default_midline_offset_px};

/* 旧版最长白列起点参考变量，当前起点改为按起始行最长白段寻找，暂时停用但不删除。
int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，[1]是第某列
int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，[1]是第某列
*/
uint16_t Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
uint16_t Right_Lost_Flag[MT9V03X_H]; //右丢线数组，丢线置1，没丢线置0 

int Right_Lost_Time = 0;    //边界丢线数
int Left_Lost_Time  = 0;
int Both_Lost_Time = 0;//两边同时丢线数
int point_mode =0; //0突变找点 1向量法找点
TestMidlineMode g_test_midline_mode = TestMidlineMode::Auto;
ReliableEdgeMode g_ipm_reliable_edge_mode = ReliableEdgeMode::Auto;
namespace
{
std::mutex g_obstacle_avoid_mutex;
ObstacleAvoidDirection g_obstacle_avoid_direction = ObstacleAvoidDirection::None;
MonotonicEventTimer g_obstacle_avoid_timer;
// 更改绕行时间时只需调整此处。
constexpr auto k_obstacle_avoid_duration = std::chrono::milliseconds(2000);//绕行时间
}

uint8 My_Offine=0;

uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 start_point_l[2] = { 0 };	//左边起点的x，y值
uint8 start_point_r[2] = { 0 };	//右边起点的x，y值

uint8_t turn_point_num =0;

int16 L_D_corner_flag = 0;//左拐点存在标志
int16 L_D_corner_row = 0;//左拐点所在行
int16 L_D_corner_col = 0;//左拐点所在列
int L_D_corner_angle = 0;//左拐点角度

int16 R_D_corner_flag = 0;//右拐点存在标志
int16 R_D_corner_row = 0;//右拐点所在行
int16 R_D_corner_col = 0;//右拐点所在列
int R_D_corner_angle = 0;//右拐点角度

int16 L_U_corner_flag = 0;//左拐点存在标志
int16 L_U_corner_row = 0;//左拐点所在行
int16 L_U_corner_col = 0;//左拐点所在列
int L_U_corner_angle = 0;//左拐点角度

int16 R_U_corner_flag = 0;//右拐点存在标志
int16 R_U_corner_row = 0;//右拐点所在行
int16 R_U_corner_col = 0;//右拐点所在列
int R_U_corner_angle = 0;//右拐点角度

uint8 enable_L_D_corner=1,enable_R_D_corner=1;
uint8 enable_L_U_corner=1,enable_R_U_corner=1;

uint8 White_col[MT9V03X_W];				//每一列白列长度

Flag_Handle Image_Flag = {false,false,false,false,false,false,false};//元素标志位

const uint8 Standard_Road_Wide[MT9V03X_H]=
{
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
    35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
    45, 46, 47, 48, 49, 50, 51, 52, 53, 54,
    55, 56, 57, 58, 59, 61, 62, 63, 64, 65,
    66, 67, 68, 69, 70, 71, 72, 73, 74, 75,
    76, 77, 78, 80, 81, 82, 83, 84, 86, 87,
    88, 89, 90, 91, 92, 94, 95, 96, 97, 98,
    99, 100, 101, 102, 103, 104, 105, 106, 107, 108,
    109, 110, 111, 112, 113, 115, 116, 117, 118, 119,
    120, 121, 122, 123, 124, 125, 125, 126, 127, 128,
    129, 130, 131, 132, 133, 133, 134, 135, 136, 137
};

const char *test_midline_mode_name(TestMidlineMode mode)
{
    switch (mode)
    {
    case TestMidlineMode::Auto:
        return "AUTO";
    case TestMidlineMode::ForceLeft:
        return "LEFT";
    case TestMidlineMode::ForceRight:
        return "RIGHT";
    default:
        return "UNKNOWN";
    }
}

const char *reliable_edge_mode_name(ReliableEdgeMode mode)
{
    switch (mode)
    {
    case ReliableEdgeMode::Auto:
        return "AUTO";
    case ReliableEdgeMode::ForceLeft:
        return "LEFT";
    case ReliableEdgeMode::ForceRight:
        return "RIGHT";
    default:
        return "UNKNOWN";
    }
}

void cycle_test_midline_mode(void)
{
    if (g_test_midline_mode == TestMidlineMode::Auto)
    {
        g_test_midline_mode = TestMidlineMode::ForceLeft;
    }
    else if (g_test_midline_mode == TestMidlineMode::ForceLeft)
    {
        g_test_midline_mode = TestMidlineMode::ForceRight;
    }
    else
    {
        g_test_midline_mode = TestMidlineMode::Auto;
    }
}

void cycle_ipm_reliable_edge_mode(void)
{
    if (g_ipm_reliable_edge_mode == ReliableEdgeMode::Auto)
    {
        g_ipm_reliable_edge_mode = ReliableEdgeMode::ForceLeft;
    }
    else if (g_ipm_reliable_edge_mode == ReliableEdgeMode::ForceLeft)
    {
        g_ipm_reliable_edge_mode = ReliableEdgeMode::ForceRight;
    }
    else
    {
        g_ipm_reliable_edge_mode = ReliableEdgeMode::Auto;
    }
}
// 触发绕行：设置绕行方向和持续时间，实际中线调整在 update_track_lines() 中根据这个状态进行。
void trigger_obstacle_avoid(ObstacleAvoidDirection direction)
{
    if (direction == ObstacleAvoidDirection::None)
    {
        return;
    }

    // 当前试车方案：直接把选中的边线当临时中线使用，持续时间由
    // k_obstacle_avoid_duration 统一配置。重复触发会从当前时刻重新计时。
    // 绕行偏移当前为 -5.0 px，需结合实车轨迹继续标定。
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    g_obstacle_avoid_timer.start();
    g_obstacle_avoid_direction = direction;
    g_ipm_midline_offset_px.store(-5.0f);
}

void obstacle_avoid_timer_task(void)
{
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    if (!g_obstacle_avoid_timer.expired(k_obstacle_avoid_duration))
    {
        return;
    }

    // 到期后只恢复选择策略和偏移量，不直接改任何中线点集。
    g_obstacle_avoid_timer.reset();
    g_obstacle_avoid_direction = ObstacleAvoidDirection::None;
    g_ipm_midline_offset_px.store(k_ipm_default_midline_offset_px);
}

bool obstacle_avoid_active(void)
{
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    return g_obstacle_avoid_direction != ObstacleAvoidDirection::None;
}

ObstacleAvoidDirection current_obstacle_avoid_direction(void)
{
    std::lock_guard<std::mutex> lock(g_obstacle_avoid_mutex);
    return g_obstacle_avoid_direction;
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

void cycle_debug_view_mode(void)
{
    if (g_debug_view_mode == DebugViewMode::Composite)
    {
        g_debug_view_mode = DebugViewMode::Normal;
    }
    else if (g_debug_view_mode == DebugViewMode::Normal)
    {
        g_debug_view_mode = DebugViewMode::Ipm;
    }
    else
    {
        g_debug_view_mode = DebugViewMode::Composite;
    }
    g_debug_show_ipm_lines = (g_debug_view_mode == DebugViewMode::Ipm);
}

const char *debug_view_mode_name(void)
{
    switch (g_debug_view_mode)
    {
    case DebugViewMode::Composite:
        return "COMBO";
    case DebugViewMode::Normal:
        return "NORMAL";
    case DebugViewMode::Ipm:
        return "IPM_LINES";
    default:
        return "UNKNOWN";
    }
}

const char *ipm_midline_scene_name(IpmMidlineScene scene)
{
    switch (scene)
    {
    case IpmMidlineScene::Invalid:
        return "Invalid";
    case IpmMidlineScene::Straight:
        return "Straight";
    case IpmMidlineScene::GentleCurve:
        return "GentleCurve";
    case IpmMidlineScene::SCurve:
        return "SCurve";
    case IpmMidlineScene::SharpCurve:
        return "SharpCurve";
    default:
        return "Unknown";
    }
}

IpmMidlineSceneResult classify_ipm_midline_scene(const uint16 points[MT9V03X_H][2],
                                                 uint16 count,
                                                 uint8 highest)
{
    IpmMidlineSceneResult result;
    result.count = count;
    result.highest = highest;
    if (count < 2)
    {
        return result;
    }

    auto clamp01 = [](float value) {
        return std::clamp(value, 0.0f, 1.0f);
    };
    auto point_distance = [&](uint16 a, uint16 b) {
        const float dx = static_cast<float>(points[b][0]) - static_cast<float>(points[a][0]);
        const float dy = static_cast<float>(points[b][1]) - static_cast<float>(points[a][1]);
        return std::sqrt(dx * dx + dy * dy);
    };
    auto angle_diff = [](float to, float from) {
        constexpr float k_pi = 3.14159265358979f;
        float diff = to - from;
        while (diff > k_pi)
        {
            diff -= 2.0f * k_pi;
        }
        while (diff <= -k_pi)
        {
            diff += 2.0f * k_pi;
        }
        return diff;
    };

    result.x_min = result.x_max = points[0][0];
    result.y_min = result.y_max = points[0][1];
    for (uint16 i = 1; i < count; ++i)
    {
        result.x_min = std::min<int>(result.x_min, points[i][0]);
        result.x_max = std::max<int>(result.x_max, points[i][0]);
        result.y_min = std::min<int>(result.y_min, points[i][1]);
        result.y_max = std::max<int>(result.y_max, points[i][1]);
        result.path_length += point_distance(i - 1, i);
    }

    result.x_span = result.x_max - result.x_min;
    result.y_span = result.y_max - result.y_min;
    result.dx_total = static_cast<int>(points[count - 1][0]) - static_cast<int>(points[0][0]);
    result.x_y_ratio = static_cast<float>(result.x_span) /
                       static_cast<float>(std::max(result.y_span, 1));
    result.chord_length = point_distance(0, count - 1);
    result.straightness = result.path_length > 1e-3f ? result.chord_length / result.path_length : 0.0f;

    int last_sign = 0;
    float positive_run = 0.0f;
    float negative_run = 0.0f;
    bool positive_confirmed = false;
    bool negative_confirmed = false;
    for (uint16 i = 1; i < count; ++i)
    {
        const int dx = static_cast<int>(points[i][0]) - static_cast<int>(points[i - 1][0]);
        int sign = 0;
        if (dx >= 2)
        {
            sign = 1;
        }
        else if (dx <= -2)
        {
            sign = -1;
        }

        if (sign == 0)
        {
            continue;
        }

        if (sign > 0)
        {
            positive_run += static_cast<float>(std::abs(dx));
            positive_confirmed = positive_run >= 8.0f;
        }
        else
        {
            negative_run += static_cast<float>(std::abs(dx));
            negative_confirmed = negative_run >= 8.0f;
        }

        if (last_sign != 0 && sign != last_sign)
        {
            result.turn_count++;
        }
        last_sign = sign;
    }
    result.has_reversal = positive_confirmed && negative_confirmed && result.turn_count > 0;

    constexpr uint16 k_curv_span = 3;
    if (count > k_curv_span * 2)
    {
        float abs_curv_sum = 0.0f;
        int curv_count = 0;
        for (uint16 i = k_curv_span; i + k_curv_span < count; ++i)
        {
            const uint16 prev = i - k_curv_span;
            const uint16 next = i + k_curv_span;
            const float ds = point_distance(prev, i) + point_distance(i, next);
            if (ds < 1e-3f)
            {
                continue;
            }

            const float theta1 = std::atan2(static_cast<float>(points[i][1]) - static_cast<float>(points[prev][1]),
                                            static_cast<float>(points[i][0]) - static_cast<float>(points[prev][0]));
            const float theta2 = std::atan2(static_cast<float>(points[next][1]) - static_cast<float>(points[i][1]),
                                            static_cast<float>(points[next][0]) - static_cast<float>(points[i][0]));
            const float curv = angle_diff(theta2, theta1) / ds;
            const float abs_curv = std::fabs(curv);
            abs_curv_sum += abs_curv;
            result.signed_curv_sum += curv;
            result.max_abs_curv = std::max(result.max_abs_curv, abs_curv);
            curv_count++;
        }

        if (curv_count > 0)
        {
            result.mean_abs_curv = abs_curv_sum / static_cast<float>(curv_count);
        }
    }

    const float count_long_score = clamp01((static_cast<float>(count) - 28.0f) / 12.0f);
    const float count_short_score = clamp01((32.0f - static_cast<float>(count)) / 16.0f);
    const float y_long_score = clamp01((static_cast<float>(result.y_span) - 65.0f) / 35.0f);
    const float y_short_score = clamp01((70.0f - static_cast<float>(result.y_span)) / 38.0f);
    const float lateral_score = clamp01((static_cast<float>(result.x_span) - 14.0f) / 52.0f);
    const float low_lateral_score = 1.0f - lateral_score;
    const float straightness_score = clamp01((result.straightness - 0.88f) / 0.10f);
    const float bend_score = clamp01((result.mean_abs_curv - 0.006f) / 0.020f);
    const float sharp_curv_score = clamp01((result.max_abs_curv - 0.030f) / 0.060f);
    const float high_hightest_score = clamp01((static_cast<float>(highest) - 10.0f) / 18.0f);
    const float reversal_score = result.has_reversal
                                 ? clamp01((std::min(positive_run, negative_run) - 8.0f) / 20.0f)
                                 : 0.0f;

    result.straight_score = 0.35f * count_long_score +
                            0.25f * y_long_score +
                            0.25f * straightness_score +
                            0.15f * low_lateral_score;
    result.s_score = 0.30f * count_long_score +
                     0.20f * y_long_score +
                     0.30f * reversal_score +
                     0.20f * lateral_score;
    result.sharp_score = 0.32f * count_short_score +
                         0.30f * y_short_score +
                         0.22f * lateral_score +
                         0.16f * sharp_curv_score +
                         0.06f * high_hightest_score;
    result.gentle_score = 0.25f * count_long_score +
                          0.20f * y_long_score +
                          0.25f * lateral_score +
                          0.20f * bend_score +
                          0.10f * (1.0f - reversal_score);

    if (count < 10 || result.y_span < 18 || result.path_length < 18.0f)
    {
        result.scene = IpmMidlineScene::Invalid;
    }
    else if (result.sharp_score >= result.s_score &&
             result.sharp_score >= result.straight_score &&
             result.sharp_score >= result.gentle_score)
    {
        result.scene = IpmMidlineScene::SharpCurve;
    }
    else if (result.s_score >= result.straight_score &&
             result.s_score >= result.gentle_score)
    {
        result.scene = IpmMidlineScene::SCurve;
    }
    else if (result.straight_score >= result.gentle_score)
    {
        result.scene = IpmMidlineScene::Straight;
    }
    else
    {
        result.scene = IpmMidlineScene::GentleCurve;
    }

    return result;
}

namespace
{
int reliable_edge_lost_score(int lost_count, float lost_ratio)
{
    // 第一版只把单边丢线数量和丢线比例作为主依据。
    return lost_count * 100 + static_cast<int>(lost_ratio * 1000.0f);
}

void update_ipm_auto_reliable_edge_selection(void)// 根据连续丢线时间和丢线比例自动选择 IPM 可靠边
{
    constexpr int k_eval_rows = MT9V03X_H - 3;
    constexpr int k_switch_confirm_frames = 4;
    g_prefill_left_lost_time = Left_Lost_Time;
    g_prefill_right_lost_time = Right_Lost_Time;
    g_prefill_left_lost_ratio = static_cast<float>(Left_Lost_Time) / static_cast<float>(k_eval_rows);
    g_prefill_right_lost_ratio = static_cast<float>(Right_Lost_Time) / static_cast<float>(k_eval_rows);

    const int left_score = reliable_edge_lost_score(g_prefill_left_lost_time, g_prefill_left_lost_ratio);
    const int right_score = reliable_edge_lost_score(g_prefill_right_lost_time, g_prefill_right_lost_ratio);
    IpmReliableEdge frame_reliable_edge = g_ipm_auto_reliable_edge;
    if (left_score < right_score)
    {
        frame_reliable_edge = IpmReliableEdge::Left;
    }
    else if (right_score < left_score)
    {
        frame_reliable_edge = IpmReliableEdge::Right;
    }
    else if (data_stastics_l > data_stastics_r)
    {
        frame_reliable_edge = IpmReliableEdge::Left;
    }
    else if (data_stastics_r > data_stastics_l)
    {
        frame_reliable_edge = IpmReliableEdge::Right;
    }
    // 完全打平时保持上一帧选择，避免左右来回跳。

    if (frame_reliable_edge == g_ipm_auto_reliable_edge)
    {
        g_ipm_auto_reliable_edge_candidate = frame_reliable_edge;
        g_ipm_auto_reliable_edge_candidate_frames = 0;
        return;
    }

    if (frame_reliable_edge != g_ipm_auto_reliable_edge_candidate)
    {
        g_ipm_auto_reliable_edge_candidate = frame_reliable_edge;
        g_ipm_auto_reliable_edge_candidate_frames = 1;
    }
    else
    {
        g_ipm_auto_reliable_edge_candidate_frames++;
    }

    if (g_ipm_auto_reliable_edge_candidate_frames >= k_switch_confirm_frames)
    {
        g_ipm_auto_reliable_edge = g_ipm_auto_reliable_edge_candidate;
        g_ipm_auto_reliable_edge_candidate_frames = 0;
    }
}

ReliableEdgeMode selected_ipm_reliable_edge_mode(void)
{
    if (g_ipm_reliable_edge_mode == ReliableEdgeMode::ForceLeft ||
            g_ipm_reliable_edge_mode == ReliableEdgeMode::ForceRight)
    {
        return g_ipm_reliable_edge_mode;
    }

    return (g_ipm_auto_reliable_edge == IpmReliableEdge::Left)
           ? ReliableEdgeMode::ForceLeft
           : ReliableEdgeMode::ForceRight;
}

ReliableEdgeMode effective_ipm_reliable_edge_mode(void)
{
    // 绕行优先级最高：触发后先压过圆环/自动可靠边逻辑。
    const ObstacleAvoidDirection avoid_direction = current_obstacle_avoid_direction();
    if (avoid_direction == ObstacleAvoidDirection::Left)
    {
        return ReliableEdgeMode::ForceLeft;
    }
    if (avoid_direction == ObstacleAvoidDirection::Right)
    {
        return ReliableEdgeMode::ForceRight;
    }

    if (Image_Flag.Left_Circle)
    {
        if (island_state == 1||island_state == 2)
        {
            return ReliableEdgeMode::ForceRight;
        }
        if ( island_state == 3)
        {
            return ReliableEdgeMode::ForceLeft;
        }
        if (island_state == 4)
        {
            return ReliableEdgeMode::Auto;
        }
    }
    if (Image_Flag.Right_Circle)
    {
        if (island_state == 1 || island_state == 2)
        {
            return ReliableEdgeMode::ForceLeft;
        }
        if (island_state == 3)
        {
            return ReliableEdgeMode::ForceRight;
        }
        if (island_state == 4)
        {
            return ReliableEdgeMode::Auto;
        }
    }

    return g_ipm_reliable_edge_mode;
}
}

//===================================对边线进行处理
/*
（选择可靠边线、IPM、滤波、偏移中线、控制）
选择可靠边：

对边线进行IPM：变换矩阵 k_image_to_ipm_mat[3][3]

*/


namespace
{
    //mat2
    constexpr double k_image_to_ipm_mat[3][3] =
        {
            {3.88086826550718, 6.83393912629936, -221.200998709152},
            {0.0710985800665834, 13.1611692370406, -252.156090767037},
            {0.00156260615530949, 0.0848223384740814, 1},
    };
    // double Mat2[3][3] = {
    //     {3.88086826550718, 6.83393912629936, -221.200998709152},
    //     {0.0710985800665834, 13.1611692370406, -252.156090767037},
    //     {0.00156260615530949, 0.0848223384740814, 1},
    // };
    // double Mat2[3][3] = {
    //     {5.32608695652173, 8.07864450127876, -318.401534526854},
    //     {0.145780051150895, 16.1227621483376, -355.86189258312},
    //     {0.00383631713554987, 0.100383631713555, 1},
    // };

    // {1.54922851132135, 4.87986157380177, -49.7605197103311},
    // {-0.244732053958449, 8.58083223015027, -50.5084935687455},
    // -0.00215511349918493, 0.0612137810224406, 1},

    // double Mat2[3][3] = {
    //     {4.0251572327044, 8.36477987421383, -230.062893081761},
    //     {1.97716239515854E-15, 15.8301886792453, -275.584905660377},
    //     {1.36019776853447E-17, 0.10377358490566, 1},
    // };
// 采样距离新距离 = 3.0 × (新 track_width_ipm / 旧 track_width_ipm)
    constexpr float k_ipm_sample_distance = 3.9f; 
    // 红线近端人工锚点，只连接近端丢线后的第一个有效红线点。
    constexpr float k_redline_front_anchor_x = 79.0f;
    constexpr float k_redline_front_anchor_y = 119.0f;
    constexpr float k_redline_front_anchor_max_gap = 36.0f;
    // 只有圆环状态1~3允许边界丢线值进入 IPM 偏移；状态4~6 不再沿用该旧规则。
    bool allow_lost_edge_offset_in_circle()
    {
        return island_state >= 1 && island_state <= 3 &&
               (Image_Flag.Left_Circle || Image_Flag.Right_Circle);
    }
    constexpr float k_ipm_break_distance = 18.0f;// 断线距离，超过这个距离认为两点不连续
    constexpr int k_ipm_midline_tangent_span = 2;// 计算中线切线的跨度，单位是点数

    /*
    double Mat1[3][3]=       { { 0.594503055500757, -0.403675880267065, 9.19372040877073},
                     { 0.0180084692930395, 0.0734423608040157, 4.6055737995523},
                     { 0.0001788550643629, -0.005365651930887, 0.73788892491219}, };

    double Mat2[3][3]= { { 1.54922851132135, 4.87986157380177, -49.7605197103311},
                     { -0.244732053958449, 8.58083223015027, -50.5084935687455},
                     { -0.00215511349918493, 0.0612137810224406, 1}, };
    */

    /*
    单点坐标变换函数。输入原图坐标 (x, y)，使用内部的 k_image_to_ipm_mat 矩阵，也就是 Mat2，把它转换成 IPM 坐标 (ipm_x, ipm_y)。
    如果齐次坐标分母接近 0，或者转换后的点超出 160x120 范围，就返回 false；否则返回 true。
    */
    bool transform_image_to_ipm_point(int x, int y, int &ipm_x, int &ipm_y)
    {
        const double denominator = k_image_to_ipm_mat[2][0] * x +
                                   k_image_to_ipm_mat[2][1] * y +
                                   k_image_to_ipm_mat[2][2];
        if (std::fabs(denominator) < 1e-9)
        {
            return false;
        }

        const double transformed_x = (k_image_to_ipm_mat[0][0] * x +
                                      k_image_to_ipm_mat[0][1] * y +
                                      k_image_to_ipm_mat[0][2]) /
                                     denominator;
        const double transformed_y = (k_image_to_ipm_mat[1][0] * x +
                                      k_image_to_ipm_mat[1][1] * y +
                                      k_image_to_ipm_mat[1][2]) /
                                     denominator;
        ipm_x = static_cast<int>(std::lround(transformed_x));
        ipm_y = static_cast<int>(std::lround(transformed_y));
        return ipm_x >= 0 && ipm_x < MT9V03X_W && ipm_y >= 0 && ipm_y < MT9V03X_H;
    }

    bool is_ipm_continuous(float x1, float y1, float x2, float y2)
    {
        const float dx = x2 - x1;
        const float dy = y2 - y1;
        return dx * dx + dy * dy <= k_ipm_break_distance * k_ipm_break_distance;
    }

    bool is_ipm_span_continuous(const uint16 points[MT9V03X_H][2], uint16 start, uint16 end)
    {
        for (uint16 i = start + 1; i <= end; ++i)
        {
            if (!is_ipm_continuous(points[i - 1][0], points[i - 1][1],
                                   points[i][0], points[i][1]))
            {
                return false;
            }
        }
        return true;
    }

    void smooth_and_resample_ipm_points(uint16 points[MT9V03X_H][2], uint16 &count)
    {
        if (count < 2)
        {
            return;
        }

        float smooth_points[MT9V03X_H][2];
        for (uint16 i = 0; i < count; ++i)
        {
            smooth_points[i][0] = static_cast<float>(points[i][0]);
            smooth_points[i][1] = static_cast<float>(points[i][1]);
        }

        for (uint16 i = 1; i + 1 < count; ++i)
        {
            if (is_ipm_continuous(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1]) &&
                    is_ipm_continuous(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1]))
            {
                smooth_points[i][0] = (static_cast<float>(points[i - 1][0]) +
                                       2.0f * static_cast<float>(points[i][0]) +
                                       static_cast<float>(points[i + 1][0])) * 0.25f;
                smooth_points[i][1] = (static_cast<float>(points[i - 1][1]) +
                                       2.0f * static_cast<float>(points[i][1]) +
                                       static_cast<float>(points[i + 1][1])) * 0.25f;
            }
        }

        uint16 resampled[MT9V03X_H][2];
        uint16 resampled_count = 0;
        auto append_point = [&](float x, float y) {
            if (resampled_count >= MT9V03X_H)
            {
                return false;
            }
            resampled[resampled_count][0] = static_cast<uint16>(std::clamp<int>(static_cast<int>(std::lround(x)), 0, MT9V03X_W - 1));
            resampled[resampled_count][1] = static_cast<uint16>(std::clamp<int>(static_cast<int>(std::lround(y)), 0, MT9V03X_H - 1));
            resampled_count++;
            return true;
        };
        auto append_if_needed = [&](float x, float y) {
            if (resampled_count == 0)
            {
                return append_point(x, y);
            }
            const float dx = x - static_cast<float>(resampled[resampled_count - 1][0]);
            const float dy = y - static_cast<float>(resampled[resampled_count - 1][1]);
            if (dx * dx + dy * dy > 1.0f)
            {
                return append_point(x, y);
            }
            return true;
        };

        append_point(smooth_points[0][0], smooth_points[0][1]);
        float remain_distance = k_ipm_sample_distance;
        float last_raw_x = smooth_points[0][0];
        float last_raw_y = smooth_points[0][1];

        for (uint16 i = 1; i < count && resampled_count < MT9V03X_H; ++i)
        {
            float ax = last_raw_x;
            float ay = last_raw_y;
            const float bx = smooth_points[i][0];
            const float by = smooth_points[i][1];
            float dx = bx - ax;
            float dy = by - ay;
            float segment_len = std::sqrt(dx * dx + dy * dy);

            if (segment_len > k_ipm_break_distance)
            {
                append_if_needed(ax, ay);
                append_point(bx, by);
                remain_distance = k_ipm_sample_distance;
                last_raw_x = bx;
                last_raw_y = by;
                continue;
            }

            while (segment_len >= remain_distance && segment_len > 1e-3f && resampled_count < MT9V03X_H)
            {
                const float t = remain_distance / segment_len;
                const float sample_x = ax + dx * t;
                const float sample_y = ay + dy * t;
                append_point(sample_x, sample_y);
                ax = sample_x;
                ay = sample_y;
                dx = bx - ax;
                dy = by - ay;
                segment_len = std::sqrt(dx * dx + dy * dy);
                remain_distance = k_ipm_sample_distance;
            }

            if (segment_len > 1e-3f)
            {
                remain_distance -= segment_len;
            }
            last_raw_x = bx;
            last_raw_y = by;
        }

        append_if_needed(last_raw_x, last_raw_y);
        for (uint16 i = 0; i < resampled_count; ++i)
        {
            points[i][0] = resampled[i][0];
            points[i][1] = resampled[i][1];
        }
        count = resampled_count;
    }

    void build_ipm_offset_midline(const uint16 edge_points[MT9V03X_H][2],
                                  uint16 edge_count,
                                  bool from_left_edge,
                                  float offset_px)
    {
        Ipm_Mid_Point_Count = 0;
        if (edge_count <= k_ipm_midline_tangent_span * 2)
        {
            return;
        }

        const float direction = from_left_edge ? 1.0f : -1.0f;

        for (uint16 i = k_ipm_midline_tangent_span;
                i + k_ipm_midline_tangent_span < edge_count && Ipm_Mid_Point_Count < MT9V03X_H; ++i)
        {
            const uint16 prev = i - k_ipm_midline_tangent_span;
            const uint16 next = i + k_ipm_midline_tangent_span;
            if (!is_ipm_span_continuous(edge_points, prev, next))
            {
                // 这一小段无法计算可靠切线，跳过该点；后续有效 IPM 边线仍然继续偏移。
                continue;
            }

            const float dx = static_cast<float>(edge_points[next][0]) - static_cast<float>(edge_points[prev][0]);
            const float dy = static_cast<float>(edge_points[next][1]) - static_cast<float>(edge_points[prev][1]);
            const float len = std::sqrt(dx * dx + dy * dy);
            if (len < 1e-3f)
            {
                continue;
            }

            const float tx = dx / len;
            const float ty = dy / len;
            const float nx = -ty;
            const float ny = tx;
            const float mid_x = static_cast<float>(edge_points[i][0]) + direction * nx * offset_px;
            const float mid_y = static_cast<float>(edge_points[i][1]) + direction * ny * offset_px;
            Ipm_Mid_Points[Ipm_Mid_Point_Count][0] = static_cast<uint16>(std::clamp<int>(static_cast<int>(std::lround(mid_x)), 0, MT9V03X_W - 1));
            Ipm_Mid_Points[Ipm_Mid_Point_Count][1] = static_cast<uint16>(std::clamp<int>(static_cast<int>(std::lround(mid_y)), 0, MT9V03X_H - 1));
            Ipm_Mid_Point_Count++;
        }

        // 普通场景才使用车头锚点；圆环内保留原有的丢线边偏移逻辑。
        const bool allow_lost_edge_offset = allow_lost_edge_offset_in_circle();
        if (!allow_lost_edge_offset && Ipm_Mid_Point_Count > 0)
        {
            const float first_x = static_cast<float>(Ipm_Mid_Points[0][0]);
            const float first_y = static_cast<float>(Ipm_Mid_Points[0][1]);
            const float first_gap = std::hypot(first_x - k_redline_front_anchor_x,
                                               first_y - k_redline_front_anchor_y);

            if (first_y < k_redline_front_anchor_y - 1.0f &&
                first_gap <= k_redline_front_anchor_max_gap)
            {
                const uint16 old_count = Ipm_Mid_Point_Count;
                // 锚点到首个真实红点按约 3.9px 分段，避免单段超过蓝线 18px 断点阈值。
                const uint16 bridge_segment_count = std::max<uint16>(
                    1,
                    static_cast<uint16>(std::ceil(first_gap / k_ipm_sample_distance)));
                const uint16 prepend_count = std::min<uint16>(bridge_segment_count, MT9V03X_H);
                const uint16 kept_old_count = std::min<uint16>(
                    old_count,
                    static_cast<uint16>(MT9V03X_H - prepend_count));

                for (uint16 j = kept_old_count; j > 0; --j)
                {
                    Ipm_Mid_Points[prepend_count + j - 1][0] = Ipm_Mid_Points[j - 1][0];
                    Ipm_Mid_Points[prepend_count + j - 1][1] = Ipm_Mid_Points[j - 1][1];
                }

                for (uint16 j = 0; j < prepend_count; ++j)
                {
                    const float t = static_cast<float>(j) /
                                    static_cast<float>(bridge_segment_count);
                    const float bridge_x = k_redline_front_anchor_x +
                                           (first_x - k_redline_front_anchor_x) * t;
                    const float bridge_y = k_redline_front_anchor_y +
                                           (first_y - k_redline_front_anchor_y) * t;
                    Ipm_Mid_Points[j][0] = static_cast<uint16>(std::clamp<int>(
                        static_cast<int>(std::lround(bridge_x)), 0, MT9V03X_W - 1));
                    Ipm_Mid_Points[j][1] = static_cast<uint16>(std::clamp<int>(
                        static_cast<int>(std::lround(bridge_y)), 0, MT9V03X_H - 1));
                }
                Ipm_Mid_Point_Count = prepend_count + kept_old_count;
            }
        }
    }

    void build_ipm_bilateral_midline_from_lines(const int16 ipm_left_line[MT9V03X_H],
                                                const int16 ipm_right_line[MT9V03X_H])
    {
        Ipm_Bilateral_Mid_Point_Count = 0;
        for (int y = MT9V03X_H - 1;
                y >= 0 && Ipm_Bilateral_Mid_Point_Count < MT9V03X_H;
                --y)
        {
            const int left_x = ipm_left_line[y];
            const int right_x = ipm_right_line[y];
            if (left_x < 0 || right_x < 0 || left_x >= right_x)
            {
                continue;
            }

            const int mid_x = (left_x + right_x) / 2;
            Ipm_Bilateral_Mid_Points[Ipm_Bilateral_Mid_Point_Count][0] =
                static_cast<uint16>(std::clamp<int>(mid_x, 0, MT9V03X_W - 1));
            Ipm_Bilateral_Mid_Points[Ipm_Bilateral_Mid_Point_Count][1] =
                static_cast<uint16>(y);
            Ipm_Bilateral_Mid_Point_Count++;
        }

        smooth_and_resample_ipm_points(Ipm_Bilateral_Mid_Points, Ipm_Bilateral_Mid_Point_Count);
    }

    void draw_ipm_midline(uint16 (*img)[image_width], uint16 color)
    {
        for (uint16 i = 1; i < Ipm_Mid_Point_Count; ++i)
        {
            if (is_ipm_continuous(Ipm_Mid_Points[i - 1][0], Ipm_Mid_Points[i - 1][1],
                                  Ipm_Mid_Points[i][0], Ipm_Mid_Points[i][1]))
            {
                dbg_line(img, Ipm_Mid_Points[i - 1][0], Ipm_Mid_Points[i - 1][1],
                         Ipm_Mid_Points[i][0], Ipm_Mid_Points[i][1], color);
            }
        }
    }

}

void build_ipm_midline(ReliableEdgeMode mode)
{
    ReliableEdgeMode reliable_edge_mode = mode;
    if (reliable_edge_mode == ReliableEdgeMode::Auto)
    {
        reliable_edge_mode = selected_ipm_reliable_edge_mode();
    }
    // offset_px 是“可靠边 -> 临时中线”的法向偏移。绕行时可被运行时改成 0。
    const float offset_px = g_ipm_midline_offset_px.load();

    if (reliable_edge_mode == ReliableEdgeMode::ForceLeft)
    {
        build_ipm_offset_midline(Ipm_Left_Points, Ipm_Left_Point_Count, true, offset_px);
    }
    else
    {
        build_ipm_offset_midline(Ipm_Right_Points, Ipm_Right_Point_Count, false, offset_px);
    }
}

/*

//修改：已经改成点集形式输出到 Ipm_Left_Points / Ipm_Right_Points，同时 ipm_left_line / ipm_right_line 仍然保留行数组格式，记录每行的 IPM x 坐标，便于后续处理。转换过程中会根据可靠性和行优先原则选择点，并统计有效点数量。
从MT9V03X_H - 4--》》hightest
*/
void transform_lines_to_ipm(const uint8 left_line[MT9V03X_H], const uint8 right_line[MT9V03X_H], int16 ipm_left_line[MT9V03X_H], int16 ipm_right_line[MT9V03X_H])
{
    int16 left_source_row[MT9V03X_H];
    int16 right_source_row[MT9V03X_H];
    Ipm_Left_Point_Count = 0;
    Ipm_Right_Point_Count = 0;
    for (int row = 0; row < MT9V03X_H; ++row)
    {
        ipm_left_line[row] = -1;
        ipm_right_line[row] = -1;
        left_source_row[row] = -1;
        right_source_row[row] = -1;
    }

    for (int row = MT9V03X_H - 4; row >= std::max<int>(hightest, 0); --row)
    {
        const int left = left_line[row];
        const int right = right_line[row];
        // 只有圆环状态1~3沿用旧规则；状态4~6和其他场景均过滤丢线哨兵值。
        // 其他场景过滤左边 x=1/2，但保留 Cross_fill 后的有效补线。
        const bool allow_lost_edge_offset = allow_lost_edge_offset_in_circle();
        const bool left_control_valid = allow_lost_edge_offset
                                            ? (left > Border_Min && left < Border_Max)
                                            : (left > Border_Min + 1 && left < Border_Max);
        const bool right_control_valid = allow_lost_edge_offset
                                             ? (right > Border_Min && right < Border_Max)
                                             : (right > Border_Min && right < Border_Max - 1);
        if (left_control_valid)
        {
            int ipm_x = 0;
            int ipm_y = 0;
            if (transform_image_to_ipm_point(left, row, ipm_x, ipm_y))
            {
                if (ipm_left_line[ipm_y] < 0 || row > left_source_row[ipm_y])
                {
                    ipm_left_line[ipm_y] = static_cast<int16>(ipm_x);
                    left_source_row[ipm_y] = static_cast<int16>(row);
                }
                if (Ipm_Left_Point_Count < MT9V03X_H)
                {
                    Ipm_Left_Points[Ipm_Left_Point_Count][0] = static_cast<uint16>(ipm_x);
                    Ipm_Left_Points[Ipm_Left_Point_Count][1] = static_cast<uint16>(ipm_y);
                    Ipm_Left_Point_Count++;
                }
            }
        }
        if (right_control_valid)
        {
            int ipm_x = 0;
            int ipm_y = 0;
            if (transform_image_to_ipm_point(right, row, ipm_x, ipm_y))
            {
                if (ipm_right_line[ipm_y] < 0 || row > right_source_row[ipm_y])
                {
                    ipm_right_line[ipm_y] = static_cast<int16>(ipm_x);
                    right_source_row[ipm_y] = static_cast<int16>(row);
                }
                if (Ipm_Right_Point_Count < MT9V03X_H)
                {
                    Ipm_Right_Points[Ipm_Right_Point_Count][0] = static_cast<uint16>(ipm_x);
                    Ipm_Right_Points[Ipm_Right_Point_Count][1] = static_cast<uint16>(ipm_y);
                    Ipm_Right_Point_Count++;
                }
            }
        }
    }

    smooth_and_resample_ipm_points(Ipm_Left_Points, Ipm_Left_Point_Count);
    smooth_and_resample_ipm_points(Ipm_Right_Points, Ipm_Right_Point_Count);
}

//直接将点集转换成 IPM 线数组，保留行优先原则和可靠性选择，但不再输出点集到 Ipm_Left_Points / Ipm_Right_Points。
// void transform_points_to_ipm_line(const uint16 points[][2], uint16 count, int16 ipm_line[MT9V03X_H])
// {
//     int16 source_row[MT9V03X_H];
//     for (int row = 0; row < MT9V03X_H; ++row)
//     {
//         ipm_line[row] = -1;
//         source_row[row] = -1;
//     }

//     for (uint16 i = 0; i < count; ++i)
//     {
//         const int x = points[i][0];
//         const int y = points[i][1];
//         int ipm_x = 0;
//         int ipm_y = 0;
//         if (transform_image_to_ipm_point(x, y, ipm_x, ipm_y) &&
//                 (ipm_line[ipm_y] < 0 || y > source_row[ipm_y]))
//         {
//             ipm_line[ipm_y] = static_cast<int16>(ipm_x);
//             source_row[ipm_y] = static_cast<int16>(y);
//         }
//     }
// }

/*
黄色测试中线生成函数，只用于图传对比，不参与控制。它根据当前 TestMidlineMode 判断使用左边、右边，还是自动可靠边。现在的版本还是基于原图 Left_Line/Right_Line 做“动态宽度倍率 + 同行偏移”：
*/
void build_test_midline(TestMidlineMode mode)
{
    const bool left_good = Left_Lost_Time < 35 &&
                           data_stastics_l > 25 &&
                           continuity_change_left_flag == 0;
    const bool right_good = Right_Lost_Time < 35 &&
                            data_stastics_r > 25 &&
                            continuity_change_right_flag == 0;
    const bool use_left = (mode == TestMidlineMode::ForceLeft) ||
                          (mode == TestMidlineMode::Auto && left_good && !right_good);
    const bool use_right = (mode == TestMidlineMode::ForceRight) ||
                           (mode == TestMidlineMode::Auto && right_good && !left_good);

    for (int row = 0; row < MT9V03X_H; ++row)
    {
        Test_Mid_Line[row] = 0;
    }

    constexpr int k_sample_margin_rows = 5;
    const int valid_top_row = std::clamp<int>(hightest + k_sample_margin_rows,
                                              k_sample_margin_rows,
                                              MT9V03X_H - k_sample_margin_rows - 1);
    const int valid_bottom_row = MT9V03X_H - k_sample_margin_rows - 1;

    float width_scale_sum = 0.0f;
    int width_scale_count = 0;
    for (int row = valid_top_row; row <= valid_bottom_row; ++row)
    {
        const int left = Left_Line[row];
        const int right = Right_Line[row];
        const int measured_width = right - left;
        const int standard_width = Standard_Road_Wide[row];
        if (left > Border_Min + 2 && right < Border_Max - 2 &&
                measured_width > standard_width / 2 &&
                measured_width < standard_width * 3 / 2)
        {
            width_scale_sum += static_cast<float>(measured_width) / static_cast<float>(standard_width);
            width_scale_count++;
        }
    }

    const float width_scale = (width_scale_count > 0)
                              ? std::clamp(width_scale_sum / static_cast<float>(width_scale_count), 0.70f, 1.30f)
                              : 1.0f;

    for (int row = valid_top_row; row <= valid_bottom_row; ++row)
    {
        int mid = (static_cast<int>(Left_Line[row]) + static_cast<int>(Right_Line[row])) / 2;
        const int half_width = static_cast<int>(std::lround(static_cast<float>(Standard_Road_Wide[row]) * width_scale * 0.5f));
        if (use_left)
        {
            mid = static_cast<int>(Left_Line[row]) + half_width;
        }
        else if (use_right)
        {
            mid = static_cast<int>(Right_Line[row]) - half_width;
        }

        Test_Mid_Line[row] = static_cast<uint8>(std::clamp(mid, 0, image_width - 1));
    }

    for (int row = valid_bottom_row - 1; row >= valid_top_row; --row)
    {
        const int delta = static_cast<int>(Test_Mid_Line[row]) - static_cast<int>(Test_Mid_Line[row + 1]);
        if (delta > 8)
        {
            Test_Mid_Line[row] = static_cast<uint8>(std::clamp<int>(Test_Mid_Line[row + 1] + 4, 0, image_width - 1));
        }
        else if (delta < -8)
        {
            Test_Mid_Line[row] = static_cast<uint8>(std::clamp<int>(Test_Mid_Line[row + 1] - 4, 0, image_width - 1));
        }
    }
}
//绝对值
int my_abs(int value)
{
    // 如果输入值为非负数，则直接返回该值
    if(value>=0) return value;
    // 如果输入值为负数，则返回其相反数，即正数
    else return -value;
}

//上下限
int16 limit_a_b(int16 x, int a, int b)
{
    // 检查x是否小于下限a，如果是，则将x设置为a
    if(x<a) x = a;
    // 检查x是否大于上限b，如果是，则将x设置为b
    if(x>b) x = b;
    // 返回限制后的x值
    return x;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     大津法求阈值
  @param     image       图像数组
             col         列
             row         行
  @return    threshold   返回int阈值数值
  Sample     threshold=otsuThreshold(image_copy[0],MT9V03X_W, MT9V03X_H);
  @note      no
-------------------------------------------------------------------------------------------------------------------*/
// 大津法计算当前灰度图的二值化阈值。
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X;
    uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;

    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }
    for (MinValue = 0; MinValue < 255 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];    //前景像素点数
        PixelFore = Amount - PixelBack;         //背景像素点数
        if (PixelBack == 0 || PixelFore == 0)
        {
            continue;
        }
        OmegaBack = (double)PixelBack / Amount;//前景像素百分比
        OmegaFore = (double)PixelFore / Amount;//背景像素百分比
        PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
        MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
        MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
        if (Sigma > SigmaB)//遍历最大的类间方差g
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    return Threshold;
}

int Threshold = 0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     图像二值化处理函数
  @return    二值化之后的图像数组
  Sample     图像二值化
  @note      二值化处理，0黑，255是白，
-------------------------------------------------------------------------------------------------------------------*/
// 使用 Threshold 将 image_copy 转成黑白二值图 bin_image。
void turn_to_bin(void)
{
    uint8 i, j;

    const int lower_threshold = Threshold;
    const int upper_threshold = std::min(Threshold + 5, 255);
    const uint8 split_row = MT9V03X_H / 2;

    for (i = 0; i < MT9V03X_H; i++)
    {
        const int row_threshold = (i < split_row) ? upper_threshold : lower_threshold;

        for (j = 0; j < MT9V03X_W; j++)
        {
            if (image_copy[i][j] > row_threshold)
                bin_image[i][j] = 255;
            else
                bin_image[i][j] = 0;
        }
    }
}

// 对二值图做简单形态学滤波，补小洞并去除孤立噪点。
void image_filter(uint8(*bin_image)[MT9V03X_W])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;
    for (i = 1; i < MT9V03X_H - 1; i++)
    {
        for (j = 1; j < (MT9V03X_W - 1); j++)
        {
            //统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
                + bin_image[i][j - 1] + bin_image[i][j + 1]
                + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];
            if (num >= threshold_max && bin_image[i][j] == 0)
            {
                bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {
                bin_image[i][j] = 0;//黑
            }

        }
    }

}

/**
 * 在图像上绘制矩形框。
 * 该函数通过将图像数组的左右边缘和顶部像素点设置为0，防止边界搜索越界。
 *
 * @param image 指向图像数组的指针，图像数组的类型为uint8，大小为MT9V03X_W。
 *              图像数组由调用该函数的外部程序提供。
 */
// 给二值图画黑色边框，辅助边界搜索在图像内闭合。
void image_draw_rectan(uint8(*image)[MT9V03X_W])
{
    // 初始化循环变量i为0。
    uint8 i = 0;

    // 清除图像数组的左侧和右侧边缘像素点。
    for (i = 0; i < MT9V03X_H - 1; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][MT9V03X_W - 1] = 0;
        image[i][MT9V03X_W - 2] = 0;
    }

    // 清除图像数组的顶部边缘像素点，并临时补一层底部黑框。
    for (i = 0; i < MT9V03X_W - 1; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        image[2][i] = 0;
        image[3][i] = 0;
        image[MT9V03X_H - 1][i] = 0;
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     画线
  @param     输入起始点，终点坐标，补一条宽度为2的黑线
  @return    null
  Sample     Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-1);
             Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-1);
                                    画一个大×
  @note     补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i,x,y;
    int start=0,end=0;
    if(startX>=MT9V03X_W-1)//限幅处理
        startX=MT9V03X_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=MT9V03X_H-1)
        startY=MT9V03X_H-1;
    else if(startY<=0)
        startY=0;
    if(endX>=MT9V03X_W-1)
        endX=MT9V03X_W-1;
    else if(endX<=0)
        endX=0;
    if(endY>=MT9V03X_H-1)
        endY=MT9V03X_H-1;
    else if(endY<=0)
        endY=0;
    if(startX==endX)//一条竖线
    {
        if (startY > endY)//互换
        {
            start=endY;
            end=startY;
        }
        else
        {
            start=startY;
            end=endY;
        }
        for (i = start; i <= end; i++)
        {
            if(i<=2)
                i=2;
            bin_image[i][startX]=IMG_BLACK;
            bin_image[i-1][startX]=IMG_BLACK;
            bin_image[i-2][startX]=IMG_BLACK;
        }
    }
    else if(startY == endY)//补一条横线
    {
        if (startX > endX)//互换
        {
            start=endX;
            end=startX;
        }
        else
        {
            start=startX;
            end=endX;
        }
        for (i = start; i <= end; i++)
        {
            if(startY<=1)
                startY=2;
            bin_image[startY][i]=IMG_BLACK;
            bin_image[startY-1][i]=IMG_BLACK;
            bin_image[startY-2][i]=IMG_BLACK;
        }
    }
    else //上面两个是水平，竖直特殊情况，下面是常见情况
    {
        if(startY>endY)//起始点矫正
        {
            start=endY;
            end=startY;
        }
        else
        {
            start=startY;
            end=endY;
        }
        for (i = start; i <= end; i++)//纵向补线，保证每一行都有黑点
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//两点式变形
            if(x>=MT9V03X_W-3)
                x=MT9V03X_W-3;
            else if (x<=3)
                x=3;
            bin_image[i][x] = IMG_BLACK;
            bin_image[i][x-1] = IMG_BLACK;
            bin_image[i][x-2] = IMG_BLACK;
            bin_image[i][x-3] = IMG_BLACK;
        }
        if(startX>endX)
        {
            start=endX;
            end=startX;
        }
        else
        {
            start=startX;
            end=endX;
        }
        for (i = start; i <= end; i++)//横向补线，保证每一列都有黑点
        {
            y =(int)(startY+(endY-startY)*(i-startX)/(endX-startX));//两点式变形
            if(y>=MT9V03X_H-1)
                y=MT9V03X_H-1;
            else if (y<=0)
                y=0;
            bin_image[y][i] = IMG_BLACK;
        }
    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
        x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
    if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
    else if(y1<=0)
        y1=0;
    if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
    else if(x2<=0)
        x2=0;
    if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
    else if(y2<=0)
        y2=0;
    if (y1 == y2)
    {
        hx = (x1 + x2) / 2;
        hx = limit_a_b(hx, Border_Min, Border_Max);
        Left_Line[y1] = hx;
        return;
    }
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1; i<=a2; i++) //根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        hx = limit_a_b(hx, Border_Min, Border_Max);
        Left_Line[i]=hx;
//        ips200_draw_point((uint16)i, (uint16)Left_Line[i], RGB565_BLUE);

    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     补线的起点，终点
  @return    null
  Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的，不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1,int y1,int x2,int y2)//右补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
        x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
    if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
    else if(y1<=0)
        y1=0;
    if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
    else if(x2<=0)
        x2=0;
    if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
    else if(y2<=0)
        y2=0;
    if (y1 == y2)
    {
        hx = (x1 + x2) / 2;
        hx = limit_a_b(hx, Border_Min, Border_Max);
        Right_Line[y1] = hx;
        return;
    }
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1; i<=a2; i++) //根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        hx = limit_a_b(hx, Border_Min, Border_Max);
        Right_Line[i]=hx;
//        ips200_draw_point((uint16)i, (uint16)Right_Line[i], RGB565_BLUE);

    }
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找5个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Left_Add_Line(Left_Line[start],start,Left_Line[end],end);
    }

    else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/4.0f;//这里的k是1/斜率
        for(i=start; i<=end; i++)
        {
            Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
            Left_Line[i]=limit_a_b(Left_Line[i], Border_Min, Border_Max);
//            ips200_draw_point((uint16)i, (uint16)Left_Line[i], RGB565_YELLOW);
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Stop_Detect(void)
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(Right_Line[start],start,Right_Line[end],end);
    }
    else
    {
        k=(float)(Right_Line[start]-Right_Line[start-4])/4.0f;//这里的k是1/斜率
        for(i=start; i<=end; i++)
        {
            Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
            Right_Line[i]=limit_a_b(Right_Line[i], Border_Min, Border_Max);
//            ips200_draw_point((uint16)i,(uint16)Right_Line[i], RGB565_YELLOW);
        }
    }
}




#if 0 // 旧版最长白列起点参考函数，当前已停用但保留代码
// 从底部向上统计白列长度，寻找左右边界搜索的起始参考列。
void Get_Longest_Line(void)
{
    int16 i,j;
    uint8 start_col = 10;
    uint8 end_col =MT9V03X_W - 9;

    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列  2
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列

    for(i = 0; i < MT9V03X_W - 1; i++)		//将横向数据清零
    {
        White_col[i] = 0;
    }

    for(j = start_col; j <= end_col; j++)
    {
        for(i = MT9V03X_H - 2; i >= 2; i--)
        {
            if(bin_image[i][j] == 255)
            {
                if((bin_image[i][j]==255&&bin_image[i-1][j]==0&&bin_image[i-2][j]==0&&bin_image[i+1][j]==255))
                    break;

                White_col[j]++;
            }
        }
    }

    //从左到右找左边最长白列
    Longest_White_Column_Left[0] =0;
    for(i=start_col; i<=end_col; i++)
    {
        if (Longest_White_Column_Left[0] < White_col[i])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_col[i];//【0】是白列长度
            Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    Longest_White_Column_Right[0] = 0;//【0】是白列长度          /////
    for(i=end_col; i>=start_col; i--) //从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_col[i])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_col[i];//【0】是白列长度
            Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
        }
    }

}

#endif

/**
 * get_lost_line函数用于检测并统计在图像中丢失的线条信息
 * 该函数没有输入参数和返回值
 *
 * 本函数首先从图像的底部开始向上遍历，检查左右两边的线条数据
 * 如果检测到左边的线条数据为1，则设置相应的丢失标志
 * 如果检测到右边的线条数据为138，则设置相应的丢失标志
 *
 * 接着，再次从图像底部开始向上遍历，根据之前设置的丢失标志进行统计
 * 统计左边、右边以及双边同时丢失线条的数量
 */
// 根据左右边界是否贴边统计丢线行数，并返回丢线类型。
uint8_t get_lost_line(void)
{
    uint16 i;

    Left_Lost_Time=0;
    Right_Lost_Time=0;
    Both_Lost_Time=0;

    for(i =0; i <= MT9V03X_H-1; i++)				//将竖向数据清零
    {
        Left_Lost_Flag[i] = 0;
        Right_Lost_Flag[i] =0;
    }
    // 从图像底部开始向上遍历，检查并标记丢失的线条
//  for(i = 0; i <= data_stastics_l - 1; i++)
//  {
//    if(points_l[i][0] <=2)
//    {
//      Left_Lost_Flag[i] = 1;
//			Left_Lost_Time++;
//    }
//  }
//  for(i = 0; i <= data_stastics_r-1; i++)
//  {
//    if(points_r[i][0] >=MT9V03X_W-3)
//    {
//      Right_Lost_Flag[i] = 1;
//			Right_Lost_Time++;
//    }
//  }
    for(i = 1; i < MT9V03X_H - 2; i++)
    {
        if(Left_Line[i] <= 2)
        {
            Left_Lost_Flag[i] = 1;
            Left_Lost_Time++;
        }

        if(Right_Line[i] >= MT9V03X_W - 3)//157
        {
            Right_Lost_Flag[i] = 1;
            Right_Lost_Time++;
        }
    }
    // 赛道数据初步分析，统计左右两边及双边同时丢失线条的数量
    for (i = MT9V03X_H - 1; i > 1; i--)
    {
//    if (Left_Lost_Flag[i]  == 1)//单边丢线数
//      Left_Lost_Time++;
//    if (Right_Lost_Flag[i] == 1)
//      Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//双边丢线数
            Both_Lost_Time++;
    }

    if((Left_Lost_Time > k_lost_line_count_threshold) &&
            Right_Lost_Time > k_lost_line_count_threshold)
    {
        return 1;
    }
    else if(Left_Lost_Time > k_lost_line_count_threshold)
    {
        return 2;
    }
    else if(Right_Lost_Time > k_lost_line_count_threshold)
    {
        return 3;
    }
    else
    {
        return 0;
    }
}

// 最终丢线判定：基于十字/圆环补线后的当前边线，不改写补线前丢线统计。
bool is_lost_line(void)
{
    constexpr int k_final_lost_line_both_threshold = 55;
    int post_both_lost_time = 0;

    for (uint16 i = 1; i < MT9V03X_H - 2; i++)
    {
        if (Left_Line[i] <= 2 && Right_Line[i] >= MT9V03X_W - 3)
        {
            post_both_lost_time++;
        }
    }

    return post_both_lost_time > k_final_lost_line_both_threshold;
}

#define border_max               MT9V03X_W-2    //边界最大值
#define border_min               1           //边界最小值


/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
备    注：
example：  get_start_point(image_h-2)
 */
// uint8 Mid_start_col =0; // 旧版最长白列起点中点，当前已停用。

#if 0 // 旧版：基于最长白列估计中点，再找单像素黑白跳变。保留备查。
uint8 Mid_start_col =0;

uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y
    Image_Flag.L_Find = false;
    Image_Flag.R_Find = false;

    Mid_start_col=(Longest_White_Column_Left[1] +Longest_White_Column_Right[1])/2;
//    Mid_start_col =MT9V03X_W/2;
    //从中间往左边，先找起点
    for (i = Mid_start_col; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            Image_Flag.L_Find = true;
            break;
        }
    }

    for (i = Mid_start_col; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            Image_Flag.R_Find = true;
            break;
        }
    }

    if(l_found&&r_found)
    {
        Image_Flag.Get_Start_Point = 1;
        return 1;
    }
    else
    {
        //printf("未找到起点\n");
        Image_Flag.Get_Start_Point = 0;
        return 0;
    }
}
#endif

// 在指定起始行寻找最长白色赛道段，并用连续黑点确认左右边界起点。
// 在二值图的底部区域寻找左右边界起点。
// 优先从传入的 start_row（通常为 MT9V03X_H-2，即倒数第 2 行）开始；
// 若当前行无法同时确认左右起点，则逐行向上重试，最多搜索 7 行。
// 任意一行左右起点均成立则立即返回 1，避免因底行局部噪点/阴影导致整帧作废。
uint8 get_start_point(uint8 start_row)
{
    const int base_row = std::clamp<int>(start_row, 1, MT9V03X_H - 2);
    const int search_left = border_min + 1;
    const int search_right = border_max - 1;

    if (search_left > search_right)
    {
        Image_Flag.Get_Start_Point = 0;
        return 0;
    }

    // 底行可能因阴影、噪点或二值化波动丢失赛道边缘，
    // 向上最多试探 7 行（约 6% 图像高度），大幅降低"整帧因底行作废"的概率。
    constexpr int k_start_point_max_search_rows = 7;
    for (int row_offset = 0; row_offset < k_start_point_max_search_rows; ++row_offset)
    {
        const int row = base_row - row_offset;
        if (row < 1)
        {
            break;
        }

        // ----- 在该行找最长白色赛道段 -----
        int best_white_left = -1;
        int best_white_right = -1;
        int best_white_length = 0;

        start_point_l[0] = 0;
        start_point_l[1] = static_cast<uint8>(row);
        start_point_r[0] = 0;
        start_point_r[1] = static_cast<uint8>(row);
        Image_Flag.L_Find = false;
        Image_Flag.R_Find = false;

        int col = search_left;
        while (col <= search_right)
        {
            while (col <= search_right && bin_image[row][col] != IMG_WHITE)
            {
                col++;
            }
            if (col > search_right)
            {
                break;
            }

            const int white_left = col;
            while (col <= search_right && bin_image[row][col] == IMG_WHITE)
            {
                col++;
            }

            const int white_right = col - 1;
            const int white_length = white_right - white_left + 1;
            if (white_length > best_white_length)
            {
                best_white_left = white_left;
                best_white_right = white_right;
                best_white_length = white_length;
            }
        }

        // 该行无白色段（赛道被二值化为全黑），尝试上一行
        if (best_white_length <= 0)
        {
            continue;
        }

        const int mid_col = (best_white_left + best_white_right) / 2;

        for (int x = mid_col; x >= best_white_left; x--)
        {
            const int black_start = x - 1;
            const int black_end = black_start - (k_start_black_confirm_count - 1);
            if (black_end < 0)
            {
                continue;
            }

            bool black_confirmed = true;
            for (int bx = black_start; bx >= black_end; bx--)
            {
                if (bin_image[row][bx] != IMG_BLACK)
                {
                    black_confirmed = false;
                    break;
                }
            }

            if (black_confirmed)
            {
                start_point_l[0] = static_cast<uint8>(x);
                Image_Flag.L_Find = true;
                break;
            }
        }

        for (int x = mid_col; x <= best_white_right; x++)
        {
            const int black_start = x + 1;
            const int black_end = black_start + (k_start_black_confirm_count - 1);
            if (black_end >= MT9V03X_W)
            {
                continue;
            }

            bool black_confirmed = true;
            for (int bx = black_start; bx <= black_end; bx++)
            {
                if (bin_image[row][bx] != IMG_BLACK)
                {
                    black_confirmed = false;
                    break;
                }
            }

            if (black_confirmed)
            {
                start_point_r[0] = static_cast<uint8>(x);
                Image_Flag.R_Find = true;
                break;
            }
        }

        Image_Flag.Get_Start_Point = Image_Flag.L_Find && Image_Flag.R_Find;
        if (Image_Flag.Get_Start_Point)
        {
            return 1;
        }
    }

    // 7 行全部搜索失败：该帧确实无法找到有效边界起点，
    // 清空标志位，由上层 Image_Process 走丢线兜底流程。
    Image_Flag.L_Find = false;
    Image_Flag.R_Find = false;
    Image_Flag.Get_Start_Point = 0;
    return 0;
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2026年5月23日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */

// 从左右起点开始做八邻域边界跟踪，输出 points_l/points_r 点集和最高搜索行。
void search_l_r(uint16 break_flag, uint8(*image)[MT9V03X_W], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint16 i = 0, j = 0;

    //左边变量
    int16 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    int16 temp_l[8][2] = { {  0 } };
    uint8 temp_dir_l[8] = {0};
    int16 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    int16 search_filds_r[8][2] = { {  0 } };
    int16 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    int16 temp_r[8][2] = { {  0 } };
    uint8 temp_dir_r[8] = {0};
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针


    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

    uint8 left_run = 1;
    uint8 right_run = 1;
    uint8 left_active = 1;
    uint8 right_active = 1;

    //开启邻域循环：左右边界独立运行，某一侧停止后另一侧仍可继续向上搜索。
    while (break_flag-- && (left_active || right_active))
    {
        if (left_active && left_run)
        {
            if (l_data_statics >= USE_num)
            {
                left_active = 0;
                left_run = 0;
            }
            else
            {
                //左边
                for (i = 0; i < 8; i++)//传递8F坐标
                {
                    search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
                    search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
                }
                //中心坐标点填充到已经找到的点内
                points_l[l_data_statics][0] = center_point_l[0];//x
                points_l[l_data_statics][1] = center_point_l[1];//y
                l_data_statics++;//索引加一

                if (center_point_l[1] <= k_search_top_stop_row)
                {
                    left_active = 0;
                    left_run = 0;
                }
                else
                {
                    index_l = 0;//先清零，后使用
                    for (i = 0; i < 8; i++)
                    {
                        temp_l[i][0] = 0;//先清零，后使用
                        temp_l[i][1] = 0;//先清零，后使用
                        temp_dir_l[i] = 0;
                    }

                    //左边判断
                    for (i = 0; i < 8; i++)
                    {
                        if (safe_image_pixel(image, search_filds_l[i][0], search_filds_l[i][1]) == 0
                                && safe_image_pixel(image, search_filds_l[(i + 1) & 7][0], search_filds_l[(i + 1) & 7][1]) == 255
                                && index_l < 8)
                        {
                            temp_l[index_l][0] = search_filds_l[(i)][0];
                            temp_l[index_l][1] = search_filds_l[(i)][1];
                            temp_dir_l[index_l] = i;
                            index_l++;
                        }
                    }

                    if (index_l)
                    {
                        uint8 selected_l = 0;
                        //更新坐标点
                        center_point_l[0] = temp_l[0][0];//x
                        center_point_l[1] = temp_l[0][1];//y
                        for (j = 0; j < index_l; j++)
                        {
                            if (center_point_l[1] > temp_l[j][1])
                            {
                                center_point_l[0] = temp_l[j][0];//x
                                center_point_l[1] = temp_l[j][1];//y
                                selected_l = j;
                            }
                        }
                        dir_l[l_data_statics - 1] = temp_dir_l[selected_l];//记录最终选中的生长方向
                    }
                    else
                    {
                        left_active = 0;
                        left_run = 0;
                    }
                }
            }
        }

        if (right_active && right_run)
        {
            if (r_data_statics >= USE_num)
            {
                right_active = 0;
                right_run = 0;
            }
            else
            {
                //右边
                for (i = 0; i < 8; i++)//传递8F坐标
                {
                    search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
                    search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
                }
                //中心坐标点填充到已经找到的点内
                points_r[r_data_statics][0] = center_point_r[0];//x
                points_r[r_data_statics][1] = center_point_r[1];//y
                r_data_statics++;//索引加一

                if (center_point_r[1] <= k_search_top_stop_row)
                {
                    right_active = 0;
                    right_run = 0;
                }
                else
                {
                    index_r = 0;//先清零，后使用
                    for (i = 0; i < 8; i++)
                    {
                        temp_r[i][0] = 0;//先清零，后使用
                        temp_r[i][1] = 0;//先清零，后使用
                        temp_dir_r[i] = 0;
                    }

                    //右边判断
                    for (i = 0; i < 8; i++)
                    {
                        if (safe_image_pixel(image, search_filds_r[i][0], search_filds_r[i][1]) == 0
                                && safe_image_pixel(image, search_filds_r[(i + 1) & 7][0], search_filds_r[(i + 1) & 7][1]) == 255
                                && index_r < 8)
                        {
                            temp_r[index_r][0] = search_filds_r[(i)][0];
                            temp_r[index_r][1] = search_filds_r[(i)][1];
                            temp_dir_r[index_r] = i;
                            index_r++;//索引加一
                        }
                    }

                    if (index_r)
                    {
                        uint8 selected_r = 0;

                        //更新坐标点
                        center_point_r[0] = temp_r[0][0];//x
                        center_point_r[1] = temp_r[0][1];//y
                        for (j = 0; j < index_r; j++)
                        {
                            if (center_point_r[1] > temp_r[j][1])
                            {
                                center_point_r[0] = temp_r[j][0];//x
                                center_point_r[1] = temp_r[j][1];//y
                                selected_r = j;
                            }
                        }
                        dir_r[r_data_statics - 1] = temp_dir_r[selected_r];//记录最终选中的生长方向
                    }
                    else
                    {
                        right_active = 0;
                        right_run = 0;
                    }
                }
            }
        }

        if (left_active && !right_active)
        {
            left_run = 1;
        }

        if (right_active && !left_active)
        {
            right_run = 1;
        }

        if (left_active && right_active && l_data_statics >= 1 && r_data_statics >= 1)
        {
            bool right_stuck = false;
            bool left_stuck = false;

            if (r_data_statics >= 3)
            {
                right_stuck = points_r[r_data_statics - 1][0] == points_r[r_data_statics - 2][0]
                              && points_r[r_data_statics - 1][0] == points_r[r_data_statics - 3][0]
                              && points_r[r_data_statics - 1][1] == points_r[r_data_statics - 2][1]
                              && points_r[r_data_statics - 1][1] == points_r[r_data_statics - 3][1];
            }

            if (l_data_statics >= 3)
            {
                left_stuck = points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0]
                             && points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0]
                             && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1]
                             && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1];
            }

            if (right_stuck || left_stuck)
            {

                break;
            }

            if (my_abs((int)points_r[r_data_statics - 1][0] - (int)points_l[l_data_statics - 1][0]) < 2
                    && my_abs((int)points_r[r_data_statics - 1][1] - (int)points_l[l_data_statics - 1][1]) < 2
               )
            {

                *hightest = (points_r[r_data_statics - 1][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点

                break;
            }

            left_run = 1;
            right_run = 1;

            const int left_y = points_l[l_data_statics - 1][1];
            const int right_y = points_r[r_data_statics - 1][1];

            if (left_y < right_y)
            {
                left_run = 0;//左边更靠上，暂停左边，等待右边追上
            }
            else if (right_y < left_y)
            {
                right_run = 0;//右边更靠上，暂停右边，等待左边追上
            }

            if (l_data_statics > 1 && dir_l[l_data_statics - 1] == 7 && right_y > left_y)//左边已经向下生长时，退回一步等待右边
            {
                center_point_l[0] = points_l[l_data_statics - 1][0];//x
                center_point_l[1] = points_l[l_data_statics - 1][1];//y
                l_data_statics--;
                left_run = 0;
            }
        }


    }


    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}

/*---------------------------------------------------------------
 【函    数】get_turning_point
 【功    能】拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
// 获取拐点信息的函数
void get_down_turning_point(void)
{
    // 初始化左拐点标志和坐标以及角度
    L_D_corner_flag = 0;
    L_D_corner_row = 0;
    L_D_corner_col = 0;
    L_D_corner_angle = 0;

    // 如果使能搜索左拐点
    if(enable_L_D_corner)
    {
        // 判断数据量是否足够且起点位置和丢失时间满足条件
        if(data_stastics_l > 9&&start_point_l[1]>=MT9V03X_H/2&&Left_Lost_Time<=MT9V03X_H/2+20)
        {
            // 遍历点集，寻找可能的拐点（仅搜索下 2/3 区域）
            uint16 end_l = (data_stastics_l * 2 / 3 > 13) ? (data_stastics_l * 2 / 3 - 13) : 0;
            for(int i = 0; i < (int)end_l; i++)
            {
                // 确保点的位置满足基本要求
                if(points_l[i+12][1]>5)
                {
                    // 使用向量法判断是否为锐角或直角
                    if((points_l[i][0] - points_l[i + 6][0]) * (points_l[i + 12][0] - points_l[i + 6][0]) +
                            (points_l[i][1] - points_l[i + 6][1]) * (points_l[i + 12][1] - points_l[i + 6][1]) >= 0)
                    {
                        // 计算并验证角度，如果符合条件，则记录左拐点信息
//                       L_D_corner_angle = Get_angle(points_l[i][0], points_l[i][1], points_l[i + 4][0], points_l[i + 4][1], points_l[i + 12][0], points_l[i + 12][1]);
                        if(points_l[i+6][0] <= (MT9V03X_W * 2) / 3 &&
                                points_l[i+6][1] >= MT9V03X_H / 3 &&
                                points_l[i+6][0]>points_l[i+12][0]&&points_l[i+6][1]<points_l[i][1]&&
                                (points_l[i + 12][0] - points_l[i + 6][0])+(points_l[i][0] - points_l[i + 6][0])<=0&&//这两行确保向量和向右上
                                ((points_l[i + 12][1] - points_l[i + 6][1])+(points_l[i][1] - points_l[i + 6][1])>=0))
                        {
                            L_D_corner_flag = 1;
                            L_D_corner_row = points_l[i+6][1];
                            L_D_corner_col = points_l[i+6][0];
                            break;
                        }
                    }
                }
            }
        }
    }

    // 初始化右拐点标志和坐标以及角度
    R_D_corner_flag = 0;
    R_D_corner_row = 0;
    R_D_corner_col = 0;
    R_D_corner_angle = 0;

    // 如果使能搜索右拐点
    if(enable_R_D_corner)
    {
        // 判断数据量是否足够且起点位置和丢失时间满足条件
        if(data_stastics_r > 9&&start_point_r[1]>=MT9V03X_H/2&&Right_Lost_Time<=MT9V03X_H/2+20)
        {
            // 遍历点集，寻找可能的拐点（仅搜索下 2/3 区域）
            uint16 end_r = (data_stastics_r * 2 / 3 > 13) ? (data_stastics_r * 2 / 3 - 13) : 0;
            for(int i = 0; i < (int)end_r; i++)
            {
                // 确保点的位置满足基本要求
                if(points_r[i+12][1]>5)
                {
                    // 使用向量法判断是否为锐角或直角
                    if((points_r[i][0] - points_r[i + 6][0]) * (points_r[i + 12][0] - points_r[i + 6][0]) +
                            (points_r[i][1] - points_r[i + 6][1]) * (points_r[i + 12][1] - points_r[i + 6][1]) >= 0)
                    {
                        // 计算并验证角度，如果符合条件，则记录右拐点信息
//                        R_D_corner_angle = Get_angle(points_r[i][0], points_r[i][1], points_r[i + 4][0], points_r[i + 4][1], points_r[i + 8][0], points_r[i + 8][1]);
                        if(points_r[i+6][0] >= MT9V03X_W / 3 &&
                                points_r[i+6][1] >= MT9V03X_H / 3 &&
                                points_r[i+12][0]>points_r[i+6][0]&&points_r[i+6][1]<points_r[i][1]&&
                                (points_r[i + 12][0] - points_r[i + 6][0])+(points_r[i][0] - points_r[i + 6][0])>=0&&//这两行确保向量和向右上
                                (points_r[i + 12][1] - points_r[i + 6][1])+(points_r[i][1] - points_r[i + 6][1])>=0)
                        {
                            R_D_corner_flag = 1;
                            R_D_corner_row = points_r[i+6][1];
                            R_D_corner_col = points_r[i+6][0];
                            break;
                        }
                    }
                }
            }
        }
    }
}

/*---------------------------------------------------------------
 【函    数】get_up_turning_point
 【功    能】拐点检测
 【参    数】无
 【返 回 值】
 【注意事项】
 ----------------------------------------------------------------*/
// 获取拐点信息的函数
void get_up_turning_point(void)
{
    // 初始化左拐点标志和坐标以及角度
    L_U_corner_flag = 0;
    L_U_corner_row = 0;
    L_U_corner_col = 0;
    L_U_corner_angle = 0;

    // 如果使能搜索左拐点
    if(enable_L_U_corner)
    {
        // 判断数据量是否足够且起点位置和丢失时间满足条件
        if(data_stastics_l > 9&&start_point_l[1]>=MT9V03X_H/2&&Left_Lost_Time<=MT9V03X_H/2+20)
        {
            // 遍历点集，寻找可能的拐点（仅搜索上 2/3 区域）
            uint16 start_l = (data_stastics_l / 3 > 7) ? (data_stastics_l / 3) : 7;
            for(int i = (int)start_l; i < data_stastics_l - 13; i++)
            {
                // 确保点的位置满足基本要求
                if(points_l[i+12][1]>5)
                {
                    // 使用向量法判断是否为钝角
                    if((points_l[i + 6][0] - points_l[i][0]) * (points_l[i + 12][0] - points_l[i + 6][0]) +
                            (points_l[i + 6][1] - points_l[i][1]) * (points_l[i + 12][1] - points_l[i + 6][1]) >= 0)
                    {
                        // 计算并验证角度，如果符合条件，则记录左拐点信息
//                       L_D_corner_angle = Get_angle(points_l[i][0], points_l[i][1], points_l[i + 4][0], points_l[i + 4][1], points_l[i + 8][0], points_l[i + 8][1]);
                        if(points_l[i+6][0] <= (MT9V03X_W * 2) / 3 &&
                                points_l[i+6][1] <= (MT9V03X_H * 2) / 3 &&
                                points_l[i+6][0]>points_l[i][0]&&
                                points_l[i+6][1]-points_l[i+12][1]>4&&
                                points_l[i][1]-points_l[i+6][1] < 4&&
                                ((points_l[i + 12][0] - points_l[i + 6][0])+(points_l[i + 6][0] - points_l[i][0]))>=0&&//这两行确保向量和向右上
                                ((points_l[i + 12][1] - points_l[i + 6][1])+(points_l[i + 6][1] - points_l[i][1]))<=0
                          )
                        {
                            L_U_corner_flag = 1;
                            L_U_corner_row = points_l[i+6][1];
                            L_U_corner_col = points_l[i+6][0];
                            break;
                        }
                    }
                }
            }
        }
    }

    // 初始化右拐点标志和坐标以及角度
    R_U_corner_flag = 0;
    R_U_corner_row = 0;
    R_U_corner_col = 0;
    R_U_corner_angle = 0;

    // 如果使能搜索右拐点
    if(enable_R_U_corner)
    {
        // 判断数据量是否足够且起点位置和丢失时间满足条件
        if(data_stastics_r > 9&&start_point_r[1]>=MT9V03X_H/2&&Right_Lost_Time<=MT9V03X_H/2+20)
        {
            // 遍历点集，寻找可能的拐点（仅搜索上 2/3 区域）
            uint16 start_r = (data_stastics_r / 3 > 7) ? (data_stastics_r / 3) : 7;
            for(int i = (int)start_r; i < data_stastics_r - 13; i++)
            {
                // 确保点的位置满足基本要求
                if(points_r[i+12][1]>5)
                {
                    // 使用向量法判断是否为锐角或直角
                    if((points_r[i + 6][0] - points_r[i][0]) * (points_r[i + 12][0] - points_r[i + 6][0])+
                            (points_r[i + 6][1] - points_r[i][1]) * (points_r[i + 12][1] - points_r[i + 6][1]) >=0)
                    {
                        // 计算并验证角度，如果符合条件，则记录右拐点信息
//                       R_D_corner_angle = Get_angle(points_r[i][0], points_r[i][1], points_r[i + 4][0], points_r[i + 4][1], points_r[i + 8][0], points_r[i + 8][1]);
                        if(points_r[i+6][0] >= MT9V03X_W / 3 &&
                                points_r[i+6][1] <= (MT9V03X_H * 2) / 3 &&
                                points_r[i][0]>points_r[i+6][0]&&
                                points_r[i+6][1]>points_r[i+12][1]+4&&
                                points_r[i][1]-points_r[i+6][1] < 4&&
                                ((points_r[i + 12][0] - points_r[i + 6][0])+(points_r[i + 6][0] - points_r[i][0]))<=0&&//这两行确保向量和向左上
                                ((points_r[i + 12][1] - points_r[i + 6][1])+(points_r[i + 6][1] - points_r[i][1]))<=0

                          )
                        {
                            R_U_corner_flag = 1;
                            R_U_corner_row = points_r[i+6][1];
                            R_U_corner_col = points_r[i+6][0];
                            break;
                        }
                    }
                }
            }
        }
    }
}

uint8 shortest_White_Column1[2];
extern int short_col_point[2];
extern bool short_col_point_is_fallback;


/*-------------------------------------------------------------------------------------------------------------------
//左圆环从左往右，右圆环从右向左
  @brief     最短白列，找环岛上拐点进环
  @param     null
  @return    null
  Sample     直接调用
  @note      环岛3是上拐点是一个v字角，但是从2进入3还没有一定就检测到上拐点，需要在状态3等待，等待的时候直行。

-------------------------------------------------------------------------------------------------------------------*/
int shortest_White_Column(uint8 x, uint8 y)
{
    return shortest_White_Column(x, y, MT9V03X_H - 1, 2);
}

int  shortest_White_Column(uint8 x,uint8 y,uint8 start_row,uint8 end_row)
{
    return shortest_White_Column(x, y, start_row, end_row, false);
}

int  shortest_White_Column(uint8 x,uint8 y,uint8 start_row,uint8 end_row,bool reverse_scan)
{
    int i, j;
    //最短长白列,[0]是最长白列的长度，[1]是第某列
//----------------------------------------------------------------------
    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for(i = 0; i < MT9V03X_W; i++)
    {
        White_col[i] = 255;
    }

    const int start_col = std::clamp<int>(x, 10, MT9V03X_W - 11);
    const int end_col = std::clamp<int>(y, start_col, MT9V03X_W - 11);
    int scan_start_row = std::clamp<int>(start_row, 2, MT9V03X_H - 1);
    int scan_end_row = std::clamp<int>(end_row, 2, scan_start_row);
    for (j = start_col; j <= end_col; j++)
    {
        White_col[j] = 0;
        for (i = scan_start_row; i >= scan_end_row; i--)
        {
            if(bin_image[i][j] == IMG_BLACK && bin_image[i-1][j] == IMG_BLACK)
                break;
            else
                White_col[j]++;
        }
    }

    shortest_White_Column1[0]=0;
    shortest_White_Column1[1]=0;
    const int scan_step = reverse_scan ? -1 : 1;
    const int scan_first_col = reverse_scan ? end_col : start_col;
    const int scan_last_col = reverse_scan ? start_col : end_col;
    for(i = scan_first_col; reverse_scan ? (i >= scan_last_col) : (i <= scan_last_col); i += scan_step)
    {
        if (shortest_White_Column1[0]==0&&White_col[i]>0&&
                White_col[i]<=White_col[i+1]&&White_col[i]<=White_col[i-1]&&
                White_col[i]<=White_col[i+3]&&White_col[i]<=White_col[i-3]&&
                White_col[i]<White_col[i+10]&&White_col[i]<White_col[i-10]
//                White_col[i]>=20&&White_col[i+1]-White_col[i]>=5&&White_col[i-1]-White_col[i]>=5&&White_col[i+2]-White_col[i]>=5&&
//          White_col[i+3]-White_col[i]>=5&&White_col[i-3]-White_col[i]>=5&&White_col[i-2]-White_col[i]>=5
                //单调性先变大然后变小
           )//shortest_White_Column_Left[0] <= White_col[i])//找最短的那一列,前5后5中最短
        {
            shortest_White_Column1[0]= White_col[i];//白列长度【0】是长度，【1】是第几列
            shortest_White_Column1[1]=i;
            short_col_point[0]=scan_start_row - White_col[i] + 1;
            short_col_point[1]=i;
            short_col_point_is_fallback=false;
            break;
        }
    }

    return shortest_White_Column1[0];
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     最短白列，找环岛上拐点进环
  @param     null
  @return    null
  Sample     直接调用
  @note      环岛3是上拐点是一个v字角，但是从2进入3还没有一定就检测到上拐点，需要在状态3等待，等待的时候直行。

-------------------------------------------------------------------------------------------------------------------*/
//int  shortest_White_Column(int x,int y)//最长白列巡线
//{
//    int i, j;
//    //最短长白列,[0]是最长白列的长度，[1]是第某列
////----------------------------------------------------------------------
//    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
//    for(i=x-3;i<y+3;i++)
//        {
//            White_Column[i] = 0;
//        }
//    for (j =x; j<=y; j++)
//    {
//        for (i = MT9V03X_H - 1; i >= 2; i--)
//        {
//            if(bin_image[i][j] == IMG_BLACK&&bin_image[i-1][j] == IMG_BLACK)
//                break;
//            else
//                White_Column[j]++;
//        }
//    }
//    shortest_White_Column1[0]=0;
//    shortest_White_Column1[1]=0;
//    for(i=x;i<=y;i++)
//    {
//        if (shortest_White_Column1[0]==0&&White_Column[i]<=White_Column[i+1]&&White_Column[i]<=White_Column[i-1]&&
//            White_Column[i]<White_Column[i+4]&&White_Column[i]<White_Column[i-4]&&
//            White_Column[i]<White_Column[i+6]&&White_Column[i]<White_Column[i-6]
////                White_Column[i]>=20&&White_Column[i+1]-White_Column[i]>=5&&White_Column[i-1]-White_Column[i]>=5&&White_Column[i+2]-White_Column[i]>=5&&
////          White_Column[i+3]-White_Column[i]>=5&&White_Column[i-3]-White_Column[i]>=5&&White_Column[i-2]-White_Column[i]>=5
//                                                                             //单调性先变大然后变小
//        )//shortest_White_Column_Left[0] <= White_Column[i])//找最短的那一列,前5后5中最短
//        {
//            shortest_White_Column1[0]= White_Column[i];//白列长度【0】是长度，【1】是第几列
//            shortest_White_Column1[1]=i;
//            //ips200_show_int(100, 270, shortest_White_Column1[1], 2);//第几列
//            break;
//        }
//    }
//    return shortest_White_Column1[0];
//}

float hd[3][3] = {{ 0.824413061073802, -0.516124387136357, -2.55970487591164},
    { 0.0141149526528835, 0.133790250227907, 4.47420783713753},
    { -3.48230739791546E-05, -0.00684853788256681, 0.782474472311575},
};

/**
 * 计算由Ax, Ay, Bx, By, Cx, Cy定义的三个点构成的三角形中，向量BA和向量BC之间的角度。
 *
 * @param Ax 点A的x坐标
 * @param Ay 点A的y坐标
 * @param Bx 点B的x坐标
 * @param By 点B的y坐标
 * @param Cx 点C的x坐标
 * @param Cy 点C的y坐标
 * @return 返回向量BA和向量BC之间的角度，单位为度。
 */
float Get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy)
{

    float BA = 0.00;//向量BA的模
    float BC = 0.00;
    float SBA_BC = 0.00;//向量点乘的值
    float angle = 0.00;

    float AX=((hd[0][0] * Ax + hd[0][1] * Ay + hd[0][2])/(hd[2][0] * Ax + hd[2][1] * Ay + hd[2][2]));
    float AY=((hd[1][0] * Ax + hd[1][1] * Ay + hd[1][2])/(hd[2][0] * Ax + hd[2][1] * Ay + hd[2][2]));
    float BX=((hd[0][0] * Bx + hd[0][1] * By + hd[0][2])/(hd[2][0] * Bx + hd[2][1] * By + hd[2][2]));
    float BY=((hd[1][0] * Bx + hd[1][1] * By + hd[1][2])/(hd[2][0] * Bx + hd[2][1] * By + hd[2][2]));
    float CX=((hd[0][0] * Cx + hd[0][1] * Cy + hd[0][2])/(hd[2][0] * Cx + hd[2][1] * Cy + hd[2][2]));
    float CY=((hd[1][0] * Cx + hd[1][1] * Cy + hd[1][2])/(hd[2][0] * Cx + hd[2][1] * Cy + hd[2][2]));

    BA = sqrt((AX-BX)*(AX-BX)+(AY-BY)*(AY-BY));
    BC = sqrt((CX-BX)*(CX-BX)+(CY-BY)*(CY-BY));

    SBA_BC = (AX-BX)*(CX-BX)+(AY-BY)*(CY-BY);

    angle =  acos(SBA_BC*1.00/(BA*BC));

    return angle*57.3;
}
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */

// 将左边界跟踪点集按行整理成 Left_Line。
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //初始化
    for (i = 0; i<MT9V03X_H; i++)
    {
        Left_Line[i] = border_min;
    }
    h = MT9V03X_H - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            Left_Line[h] = points_l[j][0]+1;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */

// 将右边界跟踪点集按行整理成 Right_Line。
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < MT9V03X_H; i++)
    {
        Right_Line[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = MT9V03X_H - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            Right_Line[h] = points_r[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

bool Get_K_b(uint8 x1,uint8 y1,uint8 x2,uint8 y2, float* slope_rate, float* intercept)//
{
    if (x1 == x2 || y1 == y2)
    {
        *slope_rate = 0.0f;
        *intercept = 0.0f;
        return false;
    }

    *slope_rate = (float)(y2 - y1) / (x2 - x1);
    if (std::fabs(*slope_rate) < 1e-6f)
    {
        *slope_rate = 0.0f;
        *intercept = 0.0f;
        return false;
    }

    *intercept = y1 - (*slope_rate * x1);
    return true;
}


uint8_t break_flag=0;
void get_turning_point(void)
{
    uint16_t i;
    break_flag=0;
    L_U_corner_flag = 0;
    L_U_corner_row = 0;//行
    L_U_corner_col = 0;//列

    L_D_corner_flag = 0;
    L_D_corner_row = 0;
    L_D_corner_col = 0;

    R_U_corner_flag = 0;
    R_U_corner_row = 0;
    R_U_corner_col = 0;

    R_D_corner_flag = 0;
    R_D_corner_row = 0;
    R_D_corner_col = 0;

    for (i = 20; i < MT9V03X_H / 2-20; i++)//寻找左上拐点
    {
        if(Left_Line[i]<Left_Line[i-10])//上面的值大于该点的值，避免圆环中拐点
        {

            if ((abs(Left_Line[i] - Left_Line[i - 1]) <= 5)
                    && (abs(Left_Line[i - 1] - Left_Line[i - 2]) <= 5)
                    && (abs(Left_Line[i - 2] - Left_Line[i - 3]) <= 3)
                    && (Left_Line[i] - Left_Line[i + 3] >= 8)
                    &&(Left_Line[i+2] - Left_Line[i + 3] <=3))
            {
                L_U_corner_row = i;//传递y坐标
                L_U_corner_col = Left_Line[i];
                L_U_corner_flag = 1;
                break;
            }
        }
    }

    for (i =  20; i < MT9V03X_H / 2-20; i++)//寻找右上拐点
    {
        if(Right_Line[i]>Right_Line[i-10])
        {
            if ((abs(Right_Line[i] - Right_Line[i - 1]) <= 5)
                    && (abs(Right_Line[i - 1] - Right_Line[i - 2]) <= 5)
                    && (abs(Right_Line[i - 2] - Right_Line[i - 3]) <= 3)
                    && (Right_Line[i + 3] - Right_Line[i] >= 8)
                    && (Right_Line[i+2] - Right_Line[i + 3] <=3))
            {
                R_U_corner_row = i;//传递y坐标
                R_U_corner_col = Right_Line[i];
                R_U_corner_flag = 1;
                break;
            }
        }
    }

    for (i = MT9V03X_H - 15; i > MT9V03X_H / 2 - 20; i--)//寻找左下拐点
    {

        if ((abs(Left_Line[i] - Left_Line[i + 1]) <= 5)
                && (abs(Left_Line[i + 1] - Left_Line[i + 2]) <= 5)
                && (abs(Left_Line[i + 2] - Left_Line[i + 3]) <= 3)
                && (Left_Line[i] - Left_Line[i - 3] >= 8))
        {

            L_D_corner_row = i;//传递y坐标
            L_D_corner_col = Left_Line[i];
            L_D_corner_flag = 1;
            break;
        }
    }

    for (i = MT9V03X_H - 15; i > MT9V03X_H / 2 - 20; i--)//寻找右下拐点
    {

        if ((abs(Right_Line[i] - Right_Line[i + 1]) <= 5)
                && (abs(Right_Line[i + 1] - Right_Line[i + 2]) <= 5)
                && (abs(Right_Line[i + 2] - Right_Line[i + 3]) <= 3)
                && (Right_Line[i - 3] - Right_Line[i] >= 8))
        {
            R_D_corner_row = i;//传递y坐标
            R_D_corner_col = Right_Line[i];
            R_D_corner_flag = 1;
            break;
        }
    }

    if ((R_D_corner_row - R_U_corner_row < 0) && (R_D_corner_row) && (R_U_corner_row))
    {
        R_D_corner_row = 0;
        R_D_corner_col = 0;
        R_D_corner_flag =0;
    }
    if ((L_D_corner_row - L_U_corner_row < 0) && (L_D_corner_row) && (L_U_corner_row))
    {
        L_D_corner_row = 0;
        L_D_corner_col = 0;
        L_D_corner_flag = 0;
    }
    if ((R_U_corner_row - R_D_corner_row > 0) && (R_D_corner_row) && (R_U_corner_row))
    {
        R_U_corner_row = 0;
        R_U_corner_col = 0;
        R_U_corner_flag = 0;
    }
    if ((L_U_corner_row - L_D_corner_row > 0) && (L_D_corner_row) && (L_U_corner_row))
    {
        L_U_corner_row = 0;
        L_U_corner_col = 0;
        L_U_corner_flag = 0;
    }

    turn_point_num=0;
    if(L_U_corner_flag ==1) turn_point_num ++;
    if(R_U_corner_flag ==1) turn_point_num ++;
    if(L_D_corner_flag ==1) turn_point_num ++;
    if(R_D_corner_flag ==1) turn_point_num ++;

}

void validate_and_count_turn_points(void)
{
    if ((R_D_corner_row - R_U_corner_row < 0) && (R_D_corner_row) && (R_U_corner_row))
    {
        R_D_corner_row = 0;
        R_D_corner_col = 0;
        R_D_corner_flag = 0;
    }
    if ((L_D_corner_row - L_U_corner_row < 0) && (L_D_corner_row) && (L_U_corner_row))
    {
        L_D_corner_row = 0;
        L_D_corner_col = 0;
        L_D_corner_flag = 0;
    }
    if ((R_U_corner_row - R_D_corner_row > 0) && (R_D_corner_row) && (R_U_corner_row))
    {
        R_U_corner_row = 0;
        R_U_corner_col = 0;
        R_U_corner_flag = 0;
    }
    if ((L_U_corner_row - L_D_corner_row > 0) && (L_D_corner_row) && (L_U_corner_row))
    {
        L_U_corner_row = 0;
        L_U_corner_col = 0;
        L_U_corner_flag = 0;
    }

    turn_point_num = 0;
    if(L_U_corner_flag == 1) turn_point_num++;
    if(R_U_corner_flag == 1) turn_point_num++;
    if(L_D_corner_flag == 1) turn_point_num++;
    if(R_D_corner_flag == 1) turn_point_num++;
}

void circle_get_turning_point(void)    //圆环拐点 寻找比十字拐点更严格一点
{
    uint16_t i,j;

    L_U_corner_flag = 0;
    L_U_corner_row = 0;
    L_U_corner_col = 0;


    L_D_corner_flag = 0;
    L_D_corner_row = 0;
    L_D_corner_col = 0;

    R_U_corner_flag = 0;
    R_U_corner_row = 0;
    R_U_corner_col = 0;

    R_D_corner_flag = 0;
    R_D_corner_row = 0;
    R_D_corner_col = 0;



    for (i = 30; i < MT9V03X_H / 2 + 20; i++)//寻找左上拐点
    {
//        if ((abs(Left_Line[i] - Left_Line[i - 1]) <= 3)
//                && (abs(Left_Line[i - 1] - Left_Line[i - 2]) <= 3)
//                && (abs(Left_Line[i - 2] - Left_Line[i - 3]) <= 2)
//                && (Left_Line[i] - Left_Line[i + 3] >= 15)
//                && (Left_Line[i+2] - Left_Line[i + 3] <=3))
        if(//只找第一个符合条件的点
            abs(Left_Line[i]-Left_Line[i-1])<=5&&
            abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
            abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
            (Left_Line[i]-Left_Line[i+2])>=8&&
            (Left_Line[i]-Left_Line[i+3])>=15&&
            (Left_Line[i]-Left_Line[i+4])>=15)
        {
            L_U_corner_row = i;//传递y坐标
            L_U_corner_col = Left_Line[i];
            L_U_corner_flag = 1;
            break;
        }
    }

    for (i = MT9V03X_H - 15; i > MT9V03X_H / 2 - 20; i--)//寻找左下拐点
    {
        //printf("L_Board[%d] = %d\r\n", i, L_Border[i]);
        if ((abs(Left_Line[i] - Left_Line[i + 1]) <= 5)
                && (abs(Left_Line[i + 1] - Left_Line[i + 2]) <= 5)
                && (abs(Left_Line[i + 2] - Left_Line[i + 3]) <= 5)
                && (Left_Line[i] - Left_Line[i - 3] >= 7))
        {
            L_D_corner_row = i;//传递y坐标
            L_D_corner_col = Left_Line[i];
            L_D_corner_flag = 1;
            break;
        }
    }

    for (i =  15; i < MT9V03X_H / 2 + 30; i++)//寻找右上拐点
    {
        if ((abs(Right_Line[i] - Right_Line[i - 1]) <= 3)
                && (abs(Right_Line[i - 1] - Right_Line[i - 2]) <= 3)
                && (abs(Right_Line[i - 2] - Right_Line[i - 3]) <= 2)
                && (Right_Line[i + 3] - Right_Line[i] >= 15)
                && (Right_Line[i+2] - Right_Line[i + 3] <=3))
        {

            for(j = 5; j < i; j++)
            {
                if(Right_Line[j]>Right_Line[i])
                {
                    break_flag=1;
                    break;
                }
            }
            if(break_flag == 1)
            {
                break;
            }
            else
            {
                R_U_corner_row = i;//传递y坐标
                R_U_corner_col = Right_Line[i];
                R_U_corner_flag = 1;
                break;
            }
        }
    }

    for (i = MT9V03X_H - 15; i > MT9V03X_H / 2 - 20; i--)//寻找右下拐点
    {

        if ((abs(Right_Line[i] - Right_Line[i + 1]) <= 5)
                && (abs(Right_Line[i + 1] - Right_Line[i + 2]) <= 5)
                && (abs(Right_Line[i + 2] - Right_Line[i + 3]) <= 5)
                && (Right_Line[i - 3] - Right_Line[i] >= 7))
        {
            R_D_corner_row = i;//传递y坐标
            R_D_corner_col = Right_Line[i];
            R_D_corner_flag = 1;
            break;
        }
    }

    if ((R_D_corner_row - R_U_corner_row < 0) && (R_D_corner_row) && (R_U_corner_row))
    {
        R_D_corner_row = 0;
        R_D_corner_col = 0;
        R_D_corner_flag =0;
    }
    if ((L_D_corner_row - L_U_corner_row < 0) && (L_D_corner_row) && (L_U_corner_row))
    {
        L_D_corner_row = 0;
        L_D_corner_col = 0;
        L_D_corner_flag = 0;
    }
    if ((R_U_corner_row - R_D_corner_row > 0) && (R_D_corner_row) && (R_U_corner_row))
    {
        R_U_corner_row = 0;
        R_U_corner_col = 0;
        R_U_corner_flag = 0;
    }
    if ((L_U_corner_row - L_D_corner_row > 0) && (L_D_corner_row) && (L_U_corner_row))
    {
        L_U_corner_row = 0;
        L_U_corner_col = 0;
        L_U_corner_flag = 0;
    }

    turn_point_num=0;
    if(L_U_corner_flag ==1) turn_point_num ++;
    if(R_U_corner_flag ==1) turn_point_num ++;
    if(L_D_corner_flag ==1) turn_point_num ++;
    if(R_D_corner_flag ==1) turn_point_num ++;

}

uint8_t island_state =0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_down_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return left_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=	hightest)
        end=hightest;
    if(end<=5)
        end=5;
    for(i=start; i>=end; i--)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
                abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
                abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
                abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
                (Left_Line[i]-Left_Line[i-2])>=5&&
                (Left_Line[i]-Left_Line[i-3])>=10&&
                (Left_Line[i]-Left_Line[i-4])>=10)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Left_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Left_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int left_up_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return left_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=	hightest)
        end=hightest;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start; i>=end; i--)
    {
        if(left_up_line==0&&//只找第一个符合条件的点
                abs(Left_Line[i]-Left_Line[i-1])<=5&&
                abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
                abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
                (Left_Line[i]-Left_Line[i+2])>=8&&
                (Left_Line[i]-Left_Line[i+3])>=15&&
                (Left_Line[i]-Left_Line[i+4])>=15)
        {
            left_up_line=i;//获取行数即可
            break;
        }
    }
    return left_up_line;//如果是MT9V03X_H-1，说明没有这么个拐点
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=	hightest)
        end=hightest;
    if(end<=5)
        end=5;
    for(i=start; i>=end; i--)
    {
        if(right_down_line==0&&//只找第一个符合条件的点
                abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
                abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
                abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
                (Right_Line[i]-Right_Line[i-2])<=-5&&
                (Right_Line[i]-Right_Line[i-3])<=-10&&
                (Right_Line[i]-Right_Line[i-4])<=-10)
        {
            right_down_line=i;//获取行数即可
            break;
        }
    }
    return right_down_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右上角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Up_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_up_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_up_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=	hightest)
        end=hightest;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start>=MT9V03X_H-1-5)
        start=MT9V03X_H-1-5;
    for(i=start; i>=end; i--)
    {
        if(right_up_line==0&&//只找第一个符合条件的点
                abs(Right_Line[i]-Right_Line[i-1])<=5&&//下面两行位置差不多
                abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
                abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
                (Right_Line[i]-Right_Line[i+2])<=-8&&
                (Right_Line[i]-Right_Line[i+3])<=-15&&
                (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            right_up_line=i;//获取行数即可
            break;
        }
    }
    return right_up_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     Continuity_Change_Left(int start,int end);
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
uint8_t Continuity_Change_Left(int start,int end)//连续性阈值设置为5
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
        return 1;
//    if(hightest<=5)//搜所截止行很矮
//       return 1;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
        end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
        t=start;
        start=end;
        end=t;
    }

    for(i=start; i>=end; i--)
    {
        if(abs(Left_Line[i]-Left_Line[i-5])>=6)//连续判断阈值是5,可更改
        {
            continuity_change_flag=i;
            break;
        }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
uint8_t Continuity_Change_Right(int start,int end)
{
    int i;
    int t;
    int continuity_change_flag=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没必要判断了
        return 1;
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
        end=5;
    if(start<end)//都是从下往上计算的，反了就互换一下
    {
        t=start;
        start=end;
        end=t;
    }

    for(i=start; i>=end; i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-5])>=6)//连续性阈值是5，可更改
        {
            continuity_change_flag=i;
            break;
        }
    }
    return continuity_change_flag;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测（左）
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护，在判断第i个点时
        start=MT9V03X_H-1-5; //要访问它前后5个点，数组两头的点要不能作为起点终点
    if(end<=5)
        end=5;
    if(start<=end)//递减计算，入口反了，直接返回0
        return monotonicity_change_line;
    for(i=start; i>=end; i--) //会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
                Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
                Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
                Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
                Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {
            //一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Left_Line[i]>=Left_Line[i+5]&&Left_Line[i]>=Left_Line[i-5]&&
                Left_Line[i]>=Left_Line[i+4]&&Left_Line[i]>=Left_Line[i-4]&&
                Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
                Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
                Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {
            //就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测（右）
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;

    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=MT9V03X_H-1-5)//数组越界保护
        start=MT9V03X_H-1-5;
    if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start; i>=end; i--) //会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
                Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
                Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
                Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
                Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {
            //一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Right_Line[i]<=Right_Line[i+5]&&Right_Line[i]<=Right_Line[i-5]&&
                Right_Line[i]<=Right_Line[i+4]&&Right_Line[i]<=Right_Line[i-4]&&
                Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
                Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
                Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1])
        {
            //就很暴力，这个数据是在前5，后5中最小的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
      @brief     角点检测
-------------------------------------------------------------------------------------------------------------------*/
uint8 right_cusp_flag = 0;//角点存在标志
int16 right_cusp_row = 0;//角点所在行
int16 right_cusp_col = 0;//角点所在列
void get_right_cusp_point(void)//右边角点
{
    int i;
    uint8 start,end;
    right_cusp_flag=0;//角点存在标志
    right_cusp_row = 0;//角点所在行
    right_cusp_col = 0;
    //如果找到了下拐点，就以下拐点为起始，否则就为最下一行
    if(R_D_corner_flag==1)
        start=R_D_corner_row-8;
    else
        start=MT9V03X_H -10;
    end=hightest + 15;
    if(data_stastics_r>17)
    {
        for(i=start; i>=end; i--) //会读取前5后5数据，所以前面对输入范围有要求
        {
            if(Right_Line[i] <Right_Line[i+8]&&Right_Line[i] <Right_Line[i-8]&&
                    Right_Line[i] <Right_Line[i+6]&&Right_Line[i] <Right_Line[i-6]&&
                    Right_Line[i]<=Right_Line[i+4]&&Right_Line[i]<=Right_Line[i-4]&&
                    Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
                    Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1]&&
                    my_abs(Right_Line[i]-Right_Line[i-6])>=10)
            {
                //这个数据是在前后邻域中最小的，那就是右侧尖点
                right_cusp_flag = 1;//角点存在标志
                right_cusp_row = i;//角点所在行
                right_cusp_col = Right_Line[i];//角点所在列
//                if((R_down_corner_flag==1)&&(my_abs(right_cusp_row-R_down_corner_row)<10))
//                {
//                    right_cusp_flag = 0;//角点存在标志
//                }
                break;
            }
        }
    }
}
uint8 left_cusp_flag = 0;//角点存在标志
int16 left_cusp_row = 0;//角点所在行
int16 left_cusp_col = 0;//角点所在列
void get_left_cusp_point(void)//左边角点
{
    int i;
    uint8 start,end;
    left_cusp_flag=0;//角点存在标志
    left_cusp_row=0;//角点所在行
    left_cusp_col=0;//角点所在列
    //如果找到了下拐点，就以下拐点为起始，否则就为最下一行
    if(L_D_corner_flag==1)
        start=L_D_corner_row-9;
    else
        start=MT9V03X_H -10;
    end=hightest + 15;
    if(data_stastics_l>17)
    {
        for(i=start; i>=end; i--) //会读取前5后5数据，所以前面对输入范围有要求
        {
            if(Left_Line[i] >Left_Line[i+8]&&Left_Line[i]>Left_Line[i-8]&&
                    Left_Line[i] >Left_Line[i+6]&&Left_Line[i]>Left_Line[i-6]&&
                    Left_Line[i]>=Left_Line[i+4]&&Left_Line[i]>=Left_Line[i-4]&&
                    Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
                    Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1]&&
                    my_abs(Left_Line[i]-Left_Line[i-6])>=10)
            {
                //这个数据是在前后邻域中最大的，那就是左侧尖点
                left_cusp_flag = 1;//角点存在标志
                left_cusp_row = i;//角点所在行
                left_cusp_col = Left_Line[i];//角点所在列
                break;
            }
        }
    }
}


int continuity_change_right_flag=0; //连续是0
int continuity_change_left_flag=0;  //连续是0
//  int monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列
int monotonicity_change_left_flag=0;//不转折是0
int monotonicity_change_right_flag=0;//不转折是0

float slope_l_rate_temp,intercept_l_temp;

void Bu_right(void)
{
    int i;
    for(i = hightest ; i<MT9V03X_H-2; i++)
    {
        Right_Line[i] = Left_Line[i] + Standard_Road_Wide[i];
        if(Right_Line[i] <= 1) Right_Line[i]=1;
        if(Right_Line[i]>= MT9V03X_W-2) Right_Line[i]=MT9V03X_W-2;
    }
}

void Bu_left(void)
{
    int i;
    for(i = hightest ; i<MT9V03X_H-2; i++)
    {
        Left_Line[i] = Right_Line[i] - Standard_Road_Wide[i];
        if(Left_Line[i] <= 1) Left_Line[i]=1;
        if(Left_Line[i] >= MT9V03X_W-2) Left_Line[i]=MT9V03X_W-2;
    }
}

int left_down_guai[2]= {0}; //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
int right_down_guai[2]= {0}; //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
int monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列
int short_col_point[2]; // 最短白列点，[0]=行，[1]=列
bool short_col_point_is_fallback = false;
int temp_RU[2];
int temp_LU[2];

namespace
{
    enum class CircleDirection : uint8_t
    {
        Left,
        Right
    };

    const char *circle_state_event_name(uint8_t state)
    {
        switch (state)
        {
        case 1:
            return "CircleEntryDetected";
        case 2:
            return "CircleWaitUpperArc";
        case 3:
            return "CircleFindArc";
        case 4:
            return "CircleExitTrend";
        case 5:
            return "CircleRepairExitLine";
        case 6:
            return "CircleRecoverTrack";
        default:
            return "CircleIdle";
        }
    }

    MonotonicEventTimer::Duration circle_state_event_timeout(uint8_t state)
    {
        switch (state)
        {
        case 1:
            return k_circle_state1_entry_detected_timeout;
        case 2:
            return k_circle_state2_wait_upper_arc_timeout;
        case 3:
            return k_circle_state3_find_arc_timeout;
        case 4:
            return k_circle_state4_exit_trend_timeout;
        case 5:
            return k_circle_state5_repair_exit_line_timeout;
        case 6:
            return k_circle_state6_recover_track_timeout;
        default:
            return MonotonicEventTimer::Duration::zero();
        }
    }

    void clear_circle_runtime_data()
    {
        Image_Flag.Left_Circle = false;
        Image_Flag.Right_Circle = false;
        g_left_circle_entry_candidate_frames = 0;
        g_right_circle_entry_candidate_frames = 0;
        g_left_circle_state2_no_monotonicity_frames = 0;
        g_right_circle_state2_no_monotonicity_frames = 0;
        g_left_circle_exit_mono_row = 0;
        g_left_circle_exit_mono_col = 0;
        g_right_circle_exit_mono_row = 0;
        g_right_circle_exit_mono_col = 0;
        monotonicity_change_line[0] = 0;
        monotonicity_change_line[1] = 0;
        short_col_point[0] = 0;
        short_col_point[1] = 0;
        short_col_point_is_fallback = false;
        shortest_White_Column1[0] = 0;
        shortest_White_Column1[1] = 0;
        temp_RU[0] = 0;
        temp_RU[1] = 0;
        temp_LU[0] = 0;
        temp_LU[1] = 0;
        reset_circle_integrate_yaw();
    }

    // 所有状态跳转统一经过此处，保证每个状态事件从进入时重新计时。
    void set_circle_state(uint8_t new_state, MonotonicEventTimer::TimePoint now)
    {
        island_state = new_state;
        if (new_state == 0)
        {
            g_circle_state_event_timer.reset();
        }
        else
        {
            g_circle_state_event_timer.start(now);
        }
    }

    void start_circle(CircleDirection direction, MonotonicEventTimer::TimePoint now)
    {
        // 入口特征计算期间可能异步触发绕行，创建圆环状态前再做一次最终确认。
        if (obstacle_avoid_active())
        {
            g_left_circle_entry_candidate_frames = 0;
            g_right_circle_entry_candidate_frames = 0;
            return;
        }

        clear_circle_runtime_data();
        Image_Flag.Left_Circle = direction == CircleDirection::Left;
        Image_Flag.Right_Circle = direction == CircleDirection::Right;
        set_circle_state(1, now);
    }

    // 用户规则：只有状态6正常恢复赛道后启动2秒再入环冷却。
    void finish_circle_normally(MonotonicEventTimer::TimePoint now)
    {
        clear_circle_runtime_data();
        set_circle_state(0, now);
        g_circle_normal_exit_cooldown_timer.start(now);
    }

    void cancel_circle_on_timeout(MonotonicEventTimer::TimePoint now)
    {
        clear_circle_runtime_data();
        set_circle_state(0, now);
    }

    bool circle_normal_exit_cooldown_active(MonotonicEventTimer::TimePoint now)
    {
        if (!g_circle_normal_exit_cooldown_timer.running())
        {
            return false;
        }
        if (g_circle_normal_exit_cooldown_timer.expired(k_circle_normal_exit_reentry_cooldown, now))
        {
            g_circle_normal_exit_cooldown_timer.reset();
            return false;
        }
        return true;
    }

    bool cancel_circle_if_state_event_timed_out(MonotonicEventTimer::TimePoint now)
    {
        if (island_state == 0)
        {
            return false;
        }

        // 兼容调试时外部直接修改 island_state 的情况，避免活动状态没有计时基准。
        if (!g_circle_state_event_timer.running())
        {
            g_circle_state_event_timer.start(now);
            return false;
        }

        const auto timeout = circle_state_event_timeout(island_state);
        if (timeout == MonotonicEventTimer::Duration::zero() ||
            !g_circle_state_event_timer.expired(timeout, now))
        {
            return false;
        }

        std::cout << circle_state_event_name(island_state) << " timeout, cancel circle" << std::endl;
        cancel_circle_on_timeout(now);
        return true;
    }
}

// 圆环检测与补线状态机：根据拐点、丢线和 yaw 积分推进 island_state。
void Island_Detect(void)//环岛检测
{
    const auto now = MonotonicEventTimer::Clock::now();
    if (cancel_circle_if_state_event_timed_out(now))
    {
        return;
    }

    uint16 i;

    uint16 start, end;

    float slope_l_rate = 0, intercept_l = 0;
    left_down_guai[0]= 0; //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    left_down_guai[1]= 0; //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    right_down_guai[0]= 0; //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    right_down_guai[1]= 0; //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
//		short_col_point[0]=0;
//		short_col_point[1]=0;
    // monotonicity_change_line[2];//单调性改变点坐标，[0]寸某行，[1]寸某列
    monotonicity_change_left_flag=0;//不转折是0
    monotonicity_change_right_flag=0;//不转折是0
    continuity_change_right_flag=0; //连续是0
    continuity_change_left_flag=0;  //连续是0

    continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-10,15);//连续性判断
    continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-10,15);
    monotonicity_change_right_flag=Monotonicity_Change_Right(MT9V03X_H-1-10,10);
    monotonicity_change_left_flag=Monotonicity_Change_Left(MT9V03X_H-1-10,10);

    if(island_state==0)
    {
        const bool cooldown_active = circle_normal_exit_cooldown_active(now);
        // 绕行会主动改变可靠边和中线偏移，期间的边线形态不能用于新建圆环状态。
        // 此限制只放在状态 0；如果绕行前已在圆环内，状态 1~6 仍继续推进。
        const bool obstacle_avoid_blocks_entry = obstacle_avoid_active();
        if(Image_Flag.Cross_Fill!=0 || cooldown_active || obstacle_avoid_blocks_entry)
        {
            g_left_circle_entry_candidate_frames = 0;
            g_right_circle_entry_candidate_frames = 0;
        }
        else
        {
            bool left_entry_candidate = false;
            bool right_entry_candidate = false;

            if(Image_Flag.Left_Circle==0 &&
                    monotonicity_change_right_flag==0&&//右边是单调的
                    continuity_change_left_flag!=0&&//左边是不连续的
                    continuity_change_right_flag==0&&//左环岛右边是连续的
                    Left_Lost_Time>=20&& //左边丢线很多
                    Left_Lost_Time<=70&& //也不能全丢了
                    hightest<30)//搜索截止行看到很远
            {
                left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);//找左下角点
                left_down_guai[1]=Left_Line[left_down_guai[0]];
                left_entry_candidate = left_down_guai[0]>=5;
            }

            if(Image_Flag.Right_Circle==0 &&
                    monotonicity_change_left_flag==0&&
                    continuity_change_left_flag==0&&//右环岛左边是连续的
                    continuity_change_right_flag!=1&&//右边是真实不连续行号，排除大部分丢线异常
                    Right_Lost_Time>=20&&
                    Right_Lost_Time<=70&&
                    hightest<30)
            {
                right_down_guai[0]=Find_Right_Down_Point(MT9V03X_H-1,20);//找右下角点
                right_down_guai[1]=Right_Line[right_down_guai[0]];
                right_entry_candidate = right_down_guai[0]>=5;
            }

            // 任一入口条件中断都立即清零，只接受同一方向连续3帧。
            g_left_circle_entry_candidate_frames = left_entry_candidate
                ? std::min(g_left_circle_entry_candidate_frames + 1, k_circle_entry_confirm_frames)
                : 0;
            g_right_circle_entry_candidate_frames = right_entry_candidate
                ? std::min(g_right_circle_entry_candidate_frames + 1, k_circle_entry_confirm_frames)
                : 0;

            // 左右入环在同一帧互斥，一方确认后不再让另一方覆盖状态。
            if(g_left_circle_entry_candidate_frames >= k_circle_entry_confirm_frames)
            {
                start_circle(CircleDirection::Left, now);
            }
            else if(g_right_circle_entry_candidate_frames >= k_circle_entry_confirm_frames)
            {
                start_circle(CircleDirection::Right, now);
            }
        }
    }

    if(Image_Flag.Left_Circle == 1)
    {
        if(island_state == 1)
        {
            //更新拐点坐标
            left_down_guai[0]=L_D_corner_row;//行
            left_down_guai[1]=L_D_corner_col;

            if(L_D_corner_flag!=0)
            {
                //注意补线要稍微避开拐点
                Left_Add_Line(left_down_guai[1]+1,left_down_guai[0]-1,left_down_guai[1]+30,0);
            }
            else if(L_D_corner_flag==0&&continuity_change_left_flag-monotonicity_change_left_flag>5)
            {
                set_circle_state(2, now);
            }
        }
        else if(island_state==2) //2状态下方丢线，上方即将出现大弧线
        {
            monotonicity_change_line[0] = Monotonicity_Change_Left(80, 25); // 单调性改变
            //monotonicity_change_line[0]=Monotonicity_Change_Left(60,0);//单调性改变
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
           // if(monotonicity_change_line[0] > 0 && monotonicity_change_line[0]<=40)
            if(monotonicity_change_line[0] > 0 && monotonicity_change_line[0]<=60)
            {
                g_left_circle_state2_no_monotonicity_frames = 0;
                Left_Add_Line(monotonicity_change_line[1]-30,MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
            }
            else if(monotonicity_change_line[0]>60)
            {
                g_left_circle_state2_no_monotonicity_frames = 0;
                short_col_point[0] = limit_a_b(k_circle_short_col_fallback_y, 5, MT9V03X_H - 5);
                short_col_point[1] = limit_a_b(k_left_circle_short_col_fallback_x, Border_Min, Border_Max);
                short_col_point_is_fallback = true;
                set_circle_state(3, now);
            }
            else
            {
                g_left_circle_state2_no_monotonicity_frames++;
                if(g_left_circle_state2_no_monotonicity_frames >= k_circle_state2_no_monotonicity_timeout_frames)
                {
                    g_left_circle_state2_no_monotonicity_frames = 0;
                    short_col_point[0] = limit_a_b(k_circle_short_col_fallback_y, 5, MT9V03X_H - 5);
                    short_col_point[1] = limit_a_b(k_left_circle_short_col_fallback_x, Border_Min, Border_Max);
                    short_col_point_is_fallback = true;
                    set_circle_state(3, now);
                }
            }
        }
        else if(island_state==3)
        {
            shortest_White_Column1[0]=shortest_White_Column(45,140,80,15);

            if(shortest_White_Column1[0]!=0)
            {
                start = 90;
                start = limit_a_b(start,5, MT9V03X_H - 5);
                end = shortest_White_Column1[1];
                end = limit_a_b(end, Border_Min, Border_Max);
                Get_K_b(Right_Line[start], start, end, short_col_point[0], &slope_l_rate, &intercept_l);
                slope_l_rate_temp= slope_l_rate;
                intercept_l_temp=intercept_l;
                for (i = 5 ; i < 100; i++)
                {
                    Right_Line[i] = (i - intercept_l) / slope_l_rate;
                    Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);
                }
            }
            else
            {
                if(short_col_point[0] > 0)
                {
                    Right_Add_Line(short_col_point[1], short_col_point[0], MT9V03X_W, MT9V03X_H / 3 * 2);
                }
            }

            //if (car.circle_intergrate_yaw < -40 || shortest_White_Column1[0] < 30)
            if (car.circle_intergrate_yaw < -50)
            {
                set_circle_state(4, now);
            }
        }
        else if(island_state == 4)
        {
            const int right_mono = Monotonicity_Change_Right(MT9V03X_H - 5, 25);
            if(car.circle_intergrate_yaw <= -360)
            {
                set_circle_state(6, now);
            }
            else if(car.circle_intergrate_yaw < -250 &&
                    (R_D_corner_flag == 1 || (right_mono > 25 && right_mono <= 60)))
            {
                if (right_mono > 25 && right_mono <= 60)
                {
                    g_left_circle_exit_mono_row = right_mono;
                    g_left_circle_exit_mono_col = Right_Line[right_mono];
                }
                else
                {
                    g_left_circle_exit_mono_row = 0;
                    g_left_circle_exit_mono_col = 0;
                }
                set_circle_state(5, now);
            }
        }
        else if(island_state == 5)
        {
            // if(car.circle_intergrate_yaw <= -360)
            // {
            //     island_state = 6;
            // }
           // else if(R_D_corner_flag == 1)
            if (R_D_corner_flag == 1)
            {
                Right_Add_Line(R_D_corner_col,R_D_corner_row,0,0);
            }
            else
            {
                const int right_mono = Monotonicity_Change_Right(MT9V03X_H - 5, 25);
                if (right_mono > 25 && right_mono <= 60)
                {
                    g_left_circle_exit_mono_row = right_mono;
                    g_left_circle_exit_mono_col = Right_Line[right_mono];
                }

                if (g_left_circle_exit_mono_row > 25 && g_left_circle_exit_mono_row <= 60)
                {
                    Right_Add_Line(g_left_circle_exit_mono_col,g_left_circle_exit_mono_row,0,0);
                }
                else
                {
                    Right_Add_Line(MT9V03X_W,MT9V03X_H-30,0,0);
                }
            }
            if(L_U_corner_flag == 1||abs(car.circle_intergrate_yaw) < 30)
            {
                set_circle_state(6, now);
            }
        }
        else if(island_state == 6)
        {
            if(L_U_corner_flag)
            {
                Lengthen_Left_Boundry(L_U_corner_row,MT9V03X_H-5);
                temp_LU[1] = L_U_corner_col;
                temp_LU[0] = L_U_corner_row;
            }
            else
            {
                start = temp_LU[0];
                start = limit_a_b(start,5, MT9V03X_H - 5);
                end = MT9V03X_H - 5;
                end = limit_a_b(end, 5, MT9V03X_H - 5);
                Get_K_b(Left_Line[start], start, Left_Line[start]-20, end, &slope_l_rate, &intercept_l);

                for (i = 3; i < MT9V03X_H-5; i++)
                {
                    Left_Line[i] = (i - intercept_l) / slope_l_rate+20;
                    Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);
                }
            }

            if(Right_Lost_Time<20||Left_Lost_Time<20)
            {
                finish_circle_normally(now);
            }
        }
    }



    if(Image_Flag.Right_Circle == 1)
    {
        if(island_state == 1)
        {
//            right_down_guai[0]=Find_Right_Down_Point(MT9V03X_H-10, 50);
//            right_down_guai[1]=Right_Line[right_down_guai[0]];
//
//            if(right_down_guai[0]!=0)
//            {
//                Right_Add_Line(right_down_guai[1]-1,right_down_guai[0]-1,right_down_guai[1]-30,0);
//            }
            right_down_guai[0]=R_D_corner_row;
            right_down_guai[1]=R_D_corner_col;

            if(R_D_corner_flag!=0)
            {
                Right_Add_Line(right_down_guai[1]-1,right_down_guai[0]-1,right_down_guai[1]-30,0);
            }
            else if(R_D_corner_flag==0&&continuity_change_right_flag-monotonicity_change_right_flag> 5)
            {
                set_circle_state(2, now);
            }
        }
        else if(island_state==2) //2状态下方丢线，上方即将出现大弧线
        {
            monotonicity_change_line[0] = Monotonicity_Change_Right(80, 25); // 单调性改变
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
            if(monotonicity_change_line[0] > 0 && monotonicity_change_line[0]<=60)//&&monotonicity_change_line[0]>=20)//右下角再不丢线进3
            {
                g_right_circle_state2_no_monotonicity_frames = 0;
                Right_Add_Line(monotonicity_change_line[1]+30,MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
            }
            else if(monotonicity_change_line[0]>60)
            {
                g_right_circle_state2_no_monotonicity_frames = 0;
                short_col_point[0] = limit_a_b(k_circle_short_col_fallback_y, 5, MT9V03X_H - 5);
                short_col_point[1] = limit_a_b(k_right_circle_short_col_fallback_x, Border_Min, Border_Max);
                short_col_point_is_fallback = true;
                set_circle_state(3, now);
            }
            else
            {
                g_right_circle_state2_no_monotonicity_frames++;
                if(g_right_circle_state2_no_monotonicity_frames >= k_circle_state2_no_monotonicity_timeout_frames)
                {
                    g_right_circle_state2_no_monotonicity_frames = 0;
                    short_col_point[0] = limit_a_b(k_circle_short_col_fallback_y, 5, MT9V03X_H - 5);
                    short_col_point[1] = limit_a_b(k_right_circle_short_col_fallback_x, Border_Min, Border_Max);
                    short_col_point_is_fallback = true;
                    set_circle_state(3, now);
                }
            }
        }
        else if(island_state==3) //下面已经出现大弧线，且上方出现角点//根本不进3里头，直接瞬间调到4里
        {

            shortest_White_Column1[0]=shortest_White_Column(19,114,80,15,true);//从左往右
//            ips200_show_string(0,210,"shortestwhitecolumn1[0]:");
//            ips200_show_int(200,210,shortest_White_Column1[0],3);
            if(shortest_White_Column1[0]!=0)
            {
//                        if(MT9V03X_H-shortest_White_Column1[0]<=50)
                start = 90;//起点
                start = limit_a_b(start,5, MT9V03X_H - 5);//限幅
                end = shortest_White_Column1[1];//终点
                end = limit_a_b(end, Border_Min, Border_Max);
                Get_K_b(Left_Line[start], start, end, short_col_point[0], &slope_l_rate, &intercept_l);
                slope_l_rate_temp= slope_l_rate;
                intercept_l_temp=intercept_l;
                for (i = 5 ; i < 100; i++)
                {
                    Left_Line[i] = (i - intercept_l_temp) / slope_l_rate_temp; // y = kx+b
                    Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);
                }
                // 等效于Left_Add_Line(short_col_point[1],short_col_point[0], 0,60);             
            }


            //=========================================================================
//                            else if(shortest_White_Column1[0]<30)
//                        else if(MT9V03X_H-shortest_White_Column1[0]>50)//拐点出现在一定范围内，认为是拐点出现
            else
            {
                if(short_col_point[0] > 0)
                {
                    Left_Add_Line(short_col_point[1],short_col_point[0],0,MT9V03X_H/3*2);
                }
            }
            //if(car.circle_intergrate_yaw > 40 || shortest_White_Column1[0] < 30)
            if (car.circle_intergrate_yaw > 50)
            {
                set_circle_state(4, now);
            }
        }
        else if(island_state == 4)
        {
            const int left_mono = Monotonicity_Change_Left(MT9V03X_H - 5, 25);
            if(car.circle_intergrate_yaw >= 360)
            {
                set_circle_state(6, now);
            }
            else if(car.circle_intergrate_yaw >250 &&
                    (L_D_corner_flag == 1 || (left_mono > 25 && left_mono <= 60)))
            {
                if (left_mono > 25 && left_mono <= 60)
                {
                    g_right_circle_exit_mono_row = left_mono;
                    g_right_circle_exit_mono_col = Left_Line[left_mono];
                }
                else
                {
                    g_right_circle_exit_mono_row = 0;
                    g_right_circle_exit_mono_col = 0;
                }
                set_circle_state(5, now);
            }
        }
        else if(island_state == 5)
        {
            // if(car.circle_intergrate_yaw >= 360)
            // {
            //     island_state = 6;
            // }
          //  else if(L_D_corner_flag == 1)
           if (L_D_corner_flag == 1)
            {
                Left_Add_Line(L_D_corner_col,L_D_corner_row,MT9V03X_W,0);
            }
            else
            {
                const int left_mono = Monotonicity_Change_Left(MT9V03X_H - 5, 25);
                if (left_mono > 25 && left_mono <= 60)
                {
                    g_right_circle_exit_mono_row = left_mono;
                    g_right_circle_exit_mono_col = Left_Line[left_mono];
                }

                if (g_right_circle_exit_mono_row > 25 && g_right_circle_exit_mono_row <= 60)
                {
                    Left_Add_Line(g_right_circle_exit_mono_col,g_right_circle_exit_mono_row,MT9V03X_W,0);
                }
                else
                {
                    Left_Add_Line(0,MT9V03X_H-30,MT9V03X_W,0);
                }
            }
            if(R_U_corner_flag == 1 || abs(car.circle_intergrate_yaw) < 30)
            {
                set_circle_state(6, now);
            }

        }
        else if(island_state == 6)
        {
            if(R_U_corner_flag)
            {
                Lengthen_Right_Boundry(R_U_corner_row,MT9V03X_H-5);

                temp_RU[1] = R_U_corner_col;
                temp_RU[0] = R_U_corner_row;
            }
            else
            {

                start = temp_RU[0] ;//起点
                start = limit_a_b(start,5, MT9V03X_H - 5);//限幅
                end = MT9V03X_H - 5;//终点
                end = limit_a_b(end, 5, MT9V03X_H - 5);
                Get_K_b(Right_Line[start], start, Right_Line[start]+20, end, &slope_l_rate, &intercept_l);
                slope_l_rate_temp = slope_l_rate;
                intercept_l_temp = intercept_l;
                //       Get_K_b(start_point_r[0], start, MT9V03X_W-2, end, &slope_l_rate, &intercept_l);
                {
                    for (i = 3  ; i < MT9V03X_H-5; i++)
                    {
                        Right_Line[i] = (i - intercept_l_temp) / slope_l_rate_temp - 20; // y = kx+b
                        Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);
                    }
                }

                // K_Add_Boundry_Right(Get_Left_K(40,MT9V03X_H-10),MT9V03X_W-20,MT9V03X_H,10);
                //Bu_right();
            }

//            if(continuity_change_right_flag == 0&&continuity_change_left_flag ==0)
            if(Right_Lost_Time<20||Left_Lost_Time<20)
            {
                finish_circle_normally(now);
            }

        }


    }

}




// 十字补线：当前帧满足双边丢线和拐点组合时，按直线重建左右边界。
void Cross_fill(void)
{
    uint16 i;

    uint16 start, end;

    float slope_l_rate = 0, intercept_l = 0;


    if(get_lost_line() == 1)
    {
        if(turn_point_num >= 2)
        {
            Image_Flag.Cross_Fill =true;
        }
//        if(Image_Flag.Cross_Fill == true) Set_Beepfreq(1);

        if ((L_D_corner_flag) && (L_U_corner_flag) && (R_D_corner_flag) && (R_U_corner_flag))//同时找到四个拐点 1111
        {
            Image_Flag.Cross_Fill = 1;
            //计算斜率,左边斜率
            Get_K_b(Left_Line[L_D_corner_row], L_D_corner_row, Left_Line[L_U_corner_row],L_U_corner_row,&slope_l_rate,&intercept_l);
            for (i = L_D_corner_row; i > L_U_corner_row; i--)
            {
                Left_Line[i] = (i - intercept_l)/ slope_l_rate;//y = kx+b
                Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);//限幅
            }

            //计算斜率,右边斜率
            Get_K_b(Right_Line[R_D_corner_row], R_D_corner_row, Right_Line[R_U_corner_row], R_U_corner_row, &slope_l_rate, &intercept_l);
            for (i = R_D_corner_row; i > R_U_corner_row; i--)
            {
                Right_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);//限幅
            }
        }
        else if ((L_D_corner_flag) && (L_U_corner_flag) && (!R_D_corner_flag) && (R_U_corner_flag))//右斜入十字 1101
        {
            Image_Flag.Cross_Fill = 1;
            //计算斜率
            Get_K_b(Left_Line[L_U_corner_row], L_U_corner_row, Left_Line[L_D_corner_row], L_D_corner_row, &slope_l_rate, &intercept_l);
            for (i = L_D_corner_row; i > L_U_corner_row; i--)
            {
                Left_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);//限幅
            }

            //计算斜率
            start = R_U_corner_row - 5;//起点
            start = limit_a_b(start, 5, MT9V03X_H-5);//限幅
            end = R_U_corner_row;//终点
            Get_K_b(Right_Line[start], start, Right_Line[end], end, &slope_l_rate, &intercept_l);
            for (i = R_U_corner_row; i < MT9V03X_H - 2; i++)
            {
                Right_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);
            }
        }
        else if ((!L_D_corner_flag) && (L_U_corner_flag) && (R_U_corner_flag) && (R_D_corner_flag))//左斜入十字 0111
        {
            Image_Flag.Cross_Fill = 1;
            //计算斜率
            start = L_U_corner_row - 5;
            start = limit_a_b(start, 5, MT9V03X_H - 5);
            end = L_U_corner_row;
            Get_K_b(Left_Line[start], start, Left_Line[end], end, &slope_l_rate, &intercept_l);
            for (i = L_U_corner_row; i < MT9V03X_H - 2; i++)
            {
                Left_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);//限幅
            }

            //计算斜率
            Get_K_b(Right_Line[R_U_corner_row], R_U_corner_row, Right_Line[R_D_corner_row], R_D_corner_row, &slope_l_rate, &intercept_l);
            for (i = R_D_corner_row; i > R_U_corner_row; i--)
            {
                Right_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);//限幅
            }
        }
        else  if((L_U_corner_flag) && (!R_U_corner_flag) && (L_D_corner_flag)&& (R_D_corner_flag))
        {

            Get_K_b(Left_Line[L_D_corner_row], L_D_corner_row, Left_Line[L_U_corner_row],L_U_corner_row,&slope_l_rate,&intercept_l);
            for (i = L_D_corner_row; i > L_U_corner_row; i--)
            {
                Left_Line[i] = (i - intercept_l)/ slope_l_rate;//y = kx+b
                Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);//限幅
            }

            //计算斜率
            start = R_D_corner_row + 5;//起点
            start = limit_a_b(start,5, MT9V03X_H - 5);//限幅
            end = R_D_corner_row + 2;//终点
            end = limit_a_b(end, 5, MT9V03X_H - 5);
            Get_K_b(Right_Line[start], start, Right_Line[end], end, &slope_l_rate, &intercept_l);
            for (i = 5; i < R_D_corner_row+5; i++)
            {
                Right_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);
            }
        }
        else if((!L_U_corner_flag) && (R_U_corner_flag) && (L_D_corner_flag)&& (R_D_corner_flag))
        {

            start = L_D_corner_row + 5;//起点
            start = limit_a_b(start,5, MT9V03X_H - 5);//限幅
            end = L_D_corner_row + 2;//终点
            end = limit_a_b(end, 5, MT9V03X_H - 5);
            Get_K_b(Left_Line[start], start, Left_Line[end], end, &slope_l_rate, &intercept_l);
            for (i = 5 ; i < L_D_corner_row+5; i++)
            {
                Left_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);
            }

            Get_K_b(Right_Line[R_D_corner_row], R_D_corner_row, Right_Line[R_U_corner_row], R_U_corner_row, &slope_l_rate, &intercept_l);
            for (i = R_D_corner_row; i > R_U_corner_row; i--)
            {
                Right_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);//限幅
            }
        }
        else if ((L_U_corner_flag) && (R_U_corner_flag) && (!L_D_corner_flag)&& (!R_D_corner_flag) /*&& (bin_image[10][MT9V03X_H - 15]) && (bin_image[119][MT9V03X_H - 15])*/)//只有上面两个点 0101
        {
            Image_Flag.Cross_Fill = 2;
            //计算斜率
            start = L_U_corner_row - 7;
            start = limit_a_b(start, 5, MT9V03X_H - 5);
            end = L_U_corner_row-2;
            end = limit_a_b(end, 5, MT9V03X_H - 5);
            Get_K_b(Left_Line[start], start, Left_Line[end], end, &slope_l_rate, &intercept_l);

            //	if(slope_l_rate>=-1.44) slope_l_rate=-1.44;
            for (i = L_U_corner_row; i < MT9V03X_H - 2; i++)
            {
                Left_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);//限幅
                //  ips200_show_float(50,140,slope_l_rate,3,3);
            }

            //计算斜率
            start = R_U_corner_row - 7;//起点
            start = limit_a_b(start, 5, MT9V03X_H - 5);//限幅
            end = R_U_corner_row-2;//终点
            end = limit_a_b(end, 5, MT9V03X_H - 5);
            Get_K_b(Right_Line[start], start, Right_Line[end], end, &slope_l_rate, &intercept_l);
            //if(slope_l_rate<=1.44) slope_l_rate = 1.44;
            for (i = R_U_corner_row; i < MT9V03X_H - 2; i++)
            {
                Right_Line[i] = (i - intercept_l) / slope_l_rate;//y = kx+b
                Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);
            }

            // ips200_show_float(150,140,slope_l_rate,3,3);
            // printf("找到十字\r\n");
        }
        else
        {
            Image_Flag.Cross_Fill = false;
        }
    }
    else
    {
        Image_Flag.Cross_Fill = false;
    }

}


// 统一入口：根据 point_mode 调用普通/圆环拐点检测，并更新四个拐点标志。
void get_turn_point(void)
{
    switch(point_mode)
    {
        case 0:
        {
            get_turning_point();
        }
        break;
        case 1:
        {
            get_down_turning_point();	  //获取下拐点
            get_up_turning_point();		  //获取上拐点
            validate_and_count_turn_points();
        }
        break;
    }
}
// 限制相邻行中线跳变，让 Mid_Line 沿纵向更平滑。
void fit_midline(void)//拟和中线，并进行滤波
{
    int i;    for (i = MT9V03X_H - 2; i >= 1; i--)
    {
        if(Mid_Line[i]-Mid_Line[i+1]>0)
        {
            if(Mid_Line[i]-Mid_Line[i+1]>8)
            {
                Mid_Line[i]=Mid_Line[i+1]+4;
            }
        }
        else if(Mid_Line[i]-Mid_Line[i+1]<0)
        {
            if(Mid_Line[i+1]-Mid_Line[i]>8)
            {
                Mid_Line[i]=Mid_Line[i+1]-4;
            }
        }
    }
}

// 对 Mid_Line 做五点滑动平均，生成最终发布用的 End_Mid_Line。
void HDPJ_lvbo(void)
{
    int i;    for (i = MT9V03X_H - 3; i >= 3; i--)
    {
        End_Mid_Line[i]=(Mid_Line[i+2]
                         +Mid_Line[i+1]
                         +Mid_Line[i]
                         +Mid_Line[i-1]
                         +Mid_Line[i-2])/5;
    }
}

/*-------------------------------------------------------------------------------------*/
// 单帧视觉处理主流程：二值化、搜边界、元素补线、生成中线。
void Image_Process(void)
{

    uint16 i;
    hightest =0;
    Threshold = otsuThreshold(image_copy[0], MT9V03X_W, MT9V03X_H)+10;	//大津法计算阈值
    turn_to_bin();					//图像二值化
    image_filter(bin_image);		//滤波
    image_draw_rectan(bin_image);//画方
//	Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-5);//测试画线
//	Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-5);
    data_stastics_l = 0;
    data_stastics_r = 0;
    Ipm_Left_Point_Count = 0;
    Ipm_Right_Point_Count = 0;
    Ipm_Mid_Point_Count = 0;
    Ipm_Bilateral_Mid_Point_Count = 0;
    for (int row = 0; row < MT9V03X_H; ++row)
    {
        Ipm_Left_Line[row] = -1;
        Ipm_Right_Line[row] = -1;
    }
    //check_cheku(90,30,4);									//找斑马线
    if(get_start_point(MT9V03X_H - 2))
    {
        search_l_r((uint16)USE_num,bin_image,&data_stastics_l, &data_stastics_r,start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1],&hightest);		//八邻域找边界

        get_left(data_stastics_l);		//取出边界
        get_right(data_stastics_r);

        get_turn_point();//获取拐点（突变点或向量法）
//        get_right_cusp_point();
//        get_left_cusp_point();
        get_lost_line();						//找左右丢线行数
        update_ipm_auto_reliable_edge_selection();
//        Zebra_Stripes_Detect(100,50);

//        if(Image_Flag.Zerba == 1&& complete == 0)
//        {
//            complete =1;
//        }
//        else if(Image_Flag.Zerba == 1&& complete == 2)
//        {
//            complete=3;
//        }
        if(mcx.start_finish == false)
        {
            Cross_fill();							//十字补线
//            circle_test();
            Island_Detect();
        }

        // test1 = static_cast<float>(island_state);                     // 圆环状态机当前状态
        // test2 = static_cast<float>(continuity_change_left_flag);       // 左圆环左边界连续性变化行
        // test3 = static_cast<float>(monotonicity_change_left_flag);     // 左圆环左边界单调性变化行
    }
//		Left_Add_Line(1,MT9V03X_H-5,MT9V03X_W-1,0);
    for (i = 0; i <  MT9V03X_H; i++)
    {
        Mid_Line[i] = (Right_Line[i] + Left_Line[i]) >> 1;//求中线
       // Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }
    transform_lines_to_ipm(Left_Line, Right_Line, Ipm_Left_Line, Ipm_Right_Line);
    build_ipm_bilateral_midline_from_lines(Ipm_Left_Line, Ipm_Right_Line);
    build_ipm_midline(effective_ipm_reliable_edge_mode());
    // build_test_midline(g_test_midline_mode);  // 旧方案：原图左右拟合黄线，已废弃，改用 IPM 中线
//   print_road_width_calibration();
    fit_midline();
    HDPJ_lvbo();

//    clear_block();
}

namespace
{
constexpr int k_composite_view_w = 80;
constexpr int k_composite_view_h = 60;

uint16 debug_color(uint16 color)
{
    return static_cast<uint16>((color << 8) | (color >> 8));
}

const char *debug_track_scene_short_name(TrackScene scene)
{
    switch (scene)
    {
    case TrackScene::Straight:
        return "ST";
    case TrackScene::GentleCurve:
        return "GC";
    case TrackScene::SharpCurve:
        return "SH";
    case TrackScene::ObstacleAvoid:
        return "OBS";
    case TrackScene::Circle:
        return "CIR";
    case TrackScene::LostLine:
        return "LOS";
    default:
        return "UNK";
    }
}

ReliableEdgeMode debug_effective_selected_edge(void)
{
    ReliableEdgeMode reliable_edge_mode = effective_ipm_reliable_edge_mode();
    if (reliable_edge_mode == ReliableEdgeMode::Auto)
    {
        reliable_edge_mode = selected_ipm_reliable_edge_mode();
    }
    return reliable_edge_mode;
}

void draw_debug_scaled_point(uint16 (*img)[image_width],
                             int dst_x,
                             int dst_y,
                             int dst_w,
                             int dst_h,
                             int src_x,
                             int src_y,
                             uint16 color)
{
    const int x = dst_x + src_x * dst_w / image_width;
    const int y = dst_y + src_y * dst_h / image_height;
    dbg_point(img, x, y, color);
}

void draw_debug_scaled_line(uint16 (*img)[image_width],
                            int dst_x,
                            int dst_y,
                            int dst_w,
                            int dst_h,
                            int src_x1,
                            int src_y1,
                            int src_x2,
                            int src_y2,
                            uint16 color)
{
    const int x1 = dst_x + src_x1 * dst_w / image_width;
    const int y1 = dst_y + src_y1 * dst_h / image_height;
    const int x2 = dst_x + src_x2 * dst_w / image_width;
    const int y2 = dst_y + src_y2 * dst_h / image_height;
    dbg_line(img, x1, y1, x2, y2, color);
}

// 右下混合 IPM 视角同时包含视觉 y=0~119 和车身参考 y=138，与普通原图视角分开映射。
constexpr int k_composite_ipm_vehicle_x = 79;
constexpr int k_composite_ipm_display_max_y = 138;

int composite_ipm_display_x(int dst_x, int dst_w, int source_x)
{
    const int clamped_x = std::clamp(source_x, 0, image_width - 1);
    return dst_x + clamped_x * dst_w / image_width;
}

int composite_ipm_display_y(int dst_y, int dst_h, int source_y)
{
    const int clamped_y = std::clamp(source_y, 0, k_composite_ipm_display_max_y);
    return dst_y + clamped_y * (dst_h - 1) / k_composite_ipm_display_max_y;
}

void draw_composite_ipm_scaled_line(uint16 (*img)[image_width],
                                    int dst_x,
                                    int dst_y,
                                    int dst_w,
                                    int dst_h,
                                    int source_x1,
                                    int source_y1,
                                    int source_x2,
                                    int source_y2,
                                    uint16 color)
{
    dbg_line(img,
             composite_ipm_display_x(dst_x, dst_w, source_x1),
             composite_ipm_display_y(dst_y, dst_h, source_y1),
             composite_ipm_display_x(dst_x, dst_w, source_x2),
             composite_ipm_display_y(dst_y, dst_h, source_y2),
             color);
}

void draw_scaled_source_image(uint16 (*img)[image_width],
                              const uint8 (*src)[image_width],
                              int dst_x,
                              int dst_y,
                              int dst_w,
                              int dst_h)
{
    for (int y = 0; y < dst_h; ++y)
    {
        const int src_y = y * image_height / dst_h;
        for (int x = 0; x < dst_w; ++x)
        {
            const int src_x = x * image_width / dst_w;
            const uint8 g = src[src_y][src_x];
            const uint16 r = (g >> 3) & 0x1F;
            const uint16 gr = (g >> 2) & 0x3F;
            const uint16 b = (g >> 3) & 0x1F;
            const uint16 color = static_cast<uint16>((r << 11) | (gr << 5) | b);
            img[dst_y + y][dst_x + x] = debug_color(color);
        }
    }
}

void draw_debug_panel_border(uint16 (*img)[image_width],
                             int x,
                             int y,
                             int w,
                             int h,
                             uint16 color)
{
    dbg_line(img, x, y, x + w - 1, y, color);
    dbg_line(img, x, y + h - 1, x + w - 1, y + h - 1, color);
    dbg_line(img, x, y, x, y + h - 1, color);
    dbg_line(img, x + w - 1, y, x + w - 1, y + h - 1, color);
}

void draw_composite_normal_view(uint16 (*img)[image_width],
                                int dst_x,
                                int dst_y,
                                int dst_w,
                                int dst_h,
                                bool show_binary)
{
    draw_scaled_source_image(img, show_binary ? bin_image : image_copy, dst_x, dst_y, dst_w, dst_h);

    const uint16 left_color = debug_color(RGB565_RED);
    const uint16 right_color = debug_color(RGB565_BLUE);
    const uint16 mid_color = debug_color(RGB565_GREEN);
    const uint16 border_color = debug_color(RGB565_GRAY);

    draw_debug_panel_border(img, dst_x, dst_y, dst_w, dst_h, border_color);

    for (int row = 0; row < image_height; row += 2)
    {
        draw_debug_scaled_point(img, dst_x, dst_y, dst_w, dst_h, left_edge_line[row], row, left_color);
        draw_debug_scaled_point(img, dst_x, dst_y, dst_w, dst_h, right_edge_line[row], row, right_color);
        draw_debug_scaled_point(img, dst_x, dst_y, dst_w, dst_h, mid_line[row], row, mid_color);
    }
}

void draw_composite_ipm_view(uint16 (*img)[image_width],
                             int dst_x,
                             int dst_y,
                             int dst_w,
                             int dst_h)
{
    const uint16 bg_color = debug_color(RGB565_WHITE);
    const uint16 edge_color = debug_color(RGB565_BLACK);
    const uint16 mid_color = debug_color(RGB565_RED);
    const uint16 control_mid_color = debug_color(RGB565_BLUE);
    const uint16 preview_color = debug_color(RGB565_RED);
    const uint16 raw_last_y_color = debug_color(static_cast<uint16>(0x87F0)); // 浅绿色
    const uint16 border_color = debug_color(RGB565_GRAY);

    dbg_fill_rect(img, dst_x, dst_y, dst_x + dst_w - 1, dst_y + dst_h - 1, bg_color);
    draw_debug_panel_border(img, dst_x, dst_y, dst_w, dst_h, border_color);

    auto draw_point_line = [&](const uint16 points[MT9V03X_H][2], uint16 count, uint16 color) {
        for (uint16 i = 1; i < count; ++i)
        {
            if (is_ipm_continuous(points[i - 1][0], points[i - 1][1],
                                  points[i][0], points[i][1]))
            {
                draw_composite_ipm_scaled_line(img,
                                               dst_x,
                                               dst_y,
                                               dst_w,
                                               dst_h,
                                               points[i - 1][0],
                                               points[i - 1][1],
                                               points[i][0],
                                               points[i][1],
                                               color);
            }
        }
    };

    draw_point_line(Ipm_Left_Points, Ipm_Left_Point_Count, edge_color);
    draw_point_line(Ipm_Right_Points, Ipm_Right_Point_Count, edge_color);

    if (Control_Ipm_Debug_Scene == TrackScene::LostLine)
    {
        draw_point_line(Ipm_Bilateral_Mid_Points, Ipm_Bilateral_Mid_Point_Count, mid_color);
    }
    else
    {
        draw_point_line(Ipm_Mid_Points, Ipm_Mid_Point_Count, mid_color);
    }

    // 控制预矄从车身 (79,138) 指向蓝线首点；调试点集的 y 会被限制119，因此单独补画这段。
    if (Control_Ipm_Extended_Mid_Point_Count > 0)
    {
        draw_composite_ipm_scaled_line(img,
                                       dst_x,
                                       dst_y,
                                       dst_w,
                                       dst_h,
                                       k_composite_ipm_vehicle_x,
                                       k_composite_ipm_display_max_y,
                                       Control_Ipm_Extended_Mid_Points[0][0],
                                       Control_Ipm_Extended_Mid_Points[0][1],
                                       control_mid_color);
    }
    draw_point_line(Control_Ipm_Extended_Mid_Points, Control_Ipm_Extended_Mid_Point_Count, control_mid_color);

    // y=119 是视觉原始 IPM 的最后一行，用浅绿线分隔车身参考区域。
    draw_composite_ipm_scaled_line(img,
                                   dst_x,
                                   dst_y,
                                   dst_w,
                                   dst_h,
                                   0,
                                   119,
                                   image_width - 1,
                                   119,
                                   raw_last_y_color);

    if (Control_Ipm_Preview_Target_Valid)
    {
        const int x = composite_ipm_display_x(dst_x, dst_w, Control_Ipm_Preview_Target[0]);
        const int y = composite_ipm_display_y(dst_y, dst_h, Control_Ipm_Preview_Target[1]);
        dbg_circle(img, x, y, 3, preview_color);
    }
}

void draw_lost_bar(uint16 (*img)[image_width],
                   int x,
                   int y,
                   const char *label,
                   int value,
                   uint16 color,
                   uint16 text_color,
                   uint16 bg_color)
{
    constexpr int k_bar_w = 28;
    constexpr int k_bar_h = 3;
    constexpr int k_label_w = 8;
    const int filled = std::clamp(value * k_bar_w / MT9V03X_H, 0, k_bar_w);

    dbg_text_3x5(img, x, y, label, text_color, bg_color, true);
    dbg_fill_rect(img, x + k_label_w, y + 1, x + k_label_w + k_bar_w - 1, y + k_bar_h, debug_color(RGB565_GRAY));
    if (filled > 0)
    {
        dbg_fill_rect(img, x + k_label_w, y + 1, x + k_label_w + filled - 1, y + k_bar_h, color);
    }
}

void draw_circle_state_blocks(uint16 (*img)[image_width],
                              int x,
                              int y,
                              uint16 color,
                              uint16 empty_color)
{
    const int state_count = std::clamp<int>(island_state, 0, 6);
    for (int block = 0; block < 6; ++block)
    {
        const int x0 = x + block * 5;
        const uint16 block_color = block < state_count ? color : empty_color;
        dbg_fill_rect(img, x0, y, x0 + 3, y + 4, block_color);
    }
}

void draw_composite_status_panel(uint16 (*img)[image_width], int x, int y, int w, int h)
{
    const uint16 bg_color = debug_color(RGB565_BLACK);
    const uint16 text_color = debug_color(RGB565_CYAN);
    const uint16 lost_color = debug_color(RGB565_RED);
    const uint16 left_color = debug_color(RGB565_RED);
    const uint16 right_color = debug_color(RGB565_BLUE);
    const uint16 both_color = debug_color(RGB565_YELLOW);
    const uint16 state_color = debug_color(RGB565_PURPLE);
    const uint16 empty_color = debug_color(RGB565_GRAY);
    const uint16 avoid_color = debug_color(RGB565_RED);

    dbg_fill_rect(img, x, y, x + w - 1, y + h - 1, bg_color);

    char scene_text[16] = {0};
    std::snprintf(scene_text, sizeof(scene_text), "SCN:%s", debug_track_scene_short_name(Control_Ipm_Debug_Scene));
    dbg_text_3x5(img, x + 2, y + 2, scene_text, text_color, bg_color, true);

    if (Image_Flag.Left_Circle || Image_Flag.Right_Circle || island_state != 0)
    {
        char circle_text[12] = {0};
        const char circle_dir = Image_Flag.Left_Circle ? 'L' : (Image_Flag.Right_Circle ? 'R' : '-');
        std::snprintf(circle_text, sizeof(circle_text), "C%c", circle_dir);
        dbg_text_3x5(img, x + 2, y + 9, circle_text, state_color, bg_color, true);
        draw_circle_state_blocks(img, x + 14, y + 9, state_color, empty_color);

        char yaw_text[16] = {0};
        std::snprintf(yaw_text, sizeof(yaw_text), "YAW:%+.0f", car.circle_intergrate_yaw);
        dbg_text_3x5(img, x + 2, y + 15, yaw_text, state_color, bg_color, true);
    }

    draw_lost_bar(img, x + 2, y + 20, "L", Left_Lost_Time, left_color, text_color, bg_color);
    draw_lost_bar(img, x + 2, y + 27, "R", Right_Lost_Time, right_color, text_color, bg_color);
    draw_lost_bar(img, x + 2, y + 34, "B", Both_Lost_Time, both_color, text_color, bg_color);
    if (is_lost_line())
    {
        dbg_text_3x5(img, x + 2, y + 43, "FLOS", lost_color, bg_color, true);
    }

    // 右上状态区在发生绕行时显示绕行方向。
    if (obstacle_avoid_active())
    {
        char avoid_text[12] = {0};
        std::snprintf(avoid_text,
                      sizeof(avoid_text),
                      "AVD:%s",
                      obstacle_avoid_direction_name(current_obstacle_avoid_direction()));
        dbg_text_3x5(img, x + 2, y + 50, avoid_text, avoid_color, bg_color, true);
    }
}

void draw_composite_reliable_edge_panel(uint16 (*img)[image_width], int x, int y, int w, int h)
{
    const uint16 bg_color = debug_color(RGB565_BLACK);
    const uint16 text_color = debug_color(RGB565_YELLOW);

    dbg_fill_rect(img, x, y, x + w - 1, y + h - 1, bg_color);

    const char *edge_text = "REL:R";
    if (Control_Ipm_Debug_Scene == TrackScene::LostLine)
    {
        edge_text = "REL:BOTH";
    }
    else if (debug_effective_selected_edge() == ReliableEdgeMode::ForceLeft)
    {
        edge_text = "REL:L";
    }

    dbg_text_3x5(img, x + 2, y + 2, edge_text, text_color, bg_color, true);

    char offset_text[16] = {0};
    std::snprintf(offset_text, sizeof(offset_text), "OFF:%4.1f", g_ipm_midline_offset_px.load());
    dbg_text_3x5(img, x + 2, y + 11, offset_text, text_color, bg_color, true);

    // 左下状态区显示当前图像处理帧率。
    char fps_text[16] = {0};
    std::snprintf(fps_text, sizeof(fps_text), "FPS:%4.1f", g_fps_value);
    dbg_text_3x5(img, x + 2, y + 20, fps_text, text_color, bg_color, true);

    // Line_Error 是 pure pursuit 最终发布的 alpha 角，单位为度。
    char alpha_text[16] = {0};
    std::snprintf(alpha_text, sizeof(alpha_text), "ALP:%+.1f", Line_Error);
    dbg_text_3x5(img, x + 2, y + 29, alpha_text, text_color, bg_color, true);

    const WheelControlTelemetry wheel = wheel_control_telemetry_snapshot();
    char left_pwm_text[16] = {0};
    char right_pwm_text[16] = {0};
    std::snprintf(left_pwm_text, sizeof(left_pwm_text), "PL:%+.1f", wheel.left_pwm);
    std::snprintf(right_pwm_text, sizeof(right_pwm_text), "PR:%+.1f", wheel.right_pwm);
    dbg_text_3x5(img, x + 2, y + 38, left_pwm_text, text_color, bg_color, true);
    dbg_text_3x5(img, x + 2, y + 47, right_pwm_text, text_color, bg_color, true);
}

void build_composite_debug_image(uint16 (*img)[image_width], bool show_binary)
{
    const uint16 bg_color = debug_color(RGB565_BLACK);
    for (int row = 0; row < image_height; ++row)
    {
        for (int col = 0; col < image_width; ++col)
        {
            img[row][col] = bg_color;
        }
    }

    draw_composite_normal_view(img, 0, 0, k_composite_view_w, k_composite_view_h, show_binary);
    draw_composite_status_panel(img, 80, 0, 80, 60);
    draw_composite_reliable_edge_panel(img, 0, 60, 80, 60);
    draw_composite_ipm_view(img, 80, 60, k_composite_view_w, k_composite_view_h);
}

void draw_debug_status_bar(uint16 (*img)[image_width])
{
    const uint16 lost_color = debug_color(RGB565_RED);
    const uint16 ok_color = debug_color(RGB565_GREEN);
    const uint16 cross_color = debug_color(RGB565_YELLOW);
    const uint16 circle_color = debug_color(RGB565_CYAN);
    const uint16 empty_color = debug_color(RGB565_GRAY);
    const uint16 bg_color = debug_color(RGB565_BLACK);

    dbg_fill_rect(img, 0, 0, image_width - 1, 7, bg_color);

    const int left_lost_width = std::clamp(Left_Lost_Time * 30 / MT9V03X_H, 0, 30);
    const int right_lost_width = std::clamp(Right_Lost_Time * 30 / MT9V03X_H, 0, 30);
    const int both_lost_width = std::clamp(Both_Lost_Time * 30 / MT9V03X_H, 0, 30);

    dbg_fill_rect(img, 0, 0, 29, 1, ok_color);
    if (left_lost_width > 0)
    {
        dbg_fill_rect(img, 0, 0, left_lost_width - 1, 1, lost_color);
    }
    dbg_fill_rect(img, 0, 3, 29, 4, ok_color);
    if (right_lost_width > 0)
    {
        dbg_fill_rect(img, 0, 3, right_lost_width - 1, 4, lost_color);
    }
    dbg_fill_rect(img, 0, 6, 29, 7, ok_color);
    if (both_lost_width > 0)
    {
        dbg_fill_rect(img, 0, 6, both_lost_width - 1, 7, lost_color);
    }

    if (Image_Flag.Cross_Fill)
    {
        dbg_fill_rect(img, 34, 1, 44, 6, cross_color);
    }
    if (Image_Flag.Left_Circle || Image_Flag.Right_Circle || island_state != 0)
    {
        char circle_text[4] = {0};
        const char circle_dir = Image_Flag.Left_Circle ? 'L' :
                                (Image_Flag.Right_Circle ? 'R' : '-');
        std::snprintf(circle_text,
                      sizeof(circle_text),
                      "%c%d",
                      circle_dir,
                      std::clamp<int>(island_state, 0, 6));
        dbg_text_3x5(img, 48, 1, circle_text, circle_color, bg_color, false);
        draw_circle_state_blocks(img, 58, 1, circle_color, empty_color);
    }
}

void draw_debug_track_lines(uint16 (*img)[image_width])
{
    const uint16 left_color = debug_color(RGB565_RED);
    const uint16 right_color = debug_color(RGB565_BLUE);
    const uint16 mid_color = debug_color(RGB565_GREEN);
    const uint16 center_color = debug_color(RGB565_GRAY);

    dbg_line(img, image_width / 2, 0, image_width / 2, image_height - 1, center_color);

    for (int row = 0; row < image_height; ++row)
    {
        dbg_point(img, left_edge_line[row], row, left_color);
        dbg_point(img, right_edge_line[row], row, right_color);
        dbg_point(img, mid_line[row], row, mid_color);
        // 旧方案：黄色拟合测试中线（Test_Mid_Line），已废弃，改用 IPM 视角查看可靠边
    }
}

void draw_debug_search_points(uint16 (*img)[image_width])
{
    const uint16 left_trace_color = debug_color(RGB565_RED);
    const uint16 right_trace_color = debug_color(RGB565_BLUE);
    const uint16 start_left_color = debug_color(RGB565_GREEN);
    const uint16 start_right_color = debug_color(RGB565_CYAN);
    const uint16 highest_color = debug_color(RGB565_GRAY);

    dbg_trace_points(img, points_l, data_stastics_l, left_trace_color, 4);
    dbg_trace_points(img, points_r, data_stastics_r, right_trace_color, 4);

    if (Image_Flag.L_Find)
    {
        dbg_cross(img, start_point_l[0], start_point_l[1], start_left_color, 3);
    }
    if (Image_Flag.R_Find)
    {
        dbg_cross(img, start_point_r[0], start_point_r[1], start_right_color, 3);
    }

    if (hightest < image_height)
    {
        dbg_line(img, 0, hightest, image_width - 1, hightest, highest_color);
    }
}

void draw_debug_corners(uint16 (*img)[image_width])
{
    const uint16 left_down_color = debug_color(RGB565_PINK);
    const uint16 right_down_color = debug_color(RGB565_BROWN);
    const uint16 monotonicity_color = debug_color(RGB565_YELLOW);
    const uint16 short_col_color = debug_color(RGB565_CYAN);

    // 普通四拐点先不画，避免和圆环状态机实际使用的点混在一起。
//    if (L_U_corner_flag)
//    {
//        dbg_rect(img, L_U_corner_col, L_U_corner_row, 4, debug_color(RGB565_YELLOW));
//    }
//    if (L_D_corner_flag)
//    {
//        dbg_rect(img, L_D_corner_col, L_D_corner_row, 4, debug_color(RGB565_PINK));
//    }
//    if (R_U_corner_flag)
//    {
//        dbg_rect(img, R_U_corner_col, R_U_corner_row, 4, debug_color(RGB565_CYAN));
//    }
//    if (R_D_corner_flag)
//    {
//        dbg_rect(img, R_D_corner_col, R_D_corner_row, 4, debug_color(RGB565_BROWN));
//    }

    const bool circle_active = Image_Flag.Left_Circle || Image_Flag.Right_Circle;

    if (circle_active && left_down_guai[0] >= 5)
    {
        dbg_rect(img, left_down_guai[1], left_down_guai[0], 4, left_down_color);
    }
    if (circle_active && right_down_guai[0] >= 5)
    {
        dbg_rect(img, right_down_guai[1], right_down_guai[0], 4, right_down_color);
    }
    if (circle_active && island_state >= 2 && monotonicity_change_line[0] >= 5)
    {
        dbg_circle(img, monotonicity_change_line[1], monotonicity_change_line[0], 5, monotonicity_color);
    }
    if (circle_active && island_state >= 3 && short_col_point[0] > 0)
    {
        if (short_col_point_is_fallback)
        {
            dbg_circle(img, short_col_point[1], short_col_point[0], 5, short_col_color);
        }
        else
        {
            dbg_cross(img, short_col_point[1], short_col_point[0], short_col_color, 4);
        }
    }
}
}

void build_ipm_lines_debug_image(uint16 (*img)[image_width])
{
    const uint16 bg_color = debug_color(RGB565_WHITE);
    const uint16 line_color = debug_color(RGB565_BLACK);
    const uint16 raw_mid_color = debug_color(RGB565_RED);
    const uint16 extended_mid_color = debug_color(RGB565_BLUE);
    const uint16 raw_last_y_color = debug_color(RGB565_CYAN);
    const uint16 preview_target_color = debug_color(RGB565_RED);
    auto track_scene_text = [](TrackScene scene) {
        switch (scene)
        {
        case TrackScene::Straight:
            return "SCN:Straight";
        case TrackScene::GentleCurve:
            return "SCN:Gentle";
        case TrackScene::SharpCurve:
            return "SCN:Sharp";
        case TrackScene::ObstacleAvoid:
            return "SCN:Obstacle";
        case TrackScene::Circle:
            return "SCN:Circle";
        case TrackScene::LostLine:
            return "SCN:Lost";
        default:
            return "SCN:Unknown";
        }
    };
    for (int row = 0; row < image_height; ++row)
    {
        for (int col = 0; col < image_width; ++col)
        {
            img[row][col] = bg_color;
        }
    }

    for (uint16 i = 1; i < Ipm_Left_Point_Count; ++i)
    {
        if (is_ipm_continuous(Ipm_Left_Points[i - 1][0], Ipm_Left_Points[i - 1][1],
                              Ipm_Left_Points[i][0], Ipm_Left_Points[i][1]))
        {
            dbg_line(img, Ipm_Left_Points[i - 1][0], Ipm_Left_Points[i - 1][1],
                     Ipm_Left_Points[i][0], Ipm_Left_Points[i][1], line_color);
        }
    }
    for (uint16 i = 1; i < Ipm_Right_Point_Count; ++i)
    {
        if (is_ipm_continuous(Ipm_Right_Points[i - 1][0], Ipm_Right_Points[i - 1][1],
                              Ipm_Right_Points[i][0], Ipm_Right_Points[i][1]))
        {
            dbg_line(img, Ipm_Right_Points[i - 1][0], Ipm_Right_Points[i - 1][1],
                     Ipm_Right_Points[i][0], Ipm_Right_Points[i][1], line_color);
        }
    }

    draw_ipm_midline(img, raw_mid_color);
    for (uint16 i = 1; i < Control_Ipm_Extended_Mid_Point_Count && i < MT9V03X_H; ++i)
    {
        if (is_ipm_continuous(Control_Ipm_Extended_Mid_Points[i - 1][0],
                              Control_Ipm_Extended_Mid_Points[i - 1][1],
                              Control_Ipm_Extended_Mid_Points[i][0],
                              Control_Ipm_Extended_Mid_Points[i][1]))
        {
            dbg_line(img,
                     Control_Ipm_Extended_Mid_Points[i - 1][0],
                     Control_Ipm_Extended_Mid_Points[i - 1][1],
                     Control_Ipm_Extended_Mid_Points[i][0],
                     Control_Ipm_Extended_Mid_Points[i][1],
                     extended_mid_color);
        }
    }
    if (Control_Ipm_Extended_Mid_Point_Count > 0)
    {
        dbg_cross(img,
                  Control_Ipm_Extended_Mid_Points[0][0],
                  Control_Ipm_Extended_Mid_Points[0][1],
                  extended_mid_color,
                  3);
        const uint16 last_idx = std::min<uint16>(Control_Ipm_Extended_Mid_Point_Count, MT9V03X_H) - 1;
        dbg_cross(img,
                  Control_Ipm_Extended_Mid_Points[last_idx][0],
                  Control_Ipm_Extended_Mid_Points[last_idx][1],
                  extended_mid_color,
                  3);
    }
    if (Control_Ipm_Raw_Last_Valid_Y >= 0 && Control_Ipm_Raw_Last_Valid_Y < MT9V03X_H)
    {
        dbg_line(img, 0, Control_Ipm_Raw_Last_Valid_Y, MT9V03X_W - 1,
                 Control_Ipm_Raw_Last_Valid_Y, raw_last_y_color);
    }
    if (Control_Ipm_Preview_Target_Valid)
    {
        dbg_circle(img,
                   Control_Ipm_Preview_Target[0],
                   Control_Ipm_Preview_Target[1],
                   5,
                   preview_target_color);
    }

    ReliableEdgeMode reliable_edge_mode = effective_ipm_reliable_edge_mode();
    if (reliable_edge_mode == ReliableEdgeMode::Auto)
    {
        reliable_edge_mode = selected_ipm_reliable_edge_mode();
    }
    if (reliable_edge_mode == ReliableEdgeMode::ForceLeft)
    {
        dbg_text_6x8(img, 2, 0, "REL:L", debug_color(RGB565_RED), bg_color, false);
    }
    else
    {
        dbg_text_6x8(img, 2, 0, "REL:R", debug_color(RGB565_RED), bg_color, false);
    }
    char debug_text[32] = {0};
    std::snprintf(debug_text,
                  sizeof(debug_text),
                  "EXT:%u RAWY:%d",
                  static_cast<unsigned>(Control_Ipm_Extended_Mid_Point_Count),
                  static_cast<int>(Control_Ipm_Raw_Last_Valid_Y));
    dbg_text_6x8(img, 2, 10, debug_text, extended_mid_color, bg_color, false);

    char offset_text[24] = {0};
    std::snprintf(offset_text, sizeof(offset_text), "OFF:%4.1f", g_ipm_midline_offset_px.load());
    dbg_text_6x8(img, 2, 20, offset_text, debug_color(RGB565_PURPLE), bg_color, false);
    if (obstacle_avoid_active())
    {
        char avoid_text[16] = {0};
        std::snprintf(avoid_text,
                      sizeof(avoid_text),
                      "AVD:%s",
                      obstacle_avoid_direction_name(current_obstacle_avoid_direction()));
        dbg_text_6x8(img, 2, 30, avoid_text, debug_color(RGB565_RED), bg_color, false);
    }
    if (Image_Flag.Left_Circle || Image_Flag.Right_Circle || island_state != 0)
    {
        char yaw_text[16] = {0};
        std::snprintf(yaw_text, sizeof(yaw_text), "YAW:%+.0f", car.circle_intergrate_yaw);
        dbg_text_6x8(img, 2, 40, yaw_text, debug_color(RGB565_CYAN), bg_color, false);
    }

    char error_text[24] = {0};
    std::snprintf(error_text, sizeof(error_text), "ERR:%+.1f", Line_Error);
    dbg_text_6x8(img, image_width - 8 * 6 - 2, 0,
                 error_text, debug_color(RGB565_PURPLE), bg_color, false);
    dbg_text_6x8(img, 2, image_height - 8,
                 track_scene_text(Control_Ipm_Debug_Scene),
                 debug_color(RGB565_PURPLE), bg_color, false);
}

// 构建 SCC8660 彩色调试图。只写 debug_image，不回写算法使用的灰度图/二值图。
void build_debug_image(bool show_binary)
{
    if (g_debug_view_mode == DebugViewMode::Composite)
    {
        build_composite_debug_image(debug_image, show_binary);
        return;
    }

    if (g_debug_view_mode == DebugViewMode::Ipm)
    {
        build_ipm_lines_debug_image(debug_image);
        return;
    }

    dbg_from_gray(debug_image,
                  show_binary ? bin_image : image_copy,
                  nullptr,
                  nullptr,
                  RGB565_BLACK,
                  true);

    draw_debug_track_lines(debug_image);//画边界线
    draw_debug_search_points(debug_image);
    draw_debug_corners(debug_image);
    draw_debug_status_bar(debug_image);
    // IPM 实际采用的可靠边。使用紧凑文字，避免覆盖 x=58~86 的圆环状态条。
    const char *edge_text = "REL:R";
    if (Control_Ipm_Debug_Scene == TrackScene::LostLine)
    {
        edge_text = "REL:B";
    }
    else if (debug_effective_selected_edge() == ReliableEdgeMode::ForceLeft)
    {
        edge_text = "REL:L";
    }
    dbg_text_3x5(debug_image, 90, 1, edge_text,
                 debug_color(RGB565_YELLOW), debug_color(RGB565_BLACK), false);
    if (Image_Flag.Left_Circle || Image_Flag.Right_Circle || island_state != 0)
    {
        char yaw_text[16] = {0};
        std::snprintf(yaw_text, sizeof(yaw_text), "YAW:%+.0f", car.circle_intergrate_yaw);
        dbg_text_3x5(debug_image, 48, 9, yaw_text,
                     debug_color(RGB565_CYAN), debug_color(RGB565_BLACK), false);
    }
    // 当前赛道场景（右上角）
    {
        const char *scene_str = "Unknown";
        switch (Control_Ipm_Debug_Scene)
        {
        case TrackScene::Straight:   scene_str = "Straight"; break;
        case TrackScene::GentleCurve: scene_str = "Gentle";   break;
        case TrackScene::SharpCurve:  scene_str = "Sharp";    break;
        case TrackScene::ObstacleAvoid: scene_str = "Obstacle"; break;
        case TrackScene::Circle:      scene_str = "Circle";   break;
        case TrackScene::LostLine:    scene_str = "LOST";     break;
        default: break;
        }
        dbg_text_6x8(debug_image, 112, 0, scene_str, debug_color(RGB565_CYAN), debug_color(RGB565_BLACK), false);
    }
}

namespace
{
// 每帧处理前重置边界/中线为安全默认值，避免沿用上一帧脏数据。
void reset_track_lines(void)
{
    for (int row = 0; row < image_height; ++row)
    {
        Left_Line[row] = k_default_left;
        Right_Line[row] = k_default_right;
        Mid_Line[row] = k_default_mid;
        End_Mid_Line[row] = k_default_mid;
        Test_Mid_Line[row] = k_default_mid;      
    }
}

}

// 将内部 Left/Right/End_Mid_Line 转成图传和控制共用的小写边界数组。
void update_track_lines(void)
{
    for (int row = 0; row < image_height; ++row)
    {
        const int left = std::clamp<int>(Left_Line[row], 0, image_width - 1);
        const int right = std::clamp<int>(Right_Line[row], 0, image_width - 1);
        int mid = End_Mid_Line[row] != 0 ? End_Mid_Line[row] : Mid_Line[row];
        if (mid == 0 && right > left)
        {
            mid = (left + right) / 2;
        }

        left_edge_line[row] = (uint8)left;
        right_edge_line[row] = (uint8)right;
        mid_line[row] = (uint8)std::clamp<int>(mid, 0, image_width - 1);
        boundary_y_line[row] = (uint8)row;
    }
}

// 视觉任务入口：取 UVC 图像、缩放、运行单帧处理，并发布图传/控制数据。
bool image_test(void)
{
    static uint8 lost_frame_count = 0;

    if (uvc_dev.wait_image_refresh() < 0)
    {
        lost_frame_count++;
        std::cout << "摄像头采集异常，连续丢帧: " << static_cast<int>(lost_frame_count) << std::endl;
        if (lost_frame_count >= k_max_lost_frame_count)
        {
            std::cout << "摄像头连续多次丢帧，程序退出" << std::endl;
            exit(0);
        }
        return false;
    }
    lost_frame_count = 0;

    rgay_image = uvc_dev.get_gray_image_ptr();
    if (rgay_image == nullptr)
    {
        std::lock_guard<std::mutex> lock(g_image_mutex);
        reset_track_lines();//
        return false;//图像异常，跳出循环，保持上次结果不变，等待下一帧图像
    }

    {
        //降低分辨率
        std::lock_guard<std::mutex> lock(g_image_mutex);
        cv::Mat src_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, rgay_image);
        cv::Mat dst_mat(image_height, image_width, CV_8UC1, image_copy);
        cv::resize(src_mat, dst_mat, cv::Size(image_width, image_height), 0, 0, cv::INTER_AREA);

        //正式入口
        reset_track_lines();
        update_circle_integrate_yaw();
        Image_Process();
        update_track_lines();
        refresh_control_ipm_debug_midline();
    }
    return true;
}
