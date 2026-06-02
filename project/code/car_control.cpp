#include "car_control.hpp"
#include "image_test.hpp"
#include "speed_strategy.hpp"
#include "imu.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

extern TrackInfo g_track_info;
extern uint8_t island_state;

float vision_target_yaw = 0.0f; // 控制模块发布给角度环的目标航向角
float Line_Error = 0.0f;        // 控制模块根据 IPM 中线计算出的 pure pursuit 角度误差，单位：度
uint16 Control_Ipm_Extended_Mid_Points[MT9V03X_H][2] = {{0}};
uint16 Control_Ipm_Extended_Mid_Point_Count = 0;
int16 Control_Ipm_Line_X_By_Y[MT9V03X_H] = {0};
int16 Control_Ipm_Raw_Last_Valid_Y = -1;
bool Control_Ipm_Preview_Target_Valid = false;
uint16 Control_Ipm_Preview_Target[2] = {0, 0};
TrackScene Control_Ipm_Debug_Scene = TrackScene::Straight;

namespace
{
constexpr float k_ipm_control_sample_distance = 3.0f;
constexpr float k_ipm_control_break_distance = 18.0f;
constexpr uint16 k_ipm_control_max_points = 30;
// IPM 控制坐标系下的车身参考点：pure pursuit 的所有角度都从这里指向预瞄点。
constexpr float k_vehicle_x = 80.0f;
constexpr float k_vehicle_y = 119.0f;
constexpr float k_pi = 3.14159265358979323846f;
constexpr uint16 k_control_work_capacity = MT9V03X_H + 2;

struct LegacyPreviewYawParam
{
    float lateral_gain; // 横向纠偏力度
    float heading_gain; // 提前看弯力度
    float max_delta_yaw;// 目标角度最大限幅
};

struct PurePursuitParam
{
    float lookahead_dist; // 在补全后的 IPM 中线上的预瞄距离，单位：IPM 像素
    float max_alpha_deg;  // pure pursuit 角度误差限幅，单位：度
};

float wrap_to_180_control(float angle)
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

LegacyPreviewYawParam get_legacy_preview_yaw_param(TrackScene scene)
{
    if (scene == TrackScene::Circle)
    {
        if (island_state <= 3)      // 入环
            return {28.0f, 26.0f, 40.0f};
        else if (island_state <= 4) // 环内
            return {22.0f, 16.0f, 30.0f};
        else                        // 出环
            return {18.0f, 12.0f, 24.0f};
    }
    if (scene == TrackScene::SharpCurve)
    {
        return {25.0f, 17.0f, 35.0f};
    }
    if (scene == TrackScene::GentleCurve)
    {
        return {21.0f, 15.0f, 28.0f};
    }
    return {17.0f, 10.0f, 22.0f};
}

PurePursuitParam get_pure_pursuit_param(TrackScene scene)
{
    // 预瞄距离按场景固定给初值。急弯/环内刻意短一些，避免远端外推线过早主导转向。
    if (scene == TrackScene::Circle)
    {
        if (island_state <= 3)      // 入环，稍远一点看入口趋势
            return {30.0f, 40.0f};
        else if (island_state <= 4) // 环内，短预瞄避免外推线主导
            return {22.0f, 34.0f};
        else                        // 出环
            return {28.0f, 30.0f};
    }
    if (scene == TrackScene::SharpCurve)
    {
        return {24.0f, 42.0f};
    }
    if (scene == TrackScene::GentleCurve)
    {
        return {34.0f, 35.0f};
    }
    return {45.0f, 25.0f};
}

float point_distance(float x1, float y1, float x2, float y2)
{
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

bool is_ipm_control_continuous(float x1, float y1, float x2, float y2)
{
    return point_distance(x1, y1, x2, y2) <= k_ipm_control_break_distance;
}

void append_work_point(float points[k_control_work_capacity][2],
                       uint16 &count,
                       float x,
                       float y)
{
    if (count >= k_control_work_capacity)
    {
        return;
    }

    if (count > 0 &&
        point_distance(points[count - 1][0], points[count - 1][1], x, y) < 0.5f)
    {
        return;
    }

    points[count][0] = x;
    points[count][1] = y;
    count++;
}

void append_resampled_point(uint16 points[MT9V03X_H][2],
                            uint16 &count,
                            float x,
                            float y)
{
    if (count >= k_ipm_control_max_points || count >= MT9V03X_H)
    {
        return;
    }

    const uint16 out_x = static_cast<uint16>(
        std::clamp<int>(static_cast<int>(std::lround(x)), 0, MT9V03X_W - 1));
    const uint16 out_y = static_cast<uint16>(
        std::clamp<int>(static_cast<int>(std::lround(y)), 0, MT9V03X_H - 1));

    if (count > 0 &&
        points[count - 1][0] == out_x &&
        points[count - 1][1] == out_y)
    {
        return;
    }

    points[count][0] = out_x;
    points[count][1] = out_y;
    count++;
}

bool find_ipm_boundary_intersection(float x0,
                                    float y0,
                                    float dx,
                                    float dy,
                                    float &boundary_x,
                                    float &boundary_y)
{
    // 远端外推必须沿 IPM 前方（y 变小）延伸；方向异常时直接放弃外推。
    constexpr float k_eps = 1e-3f;
    constexpr float k_inside_eps = 1e-2f;
    const float dir_len = std::sqrt(dx * dx + dy * dy);
    if (dir_len < k_eps || dy >= -0.1f)
    {
        return false;
    }

    float best_t = -1.0f;
    auto consider_t = [&](float t) {
        if (t <= k_eps)
        {
            return;
        }
        const float x = x0 + dx * t;
        const float y = y0 + dy * t;
        if (x < -k_inside_eps || x > static_cast<float>(MT9V03X_W - 1) + k_inside_eps ||
            y < -k_inside_eps || y > static_cast<float>(MT9V03X_H - 1) + k_inside_eps)
        {
            return;
        }
        if (best_t < 0.0f || t < best_t)
        {
            best_t = t;
        }
    };

    if (dx > k_eps)
    {
        consider_t((static_cast<float>(MT9V03X_W - 1) - x0) / dx);
    }
    else if (dx < -k_eps)
    {
        consider_t(-x0 / dx);
    }

    if (dy > k_eps)
    {
        consider_t((static_cast<float>(MT9V03X_H - 1) - y0) / dy);
    }
    else if (dy < -k_eps)
    {
        consider_t(-y0 / dy);
    }

    if (best_t < 0.0f)
    {
        return false;
    }

    boundary_x = std::clamp(x0 + dx * best_t, 0.0f, static_cast<float>(MT9V03X_W - 1));
    boundary_y = std::clamp(y0 + dy * best_t, 0.0f, static_cast<float>(MT9V03X_H - 1));
    return true;
}

void fill_line_x_by_y(const uint16 points[MT9V03X_H][2],
                      uint16 count,
                      int16 line_x_by_y[MT9V03X_H])
{
    // 暂时只给后续调试使用：把控制用有序点集投影成“某个 y 对应的 x”。
    // 真实控制仍然基于点集弧长选预瞄点，不按行取中线。
    std::fill(line_x_by_y, line_x_by_y + MT9V03X_H, static_cast<int16>(-1));
    if (count == 0)
    {
        return;
    }

    for (uint16 i = 0; i < count; ++i)
    {
        line_x_by_y[points[i][1]] = static_cast<int16>(points[i][0]);
    }

    for (uint16 i = 1; i < count; ++i)
    {
        const float x1 = static_cast<float>(points[i - 1][0]);
        const float y1 = static_cast<float>(points[i - 1][1]);
        const float x2 = static_cast<float>(points[i][0]);
        const float y2 = static_cast<float>(points[i][1]);
        if (std::fabs(y2 - y1) < 1e-3f)
        {
            continue;
        }

        const int y_begin = std::clamp<int>(static_cast<int>(std::floor(std::min(y1, y2))), 0, MT9V03X_H - 1);
        const int y_end = std::clamp<int>(static_cast<int>(std::ceil(std::max(y1, y2))), 0, MT9V03X_H - 1);
        for (int y = y_begin; y <= y_end; ++y)
        {
            const float t = (static_cast<float>(y) - y1) / (y2 - y1);
            if (t < 0.0f || t > 1.0f)
            {
                continue;
            }
            const int x = static_cast<int>(std::lround(x1 + (x2 - x1) * t));
            line_x_by_y[y] = static_cast<int16>(std::clamp<int>(x, 0, MT9V03X_W - 1));
        }
    }
}

void resample_control_path(const float work_points[k_control_work_capacity][2],
                           uint16 work_count,
                           uint16 extended_points[MT9V03X_H][2],
                           uint16 &extended_count)
{
    // 近端补线和远端外推会产生不同长度的线段，这里统一重采样成 3px 左右的点距。
    // 只保留从车身点开始的前 30 个点，让控制侧看的是近中场，远端自然截断。
    extended_count = 0;
    if (work_count == 0)
    {
        return;
    }

    append_resampled_point(extended_points, extended_count, work_points[0][0], work_points[0][1]);
    float remain_distance = k_ipm_control_sample_distance;

    for (uint16 i = 1; i < work_count && extended_count < k_ipm_control_max_points; ++i)
    {
        float ax = work_points[i - 1][0];
        float ay = work_points[i - 1][1];
        const float bx = work_points[i][0];
        const float by = work_points[i][1];
        float dx = bx - ax;
        float dy = by - ay;
        float segment_len = std::sqrt(dx * dx + dy * dy);
        if (segment_len < 1e-3f)
        {
            continue;
        }

        while (segment_len >= remain_distance &&
               segment_len > 1e-3f &&
               extended_count < k_ipm_control_max_points)
        {
            const float t = remain_distance / segment_len;
            const float sample_x = ax + dx * t;
            const float sample_y = ay + dy * t;
            append_resampled_point(extended_points, extended_count, sample_x, sample_y);

            ax = sample_x;
            ay = sample_y;
            dx = bx - ax;
            dy = by - ay;
            segment_len = std::sqrt(dx * dx + dy * dy);
            remain_distance = k_ipm_control_sample_distance;
        }

        if (segment_len > 1e-3f)
        {
            remain_distance -= segment_len;
        }
    }

    if (extended_count < 2 && work_count > 1)
    {
        append_resampled_point(extended_points,
                               extended_count,
                               work_points[work_count - 1][0],
                               work_points[work_count - 1][1]);
    }
}

bool extend_ipm_midline(const uint16 mid_points[MT9V03X_H][2],
                        uint16 mid_count,
                        uint16 extended_points[MT9V03X_H][2],
                        uint16 &extended_count,
                        int16 line_x_by_y[MT9V03X_H],
                        int16 &raw_last_valid_y)
{
    // 控制侧只在局部副本上补全/外推，不回写 Ipm_Mid_Points。
    // raw_last_valid_y 记录原始中线真实看到的最后一个 y，后面调试时不能被外推终点替代。
    extended_count = 0;
    raw_last_valid_y = -1;
    std::fill(line_x_by_y, line_x_by_y + MT9V03X_H, static_cast<int16>(-1));

    mid_count = std::min<uint16>(mid_count, MT9V03X_H);
    if (mid_count == 0)
    {
        return false;
    }

    uint16 valid_count = 1;
    for (uint16 i = 1; i < mid_count; ++i)
    {
        // 原始点集一旦出现大跳变，就只使用跳变前的第一段连续中线。
        // 这样可以避免把断裂、误检点强行连成控制路径。
        if (!is_ipm_control_continuous(mid_points[i - 1][0], mid_points[i - 1][1],
                                       mid_points[i][0], mid_points[i][1]))
        {
            break;
        }
        valid_count = i + 1;
    }

    raw_last_valid_y = static_cast<int16>(mid_points[valid_count - 1][1]);

    float work_points[k_control_work_capacity][2] = {{0.0f}};
    uint16 work_count = 0;
    // 近端补全：无论视觉第一点在哪里，控制用路径都从车身参考点开始。
    append_work_point(work_points, work_count, k_vehicle_x, k_vehicle_y);
    for (uint16 i = 0; i < valid_count; ++i)
    {
        append_work_point(work_points,
                          work_count,
                          static_cast<float>(mid_points[i][0]),
                          static_cast<float>(mid_points[i][1]));
    }

    if (valid_count >= 8)
    {
        // 远端外推：用末端和倒数第 7 个点估计末端切线，只用于补足预瞄长度。
        // 外推不代表真实识别到了未来赛道，因此急弯和环内会用更短预瞄。
        const uint16 last_index = valid_count - 1;
        const uint16 prev_index = valid_count - 7;
        const float last_x = static_cast<float>(mid_points[last_index][0]);
        const float last_y = static_cast<float>(mid_points[last_index][1]);
        const float dx = last_x - static_cast<float>(mid_points[prev_index][0]);
        const float dy = last_y - static_cast<float>(mid_points[prev_index][1]);

        float boundary_x = last_x;
        float boundary_y = last_y;
        if (find_ipm_boundary_intersection(last_x, last_y, dx, dy, boundary_x, boundary_y) &&
            point_distance(last_x, last_y, boundary_x, boundary_y) > k_ipm_control_sample_distance)
        {
            append_work_point(work_points, work_count, boundary_x, boundary_y);
        }
    }

    resample_control_path(work_points, work_count, extended_points, extended_count);
    fill_line_x_by_y(extended_points, extended_count, line_x_by_y);
    return extended_count >= 2;
}

float normalized_midline_error(uint8 row)
{
    const int safe_row = std::clamp<int>(row, 0, MT9V03X_H - 1);
    int mid = End_Mid_Line[safe_row] != 0 ? End_Mid_Line[safe_row] : Mid_Line[safe_row];
    mid = std::clamp(mid, 0, image_width - 1);
    return std::clamp((mid - image_width / 2.0f) / (image_width / 2.0f), -1.0f, 1.0f);
}

float calc_legacy_preview_delta_yaw(TrackScene scene)
{
    uint8 near_row = 85;
    uint8 far_row = 55;
    const LegacyPreviewYawParam param = get_legacy_preview_yaw_param(scene);
    if (scene == TrackScene::Circle)
    {
        if (island_state <= 3)
        {
            near_row = 95;
            far_row = 45;
        }
        else if (island_state <= 4)
        {
            near_row = 85;
            far_row = 55;
        }
        else
        {
            near_row = 80;
            far_row = 60;
        }
    }
    else if (scene == TrackScene::SharpCurve)
    {
        near_row = 88;
        far_row = 50;
    }

    const float near_error = normalized_midline_error(near_row);
    const float far_error = normalized_midline_error(far_row);
    const float heading_error = far_error - near_error;
    const float delta_yaw = near_error * param.lateral_gain + heading_error * param.heading_gain;
    return std::clamp(delta_yaw, -param.max_delta_yaw, param.max_delta_yaw);
}

bool calc_preview_target_yaw(TrackScene scene,
                             float &target_yaw,
                             float &alpha_deg,
                             uint16 preview_target[2])
{
    // 新控制主路径：IPM 中线点集 -> 控制侧补全/外推 -> 按预瞄距离取 target -> atan2 得到角度误差。
    uint16 extended_points[MT9V03X_H][2] = {{0}};
    uint16 extended_count = 0;
    int16 line_x_by_y[MT9V03X_H] = {0};
    int16 raw_last_valid_y = -1;
    if (!extend_ipm_midline(Ipm_Mid_Points,
                            Ipm_Mid_Point_Count,
                            extended_points,
                            extended_count,
                            line_x_by_y,
                            raw_last_valid_y))
    {
        return false;
    }

    const PurePursuitParam param = get_pure_pursuit_param(scene);
    // 当前点距固定为 3px，所以可以把预瞄距离直接换成目标点索引。
    uint16 target_idx = static_cast<uint16>(
        std::lround(param.lookahead_dist / k_ipm_control_sample_distance));
    target_idx = std::clamp<uint16>(target_idx, 1, extended_count - 1);

    const float target_x = static_cast<float>(extended_points[target_idx][0]);
    const float target_y = static_cast<float>(extended_points[target_idx][1]);
    preview_target[0] = extended_points[target_idx][0];
    preview_target[1] = extended_points[target_idx][1];
    const float dx = target_x - k_vehicle_x;
    const float dy = k_vehicle_y - target_y;
    if (std::fabs(dx) < 1e-3f && std::fabs(dy) < 1e-3f)
    {
        return false;
    }

    // alpha 是车头正前方到预瞄点的夹角。后级仍然使用 yaw 角度环，所以发布 yaw + alpha。
    alpha_deg = std::atan2(dx, dy) * 180.0f / k_pi;
    alpha_deg = std::clamp(alpha_deg, -param.max_alpha_deg, param.max_alpha_deg);
    target_yaw = wrap_to_180_control(yaw + alpha_deg);
    (void)line_x_by_y;
    (void)raw_last_valid_y;
    return true;
}
}

void refresh_control_ipm_debug_midline(void)
{
    uint16 extended_points[MT9V03X_H][2] = {{0}};
    uint16 extended_count = 0;
    int16 line_x_by_y[MT9V03X_H] = {0};
    int16 raw_last_valid_y = -1;

    extend_ipm_midline(Ipm_Mid_Points,
                       Ipm_Mid_Point_Count,
                       extended_points,
                       extended_count,
                       line_x_by_y,
                       raw_last_valid_y);

    Control_Ipm_Extended_Mid_Point_Count = extended_count;
    Control_Ipm_Raw_Last_Valid_Y = raw_last_valid_y;
    for (uint16 i = 0; i < MT9V03X_H; ++i)
    {
        Control_Ipm_Line_X_By_Y[i] = line_x_by_y[i];
        Control_Ipm_Extended_Mid_Points[i][0] = extended_points[i][0];
        Control_Ipm_Extended_Mid_Points[i][1] = extended_points[i][1];
    }
}

//=========================================================================================
float Sum = 0;
uint16_t Weigth_Sum = 0;
uint8_t midline1_fff, midline1_ff, midline1_f;
uint8_t midline2_fff, midline2_ff, midline2_f;


const uint8 Weigth1[120] =//更近一点
 {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 2, 2, 2, 2,
    2, 4, 4, 4, 4, 4, 5, 5, 5, 5,
    5, 5, 5, 6, 6, 6, 6, 6, 6, 8,
    9, 9, 9, 10, 10, 10, 10, 10, 10, 10,
    10, 9, 9, 9, 9, 8, 8, 7, 7, 7,
    6, 7, 7, 7, 6, 6, 6, 6, 6, 5,
    5, 5, 5, 5, 4, 4, 3, 3, 3, 3,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};


const uint8 Weigth2[120] =//更远一点
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 2, 2, 2, 2,
    2, 4, 4, 4, 4, 4, 5, 5, 5, 5,
    5, 5, 5, 6, 6, 6, 6, 6, 6, 8,
    9, 9, 9, 10, 10, 10, 10, 10, 10, 10,
    10, 9, 9, 9, 9, 8, 8, 7, 7, 7,
    6, 7, 7, 7, 6, 6, 6, 6, 6, 5,
    5, 5, 5, 5, 4, 4, 3, 3, 3, 3,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,//第一行
};
float Cal_Weigth1(void)
{
    Sum = 0;
    Weigth_Sum = 0;
    float Mid_Error = 0.0f;

    for (uint8 i = 1; i < 100; i++)
    {
        Sum += End_Mid_Line[i] * Weigth1[i];
        Weigth_Sum += Weigth1[i];
    }

    if (Weigth_Sum > 0)
    {
        Mid_Error = (float)Sum / (float)Weigth_Sum;
    }

    // 检查数据有效性
    if (Mid_Error > image_width || Mid_Error < 0)
    {
        return 0.0f; // 返回安全值
    }

    // 滤波处理
    midline1_fff = midline1_ff;
    midline1_ff = midline1_f;
    midline1_f = Mid_Error;
    Mid_Error = midline1_fff * 0.20f + midline1_ff * 0.50f + midline1_f * 0.30f;

    // 归一化到 [-1, 1] 范围
    float normalized_deviation = (Mid_Error - (image_width / 2.0f)) / (image_width / 2.0f);

    // 限制在合理范围内
    if (normalized_deviation > 1.0f)
        normalized_deviation = 1.0f;
    if (normalized_deviation < -1.0f)
        normalized_deviation = -1.0f;

    return normalized_deviation;
}

float Cal_Weigth2(void)
{
    Sum = 0;
    Weigth_Sum = 0;
    float Mid_Error = 75.0f;

    for (uint8 i = 1; i < 100; i++)
    {
        Sum += End_Mid_Line[i] * Weigth2[i];
        Weigth_Sum += Weigth2[i];
    }

    if (Weigth_Sum > 0)
    {
        Mid_Error = (float)Sum / (float)Weigth_Sum;
    }

    // 检查数据有效性
    if (Mid_Error > image_width || Mid_Error < 0)
    {
        return 0.0f; // 返回安全值
    }

    // 滤波处理
    midline2_fff = midline2_ff;
    midline2_ff = midline2_f;
    midline2_f = Mid_Error;
    Mid_Error = midline2_fff * 0.20f + midline2_ff * 0.50f + midline2_f * 0.30f;

    //  归一化到 [-1, 1] 范围
    float normalized_deviation = (Mid_Error - (image_width / 2.0f)) / (image_width / 2.0f);

    // 限制在合理范围内
    if (normalized_deviation > 1.0f)
        normalized_deviation = 1.0f;
    if (normalized_deviation < -1.0f)
        normalized_deviation = -1.0f;

    return normalized_deviation;
}

// 摄像头异常或视觉丢线时调用：保持当前 yaw，不给角度环新的转向目标。
void publish_lost_line_result(void)
{
    std::lock_guard<std::mutex> lock(g_vision_result_mutex);
    g_track_info.scene = TrackScene::LostLine;
    g_track_info.deviation = 0.0f;
    Line_Error = 0.0f;
    vision_target_yaw = yaw;
}

// 控制目标发布入口。视觉模块只负责更新中线/场景特征；这里负责把视觉结果转成 target_yaw。
void update_control_target(void)
{
    std::lock_guard<std::mutex> image_lock(g_image_mutex);

    // 旧原图权重误差只作为 IPM 点集无效时的场景兜底判断，不再发布给控制闭环。
    const float legacy_deviation = std::clamp(Cal_Weigth1(), -1.0f, 1.0f);
    // 场景分类坚持使用原始真实 IPM 中线，避免远端外推改变赛道形状判断。
    const IpmMidlineSceneResult ipm_scene =
        classify_ipm_midline_scene(Ipm_Mid_Points, Ipm_Mid_Point_Count, hightest);

    TrackScene scene = TrackScene::Straight;
    const float abs_legacy_deviation = std::fabs(legacy_deviation);
    if (Image_Flag.Left_Circle || Image_Flag.Right_Circle)
    {
        scene = TrackScene::Circle;
    }
    else if (Both_Lost_Time > 55)
    {
        scene = TrackScene::LostLine;
    }
    else if (ipm_scene.scene == IpmMidlineScene::SharpCurve)
    {
        scene = TrackScene::SharpCurve;
    }
    else if (ipm_scene.scene == IpmMidlineScene::GentleCurve ||
             ipm_scene.scene == IpmMidlineScene::SCurve ||
             Image_Flag.Cross_Fill)
    {
        scene = TrackScene::GentleCurve;
    }
    else if (ipm_scene.scene == IpmMidlineScene::Invalid)
    {
        if (abs_legacy_deviation > 0.55f)
        {
            scene = TrackScene::SharpCurve;
        }
        else if (abs_legacy_deviation > 0.25f)
        {
            scene = TrackScene::GentleCurve;
        }
    }

    float target_yaw = yaw;
    float alpha_deg = 0.0f;
    uint16 preview_target[2] = {0, 0};
    bool preview_target_valid = false;
    if (scene != TrackScene::LostLine &&
        calc_preview_target_yaw(scene, target_yaw, alpha_deg, preview_target))
    {
        preview_target_valid = true;
    }
    else if (scene != TrackScene::LostLine)
    {
        // IPM 点集不足或补全失败时，退回旧的原图中线预瞄，保证控制还有保底输出。
        alpha_deg = calc_legacy_preview_delta_yaw(scene);
        target_yaw = wrap_to_180_control(yaw + alpha_deg);
    }

    std::lock_guard<std::mutex> lock(g_vision_result_mutex);
    // 对外发布的 deviation/Line_Error 现在都是角度误差，单位是度，不再是 [-1,1] 归一化横偏。
    Line_Error = (scene == TrackScene::LostLine) ? 0.0f : alpha_deg;
    g_track_info.deviation = Line_Error;
    g_track_info.scene = scene;
    Control_Ipm_Debug_Scene = scene;
    Control_Ipm_Preview_Target_Valid = preview_target_valid;
    Control_Ipm_Preview_Target[0] = preview_target[0];
    Control_Ipm_Preview_Target[1] = preview_target[1];
    vision_target_yaw = (scene == TrackScene::LostLine) ? yaw : target_yaw;
    if (g_debug_show_ipm_lines)
    {
        build_debug_image(g_debug_show_binary_image);
    }
}
