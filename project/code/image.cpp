#include "image.hpp"

#include "IPM_image.hpp"
#include "imu.hpp"
#include "speed_strategy.hpp"
#include "zgc_draw_tool.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

extern TrackInfo g_track_info;
extern uint8 *rgay_image;

float vision_target_yaw = 0.0f;
uint8 g_front_debug_display_mode = FrontDebugDisplayOriginal;
uint8 g_front_cross_kind = FrontCrossNone;
uint8 g_front_cross_state = FrontCrossState1;

uint8 g_front_mid_line[image_height] = {0};
uint8 g_front_mid_line_valid[image_height] = {0};
uint16 g_front_left_ipm_points[k_front_migration_max_points][2] = {{0}};
uint16 g_front_right_ipm_points[k_front_migration_max_points][2] = {{0}};
uint16 g_front_mid_ipm_points[k_front_migration_max_points][2] = {{0}};
uint16 g_front_left_ipm_point_count = 0;
uint16 g_front_right_ipm_point_count = 0;
uint16 g_front_mid_ipm_point_count = 0;
uint8 g_front_midline_ready = 0;
uint8 g_front_track_side_mode = 0;
uint8 g_front_track_side_control_mode = FrontTrackSideControlAuto;
uint8 g_front_line_lost = 3;
uint8 g_front_left_long_straight_flag = 0;
uint8 g_front_right_long_straight_flag = 0;
//===============yaw
uint8 g_front_left_ring_yaw_recording_flag = 0;
float g_front_left_ring_enter_unbounded_yaw = 0.0f;
float g_front_left_ring_progress_yaw = 0.0f;
//===================
FrontCornerPoint g_front_left_upper_corner = {};
FrontCornerPoint g_front_right_upper_corner = {};
FrontCornerPoint g_front_left_lower_corner = {};
FrontCornerPoint g_front_right_lower_corner = {};
void track_element(void);


namespace
{
    // Front_Car 移植参数集中放在这里，主流程保持：二值化 -> 找种子 -> 爬边 -> IPM -> 补中线。
    static constexpr int k_front_seed_search_row = image_height - 10; // 左右种子默认搜索行，往下调更看近端，往上调更看远端
    static constexpr int k_front_seed_min_row = image_height - 40;
    // 种子搜索允许到达的最远上边界，近端断边时可适当上调
    //  ==============================================================
    static constexpr uint8 k_front_binary_source_mode = 1;         // 值源模式，0=adaptive，1=clean Otsu，默认优先试 1
    static constexpr int k_front_binary_block_size = 5;            // adaptive 二值局部窗口，变大更抗噪，边也会更钝
    static constexpr int k_front_binary_clip_value = 6;            // adaptive 二值裁剪量，变大更容易判黑，边界会更收
    static constexpr int k_front_clean_binary_min_threshold = 40;  // clean Otsu 阈值下限，防止阈值过低导致大片发白
    static constexpr int k_front_clean_binary_max_threshold = 220; // clean Otsu 阈值上限，防止阈值过高导致边界被吃掉
    static constexpr int k_front_track_half_width_fallback = 20;   // 本帧种子宽度不可用时使用的半宽补偿，中线固定偏移时调它
    static constexpr int k_front_track_approx_step = 8;            // 单边补中线时的法向估计步长，变大更稳，变小更灵敏
    static constexpr int k_front_resample_dist = 3;                // 点集重采样间距，变大更平稳，变小更密更灵敏
    static constexpr int k_front_boundary_blur_kernel = 5;         // 边界点平滑核，变大更顺，但急弯细节会被抹掉
    static constexpr int k_front_mid_blur_kernel = 5;              // 中线点平滑核，变大更顺，但响应会变慢
    static constexpr int k_front_min_boundary_points = 12;         // 判定单侧边界有效的最少点数，太小易误判，太大易丢线
    static constexpr uint16 k_front_force_track_min_points = 2;    // 手动强制巡线时，该侧至少要能估计一段切线
    static constexpr int k_front_min_corner_points = 8;            // 允许做角点判定的最少边界点数，点太少时角度不稳定
    static constexpr int k_front_min_mid_valid_rows = 7;   //18       // 判定移植中线 ready 的最少有效行数，过小易误触发控制
    static constexpr int k_front_corner_mid_step = 6;
    static constexpr int k_front_corner_end_step = 12;
    static constexpr int k_front_corner_upper_scan_start = 7;
    static constexpr int k_front_corner_top_guard_row = 5;
    static constexpr int k_front_corner_vertical_delta_min = 4;
    static constexpr int k_front_corner_near_flat_delta_max = 4;
    static constexpr int k_front_corner_region_upper_max_row = (image_height * 3) / 5;
    static constexpr int k_front_corner_region_lower_min_row = image_height / 3;
    static constexpr int k_front_corner_large_window = 10;               // 角点大窗口，偏全局稳定，适合压制局部毛刺
    static constexpr int k_front_corner_small_window = 5;                // 角点小窗口，偏局部灵敏，用来补充大窗口确认
    static constexpr float k_front_corner_angle_low_deg = 70.0f;         // 角点允许的最小夹角，调大后误报更少但更容易漏检
    static constexpr float k_front_corner_angle_high_deg = 140.0f;       // 角点允许的最大夹角，调小后确认更严格
    static constexpr int k_front_corner_confirm_white_min = 6;           // 原图 3x3 白块确认最少白点数，调大后角点确认更保守
    static constexpr bool k_front_enable_split_corner_detection = false; // 先关闭四分裂拐点，只观察通用角点稳定性
    static constexpr uint16 k_front_long_straight_min_point_count = 25;
    static constexpr uint16 k_front_long_straight_min_row_span = 45;
    static constexpr uint16 k_front_long_straight_sample_step = 4;
    static constexpr uint16 k_front_long_straight_error_threshold = 3;
    static constexpr uint8 k_front_long_straight_boundary_overlap_ratio_num = 1;
    static constexpr uint8 k_front_long_straight_boundary_overlap_ratio_den = 4;
    static constexpr float k_front_left_ring_exit_switch_yaw = 315.0f; // 左圆环累计角达到该值后，允许切到出环阶段
    static constexpr int k_front_overlap_stop_count = 3;               // 爬边时连续重叠点终止阈值，避免在局部打转
    static constexpr int k_front_top_stop_row = 2;                     // 爬边提前停止的顶部行，调大可减少贴近图像顶端的噪声
    static constexpr bool k_front_draw_projected_edges = true;         // TCP 图是否额外叠加投影到 IPM 的左右边界
    static constexpr int k_front_draw_trace_step = 2;                  // 调试绘点步长，变大更稀疏，便于看整体走势
    static constexpr float k_front_pi = 3.14159265358979323846f;
    static constexpr uint8 k_front_max_lost_frame_count = 5;
    static constexpr bool k_front_enable_experimental_cross_state_machine = true; // 实验性十字状态机开关，便于整段撤回
    static constexpr int k_front_cross_single_width_tolerance = 10;               // 对齐 Front_Car CROSS_single_width
    static constexpr uint16 k_front_cross_min_original_points = 3;
    static constexpr uint16 k_front_cross_half_step_threshold = 20; // 对齐 Front_Car Step_Max / 2
    static constexpr uint8 k_front_cross_flag_threshold = 3;
    static constexpr uint8 k_front_cross_phase_threshold = 3; // 对齐 Front_Car Cross_Count_Threshold
    static constexpr int k_front_probe_search_radius = 8;     // 前探种子只在当前列附近找边，避免横向乱探
    static constexpr int k_front_probe_start_row_backoff = 1; // 前探从角点上方一行开始，明确以上探为主
    static constexpr int k_front_probe_search_rows_up = 30;   // 前探向上搜索的最大行跨度
    static constexpr uint16 k_front_probe_min_points = 6;     // 前探点太少时直接丢弃，避免把毛刺并进主边线
    static constexpr int k_front_probe_track_step = 5;        // 对齐 Front_Car WINDOW_SIZE/2，用局部切线估计前探法向
    static constexpr int k_front_probe_track_dist = 30;       // 对齐 Front_Car track_*point 的法向外推距离

    static constexpr uint8 k_binary_black = 0;
    static constexpr uint8 k_binary_white = 255;

    static uint8 g_front_binary[image_height][image_width] = {{0}};
    static uint16 g_front_left_original_points[k_front_migration_max_points][2] = {{0}};
    static uint16 g_front_right_original_points[k_front_migration_max_points][2] = {{0}};
    static uint16 g_front_mid_original_points[k_front_migration_max_points][2] = {{0}};
    static uint16 g_front_left_probe_original_points[k_front_migration_max_points][2] = {{0}};
    static uint16 g_front_right_probe_original_points[k_front_migration_max_points][2] = {{0}};
    static uint16 g_front_left_original_point_count = 0;
    static uint16 g_front_right_original_point_count = 0;
    static uint16 g_front_mid_original_point_count = 0;
    static uint16 g_front_left_probe_original_point_count = 0;
    static uint16 g_front_right_probe_original_point_count = 0;
    static int g_front_last_center_x = image_width / 2;
    static int g_front_last_seed_width = 0;
    static int g_front_last_ipm_track_width = 0;
    static uint8 g_front_binary_threshold = 0;
    static uint8 g_front_control_midline[image_height] = {0};

    static FrontCornerPoint g_front_left_corner = {};
    static FrontCornerPoint g_front_right_corner = {};

    struct FrontCrossMachine
    {
        uint8 phase_counter = 0;
        uint8 left_flag = 0;
        uint8 right_flag = 0;
    };

    static FrontCrossMachine g_front_cross_machine = {};

    struct SeedPoint
    {
        int x = 0;
        int y = 0;
    };

    struct SeedSearchResult
    {
        bool has_left_seed = false;
        bool has_right_seed = false;
        SeedPoint left_seed = {};
        SeedPoint right_seed = {};
    };

    static SeedSearchResult g_front_debug_seeds = {};

    struct FrontProbeConnectionContext
    {
        bool valid = false;
        int original_x = 0;
        int original_y = 0;
        int original_index = 0;
        int ipm_x = 0;
        int ipm_y = 0;
        int ipm_index = 0;
    };

    static bool map_front_original_to_ipm_point(int original_x, int original_y, int &ipm_x, int &ipm_y);
    static uint16 seek_front_probe_corner_index(const uint16 (*points)[2], uint16 count, bool left_side);

    enum FrontTrackSideMode : uint8
    {
        FrontTrackSideNone = 0,
        FrontTrackSideLeftBoundary = 1,
        FrontTrackSideRightBoundary = 2,
    };

    // 将value限制在[min_value, max_value]范围内
    static int clamp_int(int value, int min_value, int max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }
        if (value > max_value)
        {
            return max_value;
        }
        return value;
    }

    // 交换RGB565颜色的高低字节
    static uint16 swap_rgb565_bytes(uint16 color)
    {
        return static_cast<uint16>((color << 8) | (color >> 8));
    }

    // 根据大小端标志转换调试颜色
    static uint16 to_debug_color(uint16 color, bool big_endian)
    {
        return big_endian ? swap_rgb565_bytes(color) : color;
    }

    //========================================================================
    // 工具函数：坐标裁剪、颜色转换、小结构操作
    //========================================================================

    static void clear_track_corner_point(Track_Corner_Point_TypeDef &corner)
    {
        corner.flag = 0;
        corner.row = 0;
        corner.col = 0;
    }

    static void store_track_corner_point(Track_Corner_Point_TypeDef &corner, int x, int y)
    {
        corner.flag = 1;
        corner.row = static_cast<uint8>(clamp_int(y, 0, image_height - 1));
        corner.col = static_cast<uint8>(clamp_int(x, 0, image_width - 1));
    }

    static void reset_front_split_corner_points()
    {
        g_front_left_upper_corner = {};
        g_front_right_upper_corner = {};
        g_front_left_lower_corner = {};
        g_front_right_lower_corner = {};

        clear_track_corner_point(g_left_upper_corner);
        clear_track_corner_point(g_right_upper_corner);
        clear_track_corner_point(g_left_lower_corner);
        clear_track_corner_point(g_right_lower_corner);
    }

    // 获取运行时赛道半宽，必须使用 IPM 坐标系下的宽度
    static int get_runtime_track_half_width()
    {
        if (g_front_last_ipm_track_width >= 4)
        {
            return std::max(g_front_last_ipm_track_width / 2, 2);
        }

        return std::max(k_front_track_half_width_fallback, 2);
    }

    //========================================================================
    // 十字状态机：检测、进入、演化、退出
    //========================================================================

    static void reset_front_cross_machine()
    {
        g_front_cross_machine = {};
        g_front_cross_kind = FrontCrossNone;
        g_front_cross_state = FrontCrossState1;
        mode_element = 0;
    }

    static void enter_front_cross_machine()
    {
        g_front_cross_machine.phase_counter = 0;
        g_front_cross_kind = FrontCrossMid;
        g_front_cross_state = FrontCrossState1;
        mode_element = 1;
    }

    static int calc_front_ipm_distance(int x0, int y0, int x1, int y1)
    {
        return static_cast<int>(std::lround(std::hypot(static_cast<float>(x1 - x0),
                                                       static_cast<float>(y1 - y0))));
    }

    static bool get_front_cross_probe_point(bool left_side, int &x, int &y)
    {
        const FrontCornerPoint &corner = left_side ? g_front_left_corner : g_front_right_corner;
        if (corner.valid)
        {
            double mapped_x = 0.0;
            double mapped_y = 0.0;
            if (Transform_Point2(corner.x, corner.y, mapped_x, mapped_y))
            {
                x = clamp_int(static_cast<int>(std::lround(mapped_x)), 0, image_width - 1);
                y = clamp_int(static_cast<int>(std::lround(mapped_y)), 0, image_height - 1);
                return true;
            }
        }

        if (left_side)
        {
            if (g_front_left_ipm_point_count == 0)
            {
                return false;
            }

            x = g_front_left_ipm_points[g_front_left_ipm_point_count - 1][0];
            y = g_front_left_ipm_points[g_front_left_ipm_point_count - 1][1];
            return true;
        }

        if (g_front_right_ipm_point_count == 0)
        {
            return false;
        }

        x = g_front_right_ipm_points[g_front_right_ipm_point_count - 1][0];
        y = g_front_right_ipm_points[g_front_right_ipm_point_count - 1][1];
        return true;
    }

    static bool front_cross_boundaries_recovered()
    {
        return g_front_line_lost == 0 &&
               g_front_left_ipm_point_count > k_front_cross_half_step_threshold &&
               g_front_right_ipm_point_count > k_front_cross_half_step_threshold;
    }

    static bool front_cross_exit_frame_ready()
    {
        return front_cross_boundaries_recovered() &&
               !g_front_left_corner.valid &&
               !g_front_right_corner.valid &&
               !g_front_left_upper_corner.valid &&
               !g_front_right_upper_corner.valid;
    }

    static void judge_front_cross_state()
    {
        const bool left_corner_ready = g_front_left_corner.valid;
        const bool right_corner_ready = g_front_right_corner.valid;
        if (!left_corner_ready && !right_corner_ready)
        {
            g_front_cross_machine.left_flag = 0;
            g_front_cross_machine.right_flag = 0;
            return;
        }

        int left_x = 0;
        int left_y = 0;
        int right_x = 0;
        int right_y = 0;
        if (!get_front_cross_probe_point(true, left_x, left_y) ||
            !get_front_cross_probe_point(false, right_x, right_y))
        {
            return;
        }

        bool cross_flag = false;
        if (left_corner_ready && right_corner_ready)
        {
            cross_flag = true;
        }
        else
        {
            if (left_corner_ready &&
                right_y > left_y &&
                g_front_left_ipm_point_count > 0 &&
                g_front_left_ipm_points[g_front_left_ipm_point_count - 1][0] <= left_x &&
                g_front_right_ipm_point_count > k_front_cross_min_original_points)
            {
                ++g_front_cross_machine.left_flag;
            }
            else if (g_front_cross_machine.left_flag > 0)
            {
                --g_front_cross_machine.left_flag;
            }

            if (right_corner_ready &&
                right_y < left_y &&
                g_front_right_ipm_point_count > 0 &&
                g_front_right_ipm_points[g_front_right_ipm_point_count - 1][0] >= right_x &&
                g_front_left_ipm_point_count > k_front_cross_min_original_points)
            {
                ++g_front_cross_machine.right_flag;
            }
            else if (g_front_cross_machine.right_flag > 0)
            {
                --g_front_cross_machine.right_flag;
            }

            if (g_front_cross_machine.left_flag >= k_front_cross_flag_threshold ||
                g_front_cross_machine.right_flag >= k_front_cross_flag_threshold)
            {
                cross_flag = true;
            }
        }

        const int runtime_width = std::max(get_runtime_track_half_width() * 2, 4);
        const int distance = calc_front_ipm_distance(left_x, left_y, right_x, right_y);
        if (cross_flag &&
            distance >= runtime_width - k_front_cross_single_width_tolerance &&
            distance <= runtime_width + k_front_cross_single_width_tolerance)
        {
            enter_front_cross_machine();
            return;
        }

        if (left_corner_ready &&
            g_front_right_original_point_count < k_front_cross_min_original_points &&
            g_front_left_ipm_point_count < k_front_cross_half_step_threshold)
        {
            enter_front_cross_machine();
            return;
        }

        if (right_corner_ready &&
            g_front_left_original_point_count < k_front_cross_min_original_points &&
            g_front_right_ipm_point_count > k_front_cross_half_step_threshold)
        {
            enter_front_cross_machine();
        }
    }

    static void evolve_front_cross_state()
    {
        if (front_cross_boundaries_recovered())
        {
            ++g_front_cross_machine.phase_counter;
        }
        else
        {
            g_front_cross_machine.phase_counter = 0;
        }

        if (g_front_cross_machine.phase_counter > k_front_cross_phase_threshold)
        {
            g_front_cross_machine.phase_counter = 0;
            g_front_cross_state = FrontCrossState2;
        }
    }

    static void exit_front_cross_state()
    {
        if (front_cross_exit_frame_ready())
        {
            ++g_front_cross_machine.phase_counter;
        }
        else
        {
            g_front_cross_machine.phase_counter = 0;
        }

        if (g_front_cross_machine.phase_counter > k_front_cross_phase_threshold)
        {
            reset_front_cross_machine();
        }
    }

    [[maybe_unused]] static void update_front_cross_state_machine()
    {
        if (!k_front_enable_experimental_cross_state_machine)
        {
            reset_front_cross_machine();
            return;
        }

        if (g_front_cross_kind == FrontCrossNone)
        {
            judge_front_cross_state();
            return;
        }

        mode_element = 1;
        if (g_front_cross_state == FrontCrossState1)
        {
            evolve_front_cross_state();
        }
        else
        {
            exit_front_cross_state();
        }
    }

    // 重置前摄像头状态；大点集依赖 count 限定访问，不做整块清零以减少每帧开销
    static void clear_front_debug_state()
    {
        std::memset(g_front_mid_line, 0, sizeof(g_front_mid_line));
        std::memset(g_front_mid_line_valid, 0, sizeof(g_front_mid_line_valid));

        g_front_debug_seeds = {};
        g_front_left_original_point_count = 0;
        g_front_right_original_point_count = 0;
        g_front_mid_original_point_count = 0;
        g_front_left_probe_original_point_count = 0;
        g_front_right_probe_original_point_count = 0;
        g_front_left_ipm_point_count = 0;
        g_front_right_ipm_point_count = 0;
        g_front_mid_ipm_point_count = 0;
        g_front_midline_ready = 0;
        g_front_track_side_mode = FrontTrackSideNone;
        g_front_left_corner = {};
        g_front_right_corner = {};
        reset_front_split_corner_points();
        g_front_binary_threshold = 0;
        g_front_line_lost = 3;
        g_front_left_long_straight_flag = 0;
        g_front_right_long_straight_flag = 0;
    }

    static uint8 calc_adaptive_threshold(const uint8 (*gray)[image_width], int x, int y,
                                         int block_size = k_front_binary_block_size,
                                         int clip_value = k_front_binary_clip_value)
    {
        const int half = std::max(block_size / 2, 1);
        const int x0 = clamp_int(x - half, 0, image_width - 1);
        const int x1 = clamp_int(x + half, 0, image_width - 1);
        const int y0 = clamp_int(y - half, 0, image_height - 1);
        const int y1 = clamp_int(y + half, 0, image_height - 1);

        int sum = 0;
        int count = 0;
        for (int row = y0; row <= y1; ++row)
        {
            for (int col = x0; col <= x1; ++col)
            {
                sum += gray[row][col];
                ++count;
            }
        }

        if (count <= 0)
        {
            return 0;
        }

        const int threshold = (sum / count) - clip_value;
        return static_cast<uint8>(clamp_int(threshold, 0, 255));
    }

    // 对copy_image逐像素做自适应二值化，并将图像边界强制设为黑色
    static void seal_front_binary_border()
    {
        for (int row = 0; row < image_height; ++row)
        {
            g_front_binary[row][0] = k_binary_black;
            g_front_binary[row][image_width - 1] = k_binary_black;
        }
        for (int col = 0; col < image_width; ++col)
        {
            g_front_binary[0][col] = k_binary_black;
            g_front_binary[image_height - 1][col] = k_binary_black;
        }
    }

    static void filter_front_binary_image()
    {
        for (int row = image_height - 2; row >= 1; --row)
        {
            for (int col = 1; col < image_width - 1; ++col)
            {
                const uint8 white_count = static_cast<uint8>((
                                                                 g_front_binary[row - 1][col - 1] + g_front_binary[row - 1][col] + g_front_binary[row - 1][col + 1] +
                                                                 g_front_binary[row][col - 1] + g_front_binary[row][col + 1] +
                                                                 g_front_binary[row + 1][col - 1] + g_front_binary[row + 1][col] + g_front_binary[row + 1][col + 1]) /
                                                             255);

                if (g_front_binary[row][col] == k_binary_black && white_count >= 5)
                {
                    g_front_binary[row][col] = k_binary_white;
                }
                else if (g_front_binary[row][col] == k_binary_white && white_count <= 2)
                {
                    g_front_binary[row][col] = k_binary_black;
                }
            }
        }
    }

    static void build_front_binary_from_copy_image_adaptive()
    {
        for (int row = 0; row < image_height; ++row)
        {
            for (int col = 0; col < image_width; ++col)
            {
                const uint8 threshold = calc_adaptive_threshold(copy_image, col, row);
                g_front_binary[row][col] = (copy_image[row][col] > threshold) ? k_binary_white : k_binary_black;
            }
        }

        seal_front_binary_border();
    }

    static void build_front_binary_from_copy_image_clean_otsu()
    {
        g_front_binary_threshold = otsuThreshold(&copy_image[0][0], image_width, image_height);
        g_front_binary_threshold = static_cast<uint8>(clamp_int(g_front_binary_threshold,
                                                                k_front_clean_binary_min_threshold,
                                                                k_front_clean_binary_max_threshold));

        for (int row = 0; row < image_height; ++row)
        {
            for (int col = 0; col < image_width; ++col)
            {
                g_front_binary[row][col] = (copy_image[row][col] > g_front_binary_threshold) ? k_binary_white : k_binary_black;
            }
        }

        filter_front_binary_image();
        seal_front_binary_border();
    }

    static void build_front_binary_from_copy_image()
    {
        if (k_front_binary_source_mode == 0)
        {
            build_front_binary_from_copy_image_adaptive();
            return;
        }

        build_front_binary_from_copy_image_clean_otsu();
    }

    //========================================================================
    // 种子搜索：逐行扫描黑白跳变，找到左右边界种子点
    //========================================================================

    // 判断原图中(x,y)是否高于当前局部阈值，越界返回false
    static bool is_original_white_by_threshold(int x, int y, uint8 threshold)
    {
        if (x < 0 || x >= image_width || y < 0 || y >= image_height)
        {
            return false;
        }

        return copy_image[y][x] > threshold;
    }

    static bool is_seed_transition(int row, int x, bool left_side)
    {
        return g_front_binary[row][x] == k_binary_black &&
               (left_side ? (g_front_binary[row][x + 1] == k_binary_white)
                          : (g_front_binary[row][x - 1] == k_binary_white));
    }

    static bool find_seed_in_row(int row, int center_x, bool left_side, SeedPoint &seed)
    {
        const int start_x = clamp_int(center_x, 2, image_width - 3);
        const int end_x = left_side ? 2 : image_width - 3;
        const int step = left_side ? -1 : 1;

        for (int x = start_x; left_side ? (x >= end_x) : (x <= end_x); x += step)
        {
            if (is_seed_transition(row, x, left_side))
            {
                seed.x = x;
                seed.y = row;
                return true;
            }
        }

        return false;
    }

    // 从搜索行向上逐行扫描，分别寻找左右边界种子点并更新历史中心位置
    static SeedSearchResult find_seed_points()
    {
        SeedSearchResult result;
        const int search_row = clamp_int(k_front_seed_search_row, 2, image_height - 3);
        const int min_row = clamp_int(k_front_seed_min_row, 2, search_row);
        const int center_x = clamp_int(g_front_last_center_x, 2, image_width - 3);

        for (int row = search_row; row >= min_row; --row)
        {
            if (!result.has_left_seed)
            {
                result.has_left_seed = find_seed_in_row(row, center_x, true, result.left_seed);
            }

            if (!result.has_right_seed)
            {
                result.has_right_seed = find_seed_in_row(row, center_x, false, result.right_seed);
            }

            if (result.has_left_seed && result.has_right_seed)
            {
                break;
            }
        }

        if (result.has_left_seed && result.has_right_seed)
        {
            g_front_last_seed_width = std::max(result.right_seed.x - result.left_seed.x, 0);
            g_front_last_center_x = (result.left_seed.x + result.right_seed.x) / 2;
        }
        else if (result.has_left_seed && g_front_last_seed_width > 0)
        {
            g_front_last_center_x = result.left_seed.x + g_front_last_seed_width / 2;
        }
        else if (result.has_right_seed && g_front_last_seed_width > 0)
        {
            g_front_last_center_x = result.right_seed.x - g_front_last_seed_width / 2;
        }

        g_front_last_center_x = clamp_int(g_front_last_center_x, 2, image_width - 3);
        return result;
    }

    static void update_front_ipm_track_width_from_seeds(const SeedSearchResult &seed_result)
    {
        if (!seed_result.has_left_seed || !seed_result.has_right_seed)
        {
            return;
        }

        double left_ipm_x = 0.0;
        double left_ipm_y = 0.0;
        double right_ipm_x = 0.0;
        double right_ipm_y = 0.0;
        if (!Transform_Point2(seed_result.left_seed.x, seed_result.left_seed.y, left_ipm_x, left_ipm_y) ||
            !Transform_Point2(seed_result.right_seed.x, seed_result.right_seed.y, right_ipm_x, right_ipm_y))
        {
            return;
        }

        const int width = std::abs(static_cast<int>(std::lround(right_ipm_x)) -
                                   static_cast<int>(std::lround(left_ipm_x)));
        if (width >= 4 && width < image_width)
        {
            g_front_last_ipm_track_width = width;
        }
    }

    static constexpr int k_dir_front[4][2] = {
        {0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    static constexpr int k_dir_frontleft[4][2] = {
        {-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    static constexpr int k_dir_frontright[4][2] = {
        {1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    //========================================================================
    // 爬边：从种子沿白-黑边界八邻域追踪，产出左右原始边线点集
    //========================================================================

    // 从左种子点出发，沿黑-白边界向上追踪左边界点集
    static uint16 follow_left_boundary(const SeedSearchResult &seed_result, uint16 (*points)[2])
    {
        if (!seed_result.has_left_seed) // 没有对应种子就不爬，该侧直接返回 0 点
        {
            return 0;
        }

        int x = seed_result.left_seed.x;
        int y = seed_result.left_seed.y;
        int dir = 0;
        int turn = 0;
        int overlap_streak = 0;
        uint16 count = 0;
        uint8 threshold = 0;

        while (count < k_front_migration_max_points &&
               x >= 2 && x < image_width - 2 &&
               y > 2 && y < image_height - 2 &&
               turn < 4) // 爬点数达到缓存上限就
        {
            if (seed_result.has_right_seed &&
                std::abs(x - seed_result.right_seed.x) <= 1 &&
                std::abs(y - seed_result.right_seed.y) <= 1)
            {
                ++overlap_streak;
                if (overlap_streak >= k_front_overlap_stop_count)
                {
                    break;
                }
            }
            else
            {
                overlap_streak = 0;
            }

            if (turn == 0)
            {
                threshold = calc_adaptive_threshold(copy_image, x, y);
            }

            const bool front_white = is_original_white_by_threshold(x + k_dir_front[dir][0],
                                                                    y + k_dir_front[dir][1],
                                                                    threshold);
            const bool frontleft_white = is_original_white_by_threshold(x + k_dir_frontleft[dir][0],
                                                                        y + k_dir_frontleft[dir][1],
                                                                        threshold);

            if (!front_white)
            {
                dir = (dir + 1) & 3;
                ++turn;
                continue;
            }

            if (!frontleft_white)
            {
                x += k_dir_front[dir][0];
                y += k_dir_front[dir][1];
            }
            else // 前方和左前都白，就走左前，同时修正朝向
            {
                x += k_dir_frontleft[dir][0];
                y += k_dir_frontleft[dir][1];
                dir = (dir + 3) & 3;
            }

            points[count][0] = static_cast<uint16>(x);
            points[count][1] = static_cast<uint16>(y);
            ++count;
            turn = 0;

            if (y <= k_front_top_stop_row)
            {
                break;
            }
        }

        return count;
    }

    // 从右种子点出发，沿白-黑边界向上追踪右边界点集
    static uint16 follow_right_boundary(const SeedSearchResult &seed_result, uint16 (*points)[2])
    {
        if (!seed_result.has_right_seed)
        {
            return 0;
        }

        int x = seed_result.right_seed.x;
        int y = seed_result.right_seed.y;
        int dir = 0;
        int turn = 0;
        int overlap_streak = 0;
        uint16 count = 0;
        uint8 threshold = 0;

        while (count < k_front_migration_max_points &&
               x > 2 && x < image_width - 2 &&
               y > 2 && y < image_height - 2 &&
               turn < 4)
        {
            if (seed_result.has_left_seed &&
                std::abs(x - seed_result.left_seed.x) <= 1 &&
                std::abs(y - seed_result.left_seed.y) <= 1)
            {
                ++overlap_streak;
                if (overlap_streak >= k_front_overlap_stop_count)
                {
                    break;
                }
            }
            else
            {
                overlap_streak = 0;
            }

            if (turn == 0)
            {
                threshold = calc_adaptive_threshold(copy_image, x, y);
            }

            const bool front_white = is_original_white_by_threshold(x + k_dir_front[dir][0],
                                                                    y + k_dir_front[dir][1],
                                                                    threshold);
            const bool frontright_white = is_original_white_by_threshold(x + k_dir_frontright[dir][0],
                                                                         y + k_dir_frontright[dir][1],
                                                                         threshold);

            if (!front_white)
            {
                dir = (dir + 3) & 3;
                ++turn;
                continue;
            }

            if (!frontright_white)
            {
                x += k_dir_front[dir][0];
                y += k_dir_front[dir][1];
            }
            else
            {
                x += k_dir_frontright[dir][0];
                y += k_dir_frontright[dir][1];
                dir = (dir + 1) & 3;
            }

            points[count][0] = static_cast<uint16>(x);
            points[count][1] = static_cast<uint16>(y);
            ++count;
            turn = 0;

            if (y <= k_front_top_stop_row)
            {
                break;
            }
        }

        return count;
    }

    static bool find_seed_in_row_window(int row, int center_x, int radius, bool left_side, SeedPoint &seed)
    {
        const int left = clamp_int(center_x - radius, 2, image_width - 3);
        const int right = clamp_int(center_x + radius, 2, image_width - 3);
        int best_x = -1;
        int best_error = 1 << 30;
        const int start_x = left_side ? right : left;
        const int end_x = left_side ? left : right;
        const int step = left_side ? -1 : 1;

        for (int x = start_x; left_side ? (x >= end_x) : (x <= end_x); x += step)
        {
            if (is_seed_transition(row, x, left_side))
            {
                const int error = std::abs(x - center_x);
                if (error < best_error)
                {
                    best_error = error;
                    best_x = x;
                    if (error == 0)
                    {
                        break;
                    }
                }
            }
        }

        if (best_x < 0)
        {
            return false;
        }

        seed.x = best_x;
        seed.y = row;
        return true;
    }

    static bool get_front_probe_anchor_original(bool left_side, int &x, int &y)
    {
        if (left_side)
        {
            if (g_front_left_upper_corner.valid)
            {
                x = g_front_left_upper_corner.x;
                y = g_front_left_upper_corner.y;
                return true;
            }
            if (g_front_left_corner.valid)
            {
                x = g_front_left_corner.x;
                y = g_front_left_corner.y;
                return true;
            }
            if (g_front_left_original_point_count > 0)
            {
                x = g_front_left_original_points[g_front_left_original_point_count - 1][0];
                y = g_front_left_original_points[g_front_left_original_point_count - 1][1];
                return true;
            }
            return false;
        }

        if (g_front_right_upper_corner.valid)
        {
            x = g_front_right_upper_corner.x;
            y = g_front_right_upper_corner.y;
            return true;
        }
        if (g_front_right_corner.valid)
        {
            x = g_front_right_corner.x;
            y = g_front_right_corner.y;
            return true;
        }
        if (g_front_right_original_point_count > 0)
        {
            x = g_front_right_original_points[g_front_right_original_point_count - 1][0];
            y = g_front_right_original_points[g_front_right_original_point_count - 1][1];
            return true;
        }

        return false;
    }

    static bool try_find_front_probe_seed_at_column(bool left_side, int row, int column_x, SeedPoint &seed)
    {
        if (row < 2 || row >= image_height - 2 ||
            column_x < 2 || column_x >= image_width - 2)
        {
            return false;
        }

        if (is_seed_transition(row, column_x, left_side))
        {
            seed.x = column_x;
            seed.y = row;
            return true;
        }

        return false;
    }

    static bool find_front_probe_seed(bool left_side, int anchor_x, int anchor_y, SeedPoint &seed)
    {
        const int search_center = clamp_int(anchor_x, 2, image_width - 3);
        const int start_row = clamp_int(anchor_y - k_front_probe_start_row_backoff, 2, image_height - 3);
        const int min_row = clamp_int(anchor_y - k_front_probe_search_rows_up, 2, start_row);

        for (int row = start_row; row >= min_row; --row)
        {
            if (try_find_front_probe_seed_at_column(left_side, row, search_center, seed))
            {
                return true;
            }

            if (find_seed_in_row_window(row, search_center, k_front_probe_search_radius, left_side, seed))
            {
                return true;
            }
        }

        return false;
    }

    static uint16 follow_boundary_from_single_seed(bool left_side, const SeedPoint &seed, uint16 (*points)[2])
    {
        SeedSearchResult seed_result;
        if (left_side)
        {
            seed_result.has_left_seed = true;
            seed_result.left_seed = seed;
            return follow_left_boundary(seed_result, points);
        }

        seed_result.has_right_seed = true;
        seed_result.right_seed = seed;
        return follow_right_boundary(seed_result, points);
    }

    static void append_original_point_unique(uint16 (*points)[2], uint16 &count, int x, int y)
    {
        if (count >= k_front_migration_max_points)
        {
            return;
        }

        const int clamped_x = clamp_int(x, 0, image_width - 1);
        const int clamped_y = clamp_int(y, 0, image_height - 1);
        if (count > 0 &&
            static_cast<int>(points[count - 1][0]) == clamped_x &&
            static_cast<int>(points[count - 1][1]) == clamped_y)
        {
            return;
        }

        points[count][0] = static_cast<uint16>(clamped_x);
        points[count][1] = static_cast<uint16>(clamped_y);
        ++count;
    }

    static void append_interpolated_original_segment(uint16 (*points)[2], uint16 &count,
                                                     int x0, int y0, int x1, int y1)
    {
        const int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
        if (steps <= 1)
        {
            return;
        }

        for (int step = 1; step < steps && count < k_front_migration_max_points; ++step)
        {
            const float t = static_cast<float>(step) / static_cast<float>(steps);
            const int x = static_cast<int>(std::lround(x0 + (x1 - x0) * t));
            const int y = static_cast<int>(std::lround(y0 + (y1 - y0) * t));
            append_original_point_unique(points, count, x, y);
        }
    }

    static bool splice_front_probe_points_into_original(bool left_side,
                                                        const FrontProbeConnectionContext &context,
                                                        uint16 (*original_points)[2], uint16 &original_count,
                                                        const uint16 (*probe_points)[2], uint16 probe_count)
    {
        if (!context.valid ||
            probe_count == 0 ||
            context.original_index < 0 ||
            context.original_index >= original_count)
        {
            return false;
        }

        const uint16 probe_corner_index = seek_front_probe_corner_index(probe_points, probe_count, left_side);
        if (probe_corner_index >= probe_count)
        {
            return false;
        }

        if (static_cast<int>(probe_points[probe_corner_index][1]) >= context.original_y)
        {
            return false;
        }

        int probe_corner_ipm_x = 0;
        int probe_corner_ipm_y = 0;
        if (!map_front_original_to_ipm_point(probe_points[probe_corner_index][0],
                                             probe_points[probe_corner_index][1],
                                             probe_corner_ipm_x, probe_corner_ipm_y))
        {
            return false;
        }

        const int runtime_width = get_runtime_track_half_width() * 2;
        const int distance = calc_front_ipm_distance(context.ipm_x, context.ipm_y,
                                                     probe_corner_ipm_x, probe_corner_ipm_y);
        if (distance < runtime_width - k_front_cross_single_width_tolerance ||
            distance > runtime_width + k_front_cross_single_width_tolerance)
        {
            return false;
        }

        uint16 merged_points[k_front_migration_max_points][2] = {{0}};
        uint16 merged_count = 0;
        for (int i = 0; i <= context.original_index && merged_count < k_front_migration_max_points; ++i)
        {
            append_original_point_unique(merged_points, merged_count,
                                         original_points[i][0], original_points[i][1]);
        }

        if (merged_count == 0)
        {
            return false;
        }

        append_interpolated_original_segment(merged_points, merged_count,
                                             merged_points[merged_count - 1][0], merged_points[merged_count - 1][1],
                                             probe_points[probe_corner_index][0], probe_points[probe_corner_index][1]);

        const uint16 base_count = merged_count;
        for (uint16 i = probe_corner_index; i < probe_count && merged_count < k_front_migration_max_points; ++i)
        {
            append_original_point_unique(merged_points, merged_count,
                                         probe_points[i][0], probe_points[i][1]);
        }

        if (merged_count <= base_count)
        {
            return false;
        }

        std::memcpy(original_points, merged_points, sizeof(uint16) * 2 * merged_count);
        original_count = merged_count;
        return true;
    }

    static bool merge_front_probe_points_into_original(uint16 (*original_points)[2], uint16 &original_count,
                                                       const uint16 (*probe_points)[2], uint16 probe_count)
    {
        if (probe_count == 0)
        {
            return false;
        }

        const uint16 before_count = original_count;
        uint16 start_index = 0;

        if (original_count > 0)
        {
            const int last_x = original_points[original_count - 1][0];
            const int last_y = original_points[original_count - 1][1];
            while (start_index < probe_count &&
                   static_cast<int>(probe_points[start_index][1]) >= last_y)
            {
                ++start_index;
            }

            if (start_index >= probe_count)
            {
                return false;
            }

            append_interpolated_original_segment(original_points, original_count,
                                                 last_x, last_y,
                                                 probe_points[start_index][0], probe_points[start_index][1]);
        }

        for (uint16 i = start_index; i < probe_count && original_count < k_front_migration_max_points; ++i)
        {
            append_original_point_unique(original_points, original_count,
                                         probe_points[i][0], probe_points[i][1]);
        }

        return original_count > before_count;
    }

    // 将源点集逆透视变换到IPM世界坐标系，连续4点变换失败则终止
    static uint16 project_points_to_ipm(const uint16 (*src)[2], uint16 src_count, uint16 (*dst)[2])
    {
        uint16 dst_count = 0;
        int invalid_streak = 0;

        for (uint16 i = 0; i < src_count && dst_count < k_front_migration_max_points; ++i)
        {
            double mapped_x = 0.0;
            double mapped_y = 0.0;
            if (!Transform_Point2(static_cast<int>(src[i][0]), static_cast<int>(src[i][1]), mapped_x, mapped_y))
            {
                if (dst_count > 0 && ++invalid_streak >= 4)
                {
                    break;
                }
                continue;
            }

            const int x = static_cast<int>(std::lround(mapped_x));
            const int y = static_cast<int>(std::lround(mapped_y));
            if (x < 0 || x >= image_width || y < 0 || y >= image_height)
            {
                if (dst_count > 0 && ++invalid_streak >= 4)
                {
                    break;
                }
                continue;
            }

            invalid_streak = 0;
            if (dst_count > 0 &&
                static_cast<int>(dst[dst_count - 1][0]) == x &&
                static_cast<int>(dst[dst_count - 1][1]) == y)
            {
                continue;
            }

            dst[dst_count][0] = static_cast<uint16>(x);
            dst[dst_count][1] = static_cast<uint16>(y);
            ++dst_count;
        }

        return dst_count;
    }

    static uint16 project_points_to_original(const uint16 (*src)[2], uint16 src_count, uint16 (*dst)[2])
    {
        uint16 dst_count = 0;

        for (uint16 i = 0; i < src_count && dst_count < k_front_migration_max_points; ++i)
        {
            double mapped_x = 0.0;
            double mapped_y = 0.0;
            if (!Transform_Point1(static_cast<int>(src[i][0]), static_cast<int>(src[i][1]), mapped_x, mapped_y))
            {
                continue;
            }

            const int x = static_cast<int>(std::lround(mapped_x));
            const int y = static_cast<int>(std::lround(mapped_y));
            if (x < 0 || x >= image_width || y < 0 || y >= image_height)
            {
                continue;
            }

            if (dst_count > 0 &&
                static_cast<int>(dst[dst_count - 1][0]) == x &&
                static_cast<int>(dst[dst_count - 1][1]) == y)
            {
                continue;
            }

            dst[dst_count][0] = static_cast<uint16>(x);
            dst[dst_count][1] = static_cast<uint16>(y);
            ++dst_count;
        }

        return dst_count;
    }

    static int clip_index(int index, int min_value, int max_value)
    {
        return clamp_int(index, min_value, max_value);
    }

    static float local_angle_points_u16(const uint16 (*points)[2], uint16 count, uint16 index, uint16 dist)
    {
        if (count == 0)
        {
            return 0.0f;
        }

        const int prev_index = clip_index(static_cast<int>(index) - static_cast<int>(dist), 0, count - 1);
        const int next_index = clip_index(static_cast<int>(index) + static_cast<int>(dist), 0, count - 1);

        float dx1 = static_cast<float>(points[index][0]) - static_cast<float>(points[prev_index][0]);
        float dy1 = static_cast<float>(points[index][1]) - static_cast<float>(points[prev_index][1]);
        const float dn1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        float dx2 = static_cast<float>(points[next_index][0]) - static_cast<float>(points[index][0]);
        float dy2 = static_cast<float>(points[next_index][1]) - static_cast<float>(points[index][1]);
        const float dn2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

        if (dn1 <= 1e-4f || dn2 <= 1e-4f)
        {
            return 0.0f;
        }

        dx1 /= dn1;
        dy1 /= dn1;
        dx2 /= dn2;
        dy2 /= dn2;
        return std::atan2(dx1 * dy2 - dx2 * dy1, dx2 * dx1 + dy2 * dy1);
    }

    static int count_corner_white_neighbors(int x, int y)
    {
        if (x <= 0 || x >= image_width - 1 || y <= 0 || y >= image_height - 1)
        {
            return 0;
        }

        const uint8 threshold = calc_adaptive_threshold(copy_image, x, y, 3, 3);
        int white_count = 0;
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                if (copy_image[y + dy][x + dx] > threshold)
                {
                    ++white_count;
                }
            }
        }

        return white_count;
    }

    // Front_Car filter_point 的最小移植：从原图边界点中挑出 3 个角点候选
    static void select_front_corner_candidates(const uint16 (*points)[2], uint16 count,
                                               bool is_left_side, uint16 out_ids[3])
    {
        if (count == 0)
        {
            out_ids[0] = out_ids[1] = out_ids[2] = 0;
            return;
        }

        uint16 best_ids[3] = {0, 0, 0};
        uint32 best_values[3] = {0, 0, 0};
        for (uint16 i = 0; i < count; ++i)
        {
            const uint16 cur_x = is_left_side ? points[i][0] : static_cast<uint16>(image_width - points[i][0]);
            const uint16 cur_y = static_cast<uint16>(image_height - points[i][1]);
            const uint32 xy = static_cast<uint32>(cur_x) * static_cast<uint32>(cur_y);
            const uint32 candidate_values[3] = {
                cur_x,
                cur_y,
                xy,
            };

            for (int k = 0; k < 3; ++k)
            {
                if (candidate_values[k] > best_values[k])
                {
                    best_values[k] = candidate_values[k];
                    best_ids[k] = i;
                }
            }
        }

        out_ids[0] = best_ids[0];
        out_ids[1] = best_ids[1];
        out_ids[2] = best_ids[2];
    }

    static int find_nearest_projected_index(const uint16 (*points)[2], uint16 count, int x, int y)
    {
        if (count == 0)
        {
            return -1;
        }

        int best_index = 0;
        int best_distance = 1 << 30;
        for (uint16 i = 0; i < count; ++i)
        {
            const int dx = std::abs(static_cast<int>(points[i][0]) - x);
            const int dy = std::abs(static_cast<int>(points[i][1]) - y);
            const int distance = dx + dy;
            if (distance < best_distance)
            {
                best_distance = distance;
                best_index = i;
                if (distance <= 1)
                {
                    break;
                }
            }
        }

        return best_index;
    }

    static int find_nearest_original_index(const uint16 (*points)[2], uint16 count, int x, int y)
    {
        if (count == 0)
        {
            return -1;
        }

        int best_index = 0;
        int best_distance = 1 << 30;
        for (uint16 i = 0; i < count; ++i)
        {
            const int dx = std::abs(static_cast<int>(points[i][0]) - x);
            const int dy = std::abs(static_cast<int>(points[i][1]) - y);
            const int distance = dx + dy;
            if (distance < best_distance)
            {
                best_distance = distance;
                best_index = i;
                if (distance <= 1)
                {
                    break;
                }
            }
        }

        return best_index;
    }

    static bool map_front_original_to_ipm_point(int original_x, int original_y, int &ipm_x, int &ipm_y)
    {
        double mapped_x = 0.0;
        double mapped_y = 0.0;
        if (!Transform_Point2(original_x, original_y, mapped_x, mapped_y))
        {
            return false;
        }

        ipm_x = static_cast<int>(std::lround(mapped_x));
        ipm_y = static_cast<int>(std::lround(mapped_y));
        return ipm_x >= 0 && ipm_x < image_width && ipm_y >= 0 && ipm_y < image_height;
    }

    static bool map_front_ipm_to_original_point(int ipm_x, int ipm_y, int &original_x, int &original_y)
    {
        double mapped_x = 0.0;
        double mapped_y = 0.0;
        if (!Transform_Point1(ipm_x, ipm_y, mapped_x, mapped_y))
        {
            return false;
        }

        original_x = static_cast<int>(std::lround(mapped_x));
        original_y = static_cast<int>(std::lround(mapped_y));
        return original_x >= 0 && original_x < image_width && original_y >= 0 && original_y < image_height;
    }

    static bool build_front_probe_connection_context_from_original_index(bool left_side, int original_index,
                                                                         FrontProbeConnectionContext &context)
    {
        context = {};

        const uint16(*original_points)[2] = left_side ? g_front_left_original_points : g_front_right_original_points;
        const uint16(*ipm_points)[2] = left_side ? g_front_left_ipm_points : g_front_right_ipm_points;
        const uint16 original_count = left_side ? g_front_left_original_point_count : g_front_right_original_point_count;
        const uint16 ipm_count = left_side ? g_front_left_ipm_point_count : g_front_right_ipm_point_count;

        if (original_index < 0 ||
            original_index >= original_count ||
            ipm_count == 0)
        {
            return false;
        }

        int mapped_ipm_x = 0;
        int mapped_ipm_y = 0;
        if (!map_front_original_to_ipm_point(original_points[original_index][0],
                                             original_points[original_index][1],
                                             mapped_ipm_x, mapped_ipm_y))
        {
            return false;
        }

        const int ipm_index = find_nearest_projected_index(ipm_points, ipm_count, mapped_ipm_x, mapped_ipm_y);
        if (ipm_index < 0)
        {
            return false;
        }

        context.valid = true;
        context.original_x = original_points[original_index][0];
        context.original_y = original_points[original_index][1];
        context.original_index = original_index;
        context.ipm_x = ipm_points[ipm_index][0];
        context.ipm_y = ipm_points[ipm_index][1];
        context.ipm_index = ipm_index;
        return true;
    }

    static bool build_front_probe_connection_context_from_original_point(bool left_side, int original_x, int original_y,
                                                                         FrontProbeConnectionContext &context)
    {
        const uint16(*original_points)[2] = left_side ? g_front_left_original_points : g_front_right_original_points;
        const uint16 original_count = left_side ? g_front_left_original_point_count : g_front_right_original_point_count;

        const int original_index = find_nearest_original_index(original_points, original_count, original_x, original_y);
        if (original_index < 0)
        {
            return false;
        }

        return build_front_probe_connection_context_from_original_index(left_side, original_index, context);
    }

    static bool get_front_probe_connection_context(bool left_side, FrontProbeConnectionContext &context)
    {
        context = {};

        const uint16(*original_points)[2] = left_side ? g_front_left_original_points : g_front_right_original_points;
        const uint16 original_count = left_side ? g_front_left_original_point_count : g_front_right_original_point_count;
        if (original_count > 0)
        {
            uint16 candidate_ids[3] = {0, 0, 0};
            select_front_corner_candidates(original_points, original_count, left_side, candidate_ids);
            if (build_front_probe_connection_context_from_original_index(left_side, candidate_ids[2], context))
            {
                return true;
            }
        }

        const FrontCornerPoint &upper_corner = left_side ? g_front_left_upper_corner : g_front_right_upper_corner;
        if (upper_corner.valid &&
            build_front_probe_connection_context_from_original_point(left_side, upper_corner.x, upper_corner.y, context))
        {
            return true;
        }

        const FrontCornerPoint &corner = left_side ? g_front_left_corner : g_front_right_corner;
        if (corner.valid &&
            build_front_probe_connection_context_from_original_point(left_side, corner.x, corner.y, context))
        {
            return true;
        }

        if (original_count > 0)
        {
            return build_front_probe_connection_context_from_original_index(left_side, original_count - 1, context);
        }

        return false;
    }

    static bool track_front_probe_reference_point(bool left_side, const FrontProbeConnectionContext &context,
                                                  int &probe_ipm_x, int &probe_ipm_y)
    {
        if (!context.valid)
        {
            return false;
        }

        const uint16(*ipm_points)[2] = left_side ? g_front_left_ipm_points : g_front_right_ipm_points;
        const uint16 ipm_count = left_side ? g_front_left_ipm_point_count : g_front_right_ipm_point_count;
        if (ipm_count == 0)
        {
            return false;
        }

        const int track_index = clip_index(context.ipm_index + k_front_probe_track_step, 0, ipm_count - 1);
        const int prev_index = clip_index(track_index - k_front_probe_track_step, 0, ipm_count - 1);
        const int next_index = clip_index(track_index + k_front_probe_track_step, 0, ipm_count - 1);

        float dx = static_cast<float>(ipm_points[next_index][0]) - static_cast<float>(ipm_points[prev_index][0]);
        float dy = static_cast<float>(ipm_points[next_index][1]) - static_cast<float>(ipm_points[prev_index][1]);
        const float norm = std::sqrt(dx * dx + dy * dy);
        if (norm <= 1e-4f)
        {
            return false;
        }

        dx /= norm;
        dy /= norm;

        const int point_x = static_cast<int>(ipm_points[track_index][0]);
        const int point_y = static_cast<int>(ipm_points[track_index][1]);
        if (left_side)
        {
            probe_ipm_x = clamp_int(static_cast<int>(std::lround(point_x - dy * k_front_probe_track_dist)), 0, image_width - 1);
            probe_ipm_y = clamp_int(static_cast<int>(std::lround(point_y + dx * k_front_probe_track_dist)), 0, image_height - 1);
        }
        else
        {
            probe_ipm_x = clamp_int(static_cast<int>(std::lround(point_x + dy * k_front_probe_track_dist)), 0, image_width - 1);
            probe_ipm_y = clamp_int(static_cast<int>(std::lround(point_y - dx * k_front_probe_track_dist)), 0, image_height - 1);
        }

        return true;
    }

    static uint16 seek_front_probe_corner_index(const uint16 (*points)[2], uint16 count, bool left_side)
    {
        if (count == 0)
        {
            return 0;
        }

        const int min_y = static_cast<int>(points[count - 1][1]);
        int best_value = -1;
        uint16 best_index = count - 1;
        for (uint16 i = 0; i < count; ++i)
        {
            const int cur_x = static_cast<int>(points[i][0]);
            const int cur_y = static_cast<int>(points[i][1]);
            const int value = left_side ? std::abs((cur_y - min_y) * cur_x)
                                        : std::abs((cur_y - min_y) * (image_width - cur_x));
            if (value > best_value)
            {
                best_value = value;
                best_index = i;
            }
        }

        return best_index;
    }

    //========================================================================
    // 拐点检测：区域判定、分裂拐点存储与扫描、通用角点评分与确认
    //========================================================================

    static bool is_front_corner_point_in_target_region(int x, int y, bool want_left_region, bool want_upper_region)
    {
        if (x < 0 || x >= image_width || y < 0 || y >= image_height)
        {
            return false;
        }

        if (want_upper_region)
        {
            if (y > k_front_corner_region_upper_max_row)
            {
                return false;
            }
        }
        else if (y < k_front_corner_region_lower_min_row)
        {
            return false;
        }

        const int row_left = valid_l_bound[y];
        const int row_right = valid_r_bound[y];
        if (row_left > row_right)
        {
            return false;
        }

        const int row_span = row_right - row_left;
        if (row_span < 6)
        {
            return false;
        }

        const int left_region_max = row_left + (row_span * 2) / 3;
        const int right_region_min = row_left + row_span / 3;

        if (want_left_region)
        {
            return x >= row_left && x <= left_region_max;
        }

        return x >= right_region_min && x <= row_right;
    }

    static void store_front_split_corner(FrontCornerPoint &front_corner,
                                         Track_Corner_Point_TypeDef &legacy_corner,
                                         const uint16 (*ipm_points)[2], uint16 ipm_count,
                                         uint16 mid_index)
    {
        front_corner = {};
        if (mid_index >= ipm_count)
        {
            clear_track_corner_point(legacy_corner);
            return;
        }

        const int ipm_x = static_cast<int>(ipm_points[mid_index][0]);
        const int ipm_y = static_cast<int>(ipm_points[mid_index][1]);
        store_track_corner_point(legacy_corner, ipm_x, ipm_y);

        double mapped_x = 0.0;
        double mapped_y = 0.0;
        if (!Transform_Point1(ipm_x, ipm_y, mapped_x, mapped_y))
        {
            return;
        }

        const int original_x = static_cast<int>(std::lround(mapped_x));
        const int original_y = static_cast<int>(std::lround(mapped_y));
        if (original_x < 0 || original_x >= image_width || original_y < 0 || original_y >= image_height)
        {
            return;
        }

        front_corner.valid = true;
        front_corner.x = static_cast<uint16>(original_x);
        front_corner.y = static_cast<uint16>(original_y);
        front_corner.index = mid_index;
        front_corner.angle_deg = std::fabs(local_angle_points_u16(ipm_points, ipm_count, mid_index,
                                                                  k_front_corner_mid_step)) *
                                 180.0f / k_front_pi;
    }

    static bool try_store_front_left_lower_corner_from_index(uint16 mid_index)
    {
        if (mid_index < k_front_corner_mid_step ||
            mid_index + k_front_corner_mid_step >= g_front_left_ipm_point_count)
        {
            return false;
        }

        const int ax = g_front_left_ipm_points[mid_index - k_front_corner_mid_step][0];
        const int ay = g_front_left_ipm_points[mid_index - k_front_corner_mid_step][1];
        const int bx = g_front_left_ipm_points[mid_index][0];
        const int by = g_front_left_ipm_points[mid_index][1];
        const int cx = g_front_left_ipm_points[mid_index + k_front_corner_mid_step][0];
        const int cy = g_front_left_ipm_points[mid_index + k_front_corner_mid_step][1];

        if (cy <= k_front_corner_top_guard_row ||
            !is_front_corner_point_in_target_region(bx, by, true, false))
        {
            return false;
        }

        const int corner_dot = (ax - bx) * (cx - bx) + (ay - by) * (cy - by);
        const int corner_balance_x = (cx - bx) + (ax - bx);
        const int corner_balance_y = (cy - by) + (ay - by);

        if (corner_dot >= 0 &&
            bx > cx &&
            by < ay &&
            corner_balance_x <= 0 &&
            corner_balance_y >= 0)
        {
            store_front_split_corner(g_front_left_lower_corner, g_left_lower_corner,
                                     g_front_left_ipm_points, g_front_left_ipm_point_count, mid_index);
            return true;
        }

        return false;
    }

    static bool try_store_front_right_lower_corner_from_index(uint16 mid_index)
    {
        if (mid_index < k_front_corner_mid_step ||
            mid_index + k_front_corner_mid_step >= g_front_right_ipm_point_count)
        {
            return false;
        }

        const int ax = g_front_right_ipm_points[mid_index - k_front_corner_mid_step][0];
        const int ay = g_front_right_ipm_points[mid_index - k_front_corner_mid_step][1];
        const int bx = g_front_right_ipm_points[mid_index][0];
        const int by = g_front_right_ipm_points[mid_index][1];
        const int cx = g_front_right_ipm_points[mid_index + k_front_corner_mid_step][0];
        const int cy = g_front_right_ipm_points[mid_index + k_front_corner_mid_step][1];

        if (cy <= k_front_corner_top_guard_row ||
            !is_front_corner_point_in_target_region(bx, by, false, false))
        {
            return false;
        }

        const int corner_dot = (ax - bx) * (cx - bx) + (ay - by) * (cy - by);
        const int corner_balance_x = (cx - bx) + (ax - bx);
        const int corner_balance_y = (cy - by) + (ay - by);

        if (corner_dot >= 0 &&
            cx > bx &&
            by < ay &&
            corner_balance_x >= 0 &&
            corner_balance_y >= 0)
        {
            store_front_split_corner(g_front_right_lower_corner, g_right_lower_corner,
                                     g_front_right_ipm_points, g_front_right_ipm_point_count, mid_index);
            return true;
        }

        return false;
    }

    static bool try_store_front_left_upper_corner_from_index(uint16 mid_index)
    {
        if (mid_index < k_front_corner_mid_step ||
            mid_index + k_front_corner_mid_step >= g_front_left_ipm_point_count)
        {
            return false;
        }

        const int ax = g_front_left_ipm_points[mid_index - k_front_corner_mid_step][0];
        const int ay = g_front_left_ipm_points[mid_index - k_front_corner_mid_step][1];
        const int bx = g_front_left_ipm_points[mid_index][0];
        const int by = g_front_left_ipm_points[mid_index][1];
        const int cx = g_front_left_ipm_points[mid_index + k_front_corner_mid_step][0];
        const int cy = g_front_left_ipm_points[mid_index + k_front_corner_mid_step][1];

        if (cy <= k_front_corner_top_guard_row ||
            !is_front_corner_point_in_target_region(bx, by, true, true))
        {
            return false;
        }

        const int corner_dot = (bx - ax) * (cx - bx) + (by - ay) * (cy - by);
        const int corner_trend_x = cx - ax;
        const int corner_trend_y = cy - ay;

        if (corner_dot >= 0 &&
            bx > ax &&
            by - cy > k_front_corner_vertical_delta_min &&
            ay - by < k_front_corner_near_flat_delta_max &&
            corner_trend_x >= 0 &&
            corner_trend_y <= 0)
        {
            store_front_split_corner(g_front_left_upper_corner, g_left_upper_corner,
                                     g_front_left_ipm_points, g_front_left_ipm_point_count, mid_index);
            return true;
        }

        return false;
    }

    static bool try_store_front_right_upper_corner_from_index(uint16 mid_index)
    {
        if (mid_index < k_front_corner_mid_step ||
            mid_index + k_front_corner_mid_step >= g_front_right_ipm_point_count)
        {
            return false;
        }

        const int ax = g_front_right_ipm_points[mid_index - k_front_corner_mid_step][0];
        const int ay = g_front_right_ipm_points[mid_index - k_front_corner_mid_step][1];
        const int bx = g_front_right_ipm_points[mid_index][0];
        const int by = g_front_right_ipm_points[mid_index][1];
        const int cx = g_front_right_ipm_points[mid_index + k_front_corner_mid_step][0];
        const int cy = g_front_right_ipm_points[mid_index + k_front_corner_mid_step][1];

        if (cy <= k_front_corner_top_guard_row ||
            !is_front_corner_point_in_target_region(bx, by, false, true))
        {
            return false;
        }

        const int corner_dot = (bx - ax) * (cx - bx) + (by - ay) * (cy - by);
        const int corner_trend_x = cx - ax;
        const int corner_trend_y = cy - ay;

        if (corner_dot >= 0 &&
            ax > bx &&
            by - cy > k_front_corner_vertical_delta_min &&
            ay - by < k_front_corner_near_flat_delta_max &&
            corner_trend_x <= 0 &&
            corner_trend_y <= 0)
        {
            store_front_split_corner(g_front_right_upper_corner, g_right_upper_corner,
                                     g_front_right_ipm_points, g_front_right_ipm_point_count, mid_index);
            return true;
        }

        return false;
    }

    // 从 IPM 边线中扫描下拐点，left_side 决定扫描左侧还是右侧
    static void detect_front_lower_corner_point(bool left_side)
    {
        const uint16 count = left_side ? g_front_left_ipm_point_count : g_front_right_ipm_point_count;
        if (count <= k_front_corner_end_step)
        {
            return;
        }

        for (uint16 i = 0; i + k_front_corner_end_step < count; ++i)
        {
            const bool ok = left_side
                                ? try_store_front_left_lower_corner_from_index(i + k_front_corner_mid_step)
                                : try_store_front_right_lower_corner_from_index(i + k_front_corner_mid_step);
            if (ok)
            {
                return;
            }
        }
    }

    // 从 IPM 边线中扫描上拐点，起始偏移由 k_front_corner_upper_scan_start 控制
    static void detect_front_upper_corner_point(bool left_side)
    {
        const uint16 count = left_side ? g_front_left_ipm_point_count : g_front_right_ipm_point_count;
        if (count <= k_front_corner_end_step)
        {
            return;
        }

        for (uint16 i = k_front_corner_upper_scan_start;
             i + k_front_corner_end_step < count; ++i)
        {
            const bool ok = left_side
                                ? try_store_front_left_upper_corner_from_index(i + k_front_corner_mid_step)
                                : try_store_front_right_upper_corner_from_index(i + k_front_corner_mid_step);
            if (ok)
            {
                return;
            }
        }
    }

    // 检测左右边界上的上下拐点，结果写入四个对外分裂拐点和内部 legacy 拐点
    static void detect_front_track_corner_points()
    {
        reset_front_split_corner_points();
        detect_front_lower_corner_point(true);
        detect_front_lower_corner_point(false);
        detect_front_upper_corner_point(true);
        detect_front_upper_corner_point(false);
    }

    static bool evaluate_front_corner(const uint16 (*original_points)[2], uint16 original_count,
                                      const uint16 (*ipm_points)[2], uint16 ipm_count,
                                      bool is_left_side, FrontCornerPoint &corner)
    {
        corner = {};
        if (is_left_side ? g_front_left_long_straight_flag : g_front_right_long_straight_flag)
        {
            return false;
        }

        if (original_count < k_front_min_corner_points || ipm_count <= k_front_corner_large_window * 2)
        {
            return false;
        }

        uint16 candidate_ids[3] = {0, 0, 0};
        select_front_corner_candidates(original_points, original_count, is_left_side, candidate_ids);

        struct CandidateMapping
        {
            bool valid = false;
            uint16 original_index = 0;
            uint16 ipm_index = 0;
            uint16 original_x = 0;
            uint16 original_y = 0;
        };

        CandidateMapping candidates[3] = {};
        for (int i = 0; i < 3; ++i)
        {
            const uint16 original_index = candidate_ids[i];
            if (original_index >= original_count)
            {
                continue;
            }

            double mapped_x = 0.0;
            double mapped_y = 0.0;
            if (!Transform_Point2(static_cast<int>(original_points[original_index][0]),
                                  static_cast<int>(original_points[original_index][1]),
                                  mapped_x, mapped_y))
            {
                continue;
            }

            const int ipm_x = static_cast<int>(std::lround(mapped_x));
            const int ipm_y = static_cast<int>(std::lround(mapped_y));
            if (ipm_x < 0 || ipm_x >= image_width || ipm_y < 0 || ipm_y >= image_height)
            {
                continue;
            }

            const int nearest_index = find_nearest_projected_index(ipm_points, ipm_count, ipm_x, ipm_y);
            if (nearest_index < 0)
            {
                continue;
            }

            candidates[i].valid = true;
            candidates[i].original_index = original_index;
            candidates[i].ipm_index = static_cast<uint16>(nearest_index);
            candidates[i].original_x = original_points[original_index][0];
            candidates[i].original_y = original_points[original_index][1];
        }

        const int windows[2] = {k_front_corner_large_window, k_front_corner_small_window};
        for (int pass = 0; pass < 2; ++pass)
        {
            const int window = windows[pass];
            float best_angle_diff = -1e9f;
            int best_candidate = -1;

            for (int i = 0; i < 3; ++i)
            {
                if (!candidates[i].valid)
                {
                    continue;
                }

                const int ipm_index = candidates[i].ipm_index;
                const int im = clip_index(ipm_index, window, ipm_count - 1) - window;
                const int ip = clip_index(ipm_index, 0, ipm_count - 1 - window) + window;

                const float angle = local_angle_points_u16(ipm_points, ipm_count, ipm_index, window);
                const float angle_im = (im < window) ? 0.0f : local_angle_points_u16(ipm_points, ipm_count, im, window);
                const float angle_ip = (ip > static_cast<int>(ipm_count) - window) ? 0.0f : local_angle_points_u16(ipm_points, ipm_count, ip, window);
                const float angle_diff = std::fabs(angle) - 0.5f * (std::fabs(angle_im) + std::fabs(angle_ip));

                if (angle_diff > best_angle_diff)
                {
                    best_angle_diff = angle_diff;
                    best_candidate = i;
                }
            }

            if (best_candidate < 0)
            {
                continue;
            }

            const float angle_diff_deg = best_angle_diff * 180.0f / k_front_pi;
            const int white_count = count_corner_white_neighbors(candidates[best_candidate].original_x,
                                                                 candidates[best_candidate].original_y);
            if (angle_diff_deg > k_front_corner_angle_low_deg &&
                angle_diff_deg < k_front_corner_angle_high_deg &&
                white_count >= k_front_corner_confirm_white_min)
            {
                corner.valid = true;
                corner.index = candidates[best_candidate].original_index;
                corner.x = candidates[best_candidate].original_x;
                corner.y = candidates[best_candidate].original_y;
                corner.angle_deg = angle_diff_deg;
                return true;
            }
        }

        return false;
    }

    static void update_front_corner_candidates()
    {
        reset_front_split_corner_points();

        evaluate_front_corner(g_front_left_original_points, g_front_left_original_point_count,
                              g_front_left_ipm_points, g_front_left_ipm_point_count,
                              true, g_front_left_corner);
        evaluate_front_corner(g_front_right_original_points, g_front_right_original_point_count,
                              g_front_right_ipm_points, g_front_right_ipm_point_count,
                              false, g_front_right_corner);
        if (k_front_enable_split_corner_detection)
        {
            detect_front_track_corner_points();
        }
    }

    //========================================================================
    // 点集处理：平滑、重采样、法向偏移 → 生成中线
    //========================================================================

    // 对点集进行加权均值平滑，首尾点保持不变
    static uint16 blur_points_in_place(uint16 (*points)[2], uint16 count, int kernel)
    {
        if (count < 3 || kernel < 3)
        {
            return count;
        }

        const int half = kernel / 2;
        uint16 temp[k_front_migration_max_points][2] = {{0}};
        temp[0][0] = points[0][0];
        temp[0][1] = points[0][1];
        temp[count - 1][0] = points[count - 1][0];
        temp[count - 1][1] = points[count - 1][1];

        for (uint16 i = 1; i + 1 < count; ++i)
        {
            int sum_x = 0;
            int sum_y = 0;
            int weight_sum = 0;

            for (int offset = -half; offset <= half; ++offset)
            {
                const int index = clamp_int(static_cast<int>(i) + offset, 0, count - 1);
                const int weight = half + 1 - std::abs(offset);
                sum_x += static_cast<int>(points[index][0]) * weight;
                sum_y += static_cast<int>(points[index][1]) * weight;
                weight_sum += weight;
            }

            temp[i][0] = static_cast<uint16>(sum_x / std::max(weight_sum, 1));
            temp[i][1] = static_cast<uint16>(sum_y / std::max(weight_sum, 1));
        }

        std::memcpy(points, temp, sizeof(uint16) * 2 * count);
        return count;
    }

    // 沿点集路径按固定弧长距离等距重采样
    static uint16 resample_points_in_place(uint16 (*points)[2], uint16 count, int dist)
    {
        if (count < 2 || dist <= 0)
        {
            return count;
        }

        uint16 temp[k_front_migration_max_points][2] = {{0}};
        uint16 out_count = 0;
        temp[out_count][0] = points[0][0];
        temp[out_count][1] = points[0][1];
        ++out_count;

        float accumulated_dist = 0.0f;
        const float target_dist = static_cast<float>(dist);

        for (uint16 i = 0; i + 1 < count && out_count < k_front_migration_max_points; ++i)
        {
            float x0 = static_cast<float>(points[i][0]);
            float y0 = static_cast<float>(points[i][1]);
            const float x1 = static_cast<float>(points[i + 1][0]);
            const float y1 = static_cast<float>(points[i + 1][1]);

            float dx = x1 - x0;
            float dy = y1 - y0;
            float segment_len = std::sqrt(dx * dx + dy * dy);
            if (segment_len <= 1e-4f)
            {
                continue;
            }

            dx /= segment_len;
            dy /= segment_len;

            float dist_to_next = target_dist - accumulated_dist;
            while (dist_to_next <= segment_len && out_count < k_front_migration_max_points)
            {
                x0 += dx * dist_to_next;
                y0 += dy * dist_to_next;

                temp[out_count][0] = static_cast<uint16>(clamp_int(static_cast<int>(std::lround(x0)), 0, image_width - 1));
                temp[out_count][1] = static_cast<uint16>(clamp_int(static_cast<int>(std::lround(y0)), 0, image_height - 1));
                ++out_count;

                segment_len -= dist_to_next;
                dist_to_next = target_dist;
                accumulated_dist = 0.0f;
            }

            accumulated_dist += segment_len;
        }

        if (out_count == 0)
        {
            return 0;
        }

        std::memcpy(points, temp, sizeof(uint16) * 2 * out_count);
        return out_count;
    }

    // 沿边界切线方向向内偏移half_width，推定中线点集
    // left_side=true 时从左边线向右偏移(+x)，left_side=false 时从右边线向左偏移(-x)
    static uint16 offset_from_boundary(bool left_side, const uint16 (*src)[2], uint16 src_count,
                                       uint16 (*dst)[2], int approx_step, int half_width)
    {
        if (src_count < 2)
        {
            return 0;
        }

        uint16 dst_count = 0;
        for (uint16 i = 0; i + 1 < src_count && dst_count < k_front_migration_max_points; ++i)
        {
            const int prev_index = clamp_int(static_cast<int>(i) - approx_step, 0, src_count - 1);
            const int next_index = clamp_int(static_cast<int>(i) + approx_step, 0, src_count - 1);

            float dx = static_cast<float>(src[next_index][0]) - static_cast<float>(src[prev_index][0]);
            float dy = static_cast<float>(src[next_index][1]) - static_cast<float>(src[prev_index][1]);
            const float norm = std::sqrt(dx * dx + dy * dy);
            if (norm <= 1e-4f)
            {
                continue;
            }

            dx /= norm;
            dy /= norm;

            // 点集从近端向远端排列，近似方向是向上；左边线的赛道内侧在路径右侧，右边线的赛道内侧在路径左侧。
            const int sign = left_side ? -1 : 1;
            const int x = clamp_int(static_cast<int>(std::lround(static_cast<float>(src[i][0]) + sign * dy * half_width)), 0, image_width - 1);
            const int y = clamp_int(static_cast<int>(std::lround(static_cast<float>(src[i][1]) - sign * dx * half_width)), 0, image_height - 1);
            dst[dst_count][0] = static_cast<uint16>(x);
            dst[dst_count][1] = static_cast<uint16>(y);
            ++dst_count;
        }

        return dst_count;
    }

    // 根据左右边界点集数量和质量选择巡线基准边
    static uint8 choose_track_side(uint16 left_count, uint16 right_count)
    {
        if (left_count == 0 && right_count == 0)
        {
            return FrontTrackSideNone;
        }
        if (left_count == 0)
        {
            return FrontTrackSideRightBoundary;
        }
        if (right_count == 0)
        {
            return FrontTrackSideLeftBoundary;
        }

        // 对齐 Front_Car 原逻辑：优先用延伸更远、更稳定的一侧来补中线。
        if (left_count < right_count / 2 && left_count < 20)
        {
            return FrontTrackSideRightBoundary;
        }
        if (right_count < left_count / 2 && right_count < 20)
        {
            return FrontTrackSideLeftBoundary;
        }
        if (left_count < 10 && right_count > left_count)
        {
            return FrontTrackSideRightBoundary;
        }
        if (right_count < 10 && left_count > right_count)
        {
            return FrontTrackSideLeftBoundary;
        }

        return (left_count >= right_count) ? FrontTrackSideLeftBoundary : FrontTrackSideRightBoundary;
    }

    static uint8 resolve_front_track_side(uint16 left_count, uint16 right_count)
    {
        switch (g_front_track_side_control_mode)
        {
        case FrontTrackSideControlLeft:
            return (left_count >= k_front_force_track_min_points) ? FrontTrackSideLeftBoundary : FrontTrackSideNone;

        case FrontTrackSideControlRight:
            return (right_count >= k_front_force_track_min_points) ? FrontTrackSideRightBoundary : FrontTrackSideNone;

        case FrontTrackSideControlNone:
            return FrontTrackSideNone;

        case FrontTrackSideControlAuto:
        default:
            return choose_track_side(left_count, right_count);
        }
    }

    static void update_front_line_lost_state(uint16 left_count, uint16 right_count)
    {
        const bool left_good = left_count >= k_front_min_boundary_points;
        const bool right_good = right_count >= k_front_min_boundary_points;

        if (!left_good && !right_good)
        {
            g_front_line_lost = 3;
        }
        else if (!left_good)
        {
            g_front_line_lost = 1;
        }
        else if (!right_good)
        {
            g_front_line_lost = 2;
        }
        else
        {
            g_front_line_lost = 0;
        }
    }

    static uint16 count_front_boundary_overlap_points(const uint16 points[][2], uint16 point_count,
                                                      bool check_left_boundary, uint16 &valid_point_count)
    {
        valid_point_count = 0;
        uint16 overlap_count = 0;

        for (uint16 i = 0; i < point_count; ++i)
        {
            const int x = static_cast<int>(points[i][0]);
            const int y = static_cast<int>(points[i][1]);
            if (x < 0 || x >= image_width || y < 0 || y >= image_height)
            {
                continue;
            }

            const int row_left = valid_l_bound[y];
            const int row_right = valid_r_bound[y];
            if (row_left > row_right)
            {
                continue;
            }

            ++valid_point_count;
            const int valid_bound = check_left_boundary ? row_left : row_right;
            if (std::abs(x - valid_bound) <= 1)
            {
                ++overlap_count;
            }
        }

        return overlap_count;
    }

    static bool is_front_boundary_overlap_excessive(const uint16 points[][2], uint16 point_count,
                                                    bool check_left_boundary,
                                                    uint16 ratio_num, uint16 ratio_den)
    {
        uint16 valid_point_count = 0;
        const uint16 overlap_count = count_front_boundary_overlap_points(points, point_count,
                                                                         check_left_boundary,
                                                                         valid_point_count);
        if (valid_point_count == 0)
        {
            return true;
        }

        return overlap_count * ratio_den > valid_point_count * ratio_num;
    }

    static bool is_front_boundary_long_straight(const uint16 points[][2], uint16 point_count,
                                                bool check_left_boundary)
    {
        if (point_count < k_front_long_straight_min_point_count)
        {
            return false;
        }

        if (is_front_boundary_overlap_excessive(points, point_count, check_left_boundary,
                                                k_front_long_straight_boundary_overlap_ratio_num,
                                                k_front_long_straight_boundary_overlap_ratio_den))
        {
            return false;
        }

        uint16 bottom_index = 0;
        uint16 top_index = 0;
        for (uint16 i = 1; i < point_count; ++i)
        {
            if (points[i][1] > points[bottom_index][1])
            {
                bottom_index = i;
            }
            if (points[i][1] < points[top_index][1])
            {
                top_index = i;
            }
        }

        const int ax = static_cast<int>(points[bottom_index][0]);
        const int ay = static_cast<int>(points[bottom_index][1]);
        const int bx = static_cast<int>(points[top_index][0]);
        const int by = static_cast<int>(points[top_index][1]);
        if (std::abs(ay - by) < k_front_long_straight_min_row_span)
        {
            return false;
        }

        const int denom = std::max(std::abs(bx - ax), std::abs(by - ay));
        if (denom <= 0)
        {
            return false;
        }

        uint32 err_sum = 0;
        uint16 sample_count = 0;
        for (uint16 i = k_front_long_straight_sample_step;
             i + k_front_long_straight_sample_step < point_count;
             i += k_front_long_straight_sample_step)
        {
            const int px = static_cast<int>(points[i][0]);
            const int py = static_cast<int>(points[i][1]);
            const int err_num = std::abs((px - ax) * (by - ay) - (py - ay) * (bx - ax));
            const uint16 err = static_cast<uint16>((err_num + denom / 2) / denom);
            err_sum += err;
            ++sample_count;
        }

        if (sample_count == 0)
        {
            return false;
        }

        const uint16 mean_err = static_cast<uint16>((err_sum + sample_count / 2) / sample_count);
        return mean_err <= k_front_long_straight_error_threshold;
    }

    static void update_front_long_straight_features()
    {
        g_front_left_long_straight_flag =
            is_front_boundary_long_straight(g_front_left_ipm_points,
                                            g_front_left_ipm_point_count,
                                            true)
                ? 1
                : 0;
        g_front_right_long_straight_flag =
            is_front_boundary_long_straight(g_front_right_ipm_points,
                                            g_front_right_ipm_point_count,
                                            false)
                ? 1
                : 0;
    }

    static void rebuild_front_ipm_boundaries_from_original()
    {
        g_front_left_ipm_point_count = project_points_to_ipm(g_front_left_original_points,
                                                             g_front_left_original_point_count,
                                                             g_front_left_ipm_points);
        g_front_right_ipm_point_count = project_points_to_ipm(g_front_right_original_points,
                                                              g_front_right_original_point_count,
                                                              g_front_right_ipm_points);

        g_front_left_ipm_point_count = blur_points_in_place(g_front_left_ipm_points,
                                                            g_front_left_ipm_point_count,
                                                            k_front_boundary_blur_kernel);
        g_front_right_ipm_point_count = blur_points_in_place(g_front_right_ipm_points,
                                                             g_front_right_ipm_point_count,
                                                             k_front_boundary_blur_kernel);

        g_front_left_ipm_point_count = resample_points_in_place(g_front_left_ipm_points,
                                                                g_front_left_ipm_point_count,
                                                                k_front_resample_dist);
        g_front_right_ipm_point_count = resample_points_in_place(g_front_right_ipm_points,
                                                                 g_front_right_ipm_point_count,
                                                                 k_front_resample_dist);
    }

    // 刷新前视边线的每帧高层特征：IPM投影边线、丢线状态、拐点候选
    // 输入依赖：g_front_left/right_original_points（原始图爬边结果）
    // 产出：
    //   g_front_left/right_ipm_points + _count —— IPM空间平滑重采样后的边线点集
    //   g_front_line_lost —— 双边丢线状态（0=双边完好, 1=左丢, 2=右丢, 3=双边丢）
    //   g_front_left/right_corner + 四个分裂拐点 —— 当前帧拐点检测结果
    // 调用时机：每帧边线爬取完成后，中线/控制/前探之前
    // static void refresh_front_boundary_features()
    // {

    // }

    static bool should_try_front_probe(bool left_side)
    {
        if (g_front_cross_kind == FrontCrossNone)
        {
            return false;
        }

        if (left_side)
        {
            return g_front_left_original_point_count < k_front_cross_half_step_threshold ||
                   g_front_left_upper_corner.valid ||
                   g_front_left_corner.valid;
        }

        return g_front_right_original_point_count < k_front_cross_half_step_threshold ||
               g_front_right_upper_corner.valid ||
               g_front_right_corner.valid;
    }

    static bool try_front_probe_for_side(bool left_side, uint16 (*probe_points)[2], uint16 &probe_count)
    {
        probe_count = 0;
        if (!should_try_front_probe(left_side))
        {
            return false;
        }

        FrontProbeConnectionContext connection;
        const bool has_connection = get_front_probe_connection_context(left_side, connection);

        int anchor_x = 0;
        int anchor_y = 0;
        bool has_probe_reference = false;
        if (has_connection)
        {
            int probe_ipm_x = 0;
            int probe_ipm_y = 0;
            if (track_front_probe_reference_point(left_side, connection, probe_ipm_x, probe_ipm_y) &&
                map_front_ipm_to_original_point(probe_ipm_x, probe_ipm_y, anchor_x, anchor_y))
            {
                has_probe_reference = true;
            }
        }

        if (!has_probe_reference &&
            !get_front_probe_anchor_original(left_side, anchor_x, anchor_y))
        {
            return false;
        }

        SeedPoint probe_seed;
        if (!find_front_probe_seed(left_side, anchor_x, anchor_y, probe_seed))
        {
            return false;
        }

        probe_count = follow_boundary_from_single_seed(left_side, probe_seed, probe_points);
        if (probe_count < k_front_probe_min_points)
        {
            probe_count = 0;
            return false;
        }

        if (has_connection)
        {
            if (left_side)
            {
                if (splice_front_probe_points_into_original(true, connection,
                                                            g_front_left_original_points,
                                                            g_front_left_original_point_count,
                                                            probe_points, probe_count))
                {
                    return true;
                }
            }
            else if (splice_front_probe_points_into_original(false, connection,
                                                             g_front_right_original_points,
                                                             g_front_right_original_point_count,
                                                             probe_points, probe_count))
            {
                return true;
            }
        }

        if (left_side)
        {
            return merge_front_probe_points_into_original(g_front_left_original_points,
                                                          g_front_left_original_point_count,
                                                          probe_points, probe_count);
        }

        return merge_front_probe_points_into_original(g_front_right_original_points,
                                                      g_front_right_original_point_count,
                                                      probe_points, probe_count);
    }

    [[maybe_unused]] static bool apply_front_probe_patch()
    {
        if (g_front_cross_kind == FrontCrossNone)
        {
            return false;
        }

        bool changed = false;
        changed |= try_front_probe_for_side(true,
                                            g_front_left_probe_original_points,
                                            g_front_left_probe_original_point_count);
        changed |= try_front_probe_for_side(false,
                                            g_front_right_probe_original_points,
                                            g_front_right_probe_original_point_count);
        return changed;
    }

    static void prepend_point_in_place(uint16 (*points)[2], uint16 &count, int x, int y)
    {
        if (count >= k_front_migration_max_points)
        {
            return;
        }

        for (int i = static_cast<int>(count); i > 0; --i)
        {
            points[i][0] = points[i - 1][0];
            points[i][1] = points[i - 1][1];
        }

        points[0][0] = static_cast<uint16>(clamp_int(x, 0, image_width - 1));
        points[0][1] = static_cast<uint16>(clamp_int(y, 0, image_height - 1));
        ++count;
    }

    static void append_point_in_place(uint16 (*points)[2], uint16 &count, int x, int y)
    {
        if (count >= k_front_migration_max_points)
        {
            return;
        }

        points[count][0] = static_cast<uint16>(clamp_int(x, 0, image_width - 1));
        points[count][1] = static_cast<uint16>(clamp_int(y, 0, image_height - 1));
        ++count;
    }

    static void extend_midline_near_field_in_place(uint16 (*points)[2], uint16 &count, int half_width)
    {
        if (count == 0)
        {
            return;
        }

        const int target_y = image_height - 1;
        if (static_cast<int>(points[0][1]) >= target_y)
        {
            return;
        }

        int x = static_cast<int>(points[0][0]);
        if (g_front_left_ipm_point_count > 0 && g_front_right_ipm_point_count > 0)
        {
            x = (static_cast<int>(g_front_left_ipm_points[0][0]) + static_cast<int>(g_front_right_ipm_points[0][0])) / 2;
        }
        else if (g_front_track_side_mode == FrontTrackSideLeftBoundary && g_front_left_ipm_point_count > 0)
        {
            x = static_cast<int>(g_front_left_ipm_points[0][0]) + half_width;
        }
        else if (g_front_track_side_mode == FrontTrackSideRightBoundary && g_front_right_ipm_point_count > 0)
        {
            x = static_cast<int>(g_front_right_ipm_points[0][0]) - half_width;
        }

        prepend_point_in_place(points, count, x, target_y);
    }

    static void extend_midline_far_field_in_place(uint16 (*points)[2], uint16 &count)
    {
        if (count == 0)
        {
            return;
        }

        const int first_x = static_cast<int>(points[0][0]);
        const int last_x = static_cast<int>(points[count - 1][0]);
        if (std::abs(last_x - first_x) <= 1)
        {
            return;
        }

        int point_x = 0;
        int point_y = 0;
        bool use_fallback = true;

        if (count >= 10)
        {
            const int x0 = static_cast<int>(points[count - 1][0]);
            const int y0 = static_cast<int>(points[count - 1][1]);
            const float dx = static_cast<float>(x0) - static_cast<float>(points[count - 10][0]);
            const float dy = static_cast<float>(y0) - static_cast<float>(points[count - 10][1]);

            float t_min = -1.0f;
            if (std::fabs(dx) > 1e-3f)
            {
                const float t = (dx > 0.0f) ? ((image_width - 1.0f) - static_cast<float>(x0)) / dx
                                            : (-static_cast<float>(x0)) / dx;
                if (t >= 0.0f)
                {
                    t_min = t;
                }
            }

            if (std::fabs(dy) > 1e-3f)
            {
                const float t = (dy < 0.0f) ? (-static_cast<float>(y0)) / dy
                                            : ((image_height - 1.0f) - static_cast<float>(y0)) / dy;
                if (t >= 0.0f && (t_min < 0.0f || t < t_min))
                {
                    t_min = t;
                }
            }

            if (t_min > 0.0f)
            {
                use_fallback = false;
                point_x = static_cast<int>(std::lround(static_cast<float>(x0) + t_min * dx));
                point_y = static_cast<int>(std::lround(static_cast<float>(y0) + t_min * dy));
            }
        }

        if (use_fallback)
        {
            point_x = (last_x - first_x > 1) ? (image_width - 1) : 0;
            point_y = 0;
        }

        const int clamped_x = clamp_int(point_x, 0, image_width - 1);
        const int clamped_y = clamp_int(point_y, 0, image_height - 1);
        if (clamped_x == static_cast<int>(points[count - 1][0]) &&
            clamped_y == static_cast<int>(points[count - 1][1]))
        {
            return;
        }

        append_point_in_place(points, count, clamped_x, clamped_y);
    }

    // 将中线x坐标写入第y行，并限制在有效左右边界内
    static void stamp_mid_row(int x, int y)
    {
        if (y < 0 || y >= image_height)
        {
            return;
        }

        int min_x = 0;
        int max_x = image_width - 1;
        if (valid_l_bound[y] <= valid_r_bound[y])
        {
            min_x = valid_l_bound[y];
            max_x = valid_r_bound[y];
        }

        g_front_mid_line[y] = static_cast<uint8>(clamp_int(x, min_x, max_x));
        g_front_mid_line_valid[y] = 1;
    }

    // 将中线点集转为逐行中线数组，缺失行插值填充，并设置中线就绪标志
    static void convert_mid_points_to_row_array(const uint16 (*points)[2], uint16 count)
    {
        if (count == 0)
        {
            return;
        }

        for (uint16 i = 0; i < count; ++i)
        {
            stamp_mid_row(static_cast<int>(points[i][0]), static_cast<int>(points[i][1]));

            if (i + 1 >= count)
            {
                continue;
            }

            const int x0 = static_cast<int>(points[i][0]);
            const int y0 = static_cast<int>(points[i][1]);
            const int x1 = static_cast<int>(points[i + 1][0]);
            const int y1 = static_cast<int>(points[i + 1][1]);
            const int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
            if (steps <= 0)
            {
                continue;
            }

            for (int step = 0; step <= steps; ++step)
            {
                const float t = static_cast<float>(step) / static_cast<float>(steps);
                const int x = static_cast<int>(std::lround(x0 + (x1 - x0) * t));
                const int y = static_cast<int>(std::lround(y0 + (y1 - y0) * t));
                stamp_mid_row(x, y);
            }
        }

        int last_row = -1;
        int last_x = 0;
        for (int row = image_height - 1; row >= 0; --row)
        {
            if (!g_front_mid_line_valid[row])
            {
                continue;
            }

            const int current_x = g_front_mid_line[row];
            if (last_row < 0)
            {
                for (int fill_row = image_height - 1; fill_row > row; --fill_row)
                {
                    stamp_mid_row(current_x, fill_row);
                }
            }
            else if (last_row - row > 1)
            {
                for (int fill_row = last_row - 1; fill_row > row; --fill_row)
                {
                    const float t = static_cast<float>(fill_row - row) / static_cast<float>(last_row - row);
                    const int interp_x = static_cast<int>(std::lround(current_x * (1.0f - t) + last_x * t));
                    stamp_mid_row(interp_x, fill_row);
                }
            }

            last_row = row;
            last_x = current_x;
        }

        int valid_rows = 0;
        for (int row = 0; row < image_height; ++row)
        {
            valid_rows += g_front_mid_line_valid[row] ? 1 : 0;
        }
        g_front_midline_ready = (valid_rows >= k_front_min_mid_valid_rows) ? 1 : 0;
    }

    // 将角度规整到(-180, 180]区间
    static float wrap180(float angle)
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

    // // 将中线偏离度映射为偏航角增量，并限幅（已替换为 Pure Pursuit，暂保留）
    // static float map_deviation_to_test_yaw(float deviation)
    // {
    //     static constexpr float k_test_deviation_to_yaw_gain = 15.0f;
    //     static constexpr float k_test_yaw_limit = 14.0f;
    //
    //     float magnitude = std::fabs(deviation) * k_test_deviation_to_yaw_gain;
    //     magnitude = std::min(magnitude, k_test_yaw_limit);
    //     return std::copysign(magnitude, deviation);
    // }

    static bool build_control_midline_from_front(uint8 out_midline[image_height])
    {
        int fallback_x = clamp_int(g_front_last_center_x, 0, image_width - 1);
        bool has_valid_point = false;

        for (int row = image_height - 1; row >= 0; --row)
        {
            if (g_front_mid_line_valid[row])
            {
                fallback_x = g_front_mid_line[row];
                has_valid_point = true;
            }
            out_midline[row] = static_cast<uint8>(fallback_x);
        }

        if (!has_valid_point)
        {
            return false;
        }

        for (int row = image_height - 2; row >= 1; --row)
        {
            if (out_midline[row] - out_midline[row + 1] > 8)
            {
                out_midline[row] = static_cast<uint8>(out_midline[row + 1] + 4);
            }
            else if (out_midline[row + 1] - out_midline[row] > 8)
            {
                out_midline[row] = static_cast<uint8>(out_midline[row + 1] - 4);
            }
        }

        for (int row = image_height - 3; row >= 3; --row)
        {
            out_midline[row] = static_cast<uint8>((out_midline[row + 2] +
                                                   out_midline[row + 1] +
                                                   out_midline[row] +
                                                   out_midline[row - 1] +
                                                   out_midline[row - 2]) /
                                                  5);
        }

        return true;
    }

    // 计算指定行区间内中线的归一化偏离度，返回值∈[-1,1]
    static float calc_midline_band_deviation(const uint8 midline_rows[image_height], int row_start, int row_end)
    {
        const int clamped_start = std::clamp(row_start, 0, image_height - 1);
        const int clamped_end = std::clamp(row_end, 0, image_height - 1);
        if (clamped_start > clamped_end)
        {
            return 0.0f;
        }

        float sum = 0.0f;
        int count = 0;
        for (int row = clamped_start; row <= clamped_end; ++row)
        {
            sum += static_cast<float>(midline_rows[row]);
            ++count;
        }

        if (count <= 0)
        {
            return 0.0f;
        }

        const float mean_mid = sum / static_cast<float>(count);
        return (mean_mid - (image_width / 2.0f)) / (image_width / 2.0f);
    }

    static void fill_front_debug_image()
    {
        std::lock_guard<std::mutex> lock(g_ipm_image_mutex);
        if (g_front_debug_display_mode == FrontDebugDisplayIpm)
        {
            fill_ipm_debug_image_from_copy_image(debug_image, true);
            draw_front_car_migration_overlay(debug_image, true);
            return;
        }

        fill_front_car_migration_debug_image(debug_image, true);
    }
}

void toggle_front_debug_display_mode(void)
{
    g_front_debug_display_mode =
        (g_front_debug_display_mode == FrontDebugDisplayOriginal) ? FrontDebugDisplayIpm
                                                                  : FrontDebugDisplayOriginal;
}

const char *get_front_debug_display_mode_name(void)
{
    return (g_front_debug_display_mode == FrontDebugDisplayIpm) ? "IPM" : "Original";
}

void set_front_track_side_control_mode(uint8 mode)
{
    if (mode > FrontTrackSideControlNone)
    {
        mode = FrontTrackSideControlAuto;
    }

    g_front_track_side_control_mode = mode;
}

uint8 get_front_track_side_control_mode(void)
{
    return g_front_track_side_control_mode;
}

const char *get_front_track_side_control_mode_name(void)
{
    switch (g_front_track_side_control_mode)
    {
    case FrontTrackSideControlLeft:
        return "Left";
    case FrontTrackSideControlRight:
        return "Right";
    case FrontTrackSideControlNone:
        return "None";
    case FrontTrackSideControlAuto:
    default:
        return "Auto";
    }
}

void start_front_left_ring_yaw_tracking(void)
{
    g_front_left_ring_enter_unbounded_yaw = yaw_tracker.get_unbounded_yaw();
    g_front_left_ring_progress_yaw = 0.0f;
    g_front_left_ring_yaw_recording_flag = 1;
}

void stop_front_left_ring_yaw_tracking(void)
{
    g_front_left_ring_yaw_recording_flag = 0;
    g_front_left_ring_enter_unbounded_yaw = 0.0f;
    g_front_left_ring_progress_yaw = 0.0f;
}

void update_front_left_ring_yaw_tracking(void)
{
    if (!g_front_left_ring_yaw_recording_flag)
    {
        return;
    }

    g_front_left_ring_progress_yaw =
        g_front_left_ring_enter_unbounded_yaw - yaw_tracker.get_unbounded_yaw();
}

bool front_left_ring_exit_yaw_ready(void)
{
    return g_front_left_ring_progress_yaw >= k_front_left_ring_exit_switch_yaw;
}

//========================================================================
// 主流程：二值化 → 种子 → 爬边 → IPM投影 → 特征刷新 → 中线 → 控制
//========================================================================

// Front_Car并联移植主流程：二值化→找种子→追踪边界→逆透视→平滑→重采样→推中线
void process_front_car_migration(void)
{
    clear_front_debug_state();            // 清零所有缓冲区、点集、状态
    build_front_binary_from_copy_image(); // 对 copy_image 二值化，支持 adaptive 和 clean Otsu 两种模式

    const SeedSearchResult seeds = find_seed_points();
    g_front_debug_seeds = seeds;
    if (!seeds.has_left_seed && !seeds.has_right_seed)
    {
        return;
    }
    update_front_ipm_track_width_from_seeds(seeds);
    // 从种子点沿黑-白边界爬线，4方向Frontier追踪，产出原图边界点集
    g_front_left_original_point_count = follow_left_boundary(seeds, g_front_left_original_points);
    g_front_right_original_point_count = follow_right_boundary(seeds, g_front_right_original_points);

    rebuild_front_ipm_boundaries_from_original(); // 左右 IPM 边线点集（经投影、平滑、重采样）
    update_front_line_lost_state(g_front_left_ipm_point_count, g_front_right_ipm_point_count);
    update_front_long_straight_features();
    update_front_corner_candidates();

    track_element();
    // refresh_front_boundary_features();
    //  实验性最小切口：只更新十字 kind/state，不接管中线与控制。
    // update_front_cross_state_machine();
    //  // 仅在十字处理中尝试前探补线，补完后重新投影并刷新边界特征。
    //  if (apply_front_probe_patch())
    //  {
    //      refresh_front_boundary_features();
    //  }

    g_front_track_side_mode = resolve_front_track_side(g_front_left_ipm_point_count,
                                                       g_front_right_ipm_point_count);

    const int half_width = get_runtime_track_half_width();
    if (g_front_track_side_mode == FrontTrackSideLeftBoundary)
    {
        g_front_mid_ipm_point_count = offset_from_boundary(true, g_front_left_ipm_points,
                                                           g_front_left_ipm_point_count,
                                                           g_front_mid_ipm_points,
                                                           k_front_track_approx_step,
                                                           half_width);
    }
    else if (g_front_track_side_mode == FrontTrackSideRightBoundary)
    {
        g_front_mid_ipm_point_count = offset_from_boundary(false, g_front_right_ipm_points,
                                                           g_front_right_ipm_point_count,
                                                           g_front_mid_ipm_points,
                                                           k_front_track_approx_step,
                                                           half_width);
    }

    if (g_front_mid_ipm_point_count == 0)
    {
        g_front_track_side_mode = FrontTrackSideNone;
        return;
    }

    g_front_mid_ipm_point_count = blur_points_in_place(g_front_mid_ipm_points,
                                                       g_front_mid_ipm_point_count,
                                                       k_front_mid_blur_kernel);
    extend_midline_near_field_in_place(g_front_mid_ipm_points,
                                       g_front_mid_ipm_point_count,
                                       half_width);
    g_front_mid_ipm_point_count = resample_points_in_place(g_front_mid_ipm_points,
                                                           g_front_mid_ipm_point_count,
                                                           k_front_resample_dist);
    extend_midline_far_field_in_place(g_front_mid_ipm_points, g_front_mid_ipm_point_count);

    g_front_mid_original_point_count = project_points_to_original(g_front_mid_ipm_points,
                                                                  g_front_mid_ipm_point_count,
                                                                  g_front_mid_original_points);
    convert_mid_points_to_row_array(g_front_mid_ipm_points, g_front_mid_ipm_point_count);
}

static void draw_front_debug_hud(uint16 (*img)[image_width], bool big_endian)
{
    static constexpr int k_front_debug_hud_x = 1;
    static constexpr int k_front_debug_hud_y = 1;
    static constexpr int k_front_debug_hud_line_height = 8;

    const uint16 bg_color = to_debug_color(RGB565_BLACK, big_endian);
    const uint16 left_color = to_debug_color(RGB565_RED, big_endian);
    const uint16 mode = to_debug_color(RGB565_GREEN, big_endian);

    char line[24] = {0};
    std::snprintf(line, sizeof(line), "ctrl:%3u", g_front_track_side_control_mode);
    dbg_text_6x8(img, k_front_debug_hud_x, k_front_debug_hud_y + k_front_debug_hud_line_height * 0,
                 line, left_color, bg_color, true);

    std::snprintf(line, sizeof(line), "side:%3u", g_front_track_side_mode);
    dbg_text_6x8(img, k_front_debug_hud_x, k_front_debug_hud_y + k_front_debug_hud_line_height * 1,
                 line, mode, bg_color, true);

    std::snprintf(line, sizeof(line), "lost:%3u", g_front_line_lost);
    dbg_text_6x8(img, k_front_debug_hud_x, k_front_debug_hud_y + k_front_debug_hud_line_height * 2,
                 line, mode, bg_color, true);

    // std::snprintf(line, sizeof(line), "LOR:%3u", g_front_left_original_point_count);
    // dbg_text_6x8(img, k_front_debug_hud_x, k_front_debug_hud_y + k_front_debug_hud_line_height * 2,
    //              line, left_color, bg_color, true);

    // std::snprintf(line, sizeof(line), "ROR:%3u", g_front_right_original_point_count);
    // dbg_text_6x8(img, k_front_debug_hud_x, k_front_debug_hud_y + k_front_debug_hud_line_height * 3,
    //              line, right_color, bg_color, true);

    // std::snprintf(line, sizeof(line), "LIP:%3u", g_front_left_ipm_point_count);
    // dbg_text_6x8(img, k_front_debug_hud_x, k_front_debug_hud_y + k_front_debug_hud_line_height * 4,
    //              line, left_color, bg_color, true);

    // std::snprintf(line, sizeof(line), "RIP:%3u", g_front_right_ipm_point_count);
    // dbg_text_6x8(img, k_front_debug_hud_x, k_front_debug_hud_y + k_front_debug_hud_line_height * 5,
    //              line, right_color, bg_color, true);
}

// 在图像上叠加绘制IPM边界点集和中线（仅在中线就绪后生效）
void draw_front_car_migration_overlay(uint16 (*img)[image_width], bool big_endian)
{
    const uint16 seed_color = to_debug_color(RGB565_RED, big_endian);
    auto draw_seed_circle = [&](const SeedPoint &seed, bool project_to_ipm)
    {
        int draw_x = seed.x;
        int draw_y = seed.y;
        if (project_to_ipm)
        {
            double mapped_x = 0.0;
            double mapped_y = 0.0;
            if (!Transform_Point2(seed.x, seed.y, mapped_x, mapped_y))
            {
                return;
            }

            draw_x = static_cast<int>(std::lround(mapped_x));
            draw_y = static_cast<int>(std::lround(mapped_y));
            if (draw_x < 0 || draw_x >= image_width || draw_y < 0 || draw_y >= image_height)
            {
                return;
            }
        }

        dbg_circle(img, draw_x, draw_y, 3, seed_color);
    };

    if (g_front_debug_seeds.has_left_seed)
    {
        draw_seed_circle(g_front_debug_seeds.left_seed, true);
    }
    if (g_front_debug_seeds.has_right_seed)
    {
        draw_seed_circle(g_front_debug_seeds.right_seed, true);
    }

    if (g_front_midline_ready)
    {
        if (k_front_draw_projected_edges)
        {
            dbg_trace_points(img, g_front_left_ipm_points, g_front_left_ipm_point_count,
                             to_debug_color(RGB565_MAGENTA, big_endian), k_front_draw_trace_step);
            dbg_trace_points(img, g_front_right_ipm_points, g_front_right_ipm_point_count,
                             to_debug_color(RGB565_CYAN, big_endian), k_front_draw_trace_step);
        }

        const uint16 mid_color = to_debug_color(RGB565_PURPLE, big_endian);
        for (int row = 0; row < image_height; ++row)
        {
            if (!g_front_mid_line_valid[row])
            {
                continue;
            }
            dbg_point(img, g_front_mid_line[row], row, mid_color);
        }
    }

    draw_front_debug_hud(img, big_endian);
}

void fill_front_car_migration_debug_image(uint16 (*img)[image_width], bool big_endian)
{
    dbg_from_gray(img, g_front_binary, nullptr, nullptr,
                  to_debug_color(RGB565_BLACK, big_endian), big_endian);

    const uint16 left_color = to_debug_color(RGB565_RED, big_endian);
    const uint16 right_color = to_debug_color(RGB565_BLUE, big_endian);
    const uint16 mid_color = to_debug_color(RGB565_PURPLE, big_endian);
    const uint16 left_probe_color = to_debug_color(RGB565_GREEN, big_endian);
    const uint16 right_probe_color = to_debug_color(RGB565_GREEN, big_endian);
    const uint16 left_upper_corner_color = to_debug_color(RGB565_YELLOW, big_endian);
    const uint16 left_lower_corner_color = to_debug_color(RGB565_BROWN, big_endian);
    const uint16 right_upper_corner_color = to_debug_color(RGB565_CYAN, big_endian);
    const uint16 right_lower_corner_color = to_debug_color(RGB565_GREEN, big_endian);
    const uint16 seed_row_color = to_debug_color(RGB565_RED, big_endian);

    dbg_line(img, 0, clamp_int(k_front_seed_search_row, 0, image_height - 1),
             image_width - 1, clamp_int(k_front_seed_search_row, 0, image_height - 1),
             seed_row_color);
    dbg_line(img, 0, clamp_int(k_front_seed_min_row, 0, image_height - 1),
             image_width - 1, clamp_int(k_front_seed_min_row, 0, image_height - 1),
             seed_row_color);

    dbg_trace_points(img, g_front_left_original_points, g_front_left_original_point_count,
                     left_color, k_front_draw_trace_step);
    dbg_trace_points(img, g_front_right_original_points, g_front_right_original_point_count,
                     right_color, k_front_draw_trace_step);
    dbg_trace_points(img, g_front_left_probe_original_points, g_front_left_probe_original_point_count,
                     left_probe_color, 1);
    dbg_trace_points(img, g_front_right_probe_original_points, g_front_right_probe_original_point_count,
                     right_probe_color, 1);
    dbg_trace_points(img, g_front_mid_original_points, g_front_mid_original_point_count,
                     mid_color, 1);

    if (g_front_debug_seeds.has_left_seed)
    {
        dbg_circle(img, g_front_debug_seeds.left_seed.x, g_front_debug_seeds.left_seed.y, 3, seed_row_color);
    }
    if (g_front_debug_seeds.has_right_seed)
    {
        dbg_circle(img, g_front_debug_seeds.right_seed.x, g_front_debug_seeds.right_seed.y, 3, seed_row_color);
    }

    if (g_front_left_upper_corner.valid)
    {
        // dbg_rect(img, g_front_left_upper_corner.x, g_front_left_upper_corner.y, 2, left_upper_corner_color);
        dbg_cross(img, g_front_left_upper_corner.x, g_front_left_upper_corner.y, left_upper_corner_color, 2);
    }

    if (g_front_left_lower_corner.valid)
    {
        // dbg_rect(img, g_front_left_lower_corner.x, g_front_left_lower_corner.y, 2, left_lower_corner_color);
        dbg_cross(img, g_front_left_lower_corner.x, g_front_left_lower_corner.y, left_lower_corner_color, 2);
    }

    if (g_front_right_upper_corner.valid)
    {
        // dbg_rect(img, g_front_right_upper_corner.x, g_front_right_upper_corner.y, 2, right_upper_corner_color);
        dbg_cross(img, g_front_right_upper_corner.x, g_front_right_upper_corner.y, right_upper_corner_color, 2);
    }

    if (g_front_right_lower_corner.valid)
    {
        // dbg_rect(img, g_front_right_lower_corner.x, g_front_right_lower_corner.y, 2, right_lower_corner_color);
        dbg_cross(img, g_front_right_lower_corner.x, g_front_right_lower_corner.y, right_lower_corner_color, 2);
    }

    if (!g_front_left_upper_corner.valid && !g_front_left_lower_corner.valid && g_front_left_corner.valid)
    {
        dbg_rect(img, g_front_left_corner.x, g_front_left_corner.y, 2, left_upper_corner_color);
        dbg_cross(img, g_front_left_corner.x, g_front_left_corner.y, left_upper_corner_color, 2);
    }

    if (!g_front_right_upper_corner.valid && !g_front_right_lower_corner.valid && g_front_right_corner.valid)
    {
        dbg_rect(img, g_front_right_corner.x, g_front_right_corner.y, 2, right_upper_corner_color);
        dbg_cross(img, g_front_right_corner.x, g_front_right_corner.y, right_upper_corner_color, 2);
    }

    draw_front_debug_hud(img, big_endian);
}

// 将原图空间的稀疏中线点集转为逐行数组，缺失行线性插值填充
static uint8 g_front_original_midline[image_height] = {0};
static bool g_front_original_midline_valid = false;

static void build_original_midline_from_points()
{
    g_front_original_midline_valid = false;
    if (g_front_mid_original_point_count < 2)
    {
        return;
    }

    uint8 row_has_value[image_height] = {0};

    // 逐点写入，相邻点之间线性插值
    for (uint16 i = 0; i < g_front_mid_original_point_count; ++i)
    {
        const int x = static_cast<int>(g_front_mid_original_points[i][0]);
        const int y = clamp_int(static_cast<int>(g_front_mid_original_points[i][1]), 0, image_height - 1);
        g_front_original_midline[y] = static_cast<uint8>(clamp_int(x, 0, image_width - 1));
        row_has_value[y] = 1;

        if (i + 1 >= g_front_mid_original_point_count)
        {
            continue;
        }

        const int x1 = static_cast<int>(g_front_mid_original_points[i + 1][0]);
        const int y1 = clamp_int(static_cast<int>(g_front_mid_original_points[i + 1][1]), 0, image_height - 1);
        const int dy = y1 - y;
        if (dy == 0)
        {
            continue;
        }

        const int step_count = std::abs(dy);
        for (int s = 1; s < step_count; ++s)
        {
            const float t = static_cast<float>(s) / static_cast<float>(step_count);
            const int mid_y = y + ((dy > 0) ? s : -s);
            const int mid_x = static_cast<int>(std::lround(x + (x1 - x) * t));
            g_front_original_midline[mid_y] = static_cast<uint8>(clamp_int(mid_x, 0, image_width - 1));
            row_has_value[mid_y] = 1;
        }
    }

    // 底部（最近车）补全到 image_height-1
    int bottom_x = image_width / 2;
    for (int row = image_height - 1; row >= 0; --row)
    {
        if (row_has_value[row])
        {
            bottom_x = g_front_original_midline[row];
            break;
        }
    }
    for (int row = image_height - 1; row >= 0 && !row_has_value[row]; --row)
    {
        g_front_original_midline[row] = static_cast<uint8>(bottom_x);
    }

    // 顶部补全到 0
    int top_x = image_width / 2;
    for (int row = 0; row < image_height; ++row)
    {
        if (row_has_value[row])
        {
            top_x = g_front_original_midline[row];
            break;
        }
    }
    for (int row = 0; row < image_height && !row_has_value[row]; ++row)
    {
        g_front_original_midline[row] = static_cast<uint8>(top_x);
    }

    g_front_original_midline_valid = true;
}

// 将归一化偏差(-1~1)分段平滑映射为偏航角增量（度），对齐旧工程实测曲线
static float map_deviation_to_angle(float deviation)
{
    auto smoothstep = [](float t) -> float
    {
        return t * t * (3.0f - 2.0f * t);
    };

    const float x = std::fabs(deviation);
    float y = 0.0f;

    // 激进测试版：死区去掉、增益翻倍、上限提到 75，验证是否力不够
    if (x <= 0.0f)                                                      // 原 0.03
    {
        y = 0.0f;                                                       // 原死区
    }
    else if (x <= 0.1f)
    {
        y = 12.0f * ((x - 0.0f) / 0.1f) * ((x - 0.0f) / 0.1f);        // 原 6° @0.1
    }
    else if (x <= 0.25f)
    {
        y = 12.0f + 18.0f * smoothstep((x - 0.1f) / 0.15f);            // 原 10°→16°
    }
    else if (x <= 0.4f)
    {
        y = 30.0f + 15.0f * smoothstep((x - 0.25f) / 0.15f);           // 原 9°→25°
    }
    else if (x <= 0.5f)
    {
        y = 45.0f + 15.0f * smoothstep((x - 0.4f) / 0.1f);             // 原 10°→35°
    }
    else if (x <= 0.6f)
    {
        y = 60.0f + 15.0f * smoothstep((x - 0.5f) / 0.1f);             // 原 10°→45°
    }
    else
    {
        y = 75.0f;                                                       // 原 55°
    }

    return std::copysign(y, deviation);
}

// 视觉控制：IPM中线投影回原图 → 偏差加权 → 分段映射 → 目标偏航角
// 中线来源仍是当前 IPM 偏移方案，控制方案回到旧工程的 deviation + 分段曲线
void update_vision_control_test(void)
{
    static constexpr float k_near_weight = 0.7f;  // 近端偏差权重（原图空间，重远端）
    static constexpr float k_far_weight  = 0.3f;  // 远端偏差权重
    static constexpr float k_sharp_curve_threshold = 0.15f;
    static constexpr float k_gentle_curve_threshold = 0.10f;
    static constexpr int k_far_row_start  = image_height / 4;
    static constexpr int k_far_row_end    = image_height / 2;
    static constexpr int k_near_row_start = (image_height * 3) / 5;
    static constexpr int k_near_row_end   = (image_height * 5) / 6;

    // 构建原图空间逐行中线
    build_original_midline_from_points();
    if (!g_front_original_midline_valid)
    {
        g_track_info.deviation = 0.0f;
        g_track_info.curvature = 0.0f;
        g_track_info.scene = TrackScene::LostLine;
        vision_target_yaw = yaw;
        return;
    }

    const float e_near = calc_midline_band_deviation(g_front_original_midline,
                                                     k_near_row_start, k_near_row_end);
    const float e_far  = calc_midline_band_deviation(g_front_original_midline,
                                                     k_far_row_start, k_far_row_end);
    const float deviation = k_near_weight * e_near + k_far_weight * e_far;
    const float curvature = e_far - e_near;

    g_track_info.deviation = deviation;
    g_track_info.curvature = curvature;

    if (g_front_line_lost == 3)
    {
        g_track_info.scene = TrackScene::LostLine;
        vision_target_yaw = yaw;
        return;
    }

    const float vision_delta_yaw = map_deviation_to_angle(deviation);
    vision_target_yaw = wrap180(yaw + vision_delta_yaw);

    const float abs_curvature = std::fabs(curvature);
    if (abs_curvature > k_sharp_curve_threshold)
    {
        g_track_info.scene = TrackScene::SharpCurve;
    }
    else if (abs_curvature > k_gentle_curve_threshold)
    {
        g_track_info.scene = TrackScene::GentleCurve;
    }
    else
    {
        g_track_info.scene = TrackScene::Straight;
    }
}
//==============================================元素状态机
uint8_t mode_state = 0; // 0是普通，1是十字，2，做圆环，3右圆环
uint8_t left_ring_process_state = 0; //
uint8_t left_ring_phase_counter = 0;
uint8_t left_ring_lost_streak = 0; // 连续丢线帧数，抗抖动，攒够才推进 phase_counter

void left_ring_go(void)
{
    if (mode_state==2)
    {
        if (left_ring_process_state == 1)
        {
            set_front_track_side_control_mode(FrontTrackSideControlRight);
        }
        if(left_ring_process_state == 2)
        {
            set_front_track_side_control_mode(FrontTrackSideControlLeft);
        }
        if(left_ring_process_state == 3)
        {
            set_front_track_side_control_mode(FrontTrackSideControlRight);
        }

    }

}

void track_element(void)
{
    if (mode_state == 0)
    {
        if (g_front_left_long_straight_flag == 0 &&
             g_front_right_long_straight_flag == 1 &&
              g_front_left_corner.valid==1)
        {
            mode_state = 2;
            left_ring_process_state = 1;
            left_ring_phase_counter = 0;
            stop_front_left_ring_yaw_tracking();
        }
    }
    if (mode_state==2)
    {

        if (left_ring_process_state == 1)
        {
            // 对齐原工程 Ring_state1：等圆环所在侧（左）丢线，确认进入环岛缺口
            // 两层滤波：先攒连续丢线帧数(lost_streak)，攒够3帧才给 phase_counter+1
            if (g_front_line_lost == 1 || g_front_line_lost == 3)
            {
                left_ring_lost_streak++;
                if (left_ring_lost_streak >= 10)
                {
                    left_ring_phase_counter++;
                    left_ring_lost_streak = 0;
                }
            }
            else
            {
                left_ring_lost_streak = 0;
                if (left_ring_phase_counter > 0)
                {
                    left_ring_phase_counter--;
                }
            }

            if (left_ring_phase_counter >= 5)
            {
                left_ring_process_state = 2;
                left_ring_phase_counter = 0;
                start_front_left_ring_yaw_tracking();
            }
        }

        if (left_ring_process_state==2)
        {
            update_front_left_ring_yaw_tracking();

            // 累计左转角度达到 315° → 出环，切 state 3
            if (g_front_left_ring_progress_yaw >= 315.0f)
            {
                left_ring_process_state = 3;
                left_ring_phase_counter = 0;
            }
        }

        if (left_ring_process_state == 3)
        {
            update_front_left_ring_yaw_tracking();
            set_front_track_side_control_mode(FrontTrackSideControlRight);
        }
    }
    left_ring_go();
}

//=====================================================
void image_process(void)
{
    static bool g_front_runtime_initialized = 0;
    static uint8 lost_frame_count = 0;

    if (!g_front_runtime_initialized)
    {
        init_ipm_valid_region();
        g_front_runtime_initialized = true;
    }

    if (uvc_dev.wait_image_refresh() < 0)
    {
        ++lost_frame_count;
        printf("摄像头采集异常，连续丢帧: %u\n", lost_frame_count);
        if (lost_frame_count >= k_front_max_lost_frame_count)
        {
            printf("摄像头连续多次丢帧，程序退出\n");
            exit(0);
        }
        return;
    }
    lost_frame_count = 0;

    rgay_image = uvc_dev.get_gray_image_ptr();
    refresh_copy_image_from_current_camera_image();

    process_front_car_migration();
    update_vision_control_test();
    fill_front_debug_image();
}
