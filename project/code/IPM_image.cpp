#include "IPM_image.hpp"
#include "track_element.hpp"
#include "imu.hpp"

#include <algorithm>
#include <cstring>

zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
uint8 *rgay_image;     // 灰度图像数据指针，指向摄像头采集到的灰度图像缓冲区首地址

uint8 copy_image[image_height][image_width]; // 图像处理模块使用的图像数据缓冲区，供IPM变换等后续处理使用
// cv::Mat copy_image(image_height, image_width, CV_8UC1); // 等效于uint8 copy_image[image_height][image_width];
uint16 debug_image[image_height][image_width]; // RGB565 调试图像缓冲区，供 SCC8660 彩色图传使用
uint8 bin_image[image_height][image_width]; // 逆透视后二值图，供后续搜线/丢线判断使用
std::mutex g_ipm_image_mutex;
static uint8_t ipm_work_array[image_height][image_width] = {0}; // 逆透写入临时缓冲区，避免图传线程读到半帧
static constexpr uint8 k_max_lost_frame_count = 5; // 连续丢失有效图像帧的最大容忍次数，超过后可触发安全机制
static constexpr uint8_t k_ipm_invalid_fill_value = 255; // 无效区统一填白，避免被误判成赛道黑区
static constexpr uint16 k_debug_invalid_fill_color = 0x000F; // RGB565 调试图中的无效区填充色
static constexpr int k_ipm_valid_margin = 2; // 有效边界向内收缩一点，减少边缘误判
static bool g_ipm_valid_region_initialized = false;

uint8 Threshold = 0; // 二值化阈值，动态
static constexpr uint16 k_otsu_roi_start_row = image_height / 3;
static constexpr uint8 k_threshold_default = 110;
static constexpr uint8 k_threshold_min_limit = 40;
static constexpr uint8 k_threshold_max_limit = 220;
//static constexpr uint16 k_max_search_points = USE_num;
static uint8 g_last_threshold = k_threshold_default;
uint8 mid_line[image_height]; // 中线

//==============================//========================//===============================

// 全局查表数组
int valid_l_bound[image_height];// 每一行逆透视有效区域的左边界，若 left > right 说明该行无效
int valid_r_bound[image_height];
uint8 start_point_l[2] = {0}; // start_point_l[0]：左起点 x
uint8 start_point_r[2] = {0};
uint8 g_left_start_point_fallback_flag = 0;
uint8 g_right_start_point_fallback_flag = 0;
uint8 left_edge_line[image_height] = {0};// 八邻域得到的左边界线
uint8 right_edge_line[image_height] = {0};
static constexpr int k_start_black_confirm_count = 2; // 起始点跳变判定时，至少需要连续这么多个黑点
static constexpr int k_valid_box_bottom_window = 5; // 画框底边时参考的底部行数窗口
static constexpr int k_search_top_stop_row = 0; // 八邻域搜线到达该行后停止继续向左右扩展，避免沿顶边横爬
static int g_valid_box_bottom_row = -1;   // 预计算得到的画框底线所在行
static int g_valid_box_bottom_left = -1;  // 预计算得到的画框底线左端点
static int g_valid_box_bottom_right = -1; // 预计算得到的画框底线右端点
static int g_valid_box_start_row = -1;    // 预计算得到的起始点搜索行
static int g_valid_box_start_left = -1;   // 起始点搜索行对应的有效左边界
static int g_valid_box_start_right = -1;  // 起始点搜索行对应的有效右边界
static uint8 g_left_ring_bridge_debug_flag = 0; // 本帧是否注入了左环补线，供 TCP 调试图显式标注
static int g_left_ring_bridge_debug_x0 = 0;
static int g_left_ring_bridge_debug_y0 = 0;
static int g_left_ring_bridge_debug_x1 = 0;
static int g_left_ring_bridge_debug_y1 = 0;

// 搜到的左边界点坐标数组，第一维是点的索引，第二维0/1分别是x/y坐标
uint16 points_l[k_max_search_points][2] = {{0}};
uint16 points_r[k_max_search_points][2] = {{0}};
uint8 dir_l[k_max_search_points] = {0};
uint8 dir_r[k_max_search_points] = {0};
uint16 g_left_point_count = 0;// 搜到的左边界点数量
uint16 g_right_point_count = 0;
uint8 g_left_crossover_flag = 0;
uint8 g_right_crossover_flag = 0;
Track_Corner_Point_TypeDef g_left_upper_corner = {0, 0, 0};
Track_Corner_Point_TypeDef g_right_upper_corner = {0, 0, 0};
Track_Corner_Point_TypeDef g_left_lower_corner = {0, 0, 0};
Track_Corner_Point_TypeDef g_right_lower_corner = {0, 0, 0};
uint8 g_left_long_straight_flag = 0;
uint8 g_right_long_straight_flag = 0;
uint8 g_track_reference_width_valid = 0;
uint8 g_track_reference_width = 0;
uint8 g_track_reference_center = image_width / 2;
uint8 g_track_reference_row = 0;
static uint8 g_track_reference_width_by_row[image_height] = {0}; // 单边巡线补宽时使用的逐行参考宽度
static uint8 g_track_reference_width_by_row_valid[image_height] = {0};
static constexpr uint8 k_search_crossover_stop_count = 2;//多次
static constexpr int k_corner_mid_step = 6;
static constexpr int k_corner_end_step = 12;
static constexpr int k_corner_upper_scan_start = 7;
static constexpr int k_corner_top_guard_row = 5;
static constexpr int k_corner_vertical_delta_min = 4;
static constexpr int k_corner_near_flat_delta_max = 4;
static constexpr int k_corner_region_upper_max_row = (image_height * 3) / 5;
static constexpr int k_corner_region_lower_min_row = image_height / 3;
static constexpr uint16 k_corner_upper_dir_candidate_skip = 2;      // 第 3 个候选
static constexpr uint16 k_corner_lower_dir_candidate_back_skip = 1; // 倒数第 2 个候选
static constexpr uint16 k_long_straight_min_point_count = 35;
static constexpr uint16 k_long_straight_min_row_span = 50;
static constexpr uint16 k_long_straight_sample_step = 4;
static constexpr uint16 k_long_straight_dir_change_limit = 3;
static constexpr uint16 k_long_straight_error_threshold = 3;
static constexpr uint8 k_long_straight_boundary_overlap_ratio_num = 1;
static constexpr uint8 k_long_straight_boundary_overlap_ratio_den = 4;
static constexpr uint8 k_track_width_init_confirm_rows = 3;
static constexpr int k_track_width_init_width_tolerance = 1;
static constexpr int k_track_width_init_edge_tolerance = 1;
static constexpr int k_left_ring_prepare_max_row = 29; // 左环第一段内，左上拐点先进入上三十行，才允许准备切到第二段
static constexpr int k_left_ring_inside_ready_min_row = 50; // 左上拐点下移到该行后，允许从左环入口切到环内
static constexpr int k_left_ring_inside_ready_max_row = 60;
static constexpr float k_left_ring_exit_switch_yaw = 315.0f; // 左环累计角达到该值后，切到第三段并恢复右边巡线

static int clamp_int(int value, int min_value, int max_value)// 整数范围限制函数，超出范围的部分会被压缩到边界值
{
    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }
    return value;
}

static int abs_int(int value)
{
    return (value >= 0) ? value : -value;
}

static inline uint16 swap_rgb565_bytes(uint16 color)
{
    return static_cast<uint16>((color << 8) | (color >> 8));
}

static inline uint8 get_safe_pixel(int x, int y)
{
    if (x < 0 || x >= image_width || y < 0 || y >= image_height)
    {
        return 255;
    }

    return bin_image[y][x];
}

static void clear_corner_point(Track_Corner_Point_TypeDef &corner)
{
    corner.flag = 0;
    corner.row = 0;
    corner.col = 0;
}

static void reset_track_corner_points(void)
{
    clear_corner_point(g_left_upper_corner);
    clear_corner_point(g_right_upper_corner);
    clear_corner_point(g_left_lower_corner);
    clear_corner_point(g_right_lower_corner);
}

static bool is_corner_point_in_target_region(int x, int y, bool want_left_region, bool want_upper_region)
{
    if (x < 0 || x >= image_width || y < 0 || y >= image_height)
    {
        return false;
    }

    if (want_upper_region)
    {
        if (y > k_corner_region_upper_max_row)
        {
            return false;
        }
    }
    else if (y < k_corner_region_lower_min_row)
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

static void store_corner_point(Track_Corner_Point_TypeDef &corner, int x, int y)
{
    corner.flag = 1;
    corner.row = static_cast<uint8>(clamp_int(y, 0, image_height - 1));
    corner.col = static_cast<uint8>(clamp_int(x, 0, image_width - 1));
}

static bool is_left_search_point_in_safe_region(int x, int y)
{
    if (x < 0 || x >= image_width || y < 0 || y >= image_height)
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
    if (row_span < 2)
    {
        return false;
    }

    const int left_limit = row_left + (row_span * 2) / 3;
    return x >= row_left && x <= left_limit;
}

static bool is_right_search_point_in_safe_region(int x, int y)
{
    if (x < 0 || x >= image_width || y < 0 || y >= image_height)
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
    if (row_span < 2)
    {
        return false;
    }

    const int right_limit = row_left + row_span / 3;
    return x >= right_limit && x <= row_right;
}

static bool select_next_search_point(const int candidates[][2], uint8 candidate_count,
                                     bool prefer_left_region,
                                     int &next_x, int &next_y,
                                     bool &used_safe_region)
{
    if (candidate_count == 0)
    {
        used_safe_region = false;
        return false;
    }

    int best_safe_index = -1;
    int best_any_index = 0;

    for (uint8 i = 0; i < candidate_count; ++i)
    {
        const int x = candidates[i][0];
        const int y = candidates[i][1];

        if (candidates[best_any_index][1] > y)
        {
            best_any_index = i;
        }

        const bool in_safe_region = prefer_left_region ?
                                    is_left_search_point_in_safe_region(x, y) :
                                    is_right_search_point_in_safe_region(x, y);
        if (!in_safe_region)
        {
            continue;
        }

        if (best_safe_index < 0 || candidates[best_safe_index][1] > y)
        {
            best_safe_index = i;
        }
    }

    used_safe_region = (best_safe_index >= 0);
    const int chosen_index = used_safe_region ? best_safe_index : best_any_index;
    next_x = candidates[chosen_index][0];
    next_y = candidates[chosen_index][1];
    return true;
}

static uint8 calc_dir_step_diff(uint8 dir_a, uint8 dir_b)
{
    int diff = abs_int((int)dir_a - (int)dir_b);
    if (diff > 4)
    {
        diff = 8 - diff;
    }
    return static_cast<uint8>(diff);
}

static uint16 count_boundary_overlap_points(const uint16 points[][2], uint16 point_count,
                                            bool check_left_boundary, uint16 &valid_point_count)
{
    valid_point_count = 0;
    uint16 overlap_count = 0;

    for (uint16 i = 0; i < point_count && i < k_max_search_points; ++i)
    {
        const int x = points[i][0];
        const int y = points[i][1];

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
        const int target_bound = check_left_boundary ? row_left : row_right;
        if (x == target_bound)
        {
            ++overlap_count;
        }
    }

    return overlap_count;
}

static bool is_boundary_overlap_excessive(const uint16 points[][2], uint16 point_count,
                                          bool check_left_boundary,
                                          uint16 ratio_num, uint16 ratio_den)
{
    uint16 valid_point_count = 0;
    const uint16 overlap_count = count_boundary_overlap_points(points, point_count,
                                                               check_left_boundary, valid_point_count);
    if (valid_point_count == 0)
    {
        return true;
    }

    return overlap_count * ratio_den > valid_point_count * ratio_num;
}

static bool is_boundary_long_straight(const uint16 points[][2], uint16 point_count,
                                      const uint8 *dirs, uint8 crossover_flag,
                                      bool check_left_boundary)
{
    if (crossover_flag || point_count < k_long_straight_min_point_count)
    {
        return false;
    }

    if (is_boundary_overlap_excessive(points, point_count, check_left_boundary,
                                      k_long_straight_boundary_overlap_ratio_num,
                                      k_long_straight_boundary_overlap_ratio_den))
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

    const int ax = points[bottom_index][0];
    const int ay = points[bottom_index][1];
    const int bx = points[top_index][0];
    const int by = points[top_index][1];
    const int row_span = abs_int(ay - by);
    if (row_span < k_long_straight_min_row_span)
    {
        return false;
    }

    uint16 dir_change_count = 0;
    for (uint16 i = 1; i < point_count; ++i)
    {
        if (calc_dir_step_diff(dirs[i], dirs[i - 1]) >= 2)
        {
            ++dir_change_count;
        }
    }
    if (dir_change_count > k_long_straight_dir_change_limit)
    {
        return false;
    }

    const int denom = std::max(abs_int(bx - ax), abs_int(by - ay));
    if (denom <= 0)
    {
        return false;
    }

    uint32 err_sum = 0;
    uint16 sample_count = 0;
    for (uint16 i = k_long_straight_sample_step;
         i + k_long_straight_sample_step < point_count;
         i += k_long_straight_sample_step)
    {
        const int px = points[i][0];
        const int py = points[i][1];
        const int err_num = abs_int((px - ax) * (by - ay) - (py - ay) * (bx - ax));
        const uint16 err = static_cast<uint16>((err_num + denom / 2) / denom);
        err_sum += err;
        ++sample_count;
    }

    if (sample_count == 0)
    {
        return false;
    }

    const uint16 mean_err = static_cast<uint16>((err_sum + sample_count / 2) / sample_count);
    return mean_err <= k_long_straight_error_threshold;
}

void detect_track_long_straight_features(void)
{
    g_left_long_straight_flag = is_boundary_long_straight(points_l, g_left_point_count,
                                                          dir_l, g_left_crossover_flag, true) ? 1 : 0;
    g_right_long_straight_flag = is_boundary_long_straight(points_r, g_right_point_count,
                                                           dir_r, g_right_crossover_flag, false) ? 1 : 0;
}

static void reset_track_search_results(void)
{
    g_left_point_count = 0;
    g_right_point_count = 0;
    g_left_crossover_flag = 0;
    g_right_crossover_flag = 0;
    g_left_long_straight_flag = 0;
    g_right_long_straight_flag = 0;

    std::memset(points_l, 0, sizeof(points_l));
    std::memset(points_r, 0, sizeof(points_r));
    std::memset(dir_l, 0, sizeof(dir_l));
    std::memset(dir_r, 0, sizeof(dir_r));
}

static void fill_track_lines_from_valid_region(void)
{
    const int fallback_center = image_width / 2;

    for (int row = 0; row < image_height; ++row)
    {
        int left = valid_l_bound[row];
        int right = valid_r_bound[row];

        if (left > right)
        {
            left = clamp_int(fallback_center - 1, 0, image_width - 1);
            right = clamp_int(fallback_center + 1, 0, image_width - 1);
        }

        left = clamp_int(left, 0, image_width - 1);
        right = clamp_int(right, 0, image_width - 1);

        if (left > right)
        {
            std::swap(left, right);
        }

        left_edge_line[row] = static_cast<uint8>(left);
        right_edge_line[row] = static_cast<uint8>(right);
        mid_line[row] = static_cast<uint8>((left + right) / 2);
    }
}

static void draw_line_on_binary_image(uint8 (*image)[image_width],
                                      int x0, int y0, int x1, int y1,
                                      uint8 color)
{
    x0 = clamp_int(x0, 0, image_width - 1);
    y0 = clamp_int(y0, 0, image_height - 1);
    x1 = clamp_int(x1, 0, image_width - 1);
    y1 = clamp_int(y1, 0, image_height - 1);

    int dx = abs_int(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs_int(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true)
    {
        image[y0][x0] = color;

        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        const int e2 = err << 1;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static void clear_left_ring_bridge_debug_state(void)
{
    g_left_ring_bridge_debug_flag = 0;
    g_left_ring_bridge_debug_x0 = 0;
    g_left_ring_bridge_debug_y0 = 0;
    g_left_ring_bridge_debug_x1 = 0;
    g_left_ring_bridge_debug_y1 = 0;
}

static void draw_left_ring_inside_bridge_on_binary_image(uint8 (*image)[image_width])
{
    const int x0 = 0;
    const int y0 = 0;
    const int x1 = clamp_int(static_cast<int>(start_point_r[0]), 0, image_width - 1);
    const int y1 = clamp_int(static_cast<int>(start_point_r[1]), 0, image_height - 1);

    g_left_ring_bridge_debug_flag = 1;
    g_left_ring_bridge_debug_x0 = x0;
    g_left_ring_bridge_debug_y0 = y0;
    g_left_ring_bridge_debug_x1 = x1;
    g_left_ring_bridge_debug_y1 = y1;

    for (int row_offset = -1; row_offset <= 1; ++row_offset)
    {
        draw_line_on_binary_image(image,
                                  x0, y0 + row_offset,
                                  x1, y1 + row_offset,
                                  0);
    }
}

static void seed_track_width_reference_profile(uint8 width)
{
    const uint8 safe_width = static_cast<uint8>(clamp_int(static_cast<int>(width), 2, image_width - 1));
    for (int row = 0; row < image_height; ++row)
    {
        g_track_reference_width_by_row[row] = safe_width;
        g_track_reference_width_by_row_valid[row] = 1;
    }
}

static int get_single_side_reference_width_for_row(int row, int fallback_width)
{
    if (row >= 0 && row < image_height && g_track_reference_width_by_row_valid[row])
    {
        return std::max(static_cast<int>(g_track_reference_width_by_row[row]), 2);
    }

    if (g_track_reference_width_valid)
    {
        return std::max(static_cast<int>(g_track_reference_width), 2);
    }

    return std::max(fallback_width, 2);
}

void update_mid_line_by_track_mode(uint8 track_mode)
{
    for (int row = 0; row < image_height; ++row)
    {
        int limit_left = 0;
        int limit_right = image_width - 1;
        if (valid_l_bound[row] <= valid_r_bound[row])
        {
            limit_left = valid_l_bound[row];
            limit_right = valid_r_bound[row];
        }

        int left = clamp_int(static_cast<int>(left_edge_line[row]), limit_left, limit_right);
        int right = clamp_int(static_cast<int>(right_edge_line[row]), limit_left, limit_right);
        if (left > right)
        {
            std::swap(left, right);
        }

        int track_width = right - left;
        if (track_mode != 0)
        {
            track_width = get_single_side_reference_width_for_row(row, track_width);
        }
        if (track_width < 2)
        {
            track_width = 2;
        }

        int mid = (left + right) / 2;
        if (track_mode == 1)
        {
            const int virtual_right = clamp_int(left + track_width, limit_left, limit_right);
            mid = (left + virtual_right) / 2;
        }
        else if (track_mode == 2)
        {
            const int virtual_left = clamp_int(right - track_width, limit_left, limit_right);
            mid = (virtual_left + right) / 2;
        }

        mid_line[row] = static_cast<uint8>(clamp_int(mid, 0, image_width - 1));
    }
}

static void search_l_r(uint16 *l_stastic, uint16 *r_stastic,
                       uint8 l_start_x, uint8 l_start_y,
                       uint8 r_start_x, uint8 r_start_y,
                       uint8 *highest)
{
    uint16 i = 0;

    int search_filds_l[8][2] = {{0}};
    uint8 index_l = 0;
    int temp_l[8][2] = {{0}};
    int center_point_l[2] = {l_start_x, l_start_y};
    uint16 l_data_statics = *l_stastic;
    static constexpr int8_t seeds_l[8][2] = {
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}};

    int search_filds_r[8][2] = {{0}};
    int center_point_r[2] = {r_start_x, r_start_y};
    uint8 index_r = 0;
    int temp_r[8][2] = {{0}};
    uint16 r_data_statics = *r_stastic;
    static constexpr int8_t seeds_r[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

    uint8 left_run = 1;
    uint8 right_run = 1;
    uint8 left_active = 1;
    uint8 right_active = 1;
    uint8 left_crossover_streak = 0;
    uint8 right_crossover_streak = 0;
    int break_flag = k_max_search_points;

    while (break_flag-- && (left_active || right_active))
    {
        if (left_active && left_run)
        {
            if (l_data_statics >= k_max_search_points)
            {
                left_active = 0;
                left_run = 0;
            }
            else
            {
                for (i = 0; i < 8; ++i)
                {
                    search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
                    search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
                }

                points_l[l_data_statics][0] = static_cast<uint16>(center_point_l[0]);
                points_l[l_data_statics][1] = static_cast<uint16>(center_point_l[1]);
                ++l_data_statics;

                if (center_point_l[1] <= k_search_top_stop_row)
                {
                    left_active = 0;
                    left_run = 0;
                    continue;
                }

                index_l = 0;
                for (i = 0; i < 8; ++i)
                {
                    const uint8 p1 = get_safe_pixel(search_filds_l[i][0], search_filds_l[i][1]);
                    const uint8 p2 = get_safe_pixel(search_filds_l[(i + 1) & 7][0], search_filds_l[(i + 1) & 7][1]);

                    if (p1 == 0 && p2 == 255 && index_l < 8)
                    {
                        temp_l[index_l][0] = search_filds_l[i][0];
                        temp_l[index_l][1] = search_filds_l[i][1];
                        ++index_l;
                        dir_l[l_data_statics - 1] = static_cast<uint8>(i);
                    }
                }

                if (index_l > 0)
                {
                    bool used_safe_region = false;
                    if (select_next_search_point(temp_l, index_l, true,
                                                 center_point_l[0], center_point_l[1],
                                                 used_safe_region))
                    {
                        if (used_safe_region)
                        {
                            left_crossover_streak = 0;
                        }
                        else
                        {
                            ++left_crossover_streak;
                            if (left_crossover_streak >= k_search_crossover_stop_count)
                            {
                                g_left_crossover_flag = 1;
                                left_active = 0;
                                left_run = 0;
                            }
                        }
                    }
                }
                else
                {
                    left_active = 0;
                    left_run = 0;
                }
            }
        }

        if (right_active && right_run)
        {
            if (r_data_statics >= k_max_search_points)
            {
                right_active = 0;
                right_run = 0;
            }
            else
            {
                for (i = 0; i < 8; ++i)
                {
                    search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
                    search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
                }

                points_r[r_data_statics][0] = static_cast<uint16>(center_point_r[0]);
                points_r[r_data_statics][1] = static_cast<uint16>(center_point_r[1]);
                ++r_data_statics;

                if (center_point_r[1] <= k_search_top_stop_row)
                {
                    right_active = 0;
                    right_run = 0;
                    continue;
                }

                index_r = 0;
                for (i = 0; i < 8; ++i)
                {
                    const uint8 p1 = get_safe_pixel(search_filds_r[i][0], search_filds_r[i][1]);
                    const uint8 p2 = get_safe_pixel(search_filds_r[(i + 1) & 7][0], search_filds_r[(i + 1) & 7][1]);

                    if (p1 == 0 && p2 == 255 && index_r < 8)
                    {
                        temp_r[index_r][0] = search_filds_r[i][0];
                        temp_r[index_r][1] = search_filds_r[i][1];
                        ++index_r;
                        dir_r[r_data_statics - 1] = static_cast<uint8>(i);
                    }
                }

                if (index_r > 0)
                {
                    bool used_safe_region = false;
                    if (select_next_search_point(temp_r, index_r, false,
                                                 center_point_r[0], center_point_r[1],
                                                 used_safe_region))
                    {
                        if (used_safe_region)
                        {
                            right_crossover_streak = 0;
                        }
                        else
                        {
                            ++right_crossover_streak;
                            if (right_crossover_streak >= k_search_crossover_stop_count)
                            {
                                g_right_crossover_flag = 1;
                                right_active = 0;
                                right_run = 0;
                            }
                        }
                    }
                }
                else
                {
                    right_active = 0;
                    right_run = 0;
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
            if ((r_data_statics >= 3 &&
                 points_r[r_data_statics - 1][0] == points_r[r_data_statics - 2][0] &&
                 points_r[r_data_statics - 1][0] == points_r[r_data_statics - 3][0] &&
                 points_r[r_data_statics - 1][1] == points_r[r_data_statics - 2][1] &&
                 points_r[r_data_statics - 1][1] == points_r[r_data_statics - 3][1]) ||
                (l_data_statics >= 3 &&
                 points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
                 points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
                 points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
                 points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
            {
                break;
            }

            if (abs_int((int)points_r[r_data_statics - 1][0] - (int)points_l[l_data_statics - 1][0]) < 2 &&
                abs_int((int)points_r[r_data_statics - 1][1] - (int)points_l[l_data_statics - 1][1]) < 2)
            {
                *highest = static_cast<uint8>((points_r[r_data_statics - 1][1] + points_l[l_data_statics - 1][1]) >> 1);
                break;
            }

            left_run = 1;
            right_run = 1;
            const int left_y = points_l[l_data_statics - 1][1];
            const int right_y = points_r[r_data_statics - 1][1];

            if (left_y < right_y)
            {
                left_run = 0;
            }
            else if (right_y < left_y)
            {
                right_run = 0;
            }

            if (dir_l[l_data_statics - 1] == 7 && right_y > left_y)
            {
                center_point_l[0] = points_l[l_data_statics - 1][0];
                center_point_l[1] = points_l[l_data_statics - 1][1];
                --l_data_statics;
                left_run = 0;
            }
        }
    }

    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

static void update_track_width_reference_profile_from_rows(const int *left_by_row,
                                                           const int *right_by_row)
{
    if (mode_element != 0)
    {
        return;
    }

    for (int row = 0; row < image_height; ++row)
    {
        if (left_by_row[row] < 0 || right_by_row[row] < 0)
        {
            continue;
        }

        const int row_left = valid_l_bound[row];
        const int row_right = valid_r_bound[row];
        if (row_left > row_right)
        {
            continue;
        }

        const int left = clamp_int(left_by_row[row], row_left, row_right);
        const int right = clamp_int(right_by_row[row], row_left, row_right);
        if (left >= right)
        {
            continue;
        }

        const int measured_width = right - left;
        if (measured_width < 2)
        {
            continue;
        }

        int filtered_width = measured_width;
        if (g_track_reference_width_by_row_valid[row])
        {
            filtered_width = (static_cast<int>(g_track_reference_width_by_row[row]) * 3 +
                              measured_width + 2) / 4;
        }

        g_track_reference_width_by_row[row] =
            static_cast<uint8>(clamp_int(filtered_width, 2, image_width - 1));
        g_track_reference_width_by_row_valid[row] = 1;
    }
}

static void convert_points_to_line_arrays(void)// 把搜到的离散点转换成每行一个边界坐标的形式，方便后续处理使用
{
    int left_by_row[image_height];
    int right_by_row[image_height];
    int top_left_found_row = image_height;
    int top_right_found_row = image_height;
    bool left_found_any = false;
    bool right_found_any = false;

    for (int y = 0; y < image_height; ++y)
    {
        left_by_row[y] = -1;
        right_by_row[y] = -1;
    }

    for (uint16 i = 0; i < g_left_point_count && i < k_max_search_points; ++i)
    {
        const int x = points_l[i][0];
        const int y = points_l[i][1];

        if (x < 0 || x >= image_width || y < 0 || y >= image_height)
        {
            continue;
        }

        if (left_by_row[y] < 0 || x < left_by_row[y])
        {
            left_by_row[y] = x;
        }

        if (y < top_left_found_row)
        {
            top_left_found_row = y;
        }
        left_found_any = true;
    }

    for (uint16 i = 0; i < g_right_point_count && i < k_max_search_points; ++i)
    {
        const int x = points_r[i][0];
        const int y = points_r[i][1];

        if (x < 0 || x >= image_width || y < 0 || y >= image_height)
        {
            continue;
        }

        if (right_by_row[y] < 0 || x > right_by_row[y])
        {
            right_by_row[y] = x;
        }

        if (y < top_right_found_row)
        {
            top_right_found_row = y;
        }
        right_found_any = true;
    }

    update_track_width_reference_profile_from_rows(left_by_row, right_by_row);

    int last_left = (start_point_l[0] < image_width) ? start_point_l[0] : (image_width / 2 - 1);
    int last_right = (start_point_r[0] < image_width) ? start_point_r[0] : (image_width / 2 + 1);

    for (int y = image_height - 1; y >= 0; --y)
    {
        const int row_left = valid_l_bound[y];
        const int row_right = valid_r_bound[y];
        const bool has_valid_region = (row_left <= row_right);

        if (left_by_row[y] >= 0)
        {
            last_left = left_by_row[y];
        }
        else if ((!left_found_any || y < top_left_found_row) && has_valid_region)
        {
            last_left = row_left;
        }

        if (right_by_row[y] >= 0)
        {
            last_right = right_by_row[y];
        }
        else if ((!right_found_any || y < top_right_found_row) && has_valid_region)
        {
            last_right = row_right;
        }

        if (has_valid_region)
        {
            last_left = clamp_int(last_left, row_left, row_right);
            last_right = clamp_int(last_right, row_left, row_right);
        }
        else
        {
            last_left = clamp_int(last_left, 0, image_width - 1);
            last_right = clamp_int(last_right, 0, image_width - 1);
        }

        if (last_left > last_right)
        {
            const int center = (last_left + last_right) / 2;
            last_left = center;
            last_right = center;
        }

        left_edge_line[y] = static_cast<uint8>(last_left);
        right_edge_line[y] = static_cast<uint8>(last_right);
        mid_line[y] = static_cast<uint8>((last_left + last_right) / 2);
    }
}

static void draw_track_line_overlay(uint16 (*img)[image_width])
{
    const uint16 edge_color = swap_rgb565_bytes(RGB565_GRAY);
    const uint16 mid_color = swap_rgb565_bytes(RGB565_BLACK);

    for (int row = 0; row < image_height; ++row)
    {
        dbg_point(img, left_edge_line[row], row, edge_color);
        dbg_point(img, right_edge_line[row], row, edge_color);
        dbg_point(img, mid_line[row], row, mid_color);
    }
}

static void update_track_lines_from_start_points(void)// 从起始点更新轨迹线
{
    fill_track_lines_from_valid_region();
    reset_track_search_results();

    if (g_valid_box_start_row < 0)
    {
        return;
    }

    uint16 left_count = 0;
    uint16 right_count = 0;
    uint8 highest = static_cast<uint8>(g_valid_box_start_row);

    search_l_r(&left_count, &right_count,
               start_point_l[0], start_point_l[1],
               start_point_r[0], start_point_r[1],
               &highest);

    g_left_point_count = left_count;
    g_right_point_count = right_count;

    if (g_left_point_count == 0 && g_right_point_count == 0)
    {
        return;
    }

    convert_points_to_line_arrays();
}

static bool find_longest_white_segment_in_row(const uint8 *row_ptr,
                                              int search_left, int search_right,
                                              int &best_white_left,
                                              int &best_white_right,
                                              int &best_white_length)
{
    best_white_left = -1;
    best_white_right = -1;
    best_white_length = 0;

    int col = search_left;
    while (col <= search_right)
    {
        while (col <= search_right && row_ptr[col] != 255)
        {
            ++col;
        }
        if (col > search_right)
        {
            break;
        }

        const int white_left = col;
        while (col <= search_right && row_ptr[col] == 255)
        {
            ++col;
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

    return best_white_length > 0;
}

static bool measure_track_width_from_row(const uint8 (*image)[image_width], int row,
                                         int &left_edge, int &right_edge,
                                         int &center_x, int &track_width)
{
    if (row < 0 || row >= image_height)
    {
        return false;
    }

    const int safe_left = valid_l_bound[row];
    const int safe_right = valid_r_bound[row];
    if (safe_left > safe_right)
    {
        return false;
    }

    const int search_left = std::max(safe_left + 1, 1);
    const int search_right = std::min(safe_right - 1, image_width - 2);
    if (search_left > search_right)
    {
        return false;
    }

    const uint8 *row_ptr = image[row];
    int best_white_left = -1;
    int best_white_right = -1;
    int best_white_length = 0;
    if (!find_longest_white_segment_in_row(row_ptr, search_left, search_right,
                                           best_white_left, best_white_right, best_white_length))
    {
        return false;
    }

    const int measured_left = best_white_left - 1;
    const int measured_right = best_white_right + 1;

    // 贴着有效区边界的候选不要，用来避免把六边形底部的极限边界误当成赛道宽度。
    if (measured_left <= safe_left || measured_right >= safe_right)
    {
        return false;
    }

    if (row_ptr[measured_left] != 0 || row_ptr[measured_right] != 0)
    {
        return false;
    }

    left_edge = measured_left;
    right_edge = measured_right;
    center_x = (measured_left + measured_right) / 2;
    track_width = measured_right - measured_left;
    return true;
}

static void try_init_reference_track_width(const uint8 (*image)[image_width])
{
    if (g_track_reference_width_valid || !g_ipm_valid_region_initialized)
    {
        return;
    }

    int streak_left[k_track_width_init_confirm_rows] = {0};
    int streak_right[k_track_width_init_confirm_rows] = {0};
    int streak_center[k_track_width_init_confirm_rows] = {0};
    int streak_width[k_track_width_init_confirm_rows] = {0};
    int streak_row[k_track_width_init_confirm_rows] = {0};
    uint8 streak_count = 0;

    int start_row = image_height - 2;
    if (g_valid_box_bottom_row >= 1)
    {
        start_row = g_valid_box_bottom_row - 1;
    }

    for (int row = start_row; row >= 0; --row)
    {
        int left_edge = 0;
        int right_edge = 0;
        int center_x = 0;
        int track_width = 0;
        if (!measure_track_width_from_row(image, row, left_edge, right_edge, center_x, track_width))
        {
            streak_count = 0;
            continue;
        }

        if (streak_count > 0)
        {
            const bool width_match = abs_int(track_width - streak_width[0]) <= k_track_width_init_width_tolerance;
            const bool left_match = abs_int(left_edge - streak_left[0]) <= k_track_width_init_edge_tolerance;
            const bool right_match = abs_int(right_edge - streak_right[0]) <= k_track_width_init_edge_tolerance;

            if (!(width_match && left_match && right_match))
            {
                streak_count = 0;
            }
        }

        streak_left[streak_count] = left_edge;
        streak_right[streak_count] = right_edge;
        streak_center[streak_count] = center_x;
        streak_width[streak_count] = track_width;
        streak_row[streak_count] = row;
        ++streak_count;

        if (streak_count < k_track_width_init_confirm_rows)
        {
            continue;
        }

        int width_sum = 0;
        int center_sum = 0;
        for (uint8 i = 0; i < k_track_width_init_confirm_rows; ++i)
        {
            width_sum += streak_width[i];
            center_sum += streak_center[i];
        }

        g_track_reference_width = static_cast<uint8>(clamp_int(
            (width_sum + k_track_width_init_confirm_rows / 2) / k_track_width_init_confirm_rows,
            0, image_width - 1));
        g_track_reference_center = static_cast<uint8>(clamp_int(
            (center_sum + k_track_width_init_confirm_rows / 2) / k_track_width_init_confirm_rows,
            0, image_width - 1));
        g_track_reference_row = static_cast<uint8>(clamp_int(
            streak_row[k_track_width_init_confirm_rows / 2], 0, image_height - 1));
        g_track_reference_width_valid = 1;
        seed_track_width_reference_profile(g_track_reference_width);
        return;
    }
}

void refresh_copy_image_from_current_camera_image(void)
{
    cv::Mat src_mat_320(240, 320, CV_8UC1, rgay_image);
    cv::Mat src_mat(image_height, image_width, CV_8UC1, copy_image);
    cv::resize(src_mat_320, src_mat, cv::Size(image_width, image_height), 0, 0, cv::INTER_AREA);
}

static void build_ipm_gray_frame_from_copy_image(void)
{
    cv::Mat src_mat(image_height, image_width, CV_8UC1, copy_image);
    cv::Mat dst_mat(image_height, image_width, CV_8UC1, ipm_work_array);
    dst_mat.setTo(k_ipm_invalid_fill_value);

    for (int i = 0; i < image_height; i++)
    {
        const int row_left = valid_l_bound[i];
        const int row_right = valid_r_bound[i];
        if (row_left > row_right)
        {
            continue;
        }

        for (int j = row_left; j <= row_right; j++)
        {
            double mapped_x = 0.0;
            double mapped_y = 0.0;
            if (!Transform_Point1(j, i, mapped_x, mapped_y))
            {
                continue;
            }

            const int x = cvRound(mapped_x);
            const int y = cvRound(mapped_y);
            if (x >= 0 && x < image_width && y >= 0 && y < image_height)
            {
                dst_mat.at<uchar>(i, j) = src_mat.at<uchar>(y, x);
            }
        }
    }
}

// 输出是二值化+滤波+画框完毕的 bin_image
static void build_ipm_binary_frame_from_current_camera_image(void)
{
    refresh_copy_image_from_current_camera_image();
    cv::Mat src_mat(image_height, image_width, CV_8UC1, copy_image);
    Threshold = otsuThreshold(&src_mat.data[0], src_mat.cols, src_mat.rows);
    build_ipm_gray_frame_from_copy_image();

    turn_to_bin();
    image_filter(bin_image);
    draw_valid_region_box(bin_image);
}

void fill_ipm_debug_image_from_copy_image(uint16 (*img)[image_width], bool big_endian)
{
    if (!g_ipm_valid_region_initialized)
    {
        init_ipm_valid_region();
    }

    build_ipm_gray_frame_from_copy_image();
    dbg_from_gray(img, ipm_work_array, valid_l_bound, valid_r_bound,
                  k_debug_invalid_fill_color, big_endian);

    const uint16 box_color = big_endian ? swap_rgb565_bytes(RGB565_GREEN) : RGB565_GREEN;
    for (int row = 0; row < image_height; ++row)
    {
        const int row_left = valid_l_bound[row];
        const int row_right = valid_r_bound[row];
        if (row_left > row_right)
        {
            continue;
        }

        dbg_point(img, row_left, row, box_color);
        dbg_point(img, row_right, row, box_color);
    }

    if (g_valid_box_bottom_row >= 0 && g_valid_box_bottom_left >= 0 &&
        g_valid_box_bottom_right >= g_valid_box_bottom_left)
    {
        dbg_line(img, g_valid_box_bottom_left, g_valid_box_bottom_row,
                 g_valid_box_bottom_right, g_valid_box_bottom_row, box_color);
    }
}

bool init_reference_track_width(uint16 max_attempt_frames)
{
    if (!g_ipm_valid_region_initialized)
    {
        init_ipm_valid_region();
    }

    if (max_attempt_frames == 0)
    {
        max_attempt_frames = 1;
    }

    g_track_reference_width_valid = 0;
    g_track_reference_width = 0;
    g_track_reference_center = image_width / 2;
    g_track_reference_row = 0;
    std::memset(g_track_reference_width_by_row, 0, sizeof(g_track_reference_width_by_row));
    std::memset(g_track_reference_width_by_row_valid, 0, sizeof(g_track_reference_width_by_row_valid));

    uint8 lost_frame_count = 0;
    for (uint16 attempt = 0; attempt < max_attempt_frames; ++attempt)
    {
        if (uvc_dev.wait_image_refresh() < 0)
        {
            ++lost_frame_count;
            std::cout << "参考赛道宽度初始化时摄像头采集异常，连续丢帧: "
                      << static_cast<int>(lost_frame_count) << std::endl;
            if (lost_frame_count >= k_max_lost_frame_count)
            {
                return false;
            }
            continue;
        }

        lost_frame_count = 0;
        rgay_image = uvc_dev.get_gray_image_ptr();
        build_ipm_binary_frame_from_current_camera_image();
        try_init_reference_track_width(bin_image);

        if (g_track_reference_width_valid)
        {
            std::cout << "参考赛道宽度初始化完成: width="
                      << static_cast<int>(g_track_reference_width)
                      << " center=" << static_cast<int>(g_track_reference_center)
                      << " row=" << static_cast<int>(g_track_reference_row)
                      << std::endl;
            return true;
        }
    }

    std::cout << "参考赛道宽度初始化失败，未找到稳定赛道宽度" << std::endl;
    return false;
}


// 这部分代码的目的，是把摄像头原始视角下的赛道图像转换成“近似俯视图”。
// 原图中由于透视效应，越远的赛道看起来越窄，左右边线会向远处汇聚。
// 转成俯视图后，赛道宽度在不同高度上的变化会减小，后面的按行找边界会稳定很多。
//
// 当前实现没有直接调用 OpenCV 的 warpPerspective，而是手工做“逆向采样”：
// 1. 先遍历逆透视结果图 dst_mat 上的每一个像素点。
// 2. 用 Mat1 把这个“目标图像坐标”反推回原图坐标。
// 3. 再去输入图 src_mat 中取该坐标处的像素值。
//
// 这种写法的核心优点是：
// - 目标图像的每个像素都会被主动赋值，不容易出现正向投影时的空洞。
// - 后续如果想自己控制插值、边界裁剪、有效区域判断，会更直接。
//
// Mat1: 逆透视图坐标 -> 原图坐标
// Mat2: 原图坐标 -> 逆透视图坐标
// 主流程做逆向采样时使用 Mat1。
// 如果后续想把原图里的点、搜线结果或其他标记投到逆透视图上，就使用 Mat2。
double Mat1[3][3] = {
    {0.422043010752688, -0.30336595901112, 10.942817755721},
    {5.07699415451805E-17, 0.0566331219557026, 6.05577612351806},
    {1.05590374670109E-18, -0.00396792574211929, 0.575709953129308},
};

double Mat2[3][3] = {
    {2.36942675159236, 5.49044585987261, -102.789808917197},
    {-9.55351577492572E-16, 10.1656050955414, -106.929936305732},
    {-1.09302447655162E-17, 0.0700636942675159, 1},
};

// src_w / src_h: 输入图像尺寸。这里就是缩放后的算法处理尺寸 160x120。
// image_w / image_h: 逆透视结果图尺寸。当前与输入一致，便于直接复用现有缓冲区。
static constexpr int src_w = image_width;   // 输入图像宽度
static constexpr int src_h = image_height;  // 输入图像高度
static constexpr int image_w = image_width; // 逆透视输出宽度
static constexpr int image_h = image_height;// 逆透视输出高度

double Tx = 0;
double Ty = 0;
// 通用齐次坐标透视变换。
// 输入/输出坐标含义由传入的矩阵决定：
// - Mat1: 逆透视图 -> 原图
// - Mat2: 原图 -> 逆透视图
static bool transform_point_with_matrix(const double matrix[3][3],
                                        int x, int y,
                                        double &mapped_x, double &mapped_y)
{
    const double w = 1.0;
    const double transformedX = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * w;
    const double transformedY = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * w;
    const double transformedW = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * w;

    if (transformedW > -1e-6 && transformedW < 1e-6)
    {
        mapped_x = -1.0;
        mapped_y = -1.0;
        return false;
    }

    mapped_x = transformedX / transformedW;
    mapped_y = transformedY / transformedW;
    return true;
}

// 把逆透视图中的点映射回原图坐标，供逆向采样使用。
bool Transform_Point1(int x, int y, double &mapped_x, double &mapped_y)
{
    return transform_point_with_matrix(Mat1, x, y, mapped_x, mapped_y);
}

// 把原图中的点映射到逆透视图坐标，供调试、画点或边线投影使用。
bool Transform_Point2(int x, int y, double &mapped_x, double &mapped_y)
{
    return transform_point_with_matrix(Mat2, x, y, mapped_x, mapped_y);
}

// 保留原来的两参数接口，兼容旧的调用方式。
void Transform_Point1(int x, int y)
{
    if (!Transform_Point1(x, y, Tx, Ty))
    {
        Tx = -1.0;
        Ty = -1.0;
    }
}

void Transform_Point2(int x, int y)
{
    if (!Transform_Point2(x, y, Tx, Ty))
    {
        Tx = -1.0;
        Ty = -1.0;
    }
}
//========================================================================
// 提取逆透视有效边界的函数
void init_ipm_valid_region(void)
{
    int lowest_valid_row = -1;
    for (int row = 0; row < image_h; ++row)
    {
        int left = image_w;
        int right = -1;

        for (int col = 0; col < image_w; ++col)
        {
            double mapped_x = 0.0;
            double mapped_y = 0.0;
            // 反推该点在原图的坐标
            if (!Transform_Point1(col, row, mapped_x, mapped_y))
            {
                continue;
            }

            const int src_x = cvRound(mapped_x);
            const int src_y = cvRound(mapped_y);

            // 这里判断的是当前算法真正使用的输入图尺寸，也就是 160x120。
            if (src_x >= 0 && src_x < src_w && src_y >= 0 && src_y < src_h)
            {
                if (col < left)
                {
                    left = col;
                }
                if (col > right)
                {
                    right = col;
                }
            }
        }

        if (left <= right)
        {
            const int shrink_left = left + k_ipm_valid_margin;
            const int shrink_right = right - k_ipm_valid_margin;
            if (shrink_left <= shrink_right)
            {
                valid_l_bound[row] = shrink_left;
                valid_r_bound[row] = shrink_right;
                lowest_valid_row = row;
            }
            else
            {
                valid_l_bound[row] = image_w;
                valid_r_bound[row] = -1;
            }
        }
        else
        {
            // 用 left > right 明确表示该行没有有效区域。
            valid_l_bound[row] = image_w;
            valid_r_bound[row] = -1;
        }
    }

    g_valid_box_bottom_row = -1;
    g_valid_box_bottom_left = -1;
    g_valid_box_bottom_right = -1;
    g_valid_box_start_row = -1;
    g_valid_box_start_left = -1;
    g_valid_box_start_right = -1;

    if (lowest_valid_row >= 0)
    {
        g_valid_box_bottom_row = lowest_valid_row;
        g_valid_box_bottom_left = valid_l_bound[lowest_valid_row];
        g_valid_box_bottom_right = valid_r_bound[lowest_valid_row];

        const int bottom_window_start = std::max(0, lowest_valid_row - k_valid_box_bottom_window + 1);
        for (int row = lowest_valid_row; row >= bottom_window_start; --row)
        {
            const int row_left = valid_l_bound[row];
            const int row_right = valid_r_bound[row];
            if (row_left > row_right)
            {
                continue;
            }

            g_valid_box_bottom_left = std::min(g_valid_box_bottom_left, row_left);
            g_valid_box_bottom_right = std::max(g_valid_box_bottom_right, row_right);
        }

        if (lowest_valid_row > 0 && valid_l_bound[lowest_valid_row - 1] <= valid_r_bound[lowest_valid_row - 1])
        {
            g_valid_box_start_row = lowest_valid_row - 1;
            g_valid_box_start_left = valid_l_bound[g_valid_box_start_row];
            g_valid_box_start_right = valid_r_bound[g_valid_box_start_row];
        }
    }

    g_ipm_valid_region_initialized = true;
    printf("逆透视边界查表 (LUT) 初始化完成。\n");
}
//=========================================大津法自动阈值========================================
// /*-------------------------------------------------------------------------------------------------------------------
//   @brief     大津法求阈值
//   @param     image       图像数组
//              col         列
//              row         行
//   @return    threshold   返回int阈值数值
//   Sample     threshold=otsuThreshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
//   @note      no
// -------------------------------------------------------------------------------------------------------------------*/
static uint8 filter_threshold(uint8 raw_threshold)
{
    if (raw_threshold < k_threshold_min_limit)
    {
        raw_threshold = k_threshold_min_limit;
    }
    else if (raw_threshold > k_threshold_max_limit)
    {
        raw_threshold = k_threshold_max_limit;
    }

    // 低成本时域平滑，减少赛道光照突变导致的阈值抖动。
    g_last_threshold = (uint8)((g_last_threshold * 3 + raw_threshold) / 4);
    return g_last_threshold;
}

uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width = col;
    uint16 Image_Height = row;
    uint16 X;
    uint16 Y;
    uint16 start_row = (k_otsu_roi_start_row < Image_Height) ? k_otsu_roi_start_row : 0;
    uint8 *image_data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack = 0, OmegaFore = 0, MicroBack = 0, MicroFore = 0, SigmaB = 0, Sigma = 0; // 类间方差;
    uint16 MinValue = 0, MaxValue = 0;
    uint8 Threshold = 0;

    for (Y = start_row; Y < Image_Height; Y++)
    {
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)image_data[Y * Image_Width + X]]++; // 统计每个灰度值的个数信息
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++)
        ; // 获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--)
        ; // 获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return g_last_threshold; // 退回上一帧稳定阈值，避免异常帧直接打坏二值图
    }
    if (MinValue + 1 == MaxValue)
    {
        return filter_threshold((uint8)((MinValue + MaxValue) / 2)); // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y]; //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y; // 灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];                                               // 前景像素点数
        PixelFore = Amount - PixelBack;                                                    // 背景像素点数
        if (PixelBack == 0 || PixelFore == 0)
        {
            continue;
        }
        OmegaBack = (double)PixelBack / Amount;                                            // 前景像素百分比
        OmegaFore = (double)PixelFore / Amount;                                            // 背景像素百分比
        PixelIntegralBack += HistGram[Y] * Y;                                              // 前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;                             // 背景灰度值
        MicroBack = (double)PixelIntegralBack / PixelBack;                                 // 前景灰度百分比
        MicroFore = (double)PixelIntegralFore / PixelFore;                                 // 背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // g
        if (Sigma > SigmaB)                                                                // 遍历最大的类间方差g
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    return filter_threshold(Threshold);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     图像二值化处理函数
  @return    二值化之后的图像数组
  Sample     图像二值化
  @note      二值化处理，0黑，255是白，
-------------------------------------------------------------------------------------------------------------------*/
void turn_to_bin(void)
{
    // 如果后续需要自动阈值，建议配合有效区版本的大津法一起使用。

    for (int i = 0; i < image_height; i++)
    {
        const int row_left = valid_l_bound[i];
        const int row_right = valid_r_bound[i];

        // 先把整行设成白色，只有边框保留黑色，避免大块无效区干扰显示。
        for (int j = 0; j < image_width; j++)
        {
            bin_image[i][j] = 255;
        }

        if (row_left > row_right)
        {
            continue;
        }

        for (int j = row_left; j <= row_right; j++)
        {
            if (ipm_work_array[i][j] > Threshold)
            {
                bin_image[i][j] = 255;
            }
            else
            {
                bin_image[i][j] = 0;
            }
        }

    }

    // 保留最外
    for (int i = 0; i < image_height; i++)
    {
        bin_image[i][0] = 0;
        bin_image[i][image_width - 1] = 0;
    }
    for (int j = 0; j < image_width; j++)
    {
        bin_image[0][j] = 0;
        bin_image[image_height - 1][j] = 0;
    }
}

/* * 函数名称：image_filter
 * 函数作用：在逆透视有效区域内做轻量形态学滤波，去除孤立白点并填补小黑洞。
 * 说明：滤波范围避开左右黑边框本身，避免把八邻域依赖的边框结构滤坏。
 */
void image_filter(uint8 (*image)[image_width])
{
    for (int i = image_height - 2; i >= 1; --i)
    {
        const int safe_left = valid_l_bound[i];
        const int safe_right = valid_r_bound[i];
        if (safe_left > safe_right)
        {
            continue;
        }

        const int start_j = std::max(safe_left + 1, 1);
        const int end_j = std::min(safe_right - 1, image_width - 2);
        if (start_j > end_j)
        {
            continue;
        }

        for (int j = start_j; j <= end_j; ++j)
        {
            const uint8 white_count = static_cast<uint8>((
                                                             image[i - 1][j - 1] + image[i - 1][j] + image[i - 1][j + 1] +
                                                             image[i][j - 1] + image[i][j + 1] +
                                                             image[i + 1][j - 1] + image[i + 1][j] + image[i + 1][j + 1]) /
                                                         255);

            if (image[i][j] == 0 && white_count >= 5)
            {
                image[i][j] = 255;
            }
            else if (image[i][j] == 255 && white_count <= 2)
            {
                image[i][j] = 0;
            }
        }
    }
}

/* * 函数名称：draw_valid_region_box
 * 函数作用：按照逆透视有效区的左右边界在二值图上画黑框，并把最底有效行用黑线封底。
 * 说明：底边画在最底有效行，但左右端点会参考底部附近若干行的外扩范围，避免底部某一行边界突然内收时只画出半截底线。
 */
void draw_valid_region_box(uint8 (*image)[image_width])
{
    if (!g_ipm_valid_region_initialized)
    {
        return;
    }

    for (int row = 0; row < image_height; ++row)
    {
        const int row_left = valid_l_bound[row];
        const int row_right = valid_r_bound[row];
        if (row_left > row_right)
        {
            continue;
        }

        image[row][row_left] = 0;
        image[row][row_right] = 0;
    }

    if (g_valid_box_bottom_row < 0 || g_valid_box_bottom_row >= image_height ||
        g_valid_box_bottom_left < 0 || g_valid_box_bottom_left >= image_width ||
        g_valid_box_bottom_right < 0 || g_valid_box_bottom_right >= image_width ||
        g_valid_box_bottom_left > g_valid_box_bottom_right)
    {
        return;
    }

    for (int col = g_valid_box_bottom_left; col <= g_valid_box_bottom_right; ++col)
    {
        image[g_valid_box_bottom_row][col] = 0;
    }
}

/* * 函数名称：find_start_point_by_valid_box
 * 函数作用：以画框底线的上一行作为起始点搜索行，先动态求出该行白色赛道段的中点，再向左右找跳变点。
 * 说明：左右两侧都要求至少出现连续两个黑点才确认起始点；若某一侧不满足，就退回该侧有效边线。
 */
void find_start_point_by_valid_box(uint8 (*image)[image_width])
{
    start_point_l[0] = 0;
    start_point_l[1] = 0;
    start_point_r[0] = 0;
    start_point_r[1] = 0;
    g_left_start_point_fallback_flag = 1;
    g_right_start_point_fallback_flag = 1;

    if (g_valid_box_start_row < 0)
    {
        return;
    }

    const int start_row = g_valid_box_start_row;
    const int safe_left = g_valid_box_start_left;
    const int safe_right = g_valid_box_start_right;

    // 默认回退到该行的有效边线。
    start_point_l[0] = static_cast<uint8>(safe_left);
    start_point_l[1] = static_cast<uint8>(start_row);
    start_point_r[0] = static_cast<uint8>(safe_right);
    start_point_r[1] = static_cast<uint8>(start_row);

    const int search_left = std::max(safe_left + 1, 1);
    const int search_right = std::min(safe_right - 1, image_width - 2);
    if (search_left > search_right)
    {
        return;
    }

    const uint8 *row_ptr = image[start_row];
    int best_white_left = -1;
    int best_white_right = -1;
    int best_white_length = 0;
    if (!find_longest_white_segment_in_row(row_ptr, search_left, search_right,
                                           best_white_left, best_white_right, best_white_length))
    {
        return;
    }

    const int mid_col = (best_white_left + best_white_right) / 2;

    for (int x = mid_col; x >= best_white_left; --x)
    {
        const int black_start = x - 1;
        const int black_end = black_start - (k_start_black_confirm_count - 1);
        if (black_end < safe_left)
        {
            continue;
        }

        if (row_ptr[black_start] == 0 && row_ptr[black_end] == 0)
        {
            start_point_l[0] = static_cast<uint8>(black_start);
            g_left_start_point_fallback_flag = 0;
            break;
        }
    }

    for (int x = mid_col; x <= best_white_right; ++x)
    {
        const int black_start = x + 1;
        const int black_end = black_start + (k_start_black_confirm_count - 1);
        if (black_end > safe_right)
        {
            continue;
        }

        if (row_ptr[black_start] == 0 && row_ptr[black_end] == 0)
        {
            start_point_r[0] = static_cast<uint8>(black_start);
            g_right_start_point_fallback_flag = 0;
            break;
        }
    }
}

static bool find_dir_corner_candidate_index(const uint16 points[][2], const uint8 *dirs,
                                            uint16 point_count, uint8 target_dir,
                                            bool from_back, uint16 skip_count,
                                            bool want_left_region, bool want_upper_region,
                                            uint16 &candidate_index)
{
    if (point_count <= k_corner_end_step)
    {
        return false;
    }

    uint16 hit_count = 0;
    if (!from_back)
    {
        for (uint16 i = 0; i < point_count; ++i)
        {
            if (dirs[i] != target_dir ||
                i < k_corner_mid_step ||
                i + k_corner_mid_step >= point_count)
            {
                continue;
            }

            if (!is_corner_point_in_target_region(points[i][0], points[i][1],
                                                  want_left_region, want_upper_region))
            {
                continue;
            }

            if (hit_count >= skip_count)
            {
                candidate_index = i;
                return true;
            }
            ++hit_count;
        }
    }
    else
    {
        for (int i = (int)point_count - 1; i >= 0; --i)
        {
            if (dirs[i] != target_dir ||
                i < k_corner_mid_step ||
                i + k_corner_mid_step >= point_count)
            {
                continue;
            }

            if (!is_corner_point_in_target_region(points[i][0], points[i][1],
                                                  want_left_region, want_upper_region))
            {
                continue;
            }

            if (hit_count >= skip_count)
            {
                candidate_index = static_cast<uint16>(i);
                return true;
            }
            ++hit_count;
        }
    }

    return false;
}

static bool try_store_left_lower_corner_from_index(uint16 mid_index)
{
    if (mid_index < k_corner_mid_step || mid_index + k_corner_mid_step >= g_left_point_count)
    {
        return false;
    }

    const int ax = points_l[mid_index - k_corner_mid_step][0];
    const int ay = points_l[mid_index - k_corner_mid_step][1];
    const int bx = points_l[mid_index][0];
    const int by = points_l[mid_index][1];
    const int cx = points_l[mid_index + k_corner_mid_step][0];
    const int cy = points_l[mid_index + k_corner_mid_step][1];

    if (cy <= k_corner_top_guard_row ||
        !is_corner_point_in_target_region(bx, by, true, false))
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
        store_corner_point(g_left_lower_corner, bx, by);
        return true;
    }

    return false;
}

static bool try_store_right_lower_corner_from_index(uint16 mid_index)
{
    if (mid_index < k_corner_mid_step || mid_index + k_corner_mid_step >= g_right_point_count)
    {
        return false;
    }

    const int ax = points_r[mid_index - k_corner_mid_step][0];
    const int ay = points_r[mid_index - k_corner_mid_step][1];
    const int bx = points_r[mid_index][0];
    const int by = points_r[mid_index][1];
    const int cx = points_r[mid_index + k_corner_mid_step][0];
    const int cy = points_r[mid_index + k_corner_mid_step][1];

    if (cy <= k_corner_top_guard_row ||
        !is_corner_point_in_target_region(bx, by, false, false))
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
        store_corner_point(g_right_lower_corner, bx, by);
        return true;
    }

    return false;
}

static bool try_store_left_upper_corner_from_index(uint16 mid_index)
{
    if (mid_index < k_corner_mid_step || mid_index + k_corner_mid_step >= g_left_point_count)
    {
        return false;
    }

    const int ax = points_l[mid_index - k_corner_mid_step][0];
    const int ay = points_l[mid_index - k_corner_mid_step][1];
    const int bx = points_l[mid_index][0];
    const int by = points_l[mid_index][1];
    const int cx = points_l[mid_index + k_corner_mid_step][0];
    const int cy = points_l[mid_index + k_corner_mid_step][1];

    if (cy <= k_corner_top_guard_row ||
        !is_corner_point_in_target_region(bx, by, true, true))
    {
        return false;
    }

    const int corner_dot = (bx - ax) * (cx - bx) + (by - ay) * (cy - by);
    const int corner_trend_x = cx - ax;
    const int corner_trend_y = cy - ay;

    if (corner_dot >= 0 &&
        bx > ax &&
        by - cy > k_corner_vertical_delta_min &&
        ay - by < k_corner_near_flat_delta_max &&
        corner_trend_x >= 0 &&
        corner_trend_y <= 0)
    {
        store_corner_point(g_left_upper_corner, bx, by);
        return true;
    }

    return false;
}

static bool try_store_right_upper_corner_from_index(uint16 mid_index)
{
    if (mid_index < k_corner_mid_step || mid_index + k_corner_mid_step >= g_right_point_count)
    {
        return false;
    }

    const int ax = points_r[mid_index - k_corner_mid_step][0];
    const int ay = points_r[mid_index - k_corner_mid_step][1];
    const int bx = points_r[mid_index][0];
    const int by = points_r[mid_index][1];
    const int cx = points_r[mid_index + k_corner_mid_step][0];
    const int cy = points_r[mid_index + k_corner_mid_step][1];

    if (cy <= k_corner_top_guard_row ||
        !is_corner_point_in_target_region(bx, by, false, true))
    {
        return false;
    }

    const int corner_dot = (bx - ax) * (cx - bx) + (by - ay) * (cy - by);
    const int corner_trend_x = cx - ax;
    const int corner_trend_y = cy - ay;

    if (corner_dot >= 0 &&
        ax > bx &&
        by - cy > k_corner_vertical_delta_min &&
        ay - by < k_corner_near_flat_delta_max &&
        corner_trend_x <= 0 &&
        corner_trend_y <= 0)
    {
        store_corner_point(g_right_upper_corner, bx, by);
        return true;
    }

    return false;
}

static void detect_left_lower_corner_point(void)
{
    if (g_left_point_count <= k_corner_end_step)
    {
        return;
    }

    uint16 candidate_index = 0;
    if (find_dir_corner_candidate_index(points_l, dir_l, g_left_point_count,
                                        6, true, k_corner_lower_dir_candidate_back_skip,
                                        true, false, candidate_index) &&
        try_store_left_lower_corner_from_index(candidate_index))
    {
        return;
    }

    for (uint16 i = 0; i + k_corner_end_step < g_left_point_count; ++i)
    {
        if (try_store_left_lower_corner_from_index(i + k_corner_mid_step))
        {
            return;
        }
    }
}

static void detect_right_lower_corner_point(void)
{
    if (g_right_point_count <= k_corner_end_step)
    {
        return;
    }

    uint16 candidate_index = 0;
    if (find_dir_corner_candidate_index(points_r, dir_r, g_right_point_count,
                                        6, true, k_corner_lower_dir_candidate_back_skip,
                                        false, false, candidate_index) &&
        try_store_right_lower_corner_from_index(candidate_index))
    {
        return;
    }

    for (uint16 i = 0; i + k_corner_end_step < g_right_point_count; ++i)
    {
        if (try_store_right_lower_corner_from_index(i + k_corner_mid_step))
        {
            return;
        }
    }
}

static void detect_left_upper_corner_point(void)
{
    if (g_left_point_count <= k_corner_end_step)
    {
        return;
    }

    uint16 candidate_index = 0;
    if (find_dir_corner_candidate_index(points_l, dir_l, g_left_point_count,
                                        2, false, k_corner_upper_dir_candidate_skip,
                                        true, true, candidate_index) &&
        try_store_left_upper_corner_from_index(candidate_index))
    {
        return;
    }

    for (uint16 i = k_corner_upper_scan_start; i + k_corner_end_step < g_left_point_count; ++i)
    {
        if (try_store_left_upper_corner_from_index(i + k_corner_mid_step))
        {
            return;
        }
    }
}

static void detect_right_upper_corner_point(void)
{
    if (g_right_point_count <= k_corner_end_step)
    {
        return;
    }

    uint16 candidate_index = 0;
    if (find_dir_corner_candidate_index(points_r, dir_r, g_right_point_count,
                                        2, false, k_corner_upper_dir_candidate_skip,
                                        false, true, candidate_index) &&
        try_store_right_upper_corner_from_index(candidate_index))
    {
        return;
    }

    for (uint16 i = k_corner_upper_scan_start; i + k_corner_end_step < g_right_point_count; ++i)
    {
        if (try_store_right_upper_corner_from_index(i + k_corner_mid_step))
        {
            return;
        }
    }
}

void detect_track_corner_points(void)
{
    reset_track_corner_points();
    detect_left_lower_corner_point();
    detect_right_lower_corner_point();
    detect_left_upper_corner_point();
    detect_right_upper_corner_point();
}

//=======================================================测试元素
uint8_t line_lost = 0; // 0都不丢线，1左边丢线，2右边丢线，3都丢线
#define k_lost_line_ratio_num 1//丢线阈值分子
#define k_lost_line_ratio_den 7

uint8_t g_left_ring_yaw_recording_flag = 0; // 左环累计角是否正在记录
float g_left_ring_enter_unbounded_yaw = 0.0f; // 左环进入时记录的连续 yaw
float g_left_ring_progress_yaw = 0.0f; // 左环累计角，左转时递增

uint8_t mode_element = 0; // 0正常模式，1十字，2左圆环；
uint8_t left_ring_process_mode = 0;//0未进入左圆环
static uint8_t g_left_ring_ready_for_inside_flag = 0; // 左环第一段内，是否已经满足进入第二段前的预备条件
// // 辅助函数

static void start_left_ring_yaw_tracking(void)
{
    g_left_ring_enter_unbounded_yaw = yaw_tracker.get_unbounded_yaw();
    g_left_ring_progress_yaw = 0.0f;
    g_left_ring_yaw_recording_flag = 1;
}

static void stop_left_ring_yaw_tracking(void)
{
    g_left_ring_yaw_recording_flag = 0;
    g_left_ring_enter_unbounded_yaw = 0.0f;
    g_left_ring_progress_yaw = 0.0f;
}

static void update_left_ring_yaw_tracking(void)// 这个函数需要在每一帧都调用，才能正确累计角度。
{
    if (!g_left_ring_yaw_recording_flag)
    {
        return;
    }

    g_left_ring_progress_yaw =
        g_left_ring_enter_unbounded_yaw - yaw_tracker.get_unbounded_yaw();
}

void lost_line_check(void)
{
    uint16 left_valid_point_count = 0;
    uint16 right_valid_point_count = 0;
    const uint16 left_overlap_count = count_boundary_overlap_points(points_l, g_left_point_count,
                                                                    true, left_valid_point_count);
    const uint16 right_overlap_count = count_boundary_overlap_points(points_r, g_right_point_count,
                                                                     false, right_valid_point_count);
    float left_overlap_ratio = 0.0f;
    float right_overlap_ratio = 0.0f;

    uint8 left_lost = 0;
    uint8 right_lost = 0;

    line_lost = 0;

    if (g_left_point_count == 0 || left_valid_point_count == 0)
    {
        left_lost = 1;
    }
    else
    {
        left_overlap_ratio = static_cast<float>(left_overlap_count) /
                             static_cast<float>(left_valid_point_count);

        if (left_overlap_count * k_lost_line_ratio_den >
            left_valid_point_count * k_lost_line_ratio_num)
        {
            left_lost = 1;
        }
    }

    if (g_right_point_count == 0 || right_valid_point_count == 0)
    {
        right_lost = 1;
    }
    else
    {
        right_overlap_ratio = static_cast<float>(right_overlap_count) /
                              static_cast<float>(right_valid_point_count);

        if (right_overlap_count * k_lost_line_ratio_den >
            right_valid_point_count * k_lost_line_ratio_num)
        {
            right_lost = 1;
        }
    }

    test3 = left_overlap_ratio;
    test4 = right_overlap_ratio;

    if (left_lost && right_lost)
    {
        line_lost = 3;
    }
    else if (left_lost)
    {
        line_lost = 1;
    }
    else if (right_lost)
    {
        line_lost = 2;
    }
}

void left_ring_process()
{
    //update_left_ring_yaw_tracking();

    if (left_ring_process_mode==1)
    {
       update_mid_line_by_track_mode(2);
       // 在这个时候记录yaw值，辅助进环出环
       //右边巡线
    }
    if (left_ring_process_mode==2)
    {
       // start_left_ring_yaw_tracking();
        update_left_ring_yaw_tracking();
        update_mid_line_by_track_mode(1);
        //进行左边巡线
        //并且将将左上拐点到右下起始点的线段拟合成圆弧，记录圆心坐标和半径，辅助进环出环（我是怕左边线过度爬）
    }
    if (left_ring_process_mode==3)
    {
        update_left_ring_yaw_tracking();
        update_mid_line_by_track_mode(2);
        // 左环第三段切回右边巡线，等待后续补充出环判据
    }
}

static bool is_left_upper_corner_ready_for_ring_inside(void)
{
    if (!g_left_upper_corner.flag)
    {
        return false;
    }

    const int corner_row = static_cast<int>(g_left_upper_corner.row);
    return corner_row >= k_left_ring_inside_ready_min_row &&
           corner_row <= k_left_ring_inside_ready_max_row;
}

static void draw_left_ring_ready_region_box(uint16 (*img)[image_width])
{
    const int prepare_row = clamp_int(k_left_ring_prepare_max_row, 0, image_height - 1);
    const int inside_ready_min_row = clamp_int(k_left_ring_inside_ready_min_row, 0, image_height - 1);
    const int inside_ready_max_row = clamp_int(k_left_ring_inside_ready_max_row, 0, image_height - 1);
    const uint16 prepare_color = swap_rgb565_bytes(RGB565_YELLOW);
    const uint16 inside_ready_color = swap_rgb565_bytes(RGB565_GREEN);

    dbg_line(img, 0, prepare_row, image_width - 1, prepare_row, prepare_color);
    dbg_line(img, 0, inside_ready_min_row, image_width - 1, inside_ready_min_row, inside_ready_color);
    dbg_line(img, 0, inside_ready_max_row, image_width - 1, inside_ready_max_row, inside_ready_color);
}

void track_element_update(void)
{
    lost_line_check();
    detect_track_corner_points();
    detect_track_long_straight_features();

    if (mode_element != 2 && g_left_ring_yaw_recording_flag)
    {
        stop_left_ring_yaw_tracking();
    }

    if (mode_element==0)
    {
        {//左圆环识别
            if (line_lost == 1 && g_right_long_straight_flag == 1 && g_left_lower_corner.flag == 1 && g_left_upper_corner.flag == 0)
            {
                mode_element = 2;//判定为左圆环
                left_ring_process_mode = 1;
                g_left_ring_ready_for_inside_flag = 0;
                
            }
        }
        
    }
    if (mode_element == 2)
    {
        if (left_ring_process_mode == 1)
        {
            if (g_left_upper_corner.flag &&
                g_left_upper_corner.row <= k_left_ring_prepare_max_row &&
                g_left_lower_corner.flag == 0)
            {
                g_left_ring_ready_for_inside_flag = 1;
            }

            if (g_left_ring_ready_for_inside_flag &&
                is_left_upper_corner_ready_for_ring_inside())
            {
                left_ring_process_mode = 2;
                start_left_ring_yaw_tracking();
            }
        }
        else if (left_ring_process_mode == 2)
        {
            if (g_left_ring_progress_yaw >= k_left_ring_exit_switch_yaw)
            {
                left_ring_process_mode = 3;
            }
        }
    }
    left_ring_process();
}
