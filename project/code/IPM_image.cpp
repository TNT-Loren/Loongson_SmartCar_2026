#include "IPM_image.hpp"
#include "track_element.hpp"

#include <algorithm>
#include <cstring>

zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
uint8 *rgay_image;     // 灰度图像数据指针，指向摄像头采集到的灰度图像缓冲区首地址

uint8 copy_image[image_height][image_width]; // 图像处理模块使用的图像数据缓冲区，供IPM变换等后续处理使用
// cv::Mat copy_image(image_height, image_width, CV_8UC1); // 等效于uint8 copy_image[image_height][image_width];
uint8_t ipm_image_array[image_height][image_width];// IPM变换后的图像数据缓冲区，供后续处理使用
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

float vision_target_yaw = 0.0f; // 保存为“这帧图像给出的目标航向”

//==============================//========================//===============================

// 全局查表数组
int valid_l_bound[image_height];// 每一行逆透视有效区域的左边界，若 left > right 说明该行无效
int valid_r_bound[image_height];
uint8 start_point_l[2] = {0}; // start_point_l[0]：左起点 x
uint8 start_point_r[2] = {0};
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

// 搜到的左边界点坐标数组，第一维是点的索引，第二维0/1分别是x/y坐标
uint16 points_l[k_max_search_points][2] = {{0}};
uint16 points_r[k_max_search_points][2] = {{0}};
uint8 dir_l[k_max_search_points] = {0};
uint8 dir_r[k_max_search_points] = {0};
uint16 g_left_point_count = 0;// 搜到的左边界点数量
uint16 g_right_point_count = 0;
Track_Corner_Point_TypeDef g_left_upper_corner = {0, 0, 0};
Track_Corner_Point_TypeDef g_right_upper_corner = {0, 0, 0};
Track_Corner_Point_TypeDef g_left_lower_corner = {0, 0, 0};
Track_Corner_Point_TypeDef g_right_lower_corner = {0, 0, 0};
static constexpr int k_corner_mid_step = 6;
static constexpr int k_corner_end_step = 12;
static constexpr int k_corner_upper_scan_start = 7;
static constexpr int k_corner_top_guard_row = 5;
static constexpr int k_corner_vertical_delta_min = 4;
static constexpr int k_corner_near_flat_delta_max = 4;
static constexpr int k_corner_region_upper_max_row = (image_height * 2) / 3;
static constexpr int k_corner_region_lower_min_row = image_height / 3;

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

static void reset_track_search_results(void)
{
    g_left_point_count = 0;
    g_right_point_count = 0;

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

static void search_l_r(uint16 *l_stastic, uint16 *r_stastic,
                       uint8 l_start_x, uint8 l_start_y,
                       uint8 r_start_x, uint8 r_start_y,
                       uint8 *highest)
{
    uint16 i = 0;
    uint16 j = 0;

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
                    center_point_l[0] = temp_l[0][0];
                    center_point_l[1] = temp_l[0][1];
                    for (j = 1; j < index_l; ++j)
                    {
                        if (center_point_l[1] > temp_l[j][1])
                        {
                            center_point_l[0] = temp_l[j][0];
                            center_point_l[1] = temp_l[j][1];
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
                    center_point_r[0] = temp_r[0][0];
                    center_point_r[1] = temp_r[0][1];
                    for (j = 1; j < index_r; ++j)
                    {
                        if (center_point_r[1] > temp_r[j][1])
                        {
                            center_point_r[0] = temp_r[j][0];
                            center_point_r[1] = temp_r[j][1];
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

static void convert_points_to_line_arrays(void)// 把搜到的离散点转换成每行一个边界坐标的形式，方便后续处理使用
{
    int left_by_row[image_height];
    int right_by_row[image_height];

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
    }

    int last_left = (start_point_l[0] < image_width) ? start_point_l[0] : (image_width / 2 - 1);
    int last_right = (start_point_r[0] < image_width) ? start_point_r[0] : (image_width / 2 + 1);

    for (int y = image_height - 1; y >= 0; --y)
    {
        if (left_by_row[y] >= 0)
        {
            last_left = left_by_row[y];
        }

        if (right_by_row[y] >= 0)
        {
            last_right = right_by_row[y];
        }

        const int row_left = valid_l_bound[y];
        const int row_right = valid_r_bound[y];
        if (row_left <= row_right)
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
// 当前主流程实际使用的是 Mat1，Mat2 更多是留给调试、画点或验证矩阵用。
double Mat1[3][3] = {
    {-0.0290303853733795, 0.0210777262159344, -0.819278828256909},
    {-0.00144841552112371, 0.00125159250605812, -0.837918472458018},
    {-2.06916503017672E-05, 0.000275047546694224, -0.0370441101329347},
};

double Mat2[3][3] = {
    {-31.71511450107, -95.6888683012815, 2865.85348720067},
    {6.25631869614484, -182.337557790208, 3986.01391462335},
    {0.0641673712425111, -1.30038316933205, 1},
};

// src_w / src_h: 输入图像尺寸。这里就是缩放后的算法处理尺寸 160x120。
// image_w / image_h: 逆透视结果图尺寸。当前与输入一致，便于直接复用现有缓冲区。
static constexpr int src_w = image_width;   // 输入图像宽度
static constexpr int src_h = image_height;  // 输入图像高度
static constexpr int image_w = image_width; // 逆透视输出宽度
static constexpr int image_h = image_height;// 逆透视输出高度

double Tx = 0;
double Ty = 0;
// 把“逆透视结果图”中的点 (x, y) 映射回“原图”中的采样点坐标。
// 这里使用的是齐次坐标透视变换：
// [X']   [m00 m01 m02] [x]
// [Y'] = [m10 m11 m12] [y]
// [W']   [m20 m21 m22] [1]
//
// 最终真实坐标为：
// X = X' / W'
// Y = Y' / W'
//
// 注意这里的 x、y 不是原图坐标，而是 dst_mat 中的列、行坐标。
// 经过 Mat1 变换之后，mapped_x、mapped_y 才是输入图 src_mat 里的采样位置。
bool Transform_Point1(int x, int y, double &mapped_x, double &mapped_y)
{
    const double w = 1.0;
    const double transformedX = Mat1[0][0] * x + Mat1[0][1] * y + Mat1[0][2] * w;
    const double transformedY = Mat1[1][0] * x + Mat1[1][1] * y + Mat1[1][2] * w;
    const double transformedW = Mat1[2][0] * x + Mat1[2][1] * y + Mat1[2][2] * w;

    // 透视变换最怕分母接近 0。
    // 一旦 transformedW 非常接近 0，除法结果会突然变成一个非常大的值，
    // 通常意味着这个点被映射到了原图很远的位置，已经没有可采样意义。
    // 这里直接把它记成无效点，后面不会写入 dst_mat。
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

// 保留原来的两参数接口，兼容旧的调用方式。
void Transform_Point1(int x, int y)
{
    if (!Transform_Point1(x, y, Tx, Ty))
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
    // Threshold = otsuThreshold(&ipm_image_array[0][0], image_width, image_height);

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

    if (best_white_length <= 0)
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
            break;
        }
    }
}

static void detect_left_lower_corner_point(void)
{
    if (g_left_point_count <= k_corner_end_step)
    {
        return;
    }

    for (uint16 i = 0; i + k_corner_end_step < g_left_point_count; ++i)
    {
        const int ax = points_l[i][0];
        const int ay = points_l[i][1];
        const int bx = points_l[i + k_corner_mid_step][0];
        const int by = points_l[i + k_corner_mid_step][1];
        const int cx = points_l[i + k_corner_end_step][0];
        const int cy = points_l[i + k_corner_end_step][1];

        if (cy <= k_corner_top_guard_row)
        {
            continue;
        }

        if (!is_corner_point_in_target_region(bx, by, true, false))
        {
            continue;
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

    for (uint16 i = 0; i + k_corner_end_step < g_right_point_count; ++i)
    {
        const int ax = points_r[i][0];
        const int ay = points_r[i][1];
        const int bx = points_r[i + k_corner_mid_step][0];
        const int by = points_r[i + k_corner_mid_step][1];
        const int cx = points_r[i + k_corner_end_step][0];
        const int cy = points_r[i + k_corner_end_step][1];

        if (cy <= k_corner_top_guard_row)
        {
            continue;
        }

        if (!is_corner_point_in_target_region(bx, by, false, false))
        {
            continue;
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

    for (uint16 i = k_corner_upper_scan_start; i + k_corner_end_step < g_left_point_count; ++i)
    {
        const int ax = points_l[i][0];
        const int ay = points_l[i][1];
        const int bx = points_l[i + k_corner_mid_step][0];
        const int by = points_l[i + k_corner_mid_step][1];
        const int cx = points_l[i + k_corner_end_step][0];
        const int cy = points_l[i + k_corner_end_step][1];

        if (cy <= k_corner_top_guard_row)
        {
            continue;
        }

        if (!is_corner_point_in_target_region(bx, by, true, true))
        {
            continue;
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

    for (uint16 i = k_corner_upper_scan_start; i + k_corner_end_step < g_right_point_count; ++i)
    {
        const int ax = points_r[i][0];
        const int ay = points_r[i][1];
        const int bx = points_r[i + k_corner_mid_step][0];
        const int by = points_r[i + k_corner_mid_step][1];
        const int cx = points_r[i + k_corner_end_step][0];
        const int cy = points_r[i + k_corner_end_step][1];

        if (cy <= k_corner_top_guard_row)
        {
            continue;
        }

        if (!is_corner_point_in_target_region(bx, by, false, true))
        {
            continue;
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

static void fill_debug_image(void)
{
    std::lock_guard<std::mutex> lock(g_ipm_image_mutex);

    // 灰度底图直接产出大端序 RGB565，与后面叠加标记的字节序一致，
    // 省去整帧 19200 像素的独立字节交换遍历。
    dbg_from_gray(debug_image, bin_image, valid_l_bound, valid_r_bound,
                  k_debug_invalid_fill_color, true);

    // 叠加标记颜色在写入前交换一次字节序，与底图大端序对齐。
    dbg_cross(debug_image, start_point_l[0], start_point_l[1],
              swap_rgb565_bytes(RGB565_GREEN), 2);
    dbg_cross(debug_image, start_point_r[0], start_point_r[1],
              swap_rgb565_bytes(RGB565_BLUE), 2);
    dbg_trace_points(debug_image, points_l, g_left_point_count,
                     swap_rgb565_bytes(RGB565_RED), 1);
    dbg_trace_points(debug_image, points_r, g_right_point_count,
                     swap_rgb565_bytes(RGB565_BLUE), 1);

    if (g_left_upper_corner.flag)
    {
        dbg_cross(debug_image, g_left_upper_corner.col, g_left_upper_corner.row,
                  swap_rgb565_bytes(RGB565_YELLOW), 2);
    }

    if (g_right_upper_corner.flag)
    {
        dbg_cross(debug_image, g_right_upper_corner.col, g_right_upper_corner.row,
                  swap_rgb565_bytes(RGB565_CYAN), 2);
    }

    if (g_left_lower_corner.flag)
    {
        dbg_cross(debug_image, g_left_lower_corner.col, g_left_lower_corner.row,
                  swap_rgb565_bytes(RGB565_MAGENTA), 2);
    }

    if (g_right_lower_corner.flag)
    {
        dbg_cross(debug_image, g_right_lower_corner.col, g_right_lower_corner.row,
                  swap_rgb565_bytes(RGB565_BROWN), 2);
    }
}

//=======================================================测试元素
uint8_t line_lost = 0; // 0都不丢线，1左边丢线，2右边丢线，3都丢线
#define k_lost_line_ratio_num 1//丢线阈值分子
#define k_lost_line_ratio_den 7
// // 辅助函数

void lost_line_check(void)
{
    int16 left_overlap_count = 0; // 左边丢线点数
        uint16 right_overlap_count = 0;
        float left_overlap_ratio = 0.0f;//
        float right_overlap_ratio = 0.0f;

        uint8 left_lost = 0;
        uint8 right_lost = 0;

       line_lost = 0;

        if (g_left_point_count == 0)
        {
            left_lost = 1;
        }
        else
        {
            for (uint16 i = 0; i < g_left_point_count && i < k_max_search_points; ++i)
            {
                const int x = points_l[i][0];
                const int y = points_l[i][1];

                if (y < 0 || y >= image_height)
                {
                    continue;
                }

                if (valid_l_bound[y] > valid_r_bound[y])
                {
                    continue;
                }

                if (x == valid_l_bound[y])
                {
                    ++left_overlap_count;
                }
            }

            left_overlap_ratio = static_cast<float>(left_overlap_count) /
                                 static_cast<float>(g_left_point_count);

            if (left_overlap_count * k_lost_line_ratio_den >
                g_left_point_count * k_lost_line_ratio_num)
            {
                left_lost = 1;
            }
        }

        if (g_right_point_count == 0)
        {
            right_lost = 1;
        }
        else
        {
            for (uint16 i = 0; i < g_right_point_count && i < k_max_search_points; ++i)
            {
                const int x = points_r[i][0];
                const int y = points_r[i][1];

                if (y < 0 || y >= image_height)
                {
                    continue;
                }

                if (valid_l_bound[y] > valid_r_bound[y])
                {
                    continue;
                }

                if (x == valid_r_bound[y])
                {
                    ++right_overlap_count;
                }
            }

            right_overlap_ratio = static_cast<float>(right_overlap_count) /
                                  static_cast<float>(g_right_point_count);

            if (right_overlap_count * k_lost_line_ratio_den >
                g_right_point_count * k_lost_line_ratio_num)
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

void track_element_update(void)
{
    lost_line_check();
    detect_track_corner_points();
}

    //===========================================================================
    void image_process(void)
{
    if (!g_ipm_valid_region_initialized)
    {
        init_ipm_valid_region();
    }

    static uint8 lost_frame_count = 0;

    // 阻塞式等待摄像头完成新一帧图像的采集刷新，返回<0表示采集失败/异常
    if (uvc_dev.wait_image_refresh() < 0)
    {
        lost_frame_count++;
        std::cout << "摄像头采集异常，连续丢帧: " << static_cast<int>(lost_frame_count) << std::endl;
        if (lost_frame_count >= k_max_lost_frame_count)
        {
            std::cout << "摄像头连续多次丢帧，程序退出" << std::endl;
            exit(0);
        }
        return;
    }
    lost_frame_count = 0;

    // 获取摄像头采集到的灰度图像数据指针
    rgay_image = uvc_dev.get_gray_image_ptr();
    // memcpy(copy_image, rgay_image, image_width * image_height * sizeof(uint8));
    //  (零拷贝，直接在原始数据上构建 Mat 对象)
    //  参数含义：高120, 宽160, 8位无符号单通道(灰度图), 数组首地址
    //  直接零拷贝包装原图（作为只读的 src）
    cv::Mat src_mat_320(240, 320, CV_8UC1, rgay_image);
    cv::Mat src_mat(src_h, src_w, CV_8UC1, copy_image);
    // 给输出数组也“穿上” cv::Mat 的外衣。
    cv::Mat dst_mat(image_h, image_w, CV_8UC1, ipm_work_array);
    cv::resize(src_mat_320, src_mat, cv::Size(src_w, src_h), 0, 0, cv::INTER_AREA);
    Threshold = otsuThreshold(&src_mat.data[0], src_mat.cols, src_mat.rows);
    //test2 = Threshold;
    dst_mat.setTo(k_ipm_invalid_fill_value);

    for (int i = 0; i < image_h; i++)
    {
        const int row_left = valid_l_bound[i];
        const int row_right = valid_r_bound[i];
        if (row_left > row_right)
        {
            continue;
        }

        for (int j = row_left; j <= row_right; j++)
        {
            // 把逆透视图中的点 (j, i) 映射回原图坐标。
            double mapped_x = 0.0;
            double mapped_y = 0.0;
            if (!Transform_Point1(j, i, mapped_x, mapped_y))
            {
                continue;
            }
            // 这里直接四舍五入到最近整数像素，相当于做最邻近采样。
            // 如果后续想让逆透视图更平滑，可以改成双线性插值。
            const int x = cvRound(mapped_x);
            const int y = cvRound(mapped_y);
            // 不是所有 dst_mat 中的点都能在输入图中找到合法位置。
            // 逆透视后常见情况是：
            // - 底部左右两侧会映射到原图外
            // - 靠近无穷远方向的点会被拉到极端位置
            // 所以写入前一定要做越界判断。
            if (x >= 0 && x < src_w && y >= 0 && y < src_h)
            {
                dst_mat.at<uchar>(i, j) = src_mat.at<uchar>(y, x);
            }
        }
    }

    // 旧的灰度 TCP 图传接口，先保留注释以便回退。
    // {
    //     std::lock_guard<std::mutex> lock(g_ipm_image_mutex);
    //     std::memcpy(ipm_image_array, ipm_work_array, sizeof(ipm_image_array));
    // }

    turn_to_bin();    
    image_filter(bin_image);
    draw_valid_region_box(bin_image);
    find_start_point_by_valid_box(bin_image);
    update_track_lines_from_start_points();
    track_element_update();
    fill_debug_image();
    // test1 = g_left_point_count;
    // test2 = g_right_point_count;
    test1= line_lost;
}
