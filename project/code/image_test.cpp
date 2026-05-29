#include "image_test.hpp"
#include "car_control.hpp"
#include "speed_strategy.hpp"
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

float vision_target_yaw = 0.0f;// 视觉计算的目标偏航角（供角度环使用）

extern float yaw;
extern TrackInfo g_track_info;
extern float test1, test2, test3;
extern uint8_t island_state;

namespace
{
    // constexpr 编译时常量 不可修改值 比define更安全
    constexpr uint8 k_max_lost_frame_count = 5; // 摄像头连续丢帧的最大允许次数
    constexpr uint8 k_default_left = 1;         // 默认左边界位置
    constexpr uint8 k_default_right = image_width - 2;
    constexpr uint8 k_default_mid = image_width / 2;
    constexpr int k_lost_line_count_threshold = 30;//丢线行数阈值
    constexpr uint8 k_search_top_stop_row = 0;//八邻域搜到顶行后停止，避免沿顶边横向爬线
    constexpr uint8 k_start_black_confirm_count = 2;//起点边界确认需要的连续黑点数
    constexpr int k_left_circle_short_col_fallback_x = 60;
    constexpr int k_right_circle_short_col_fallback_x = (MT9V03X_W - 1) - k_left_circle_short_col_fallback_x;
    constexpr int k_circle_short_col_fallback_y = 20;

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

float wrap_to_180_local(float angle)
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

struct PreviewYawParam
{
    uint8 near_row;
    uint8 far_row;
    float lateral_gain; // 横向纠偏力度
    float heading_gain; // 提前看弯力度
    float max_delta_yaw; // 目标角度最大限幅
};

PreviewYawParam get_preview_yaw_param(TrackScene scene)
{
    if (scene == TrackScene::Circle)
    {
        if (island_state <= 3)      // 入环
            return {95, 45, 28.0f, 26.0f, 40.0f};
        else if (island_state <= 4) // 环内
            return {85, 55, 22.0f, 16.0f, 30.0f};
        else                        // 出环
            return {80, 60, 18.0f, 12.0f, 24.0f};
    }
    if (scene == TrackScene::SharpCurve)
    {
        return {88, 50, 24.0f, 20.0f, 32.0f};
    }
    if (scene == TrackScene::GentleCurve)
    {
        return {85, 55, 21.0f, 18.0f, 30.0f};
    }
    return {85, 55, 17.0f, 13.0f, 22.0f};
}

float normalized_midline_error(uint8 row)
{
    const int safe_row = std::clamp<int>(row, 0, MT9V03X_H - 1);
    int mid = End_Mid_Line[safe_row] != 0 ? End_Mid_Line[safe_row] : Mid_Line[safe_row];
    mid = std::clamp(mid, 0, image_width - 1);
    return std::clamp((mid - image_width / 2.0f) / (image_width / 2.0f), -1.0f, 1.0f);
}

float calc_preview_delta_yaw(TrackScene scene)
{
    const PreviewYawParam param = get_preview_yaw_param(scene);
    const float near_error = normalized_midline_error(param.near_row);
    const float far_error = normalized_midline_error(param.far_row);
    const float heading_error = far_error - near_error;

    const float delta_yaw = near_error * param.lateral_gain + heading_error * param.heading_gain;
    return std::clamp(delta_yaw, -param.max_delta_yaw, param.max_delta_yaw);
}

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
float Cal_Weigth(void);
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
uint8 Road_Wide[MT9V03X_H]; //赛道宽度
uint8 bin_image_ipm[image_h][image_w];  //
uint8 sobel_image[MT9V03X_H][MT9V03X_W];
uint8_t White_Column[MT9V03X_H];//旧版白列缓存，当前有效列宽统计使用 White_col

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

float Line_Error=0;

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

void cycle_debug_view_mode(void)
{
    g_debug_show_ipm_lines = !g_debug_show_ipm_lines;
}

const char *debug_view_mode_name(void)
{
    return g_debug_show_ipm_lines ? "IPM_LINES" : "NORMAL";
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
        {1.54922851132135, 4.87986157380177, -49.7605197103311},
        {-0.244732053958449, 8.58083223015027, -50.5084935687455},
        {-0.00215511349918493, 0.0612137810224406, 1},
    };

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
}

/*
整条边线转换函数。输入原图行数组形式的 left_line / right_line，逐行把左右边界点通过 transform_image_to_ipm_point() 投影到 IPM 坐标系。输出到 ipm_left_line / ipm_right_line，格式仍然是行数组：
*/
void transform_lines_to_ipm(const uint8 left_line[MT9V03X_H], const uint8 right_line[MT9V03X_H], int16 ipm_left_line[MT9V03X_H], int16 ipm_right_line[MT9V03X_H])
{
    int16 left_source_row[MT9V03X_H];
    int16 right_source_row[MT9V03X_H];
    for (int row = 0; row < MT9V03X_H; ++row)
    {
        ipm_left_line[row] = -1;
        ipm_right_line[row] = -1;
        left_source_row[row] = -1;
        right_source_row[row] = -1;
    }

    for (int row = std::max<int>(hightest, 0); row < MT9V03X_H; ++row)
    {
        const int left = left_line[row];
        const int right = right_line[row];
        if (left > Border_Min && left < Border_Max)
        {
            int ipm_x = 0;
            int ipm_y = 0;
            if (transform_image_to_ipm_point(left, row, ipm_x, ipm_y) &&
                    (ipm_left_line[ipm_y] < 0 || row > left_source_row[ipm_y]))
            {
                ipm_left_line[ipm_y] = static_cast<int16>(ipm_x);
                left_source_row[ipm_y] = static_cast<int16>(row);
            }
        }
        if (right > Border_Min && right < Border_Max)
        {
            int ipm_x = 0;
            int ipm_y = 0;
            if (transform_image_to_ipm_point(right, row, ipm_x, ipm_y) &&
                    (ipm_right_line[ipm_y] < 0 || row > right_source_row[ipm_y]))
            {
                ipm_right_line[ipm_y] = static_cast<int16>(ipm_x);
                right_source_row[ipm_y] = static_cast<int16>(row);
            }
        }
    }
}

void transform_points_to_ipm_line(const uint16 points[][2], uint16 count, int16 ipm_line[MT9V03X_H])
{
    int16 source_row[MT9V03X_H];
    for (int row = 0; row < MT9V03X_H; ++row)
    {
        ipm_line[row] = -1;
        source_row[row] = -1;
    }

    for (uint16 i = 0; i < count; ++i)
    {
        const int x = points[i][0];
        const int y = points[i][1];
        int ipm_x = 0;
        int ipm_y = 0;
        if (transform_image_to_ipm_point(x, y, ipm_x, ipm_y) &&
                (ipm_line[ipm_y] < 0 || y > source_row[ipm_y]))
        {
            ipm_line[ipm_y] = static_cast<int16>(ipm_x);
            source_row[ipm_y] = static_cast<int16>(y);
        }
    }
}

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

    // 清除图像数组的顶部边缘像素点；底部边缘当前保留。
    for (i = 0; i < MT9V03X_W - 1; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        image[2][i] = 0;
        image[3][i] = 0;
//    image[MT9V03X_H-1][i] = 0;
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

        if(Right_Line[i] >= MT9V03X_W - 3)
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
uint8 get_start_point(uint8 start_row)
{
    const int row = std::clamp<int>(start_row, 1, MT9V03X_H - 2);
    const int search_left = border_min + 1;
    const int search_right = border_max - 1;

    int best_white_left = -1;
    int best_white_right = -1;
    int best_white_length = 0;

    start_point_l[0] = 0;
    start_point_l[1] = static_cast<uint8>(row);
    start_point_r[0] = 0;
    start_point_r[1] = static_cast<uint8>(row);
    Image_Flag.L_Find = false;
    Image_Flag.R_Find = false;

    if (search_left > search_right)
    {
        Image_Flag.Get_Start_Point = 0;
        return 0;
    }

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

    if (best_white_length <= 0)
    {
        Image_Flag.Get_Start_Point = 0;
        return 0;
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
    return Image_Flag.Get_Start_Point ? 1 : 0;
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

bool Get_K_b(uint8 x1,uint8 y1,uint8 x2,uint8 y2, float* slope_rate, float* intercept)
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

// 圆环检测与补线状态机：根据拐点、丢线和 yaw 积分推进 island_state。
void Island_Detect(void)//环岛检测
{
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

    if(Image_Flag.Cross_Fill==0&&island_state==0)//互斥
    {
        continuity_change_left_flag=Continuity_Change_Left(MT9V03X_H-10,15);//连续性判断
        continuity_change_right_flag=Continuity_Change_Right(MT9V03X_H-10,15);
        if(Image_Flag.Left_Circle==0)//左环
        {
            if(monotonicity_change_right_flag==0&&//右边是单调的
                    continuity_change_left_flag!=0&&//左边是不连续的
                    continuity_change_right_flag==0&&//左环岛右边是连续的
                    Left_Lost_Time>=20&& //左边丢线很多
                    Left_Lost_Time<=70&& //也不能全丢了
                    hightest<30//&&//搜索截止行看到很远
              )
            {
                left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);//找左下角点
                left_down_guai[1]=Left_Line[left_down_guai[0]];
                if(left_down_guai[0]>=5)//条件1很松，在这里判断拐点，位置不对，则是误判，跳出
                {
                    island_state=1;
                    reset_circle_integrate_yaw();
                    short_col_point[0]=0;
                    short_col_point[1]=0;
                    short_col_point_is_fallback=false;
                    Image_Flag.Left_Circle=1;
                }
                else//误判，归零
                {
                    island_state=0;
                    Image_Flag.Left_Circle=0;
                }
            }
        }
        if(Image_Flag.Right_Circle==0)//右环
        {
            if(monotonicity_change_left_flag==0&&
                    continuity_change_left_flag==0&&//右环岛左边是连续的
                    continuity_change_right_flag!=1&&//右边是真实不连续行号，排除连续和大部分丢线异常
                    Right_Lost_Time>=20&&           //右丢线多
                    Right_Lost_Time<=70&&           //右丢线不能太多

                    hightest<30//&&//搜索截止行看到很远
              )
            {
                right_down_guai[0]=Find_Right_Down_Point(MT9V03X_H-1,20);//右下点
                right_down_guai[1]=Right_Line[right_down_guai[0]];
                if(right_down_guai[0]>=5)//条件1很松，在这里加判拐点，位置不对，则是误判，跳出
                {
                    island_state=1;
                    reset_circle_integrate_yaw();
                    short_col_point[0]=0;
                    short_col_point[1]=0;
                    short_col_point_is_fallback=false;
                    Image_Flag.Right_Circle=1;
                }
                else
                {
                    island_state=0;
                    Image_Flag.Right_Circle=0;
                }
            }
        }
    }

    if(Image_Flag.Left_Circle == 1)
    {
        if(island_state == 1)
        {
            left_down_guai[0]=L_D_corner_row;
            left_down_guai[1]=L_D_corner_col;

            if(L_D_corner_flag!=0)
            {
                Left_Add_Line(left_down_guai[1]+1,left_down_guai[0]-1,left_down_guai[1]+30,0);
            }
            else if(L_D_corner_flag==0&&continuity_change_left_flag-monotonicity_change_left_flag>5)
            {
                island_state=2;
            }
        }
        else if(island_state==2) //2状态下方丢线，上方即将出现大弧线
        {
            monotonicity_change_line[0]=Monotonicity_Change_Left(60,0);//单调性改变
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
            if(monotonicity_change_line[0] > 0 && monotonicity_change_line[0]<=40)
            {
                Left_Add_Line(monotonicity_change_line[1]-30,MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
            }
            else if(monotonicity_change_line[0]>40)
            {
                short_col_point[0] = limit_a_b(k_circle_short_col_fallback_y, 5, MT9V03X_H - 5);
                short_col_point[1] = limit_a_b(k_left_circle_short_col_fallback_x, Border_Min, Border_Max);
                short_col_point_is_fallback = true;
                island_state=3;
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
                    Right_Add_Line(short_col_point[1],short_col_point[0],MT9V03X_W,MT9V03X_H/2);
                }
            }

            if(car.circle_intergrate_yaw < -45 && shortest_White_Column1[0] < 40)
            {
                island_state=4;
            }
        }
        else if(island_state == 4)
        {
            if(car.circle_intergrate_yaw < -260 && R_D_corner_flag == 1)
            {
                island_state=5;
            }
        }
        else if(island_state == 5)
        {
            if(R_D_corner_flag == 1)
            {
                Right_Add_Line(R_D_corner_col,R_D_corner_row,0,0);
            }
            else
            {
                Right_Add_Line(MT9V03X_W,MT9V03X_H-30,0,0);
            }
            if(L_U_corner_flag == 1||abs(car.circle_intergrate_yaw) < 30)
            {
                island_state = 6;
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

            if(Right_Lost_Time<20&&Left_Lost_Time<20)
            {
                island_state=0;
                Image_Flag.Left_Circle =0;
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
                island_state=2;
            }
        }
        else if(island_state==2) //2状态下方丢线，上方即将出现大弧线
        {
            monotonicity_change_line[0]=Monotonicity_Change_Right(60,0);//单调性改变
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
            if(monotonicity_change_line[0] > 0 && monotonicity_change_line[0]<=40)//&&monotonicity_change_line[0]>=20)//右下角再不丢线进3
            {
                Right_Add_Line(monotonicity_change_line[1]+30,MT9V03X_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
            }
            else if(monotonicity_change_line[0]>40)
            {
                short_col_point[0] = limit_a_b(k_circle_short_col_fallback_y, 5, MT9V03X_H - 5);
                short_col_point[1] = limit_a_b(k_right_circle_short_col_fallback_x, Border_Min, Border_Max);
                short_col_point_is_fallback = true;
                island_state=3;
            }
        }
        else if(island_state==3) //下面已经出现大弧线，且上方出现角点//根本不进3里头，直接瞬间调到4里
        {

            shortest_White_Column1[0]=shortest_White_Column(19,114,80,15,true);

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
                    Left_Line[i] = (i - intercept_l) / slope_l_rate ;//y = kx+b
                    Left_Line[i] = limit_a_b(Left_Line[i], Border_Min, Border_Max);
                }
            }


            //=========================================================================
//                            else if(shortest_White_Column1[0]<30)
//                        else if(MT9V03X_H-shortest_White_Column1[0]>50)//拐点出现在一定范围内，认为是拐点出现

            else
            {
                if(short_col_point[0] > 0)
                {
                    Left_Add_Line(short_col_point[1],short_col_point[0],0,MT9V03X_H/2);
                }
            }
            if(car.circle_intergrate_yaw > 45 && shortest_White_Column1[0] < 40)
            {
                island_state=4;
            }
        }
        else if(island_state == 4)
        {
            if(car.circle_intergrate_yaw >260 && L_D_corner_flag == 1)
            {

                island_state=5;
            }
        }
        else if(island_state == 5)
        {
            if(L_D_corner_flag == 1)
            {
                Left_Add_Line(L_D_corner_col,L_D_corner_row,MT9V03X_W,0);
            }
            else
            {
                Left_Add_Line(0,MT9V03X_H-30,MT9V03X_W,0);
            }
            if(R_U_corner_flag == 1 || abs(car.circle_intergrate_yaw) < 30)
            {
                island_state = 6;
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

//       Get_K_b(start_point_r[0], start, MT9V03X_W-2, end, &slope_l_rate, &intercept_l);
                {
                    for (i = 3  ; i < MT9V03X_H-5; i++)
                    {
                        Right_Line[i] = (i - intercept_l) / slope_l_rate-20 ;//y = kx+b
                        Right_Line[i] = limit_a_b(Right_Line[i], Border_Min, Border_Max);
                    }
                }

                // K_Add_Boundry_Right(Get_Left_K(40,MT9V03X_H-10),MT9V03X_W-20,MT9V03X_H,10);
                //Bu_right();
            }

//            if(continuity_change_right_flag == 0&&continuity_change_left_flag ==0)
            if(Right_Lost_Time<20&&Left_Lost_Time<20)
            {

                island_state=0;
                Image_Flag.Right_Circle =0;
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
    Threshold = otsuThreshold(image_copy[0], MT9V03X_W, MT9V03X_H);	//大津法计算阈值
    turn_to_bin();					//图像二值化
    image_filter(bin_image);		//滤波
    image_draw_rectan(bin_image);//画方
//	Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-5);//测试画线
//	Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-5);
    data_stastics_l = 0;
    data_stastics_r = 0;
    transform_points_to_ipm_line(points_l, 0, Ipm_Left_Line);
    transform_points_to_ipm_line(points_r, 0, Ipm_Right_Line);
    //check_cheku(90,30,4);									//找斑马线
    if(get_start_point(MT9V03X_H - 2))
    {
        search_l_r((uint16)USE_num,bin_image,&data_stastics_l, &data_stastics_r,start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1],&hightest);		//八邻域找边界
        transform_points_to_ipm_line(points_l, data_stastics_l, Ipm_Left_Line);
        transform_points_to_ipm_line(points_r, data_stastics_r, Ipm_Right_Line);

        get_left(data_stastics_l);		//取出边界
        get_right(data_stastics_r);

        get_turn_point();//获取拐点（突变点或向量法）
//        get_right_cusp_point();
//        get_left_cusp_point();
        get_lost_line();						//找左右丢线行数
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
    build_test_midline(g_test_midline_mode);
//   print_road_width_calibration();
    fit_midline();
    HDPJ_lvbo();
    Line_Error = Cal_Weigth();                  // 加权中线偏差（已归一化到 [-1, 1]）

//    clear_block();
}

// 计算当前中线偏差的统一接口，当前转到 car_control.cpp 的 Cal_Weigth1。
float Cal_Weigth(void)
{
    return Cal_Weigth1();
}

namespace
{
uint16 debug_color(uint16 color)
{
    return static_cast<uint16>((color << 8) | (color >> 8));
}

void draw_debug_status_bar(uint16 (*img)[image_width])
{
    const uint16 lost_color = debug_color(RGB565_RED);
    const uint16 ok_color = debug_color(RGB565_GREEN);
    const uint16 cross_color = debug_color(RGB565_YELLOW);
    const uint16 circle_color = debug_color(RGB565_CYAN);
    const uint16 mode_color = debug_color(RGB565_PURPLE);
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
    if (Image_Flag.Left_Circle || Image_Flag.Right_Circle)
    {
        dbg_fill_rect(img, 48, 1, 58, 6, circle_color);
    }
    if (island_state != 0)
    {
        const int state_count = std::clamp<int>(island_state, 0, 6);
        const int block_width = 4;
        const int block_gap = 2;
        const int block_x0 = 62;
        for (int block = 0; block < state_count; ++block)
        {
            const int x0 = block_x0 + block * (block_width + block_gap);
            dbg_fill_rect(img, x0, 1, x0 + block_width - 1, 6, mode_color);
        }
    }
}

void draw_debug_track_lines(uint16 (*img)[image_width])
{
    const uint16 left_color = debug_color(RGB565_RED);
    const uint16 right_color = debug_color(RGB565_BLUE);
    const uint16 mid_color = debug_color(RGB565_GREEN);
    const uint16 test_mid_color = debug_color(RGB565_YELLOW);
    const uint16 center_color = debug_color(RGB565_GRAY);

    dbg_line(img, image_width / 2, 0, image_width / 2, image_height - 1, center_color);

    for (int row = 0; row < image_height; ++row)
    {
        dbg_point(img, left_edge_line[row], row, left_color);
        dbg_point(img, right_edge_line[row], row, right_color);
        dbg_point(img, mid_line[row], row, mid_color);
        if (Test_Mid_Line[row] > 0)
        {
            dbg_cross(img, Test_Mid_Line[row], row, test_mid_color, 1);
        }
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
    for (int row = 0; row < image_height; ++row)
    {
        for (int col = 0; col < image_width; ++col)
        {
            img[row][col] = bg_color;
        }
    }

    for (int row = 0; row < MT9V03X_H; ++row)
    {
        if (Ipm_Left_Line[row] >= 0)
        {
            dbg_point(img, Ipm_Left_Line[row], row, line_color);
        }
        if (Ipm_Right_Line[row] >= 0)
        {
            dbg_point(img, Ipm_Right_Line[row], row, line_color);
        }
    }
}

// 构建 SCC8660 彩色调试图。只写 debug_image，不回写算法使用的灰度图/二值图。
void build_debug_image(bool show_binary)
{
    if (g_debug_show_ipm_lines)
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
    dbg_text_6x8(debug_image, 92, 0, test_midline_mode_name(g_test_midline_mode), debug_color(RGB565_YELLOW), debug_color(RGB565_BLACK), false);
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
        //Road_Wide[row] = Standard_Road_Wide[row];//暂时用不上
    }
    //update_track_lines();
}

// 发布摄像头异常/丢线时的控制结果：保持当前 yaw 并标记 LostLine。
void publish_lost_line_result(void)
{
    std::lock_guard<std::mutex> lock(g_vision_result_mutex);
    g_track_info.scene = TrackScene::LostLine;
    g_track_info.deviation = 0.0f;
    vision_target_yaw = yaw;
}

// 将视觉中线偏差转换成目标 yaw 和赛道场景，供 scheduler 控制环使用。
void update_control_target(void)
{
    const float deviation = std::clamp(Line_Error, -1.0f, 1.0f);

    TrackScene scene = TrackScene::Straight;
    const float abs_deviation = std::fabs(deviation);
    if (Image_Flag.Left_Circle || Image_Flag.Right_Circle)
    {
        scene = TrackScene::Circle;
    }
    else if (Both_Lost_Time > 55)
    {
        scene = TrackScene::LostLine;
    }
    else if (abs_deviation > 0.55f)
    {
        scene = TrackScene::SharpCurve;
    }
    else if (Image_Flag.Cross_Fill || abs_deviation > 0.25f)
    {
        scene = TrackScene::GentleCurve;
    }

    const float delta_yaw = calc_preview_delta_yaw(scene);
    const float target_yaw = wrap_to_180_local(yaw + delta_yaw);
    {
        std::lock_guard<std::mutex> lock(g_vision_result_mutex);
        g_track_info.deviation = deviation;
        g_track_info.scene = scene;
        vision_target_yaw = (scene == TrackScene::LostLine) ? yaw : target_yaw;
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
void image_test(void)
{
    static uint8 lost_frame_count = 0;

    if (uvc_dev.wait_image_refresh() < 0)
    {
        lost_frame_count++;
        std::cout << "摄像头采集异常，连续丢帧: " << static_cast<int>(lost_frame_count) << std::endl;
        publish_lost_line_result();
        if (lost_frame_count >= k_max_lost_frame_count)
        {
            std::cout << "摄像头连续多次丢帧，程序退出" << std::endl;
            exit(0);
        }
        return;
    }
    lost_frame_count = 0;

    rgay_image = uvc_dev.get_gray_image_ptr();
    if (rgay_image == nullptr)
    {
        std::lock_guard<std::mutex> lock(g_image_mutex);
        reset_track_lines();//
        publish_lost_line_result();
        return;//图像异常，跳出循环，保持上次结果不变，等待下一帧图像
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
        update_control_target();
        build_debug_image(g_debug_show_binary_image);
    }
}
