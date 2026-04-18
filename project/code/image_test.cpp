#include "image_test.hpp"
#include "math.h"

zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
uint8 *rgay_image;     // 灰度图像数据指针，指向摄像头采集到的灰度图像缓冲区首地址

uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

// 灰度图像数据拷贝缓冲区，二维数组存储，大小与摄像头采集分辨率一致，用于中转图像数据
uint8 bin_image[MT9V03X_H][MT9V03X_W]; // 二值化图像数组
uint8 Left_Line[MT9V03X_H];            // 左边线数组
uint8 Right_Line[MT9V03X_H];           // 右边线数组
uint8 Mid_Line[MT9V03X_H];             // 中线数组

uint8 Threshold = 0; // 二值化阈值，动态

int Longest_White_Column_Left[2];  // 最长白列,[0]是最长白列的长度，[1]是第某列
int Longest_White_Column_Right[2]; // 最长白列,[0]是最长白列的长度，[1]是第某列
uint8 White_col[MT9V03X_W];        // 每一列白列长度

// 智能车更关注近处赛道，下方区域的阈值比整帧更稳定。
static constexpr uint16 k_otsu_roi_start_row = MT9V03X_H / 3;
static constexpr uint8 k_threshold_default = 110;
static constexpr uint8 k_threshold_min_limit = 40;
static constexpr uint8 k_threshold_max_limit = 220;
static constexpr uint8 k_max_lost_frame_count = 5; // 连续丢失有效图像帧的最大容忍次数，超过后可触发安全机制
static constexpr uint16 k_max_search_points = USE_num;
static constexpr uint8 k_mid_overlay_value = 128;
static constexpr uint8 k_mid_overlay_half_width = 1;

uint8 start_point_l[2] = {0}; // 左边起点的x，y值
uint8 start_point_r[2] = {0}; // 右边起点的x，y值
uint16 points_l[k_max_search_points][2] = {{0}}; // 左线
uint16 points_r[k_max_search_points][2] = {{0}}; // 右线
uint16 dir_r[k_max_search_points] = {0};         // 用来存储右边生长方向
uint16 dir_l[k_max_search_points] = {0};         // 用来存储左边生长方向
uint16 data_stastics_l = 0;                  // 统计左边找到点的个数
uint16 data_stastics_r = 0;                  // 统计右边找到点的个数

static uint8 g_last_threshold = k_threshold_default;

/**
 * 计算整数的绝对值
 *
 * @param value 待处理的整数
 * @return 整数的绝对值
 */
int my_abs(int value)
{
    // 如果输入值为非负数，则直接返回该值
    if (value >= 0)
        return value;
    // 如果输入值为负数，则返回其相反数，即正数
    else
        return -value;
}

/**
 * 限制输入值x在a和b之间
 * 如果x小于a，则将x设置为a；如果x大于b，则将x设置为b
 * 此函数的目的是确保x的值不会超出[a, b]的范围
 *
 * @param x 需要限制的值
 * @param a 下限值
 * @param b 上限值
 * @return 限制后的x值
 */
int16 limit_a_b(int16 x, int a, int b)
{
    // 检查x是否小于下限a，如果是，则将x设置为a
    if (x < a)
        x = a;
    // 检查x是否大于上限b，如果是，则将x设置为b
    if (x > b)
        x = b;
    // 返回限制后的x值
    return x;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     大津法求阈值
  @param     image       图像数组
             col         列
             row         行
  @return    threshold   返回int阈值数值
  Sample     threshold=otsuThreshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
  @note      no
-------------------------------------------------------------------------------------------------------------------*/
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
    uint8 i, j;
    //    Threshold = otsuThreshold(image_copy[0], MT9V03X_W, MT9V03X_H);

    for (i = 0; i < MT9V03X_H; i++)
    {
        for (j = 0; j < MT9V03X_W; j++)
        {
            if (image_copy[i][j] > Threshold)
                bin_image[i][j] = 255;
            else
                bin_image[i][j] = 0;
        }
    }
}

void image_filter(uint8 (*bin_image)[MT9V03X_W]) // 形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;

    for (i = 1; i < MT9V03X_H - 1; i++)
    {
        for (j = 1; j < (MT9V03X_W - 1); j++)
        {
            // 统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1] + bin_image[i][j - 1] + bin_image[i][j + 1] + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = 255; // 白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {

                bin_image[i][j] = 0; // 黑
            }
        }
    }
}

/**
 * 在图像上绘制矩形框。
 * 该函数通过将图像数组的边缘像素点设置为0，从而在图像上绘制一个矩形框。
 *
 * @param image 指向图像数组的指针，图像数组的类型为uint8，大小为MT9V03X_W。
 *              图像数组由调用该函数的外部程序提供。
 */
void image_draw_rectan(uint8 (*image)[MT9V03X_W])
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

    // 清除图像数组的顶部和底部边缘像素点。
    for (i = 0; i < MT9V03X_W - 1; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        image[2][i] = 0;
        image[3][i] = 0;
        //    image[MT9V03X_H-1][i] = 0;
    }
}

static void reset_search_results(void)
{
    uint16 i = 0;

    data_stastics_l = 0;
    data_stastics_r = 0;

    for (i = 0; i < k_max_search_points; i++)
    {
        points_l[i][0] = 0;
        points_l[i][1] = 0;
        points_r[i][0] = 0;
        points_r[i][1] = 0;
        dir_l[i] = 0;
        dir_r[i] = 0;
    }

    for (i = 0; i < MT9V03X_H; i++)
    {
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W - 1;
        Mid_Line[i] = MT9V03X_W / 2;
    }
}

void Get_Longest_Line(void)
{
    int16 i, j;
    uint8 start_col = 10;
    uint8 end_col = MT9V03X_W - 9;
    uint8 mid_col = MT9V03X_W / 2;

    Longest_White_Column_Left[0] = 0;         // 最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = mid_col/2; // 默认落在左半区
    Longest_White_Column_Right[0] = 0;        // 最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = mid_col + mid_col / 2;

    for (i = 0; i < MT9V03X_W - 1; i++) // 将横向数据清零
    {
        White_col[i] = 0;
    }

    for (j = start_col; j <= end_col; j++)
    {
        for (i = MT9V03X_H - 2; i >= 2; i--)
        {
            if (bin_image[i][j] == 255)
            {
                if ((bin_image[i][j] == 255 && bin_image[i - 1][j] == 0 && bin_image[i - 2][j] == 0 && bin_image[i + 1][j] == 255))
                    break;

                White_col[j]++;
            }
        }
    }

    // 从左到右找左边最长白列
    Longest_White_Column_Left[0] = 0;
    for (i = start_col; i <= mid_col; i++)
    {
        if (Longest_White_Column_Left[0] < White_col[i]) // 找最长的那一列
        {
            Longest_White_Column_Left[0] = White_col[i]; // 【0】是白列长度
            Longest_White_Column_Left[1] = i;            // 【1】是下标，第j列
        }
    }
    // 从右到左找右左边最长白列
    Longest_White_Column_Right[0] = 0;     // 【0】是白列长度          /////
    for (i = end_col; i >= mid_col; i--) // 右边只在右半区找起始列
    {
        if (Longest_White_Column_Right[0] < White_col[i]) // 找最长的那一列
        {
            Longest_White_Column_Right[0] = White_col[i]; // 【0】是白列长度
            Longest_White_Column_Right[1] = i;            // 【1】是下标，第j列
        }
    }
}

#define border_max MT9V03X_W - 2 // 边界最大值
#define border_min 1             // 边界最小值

/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
uint8 Mid_start_col = 0;

uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0, l_found = 0, r_found = 0;
    // 清零
    start_point_l[0] = 0; // x
    start_point_l[1] = 0; // y

    start_point_r[0] = 0; // x
    start_point_r[1] = 0; // y

    Mid_start_col = (Longest_White_Column_Left[1] + Longest_White_Column_Right[1]) / 2;
    //    Mid_start_col =MT9V03X_W/2;
    // 从中间往左边，先找起点
    for (i = Mid_start_col; i > border_min; i--)
    {
        start_point_l[0] = i;         // x
        start_point_l[1] = start_row; // y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            // printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = Mid_start_col; i < border_max; i++)
    {
        start_point_r[0] = i;         // x
        start_point_r[1] = start_row; // y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            // printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if (l_found && r_found)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// 安全像素读取函数，防止八邻域越界导致单片机死机
static inline uint8 get_safe_pixel(int x, int y)
{
    // 如果坐标超出图像物理边界，强制视为白底(255)，逼迫八邻域算法向内闭合转向
    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H)
    {
        return 255;
    }
    // 假设您的二值化图像数组名为 bin_image (请根据实际情况调整为 image 或 bin_image)
    return bin_image[y][x];
}

static uint8 find_start_point_from_bottom(void)
{
    int row = 0;
    int min_row = MT9V03X_H / 2;

    for (row = MT9V03X_H - 2; row >= min_row; row--)
    {
        if (get_start_point((uint8)row))
        {
            return 1;
        }
    }

    return 0;
}

void search_l_r(uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest)
{
    uint16 i = 0, j = 0;

    // ===== 左边变量 ====
    uint8 search_filds_l[8][2] = {{0}};
    uint8 index_l = 0;
    uint8 temp_l[8][2] = {{0}};
    uint8 center_point_l[2] = {0};
    uint16 l_data_statics;
    // 左八邻域 (顺时针)
    static int8 seeds_l[8][2] = {
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}};

    // ==== 右边变量 ====
    uint8 search_filds_r[8][2] = {{0}};
    uint8 center_point_r[2] = {0};
    uint8 index_r = 0;
    uint8 temp_r[8][2] = {{0}};
    uint16 r_data_statics;
    // 右八邻域 (逆时针)
    static int8 seeds_r[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

    l_data_statics = *l_stastic;
    r_data_statics = *r_stastic;

    // 初始化起点
    center_point_l[0] = l_start_x;
    center_point_l[1] = l_start_y;
    center_point_r[0] = r_start_x;
    center_point_r[1] = r_start_y;

    // 引入独立状态机，废弃原有的 continue 逻辑
    uint8 left_run = 1;
    uint8 right_run = 1;
    uint8 left_active = 1;
    uint8 right_active = 1;
    int break_flag = k_max_search_points; // 最大搜索步数与缓冲区大小保持一致

    // 开启邻域循环
    while (break_flag-- && (left_active || right_active))
    {
        // ================= 【左侧搜索模块】 =================
        if (left_active && left_run)
        {
            if (l_data_statics >= k_max_search_points)
            {
                left_active = 0;
                left_run = 0;
            }
            else
            {
            for (i = 0; i < 8; i++)
            {
                search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
                search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
            }

            points_l[l_data_statics][0] = center_point_l[0];
            points_l[l_data_statics][1] = center_point_l[1];
            l_data_statics++;

            index_l = 0;

            // 左边判断边缘 (使用 get_safe_pixel 替代直接访问)
            for (i = 0; i < 8; i++)
            {
                uint8 p1 = get_safe_pixel(search_filds_l[i][0], search_filds_l[i][1]);
                uint8 p2 = get_safe_pixel(search_filds_l[(i + 1) & 7][0], search_filds_l[(i + 1) & 7][1]);

                if (p1 == 0 && p2 == 255)
                {
                    temp_l[index_l][0] = search_filds_l[i][0];
                    temp_l[index_l][1] = search_filds_l[i][1];
                    index_l++;
                    dir_l[l_data_statics - 1] = i;
                }
            }

            if (index_l)
            {
                center_point_l[0] = temp_l[0][0];
                center_point_l[1] = temp_l[0][1];
                for (j = 0; j < index_l; j++)
                {
                    // 寻找 Y 值最小的点 (在图像上是最靠上的点)
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

        // ================= 【右侧搜索模块】 =================
        if (right_active && right_run)
        {
            if (r_data_statics >= k_max_search_points)
            {
                right_active = 0;
                right_run = 0;
            }
            else
            {
            for (i = 0; i < 8; i++)
            {
                search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
                search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
            }

            points_r[r_data_statics][0] = center_point_r[0];
            points_r[r_data_statics][1] = center_point_r[1];
            r_data_statics++;

            index_r = 0;

            // 右边判断边缘 (使用 get_safe_pixel)
            for (i = 0; i < 8; i++)
            {
                uint8 p1 = get_safe_pixel(search_filds_r[i][0], search_filds_r[i][1]);
                uint8 p2 = get_safe_pixel(search_filds_r[(i + 1) & 7][0], search_filds_r[(i + 1) & 7][1]);

                if (p1 == 0 && p2 == 255)
                {
                    temp_r[index_r][0] = search_filds_r[i][0];
                    temp_r[index_r][1] = search_filds_r[i][1];
                    index_r++;
                    dir_r[r_data_statics - 1] = i;
                }
            }

            if (index_r)
            {
                center_point_r[0] = temp_r[0][0];
                center_point_r[1] = temp_r[0][1];
                for (j = 0; j < index_r; j++)
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

        // ================= 【结束条件与步调同步】 =================
        if (left_active && right_active && l_data_statics >= 1 && r_data_statics >= 1)
        {
            // 判定 1：死循环/原地踏步保护
            if ((r_data_statics >= 3 &&
                 points_r[r_data_statics - 1][0] == points_r[r_data_statics - 2][0] &&
                 points_r[r_data_statics - 1][0] == points_r[r_data_statics - 3][0] &&
                 points_r[r_data_statics - 1][1] == points_r[r_data_statics - 2][1] &&
                 points_r[r_data_statics - 1][1] == points_r[r_data_statics - 3][1]) ||
                (l_data_statics >= 4 &&
                 points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
                 points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
                 points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
                 points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
            {
                break;
            }

            // 判定 2：左右两边相遇判断 (已修复 my_abs 逻辑)
            if (my_abs(points_r[r_data_statics - 1][0] - points_l[l_data_statics - 1][0]) < 2 &&
                my_abs(points_r[r_data_statics - 1][1] - points_l[l_data_statics - 1][1]) < 2)
            {
                *hightest = (points_r[r_data_statics - 1][1] + points_l[l_data_statics - 1][1]) >> 1;
                break;
            }

            // 判定 3：动态步调同步 (谁的 Y 更小即在图像上方，谁就暂停等待)
            left_run = 1;
            right_run = 1;
            int left_y = points_l[l_data_statics - 1][1];
            int right_y = points_r[r_data_statics - 1][1];

            if (left_y < right_y)
            {
                left_run = 0; // 左边靠前，左边等待
            }
            else if (right_y < left_y)
            {
                right_run = 0; // 右边靠前，右边等待
            }

            // 判定 4：应对向下倒拐的特殊保护 (替代了原来的错误 continue)
            if (dir_l[l_data_statics - 1] == 7 && (right_y > left_y))
            {
                center_point_l[0] = points_l[l_data_statics - 1][0];
                center_point_l[1] = points_l[l_data_statics - 1][1];
                l_data_statics--;
                left_run = 0; // 强制左侧等待
            }
        }
    }

    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

static void convert_points_to_line_arrays(void)
{
    int16 left_by_row[MT9V03X_H];
    int16 right_by_row[MT9V03X_H];
    int y = 0;
    uint16 i = 0;
    int last_left = 0;
    int last_right = MT9V03X_W - 1;

    for (y = 0; y < MT9V03X_H; y++)
    {
        left_by_row[y] = -1;
        right_by_row[y] = -1;
    }

    for (i = 0; i < data_stastics_l && i < k_max_search_points; i++)
    {
        uint16 x = points_l[i][0];
        uint16 py = points_l[i][1];

        if (py >= MT9V03X_H || x >= MT9V03X_W)
        {
            continue;
        }

        if (left_by_row[py] < 0 || x < (uint16)left_by_row[py])
        {
            left_by_row[py] = (int16)x;
        }
    }

    for (i = 0; i < data_stastics_r && i < k_max_search_points; i++)
    {
        uint16 x = points_r[i][0];
        uint16 py = points_r[i][1];

        if (py >= MT9V03X_H || x >= MT9V03X_W)
        {
            continue;
        }

        if (right_by_row[py] < 0 || x > (uint16)right_by_row[py])
        {
            right_by_row[py] = (int16)x;
        }
    }

    for (y = MT9V03X_H - 1; y >= 0; y--)
    {
        if (left_by_row[y] >= 0)
        {
            last_left = left_by_row[y];
        }

        if (right_by_row[y] >= 0)
        {
            last_right = right_by_row[y];
        }

        last_left = limit_a_b((int16)last_left, 0, MT9V03X_W - 1);
        last_right = limit_a_b((int16)last_right, 0, MT9V03X_W - 1);

        if (last_left >= last_right)
        {
            int center = (last_left + last_right) / 2;
            last_left = limit_a_b((int16)(center - 1), 0, MT9V03X_W - 1);
            last_right = limit_a_b((int16)(center + 1), 0, MT9V03X_W - 1);
        }

        Left_Line[y] = (uint8)last_left;
        Right_Line[y] = (uint8)last_right;
        Mid_Line[y] = (uint8)((last_left + last_right) / 2);
    }
}

static void draw_track_lines(void)
{
    uint16 y = 0;
    int x = 0;
    int mid = 0;

    if (data_stastics_l == 0 && data_stastics_r == 0)
    {
        return;
    }

    for (y = 0; y < MT9V03X_H; y++)
    {
        mid = Mid_Line[y];
        for (x = mid - k_mid_overlay_half_width; x <= mid + k_mid_overlay_half_width; x++)
        {
            if (x >= 0 && x < MT9V03X_W)
            {
                bin_image[y][x] = k_mid_overlay_value;
            }
        }
    }
}

static void update_track_lines(void)
{
    uint8 highest = MT9V03X_H - 1;

    reset_search_results();
    Get_Longest_Line();

    if (!find_start_point_from_bottom())
    {
        return;
    }

    search_l_r(&data_stastics_l, &data_stastics_r,
               start_point_l[0], start_point_l[1],
               start_point_r[0], start_point_r[1],
               &highest);

    convert_points_to_line_arrays();
    draw_track_lines();
}

void image_test(void)
{
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
    memcpy(image_copy, rgay_image, UVC_WIDTH * UVC_HEIGHT * sizeof(uint8));

    Threshold = otsuThreshold(image_copy[0], MT9V03X_W, MT9V03X_H); // 大津法计算阈值
    
    // test= Threshold;
    turn_to_bin();                                                  // 图像二值化
    image_filter(bin_image);                                        // 滤波
    image_draw_rectan(bin_image);                                   // 画方
    update_track_lines();


    seekfree_assistant_camera_send();
}
