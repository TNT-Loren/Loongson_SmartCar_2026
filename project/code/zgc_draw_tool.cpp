#include "zgc_draw_tool.hpp"

// ---- 内部工具函数，仅本文件可见 ----

// 返回两整数中较小值
static inline int imin(int a, int b) { return (a < b) ? a : b; }

// 返回两整数中较大值
static inline int imax(int a, int b) { return (a > b) ? a : b; }

// 返回整数的绝对值
static inline int iabs(int v) { return (v >= 0) ? v : -v; }

// 将 v 限制在 [lo, hi] 闭区间内
static inline int iclamp(int v, int lo, int hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

//=====================================================================================================================
// 函数简介     画点函数
// 参数说明     img             目标 RGB565 图像数组指针
// 参数说明     x               坐标x方向 [0, image_width-1]
// 参数说明     y               坐标y方向 [0, image_height-1]
// 参数说明     color           RGB565 颜色值
// 返回参数     void
// 使用示例     dbg_point(debug_image, 80, 60, RGB565_RED);
// 备注信息     所有上层绘图函数最终都调用本函数落点；坐标越界静默跳过
//=====================================================================================================================
void dbg_point(uint16 (*img)[image_width], int x, int y, uint16 color)
{
    if (x >= 0 && x < image_width && y >= 0 && y < image_height)
    {
        img[y][x] = color;
    }
}

//=====================================================================================================================
// 函数简介     十字标记
// 参数说明     img             目标 RGB565 图像数组指针
// 参数说明     x               十字中心x坐标 [0, image_width-1]
// 参数说明     y               十字中心y坐标 [0, image_height-1]
// 参数说明     color           RGB565 颜色值
// 参数说明     half            十字臂半长（像素），half=2 则十字总长5像素
// 返回参数     void
// 使用示例     dbg_cross(debug_image, start_point_l[0], start_point_l[1], RGB565_GREEN, 2);
// 备注信息     水平臂和垂直臂各 (2*half+1) 个像素，交汇于 (x,y)
//=====================================================================================================================
void dbg_cross(uint16 (*img)[image_width], int x, int y, uint16 color, int half)
{
    for (int d = -half; d <= half; ++d)
    {
        dbg_point(img, x + d, y,     color); // 水平臂
        dbg_point(img, x,     y + d, color); // 垂直臂
    }
}

//=====================================================================================================================
// 函数简介     空心正方形框
// 参数说明     img             目标 RGB565 图像数组指针
// 参数说明     x               框中心x坐标 [0, image_width-1]
// 参数说明     y               框中心y坐标 [0, image_height-1]
// 参数说明     half            框半边长（像素），half=3 则外框为 7x7
// 参数说明     color           RGB565 颜色值
// 返回参数     void
// 使用示例     dbg_rect(debug_image, corner_x, corner_y, 3, RGB565_YELLOW);
// 备注信息     只描四条边，不填充内部区域；适合框选拐点、候选元素而不遮挡内部内容
//=====================================================================================================================
void dbg_rect(uint16 (*img)[image_width], int x, int y, int half, uint16 color)
{
    int x0 = x - half;
    int y0 = y - half;
    int x1 = x + half;
    int y1 = y + half;

    // 上边 + 下边
    for (int i = x0; i <= x1; ++i)
    {
        dbg_point(img, i, y0, color);
        dbg_point(img, i, y1, color);
    }
    // 左边 + 右边（四角已由上/下边覆盖）
    for (int j = y0; j <= y1; ++j)
    {
        dbg_point(img, x0, j, color);
        dbg_point(img, x1, j, color);
    }
}

//=====================================================================================================================
// 函数简介     实心矩形填充
// 参数说明     img             目标 RGB565 图像数组指针
// 参数说明     x1              矩形左上角x坐标
// 参数说明     y1              矩形左上角y坐标
// 参数说明     x2              矩形右下角x坐标
// 参数说明     y2              矩形右下角y坐标
// 参数说明     color           RGB565 颜色值
// 返回参数     void
// 使用示例     dbg_fill_rect(debug_image, 10, 20, 30, 40, RGB565_BLUE);
// 备注信息     x1/x2、y1/y2 无需保证大小顺序，内部自动交换并裁剪到有效范围
//=====================================================================================================================
void dbg_fill_rect(uint16 (*img)[image_width], int _x1, int _y1, int _x2, int _y2, uint16 color)
{
    int x1 = iclamp(imin(_x1, _x2), 0, image_width - 1);
    int x2 = iclamp(imax(_x1, _x2), 0, image_width - 1);
    int y1 = iclamp(imin(_y1, _y2), 0, image_height - 1);
    int y2 = iclamp(imax(_y1, _y2), 0, image_height - 1);

    for (int y = y1; y <= y2; ++y)
    {
        for (int x = x1; x <= x2; ++x)
        {
            img[y][x] = color;
        }
    }
}

//=====================================================================================================================
// 函数简介     Bresenham 直线
// 参数说明     img             目标 RGB565 图像数组指针
// 参数说明     x1              线段起点x坐标 [0, image_width-1]
// 参数说明     y1              线段起点y坐标 [0, image_height-1]
// 参数说明     x2              线段终点x坐标 [0, image_width-1]
// 参数说明     y2              线段终点y坐标 [0, image_height-1]
// 参数说明     color           RGB565 颜色值
// 返回参数     void
// 使用示例     dbg_line(debug_image, 0, 60, 159, 60, RGB565_GREEN);
// 备注信息     纯整数 Bresenham 算法，无浮点、无除法；单点越界由 dbg_point 自动跳过
//=====================================================================================================================
void dbg_line(uint16 (*img)[image_width], int x1, int y1, int x2, int y2, uint16 color)
{
    int dx  = iabs(x2 - x1);
    int sx  = (x1 < x2) ? 1 : -1;
    int dy  = -iabs(y2 - y1);
    int sy  = (y1 < y2) ? 1 : -1;
    int err = dx + dy;

    while (1)
    {
        dbg_point(img, x1, y1, color);

        if (x1 == x2 && y1 == y2)
        {
            break;
        }

        int e2 = err << 1;  // e2 = 2 * err

        if (e2 >= dy)
        {
            err += dy;
            x1  += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y1  += sy;
        }
    }
}

//=====================================================================================================================
// 函数简介     空心圆
// 参数说明     img             目标 RGB565 图像数组指针
// 参数说明     cx              圆心x坐标 [0, image_width-1]
// 参数说明     cy              圆心y坐标 [0, image_height-1]
// 参数说明     r               半径（像素），r<=0 时不绘制
// 参数说明     color           RGB565 颜色值
// 返回参数     void
// 使用示例     dbg_circle(debug_image, 80, 60, 10, RGB565_CYAN);
// 备注信息     Bresenham 中点画圆算法，纯整数运算；适合标注圆环元素搜圈区域
//=====================================================================================================================
void dbg_circle(uint16 (*img)[image_width], int cx, int cy, int r, uint16 color)
{
    if (r <= 0)
    {
        return;
    }

    int x = 0;
    int y = r;
    int d = 1 - r;  // 决策参数初始值

    while (x <= y)
    {
        // 利用八对称性一次画八个点
        dbg_point(img, cx + x, cy + y, color);
        dbg_point(img, cx - x, cy + y, color);
        dbg_point(img, cx + x, cy - y, color);
        dbg_point(img, cx - x, cy - y, color);
        dbg_point(img, cx + y, cy + x, color);
        dbg_point(img, cx - y, cy + x, color);
        dbg_point(img, cx + y, cy - x, color);
        dbg_point(img, cx - y, cy - x, color);

        if (d < 0)
        {
            d += (x << 1) + 3;  // d += 2*x + 3
        }
        else
        {
            d += ((x - y) << 1) + 5;  // d += 2*(x - y) + 5
            --y;
        }
        ++x;
    }
}

//=====================================================================================================================
// 函数简介     离散点轨迹描点
// 参数说明     img             目标 RGB565 图像数组指针
// 参数说明     points          点坐标数组，每个元素为 {x, y}，x 在前 y 在后
// 参数说明     count           数组中有效点的个数
// 参数说明     color           RGB565 颜色值
// 参数说明     step            采样间隔，step=1 描所有点，step=2 隔一个描一个，step<=0 自动修正为1
// 返回参数     void
// 使用示例     dbg_trace_points(debug_image, points_l, g_left_point_count, RGB565_RED, 3);
// 备注信息     适合可视化八邻域搜线路径；越界点自动跳过不会崩溃
//=====================================================================================================================
void dbg_trace_points(uint16 (*img)[image_width],
                      const uint16 points[][2], uint16 count,
                      uint16 color, int step)
{
    if (step < 1)
    {
        step = 1;
    }

    for (uint16 i = 0; i < count; i += static_cast<uint16>(step))
    {
        int x = static_cast<int>(points[i][0]);
        int y = static_cast<int>(points[i][1]);
        dbg_point(img, x, y, color);
    }
}

//=====================================================================================================================
// 函数简介     灰度图转 RGB565 图像，感知逆透视有效区
// 参数说明     dst             目标 RGB565 图像数组指针
// 参数说明     src             源灰度图像数组指针，0=黑 255=白
// 参数说明     valid_l         每行有效区左边界数组，传 nullptr 表示整行有效
// 参数说明     valid_r         每行有效区右边界数组，传 nullptr 表示整行有效
// 参数说明     invalid_color   无效区填充颜色（RGB565），内部按 big_endian 决定是否交换字节
// 参数说明     big_endian      true 时产出大端序 RGB565（逐飞 SCC8660 图传链路专用）
// 返回参数     void
// 使用示例     dbg_from_gray(debug_image, ipm_image_array, valid_l_bound, valid_r_bound, 0x000F, true);
// 备注信息     有效区内做灰度→RGB565 转换；无效区用 invalid_color 统一填充；
//              big_endian=true 时省去调用方再做整帧字节交换的开销
//=====================================================================================================================
void dbg_from_gray(uint16 (*dst)[image_width],
                   const uint8 (*src)[image_width],
                   const int *valid_l, const int *valid_r,
                   uint16 invalid_color, bool big_endian)
{
    const uint16 fill = big_endian
        ? static_cast<uint16>((invalid_color << 8) | (invalid_color >> 8))
        : invalid_color;

    // 分支外提到双重循环外面，避免每像素判断，各自保留编译器自动向量化空间
    if (big_endian)
    {
        for (int y = 0; y < image_height; ++y)
        {
            int row_left  = 0;
            int row_right = image_width - 1;
            if (valid_l != nullptr && valid_r != nullptr)
            {
                row_left  = valid_l[y];
                row_right = valid_r[y];
            }

            for (int x = 0; x < image_width; ++x)
            {
                if (x >= row_left && x <= row_right)
                {
                    const uint8 g = src[y][x];
                    const uint16 r = (g >> 3) & 0x1F;
                    const uint16 gr = (g >> 2) & 0x3F;
                    const uint16 b = (g >> 3) & 0x1F;
                    const uint16 color = static_cast<uint16>((r << 11) | (gr << 5) | b);
                    dst[y][x] = static_cast<uint16>((color << 8) | (color >> 8));
                }
                else
                {
                    dst[y][x] = fill;
                }
            }
        }
    }
    else
    {
        for (int y = 0; y < image_height; ++y)
        {
            int row_left  = 0;
            int row_right = image_width - 1;
            if (valid_l != nullptr && valid_r != nullptr)
            {
                row_left  = valid_l[y];
                row_right = valid_r[y];
            }

            for (int x = 0; x < image_width; ++x)
            {
                if (x >= row_left && x <= row_right)
                {
                    const uint8 g = src[y][x];
                    const uint16 r = (g >> 3) & 0x1F;
                    const uint16 gr = (g >> 2) & 0x3F;
                    const uint16 b = (g >> 3) & 0x1F;
                    dst[y][x] = static_cast<uint16>((r << 11) | (gr << 5) | b);
                }
                else
                {
                    dst[y][x] = fill;
                }
            }
        }
    }
}
