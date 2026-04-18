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
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width = col;
    uint16 Image_Height = row;
    uint16 X;
    uint16 Y;
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

    for (Y = 0; Y < Image_Height; Y++) // Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        // Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)image_data[Y * Image_Width + X]]++; // 统计每个灰度值的个数信息
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++)
        ; // 获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--)
        ; // 获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue; // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue; // 图像中只有二个颜色
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
    return Threshold;
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

void image_test(void)
{
    // 阻塞式等待摄像头完成新一帧图像的采集刷新，返回<0表示采集失败/异常
    if (uvc_dev.wait_image_refresh() < 0)
    {
        std::cout << " 摄像头采集异常，直接退出程序，防止卡死 " << std::endl;
        exit(0); // 摄像头采集异常，直接退出程序，防止卡死
    }

    // 获取摄像头采集到的灰度图像数据指针
    rgay_image = uvc_dev.get_gray_image_ptr();
    memcpy(image_copy, rgay_image, UVC_WIDTH * UVC_HEIGHT * sizeof(uint8));

    Threshold = otsuThreshold(image_copy[0], MT9V03X_W, MT9V03X_H); // 大津法计算阈值
    
    // test= Threshold;
    turn_to_bin();                                                  // 图像二值化
    image_filter(bin_image);                                        // 滤波
    image_draw_rectan(bin_image);                                   // 画方

    seekfree_assistant_camera_send();
}
