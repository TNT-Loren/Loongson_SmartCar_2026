#include "image.hpp"

zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
uint8 *rgay_image;     // 灰度图像数据指针，指向摄像头采集到的灰度图像缓冲区首地址

uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

static constexpr uint8 k_max_lost_frame_count = 5; // 连续丢失有效图像帧的最大容忍次数，超过后可触发安全机制

point circle_left;
point circle_right;

uint8 reference_point;
uint8 reference_rate = 20;
uint8 reference_col;

uint8 white_point;
uint8 black_point;
uint8 reference_col_line[SEARCH_H]; // 参考列绘制
uint8 remote_distance[SEARCH_W];    // 远近距离数组 记录每列的边界位置
uint8 left_edge_line[SEARCH_H];     // 左边界线
uint8 right_edge_line[SEARCH_H];    // 右边界线
uint8 mid_line[SEARCH_H];    // 中线

uint32 if_count = 0;

uint8 left_breakpoint_flag = 0;  // 左断点标志位
uint8 right_breakpoint_flag = 0; // 右断点标志位

// 限幅函数
static int16 limit_ab(int16 value, int16 min, int16 max)
{
    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }
    return value;
}

// 寻找数组中的最大值或最小值
//  model = 1 寻找最大值   model = 0 寻找最小值
static uint8 find_max_min(uint8 *array, uint8 start, uint8 end, uint8 model)
{
    uint8 i = 0, temp = 0, temp1 = 0, temp2 = 0, value = 0;
    if (start > end)
    {
        temp1 = start - end;
        temp2 = end;
        array += start;
        value = *array;
        if (model)
        {
            for (i = 0; i <= temp1; i++)
            {
                temp = *(array - i);
                if (temp > value)
                {
                    value = temp;
                    temp2 = start - i;
                }
            }
        }
        else
        {
            for (i = 0; i <= temp1; i++)
            {
                temp = *(array - i);
                if (temp < value)
                {
                    value = temp;
                    temp2 = start - i;
                }
            }
        }
    }
    else
    {
        temp1 = end - start;
        temp2 = start;
        array += start;
        value = *array;
        if (model)
        {
            for (i = 0; i <= temp1; i++)
            {
                temp = *(array + i);
                if (temp > value)
                {
                    value = temp;
                    temp2 = start + i;
                }
            }
        }
        else
        {
            for (i = 0; i <= temp1; i++)
            {
                temp = *(array + i);
                if (temp < value)
                {
                    value = temp;
                    temp2 = start + i;
                }
            }
        }
    }
    return temp2;
}

// 获取参考点、白点、黑点
// void get_reference_point(const uint8 *iamge)
// {
//     const uint8 *p = &iamge[(SEARCH_H - REFERENCEROW) * SEARCH_W];
//     uint16 temp;
//     uint32 temp_sum = 0;
// 	uint16 i = 0;
//     temp = REFERENCEROW * SEARCH_W;
//     for(i = 0; i < temp; i++)
//     {
//         temp_sum += *(p + i);
//     }
//     reference_point = (uint8)(temp_sum / temp);
//     white_point = (uint8)limit_ab((int32)reference_point * WHITEMAXRATE/10, BLACKPOINT, 255);
//     black_point = (uint8)limit_ab((int32)reference_point * BLACKMINRATE/10, BLACKPOINT, 255);
// }
void get_reference_point(const uint8 *iamge)
{
    uint16 row;
    uint16 col;
    uint32 temp_sum = 0;
    uint16 start_col = (SEARCH_W - 8) / 2;
    uint16 temp = REFERENCEROW * 8;
    const uint8 *p = &iamge[(SEARCH_H - REFERENCEROW) * SEARCH_W];

    for (row = 0; row < REFERENCEROW; row++)
    {
        for (col = 0; col < 8; col++)
        {
            temp_sum += p[row * SEARCH_W + start_col + col];
        }
    }

    reference_point = (uint8)(temp_sum / temp);
    white_point = (uint8)limit_ab((uint16)reference_point * WHITEMAXRATE / 10, BLACKPOINT, 255);
    black_point = (uint8)limit_ab((uint16)reference_point * BLACKMINRATE / 10, BLACKPOINT, 255);
}

// 获取参考列
void get_reference_col(const uint8 *image)
{
    int col;
    int row;
    uint8 i = 0;
    int16 temp1 = 0;
    int16 temp2 = 0;
    uint16 temp3 = 0;
    uint8 temp4 = SEARCH_H - 1;
    for (col = 0; col < SEARCH_W; col++)
    {
        remote_distance[col] = SEARCH_H - 1;
    }
    for (col = 0; col < SEARCH_W; col += JUMPPIXEL)
    {
        for (row = temp4 + JUMPPIXEL; row >= STOPROW; row -= JUMPPIXEL)
        {
            if (row >= SEARCH_H - 1)
            {
                row = SEARCH_H - 1;
            }
            temp1 = *(image + row * SEARCH_W + col);             // 当前点灰度值
            temp2 = *(image + (row - STOPROW) * SEARCH_W + col); // 对比点灰度值
            if (temp2 > white_point)
            {
                continue;
            }
            if (temp1 < black_point)
            {
                remote_distance[col] = (uint8)row;
                temp4 = (uint8)row;
                break;
            }
            temp3 = (uint16)(((int32)(temp2 - temp1) * 200) / (temp1 + temp2));
            if (temp3 > reference_rate || row == STOPROW)
            {
                remote_distance[col] = (uint8)row;
                temp4 = (uint8)row;
                break;
            }
        }
    }
    reference_col = find_max_min(remote_distance, 10, SEARCH_W - 10, 0);
    reference_col = (uint8)limit_ab(reference_col, 1, SEARCH_W - 2);
    for (i = 0; i < SEARCH_H - STOPROW; i++)
    {
        reference_col_line[i] = reference_col;
    }
}





// 扫线

void search_line(const uint8 *image)
{
    const uint8 *p = image;                // 当前行起始地址
    uint8 row_max = SEARCH_H - 1;          // 行最大值
    uint8 row_min = STOPROW;               // 行最小值
    uint8 col_max = SEARCH_W - JUMPPIXEL;  // 列最大值
    uint8 col_min = JUMPPIXEL;             // 列最小值
    int16 left_start_col = reference_col;  // 搜线左起始列
    int16 left_end_col = 0;                // 搜线左结束列
    int16 right_start_col = reference_col; // 搜线右起始列
    int16 right_end_col = 0;               // 搜线右结束列
    uint8 search_time = 0;                 // 搜线次数
    uint8 temp1 = 0, temp2 = 0;            // 临时变量  储存图像数据
    int temp3 = 0;                         // 临时变量  储存对比度
    int left_stop = 0;                     // 左搜线停止标志
    int right_stop = 0;                    // 右搜线停止标志
    int stop_point = 0;                    // 搜线停止点

    int col, row;

    for (row = row_max; row >= row_min; row--)
    {
        left_edge_line[row] = col_min - JUMPPIXEL;
        right_edge_line[row] = col_max + JUMPPIXEL;
    }
    for (row = row_max; row >= row_min; row--)
    {
        p = (uint8 *)&image[row * SEARCH_W];
        if (!left_stop)
        {
            search_time = 2;
            do
            {
                if (search_time == 1)
                {
                    left_start_col = reference_col;
                    left_end_col = col_min;
                }
                search_time--;
                for (col = left_start_col; col >= left_end_col; col--)
                {
                    temp1 = *(p + col);             // 当前点灰度值
                    temp2 = *(p + col - JUMPPIXEL); // 对比点灰度值
                    if (temp1 < black_point && col == left_start_col && left_start_col == reference_col)
                    {
                        left_stop = 1;
                        for (stop_point = row; stop_point >= 0; stop_point--)
                        {
                            left_edge_line[stop_point] = col_min;
                        }
                        search_time = 0;
                        break;
                    }

                    if (temp1 < black_point)
                    {
                        left_edge_line[row] = (uint8)col;
                        break;
                    }

                    if (temp2 > white_point)
                    {
                        continue;
                    }

                    temp3 = (int16)(((int32)(temp1 - temp2) * 200) / (temp1 + temp2));
                    if (temp3 > reference_rate || col == col_min)
                    {
                        left_edge_line[row] = (uint8)col - JUMPPIXEL;
                        left_start_col = (uint8)limit_ab(col + SEARCHRANGE, col, col_max);
                        left_end_col = (uint8)limit_ab(col - SEARCHRANGE, col_min, col);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }
        if (!right_stop)
        {
            search_time = 2;
            do
            {
                if (search_time == 1)
                {
                    right_start_col = reference_col;
                    right_end_col = col_max;
                }
                search_time--;
                for (col = right_start_col; col <= right_end_col; col++)
                {
                    temp1 = *(p + col);             // 当前点灰度值
                    temp2 = *(p + col + JUMPPIXEL); // 对比点灰度值
                    if (temp1 < black_point && col == right_start_col && right_start_col == reference_col)
                    {
                        right_stop = 1;
                        for (stop_point = row; stop_point >= 0; stop_point--)
                        {
                            right_edge_line[stop_point] = col_max;
                        }
                        search_time = 0;
                        break;
                    }

                    if (temp1 < black_point)
                    {
                        right_edge_line[row] = (uint8)col;
                        break;
                    }

                    if (temp2 > white_point)
                    {
                        continue;
                    }

                    temp3 = (int16)(((int32)(temp1 - temp2) * 200) / (temp1 + temp2));
                    if (temp3 > reference_rate || col == col_max)
                    {
                        right_edge_line[row] = (uint8)col + JUMPPIXEL;
                        right_start_col = (uint8)limit_ab(col - SEARCHRANGE, col_min, col);
                        right_end_col = (uint8)limit_ab(col + SEARCHRANGE, col, col_max);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }
    }
}

void search_left_circle(const uint8 *image)
{
    int col;
    int row;
    int16 temp1 = 0;
    int16 temp2 = 0;
    uint16 temp3 = 0;
    uint8 circle_flag = 0;
    for (col = CIRCLESEARCHRANGE; col < SEARCH_W - CIRCLESEARCHRANGE; col += JUMPPIXEL)
    {
        circle_right.find = 0;
        circle_left.find = 0;
        circle_flag = 0;
        for (row = SEARCH_H - CIRCLESEARCHRANGE; row >= STOPROW; row -= JUMPPIXEL)
        {
            temp1 = *(image + row * SEARCH_W + col);               // 当前点灰度值
            temp2 = *(image + (row - JUMPPIXEL) * SEARCH_W + col); // 对比点灰度值
            if (temp2 > white_point)
            {
                continue;
            }
            if (temp1 < black_point)
            {
                if (left_edge_line[row] - 10 > col)
                {
                    break;
                }
                circle_flag += 1;
                if (circle_flag == 2)
                {
                    circle_left.row = (uint8)row;
                    circle_left.col = (uint8)col;
                    circle_left.find = 1;
                    break;
                }
                continue;
            }
            temp3 = (uint16)(((int32)(temp2 - temp1) * 200) / (temp1 + temp2));
            if (temp3 > reference_rate)
            {
                if (left_edge_line[row] - 10 > col)
                {
                    break;
                }
                circle_flag += 1;
                if (circle_flag == 2)
                {
                    circle_left.row = (uint8)row;
                    circle_left.col = (uint8)col;
                    circle_left.find = 1;
                    break;
                }
            }
        }
        if (circle_left.find == 1)
        {
            break;
        }
        if (row < STOPROW - JUMPPIXEL)
        {
            circle_left.find = 0;
            break;
        }
    }
}

void search_right_circle(const uint8 *image)
{
    int col;
    int row;
    int16 temp1 = 0;
    int16 temp2 = 0;
    uint16 temp3 = 0;
    uint8 circle_flag = 0;
    for (col = SEARCH_W - CIRCLESEARCHRANGE; col > CIRCLESEARCHRANGE; col -= JUMPPIXEL)
    {
        if (circle_left.find == 0)
        {
            break;
        }
        circle_flag = 0;
        for (row = SEARCH_H - CIRCLESEARCHRANGE; row >= STOPROW; row -= JUMPPIXEL)
        {
            temp1 = *(image + row * SEARCH_W + col);               // 当前点灰度值
            temp2 = *(image + (row - JUMPPIXEL) * SEARCH_W + col); // 对比点灰度值
            if (temp2 > white_point)
            {
                continue;
            }

            if (temp1 < black_point)
            {
                if (right_edge_line[row] + 10 < col)
                {
                    break;
                }
                circle_flag += 1;
                if (circle_flag == 2)
                {
                    circle_right.row = (uint8)row;
                    circle_right.col = (uint8)col;
                    circle_right.find = 1;
                    break;
                }
                continue;
            }
            temp3 = (uint16)(((int32)(temp2 - temp1) * 200) / (temp1 + temp2));
            if (temp3 > reference_rate)
            {
                if (right_edge_line[row] + 10 < col)
                {
                    break;
                }
                circle_flag += 1;
                if (circle_flag == 2)
                {
                    circle_right.row = (uint8)row;
                    circle_right.col = (uint8)col;
                    circle_right.find = 1;
                    break;
                }
            }
        }
        if (circle_right.find == 1)
        {
            break;
        }
    }
}

void get_mid_line(void)
{
    uint8 row;
    for (row = 0; row < SEARCH_H; row++)
    {
        mid_line[row] = (left_edge_line[row] + right_edge_line[row]) / 2;
    }
}

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;  
}

float vision_target_yaw = 0.0f; // 保存为“这帧图像给出的目标航向”
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

    get_reference_point((uint8_t *)image_copy);
    get_reference_col((uint8_t *)image_copy);   
    search_line((uint8_t *)image_copy);
    get_mid_line();

    fit_midline();      // 中线拟合
    HDPJ_lvbo();        // 滑动平均滤波

    // 图像线程里，每来一帧更新一次
    float deviation = Cal_Weigth1();                   // [-1, 1]
    float vision_delta_yaw = k_dev_to_yaw * deviation; // 例如“满偏差对应 10~20 度”
    // 保存为“这帧图像给出的目标航向”
    vision_target_yaw = wrap180(yaw + vision_delta_yaw);
    test = vision_target_yaw; // 供调试观察用，后续可以删除


    // 这里适合继续做“视觉层”的计算，例如根据 mid_line 计算横向误差、预瞄方向误差，
    // 最终得到一个相对车体坐标系的 vision_delta_yaw（还需要再转多少度）。
    // 不建议在 image_test() 里直接写最终 target_yaw，因为 target_yaw 是结合当前 IMU yaw
    // 后得到的“绝对航向目标”，更适合在调度/控制线程里统一生成。

    // 原来在主循环里直接发图，保留旧写法供后续对比。
    // 现在改为由 tcp_background_thread() 统一发送，避免图像处理线程直接占用 TCP 发送时间。
    seekfree_assistant_camera_send();
}




