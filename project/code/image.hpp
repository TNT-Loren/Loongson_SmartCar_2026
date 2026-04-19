#ifndef __image_HPP__
#define __image_HPP__

#include "zf_common_headfile.hpp"

extern zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
extern uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

#define SEARCH_H UVC_HEIGHT
#define SEARCH_W UVC_WIDTH
#define REFERENCEROW 5
#define STOPROW 10
#define SEARCHRANGE 10

#define WHITEMAXRATE 13
#define BLACKMINRATE 7
#define BLACKPOINT 50

#define JUMPPIXEL 3
#define BREAKPOINT 20        // 断点阈值
#define CIRCLESEARCHRANGE 20 // 圆搜索范围

#define k_dev_to_yaw   20.0f // 假设满偏差对应 20 度，实际需要根据测试调整

extern uint8 reference_col_line[SEARCH_H]; // 参考列绘制
extern uint8 reference_point;              // 动态参考点 灰度值
extern uint8 reference_rate;               // 动态对比度  差比和
extern uint8 reference_col;                // 动态参考列
extern uint8 white_point;                  // 白点阈值
extern uint8 black_point;                  // 黑点阈值
extern uint8 remote_distance[SEARCH_W];    // 远近距离数组 记录每列的边界位置

extern uint8 left_edge_line[SEARCH_H];     // 左边界线
extern uint8 right_edge_line[SEARCH_H];    // 右边界线
extern uint8 mid_line[SEARCH_H];    // 中线

extern float vision_target_yaw; // 保存为“这帧图像给出的目标航向”

extern uint32 if_count; // 计数器

typedef struct
{
    uint8 row;
    uint8 col;
    uint8 find; // 0未找到 1找到
} point;

extern point circle_left;
extern point circle_right;

void get_reference_point(const uint8 *iamge);
void get_reference_col(const uint8 *image);
void search_line(const uint8 *image);
void get_mid_line(void);
void search_right_circle(const uint8 *image);
void search_left_circle(const uint8 *image);

void image_test(void);

#endif
