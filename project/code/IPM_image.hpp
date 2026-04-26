#ifndef __IPM_IMAGE_HPP__
#define __IPM_IMAGE_HPP__

#include "zf_common_headfile.hpp"
#include <mutex>

// 实际处理的图像宽度，320是摄像头采集的原始宽度，为了提高处理效率，后续会缩放到160宽度进行处理
#define image_width 160
#define image_height 120

extern zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
//extern uint8 copy_image[image_height][image_width]; // 用于存储缩放后的图像数据
extern uint8 copy_image[image_height][image_width]; // 图像处理模块使用的图像数据缓冲区，供IPM变换等后续处理使用
extern uint8_t ipm_image_array[image_height][image_width]; // IPM变换后的图像数据缓冲区，供后续处理使用
extern uint8 bin_image[image_height][image_width]; // 逆透视后二值图，供后续搜线/丢线判断使用
extern std::mutex g_ipm_image_mutex; // 保护图传缓冲区，避免发送线程读到半帧数据
extern int valid_l_bound[image_height]; // 每一行逆透视有效区域的左边界，若 left > right 说明该行无效
extern int valid_r_bound[image_height]; // 每一行逆透视有效区域的右边界
extern uint8 start_point_l[2]; // 左边起点的 x、y
extern uint8 start_point_r[2]; // 右边起点的 x、y
extern uint8 left_edge_line[image_height];  // 八邻域得到的左边界线
extern uint8 right_edge_line[image_height]; // 八邻域得到的右边界线

extern uint8 mid_line[image_height]; // 中线

void init_ipm_valid_region(void);// 逆透视有效区域初始化函数，预先计算每行的有效左右边界，供后续处理使用
void turn_to_bin(void);
void draw_valid_region_box(uint8 (*bin_image)[image_width]);
void find_start_point_by_valid_box(uint8 (*bin_image)[image_width]);
void image_filter(uint8 (*bin_image)[image_width]);
void image_process(void);

#endif
