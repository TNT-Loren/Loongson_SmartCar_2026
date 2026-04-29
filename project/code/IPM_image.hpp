#ifndef __IPM_IMAGE_HPP__
#define __IPM_IMAGE_HPP__

#include "image_size.hpp"
#include "zf_common_headfile.hpp"
#include <mutex>

static constexpr uint16 k_max_search_points = image_height * 5; // 原始搜线点数组容量
static constexpr uint16 k_max_search_steps = image_height * 4;  // 八邻域最大搜索步数

extern zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
//extern uint8 copy_image[image_height][image_width]; // 用于存储缩放后的图像数据
extern uint8 copy_image[image_height][image_width]; // 图像处理模块使用的图像数据缓冲区，供IPM变换等后续处理使用
extern uint8_t ipm_image_array[image_height][image_width]; // IPM变换后的图像数据缓冲区，供后续处理使用
extern uint16 debug_image[image_height][image_width]; // RGB565 调试图像缓冲区，供 SCC8660 彩色图传使用
extern uint8 bin_image[image_height][image_width]; // 逆透视后二值图，供后续搜线/丢线判断使用
extern std::mutex g_ipm_image_mutex; // 保护图传缓冲区，避免发送线程读到半帧数据
extern int valid_l_bound[image_height]; // 每一行逆透视有效区域的左边界，若 left > right 说明该行无效
extern int valid_r_bound[image_height]; // 每一行逆透视有效区域的右边界
extern uint8 start_point_l[2]; // 左边起点的 x、y
extern uint8 start_point_r[2]; // 右边起点的 x、y
extern uint8 left_edge_line[image_height];  // 八邻域得到的左边界线
extern uint8 right_edge_line[image_height]; // 八邻域得到的右边界线
extern uint16 points_l[k_max_search_points][2];
extern uint16 points_r[k_max_search_points][2];
extern uint8 dir_l[k_max_search_points]; // 八邻域搜线时每个点对应的生长方向
extern uint8 dir_r[k_max_search_points];
extern uint16 g_left_point_count;  // 本帧左边原始搜线点数
extern uint16 g_right_point_count; // 本帧右边原始搜线点数
///////////////////////////////////////////////
// extern uint16 g_left_unique_y_count;  // 本帧左边原始点集中覆盖到的不同 y 行数
// extern uint16 g_right_unique_y_count; // 本帧右边原始点集中覆盖到的不同 y 行数
// extern uint8 g_left_hit_limit;        // 本帧左边搜线是否被步数/容量上限截断
// extern uint8 g_right_hit_limit;

extern uint8 mid_line[image_height]; // 中线

void init_ipm_valid_region(void);// 逆透视有效区域初始化函数，预先计算每行的有效左右边界，供后续处理使用
void turn_to_bin(void);
void draw_valid_region_box(uint8 (*bin_image)[image_width]);
void find_start_point_by_valid_box(uint8 (*bin_image)[image_width]);
void image_filter(uint8 (*bin_image)[image_width]);
void image_process(void);

#endif
