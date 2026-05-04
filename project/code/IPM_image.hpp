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
extern uint16 debug_image[image_height][image_width]; // RGB565 调试图像缓冲区，供 SCC8660 彩色图传使用
extern uint8 bin_image[image_height][image_width]; // 逆透视后二值图，供后续搜线/丢线判断使用
extern std::mutex g_ipm_image_mutex; // 保护图传缓冲区，避免发送线程读到半帧数据
extern int valid_l_bound[image_height]; // 每一行逆透视有效区域的左边界，若 left > right 说明该行无效
extern int valid_r_bound[image_height]; // 每一行逆透视有效区域的右边界
extern uint8 start_point_l[2]; // 左边起点的 x、y
extern uint8 start_point_r[2]; // 右边起点的 x、y
extern uint8 g_left_start_point_fallback_flag;  // 左起点是否退回到了有效区边界
extern uint8 g_right_start_point_fallback_flag; // 右起点是否退回到了有效区边界
extern uint8 left_edge_line[image_height];  // 八邻域得到的左边界线
extern uint8 right_edge_line[image_height]; // 八邻域得到的右边界线
extern uint16 points_l[k_max_search_points][2];
extern uint16 points_r[k_max_search_points][2];
extern uint8 dir_l[k_max_search_points]; // 八邻域搜线时每个点对应的生长方向
extern uint8 dir_r[k_max_search_points];
extern uint16 g_left_point_count;  // 本帧左边原始搜线点数
extern uint16 g_right_point_count; // 本帧右边原始搜线点数
extern uint8 g_left_crossover_flag;  // 左边界搜线发生串边并被保护逻辑截停
extern uint8 g_right_crossover_flag; // 右边界搜线发生串边并被保护逻辑截停


extern uint8 g_track_reference_width_valid;  // 参考赛道宽度是否已初始化
extern uint8 g_track_reference_width;        // 参考赛道宽度（黑边到黑边）
extern uint8 g_track_reference_center;       // 参考赛道中心 x
extern uint8 g_track_reference_row;          // 参考赛道宽度取样所在行
///////////////////////////////////////////////
// extern uint16 g_left_unique_y_count;  // 本帧左边原始点集中覆盖到的不同 y 行数
// extern uint16 g_right_unique_y_count; // 本帧右边原始点集中覆盖到的不同 y 行数
// extern uint8 g_left_hit_limit;        // 本帧左边搜线是否被步数/容量上限截断
// extern uint8 g_right_hit_limit;

extern uint8 mid_line[image_height]; // 中线

typedef struct
{
    uint8 flag; // 0=未找到, 1=找到
    uint8 row;  // y 坐标
    uint8 col;  // x 坐标
} Track_Corner_Point_TypeDef;

extern Track_Corner_Point_TypeDef g_left_upper_corner;  // 左上拐点
extern Track_Corner_Point_TypeDef g_right_upper_corner; // 右上拐点
extern Track_Corner_Point_TypeDef g_left_lower_corner;  // 左下拐点
extern Track_Corner_Point_TypeDef g_right_lower_corner; // 右下拐点
extern uint8 g_left_long_straight_flag;   // 左边界是否为长直线
extern uint8 g_right_long_straight_flag;  // 右边界是否为长直线
extern uint8_t g_left_ring_yaw_recording_flag; // 左环累计角是否正在记录
extern float g_left_ring_enter_unbounded_yaw;  // 左环进入时记录的连续 yaw
extern float g_left_ring_progress_yaw;         // 左环累计角，左转时递增
extern uint8_t line_lost;              // 0都不丢线，1左边丢线，2右边丢线，3都丢线
extern uint8_t mode_element;              // 0正常模式，1十字，2左圆环；
extern uint8_t left_ring_process_mode;    // 0未进入左圆环

void init_ipm_valid_region(void);// 逆透视有效区域初始化函数，预先计算每行的有效左右边界，供后续处理使用
void turn_to_bin(void);
void draw_valid_region_box(uint8 (*bin_image)[image_width]);
void find_start_point_by_valid_box(uint8 (*bin_image)[image_width]);
void image_filter(uint8 (*bin_image)[image_width]);
void update_mid_line_by_track_mode(uint8 track_mode); // 0双边巡线，1左边巡线，2右边巡线
void detect_track_corner_points(void);
void detect_track_long_straight_features(void);
bool Transform_Point1(int x, int y, double &mapped_x, double &mapped_y);
bool Transform_Point2(int x, int y, double &mapped_x, double &mapped_y);
void Transform_Point1(int x, int y);
void Transform_Point2(int x, int y);
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);
void refresh_copy_image_from_current_camera_image(void);
void fill_ipm_debug_image_from_copy_image(uint16 (*img)[image_width], bool big_endian = true);
bool init_reference_track_width(uint16 max_attempt_frames = 30);

#endif
