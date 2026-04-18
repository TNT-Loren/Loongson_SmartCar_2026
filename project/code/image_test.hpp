#ifndef __image_test_HPP__
#define __image_test_HPP__

#include "zf_common_headfile.hpp"

extern zf_device_uvc uvc_dev; // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集


#define MT9V03X_H UVC_HEIGHT
#define MT9V03X_W UVC_WIDTH


extern uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];
extern uint8 bin_image[MT9V03X_H][MT9V03X_W]; // 二值化图像数组

// 定义膨胀和腐蚀的阈值区间
#define threshold_max 255 * 5 // 此参数可根据自己的需求调节
#define threshold_min 255 * 1 // 此参数可根据自己的需求调节

#define IMG_BLACK 0   // 0x00黑色
#define IMG_WHITE 255 // 0xff白色

#define Border_Max UVC_WIDTH - 2 // 边界最大值
#define Border_Min 1             // 边界最小值

//=========================
void image_test(void);

#endif