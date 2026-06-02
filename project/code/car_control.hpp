#ifndef __car_control_HPP__
#define __car_control_HPP__

#include "zf_common_headfile.hpp"
#include "image_size.hpp"

enum class TrackScene : uint8_t;

extern float vision_target_yaw;
extern float Line_Error;
extern uint16 Control_Ipm_Extended_Mid_Points[image_height][2];
extern uint16 Control_Ipm_Extended_Mid_Point_Count;
extern int16 Control_Ipm_Line_X_By_Y[image_height];
extern int16 Control_Ipm_Raw_Last_Valid_Y;
extern bool Control_Ipm_Preview_Target_Valid;
extern uint16 Control_Ipm_Preview_Target[2];
extern TrackScene Control_Ipm_Debug_Scene;

float Cal_Weigth1(void);
float Cal_Weigth2(void);
void publish_lost_line_result(void);
void update_control_target(void);
void refresh_control_ipm_debug_midline(void);

#endif
