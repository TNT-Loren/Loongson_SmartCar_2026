#ifndef __speed_strategy_HPP__
#define __speed_strategy_HPP__

#include "zf_common_headfile.hpp"
// code/speed_strategy.hpp
#pragma once
#include <cstdint>

// 赛道场景枚举
enum class TrackScene : uint8_t
{
    Straight,
    GentleCurve,
    SharpCurve,
    Circle,
    LostLine
};

// 赛道信息结构体（由视觉模块填充）
struct TrackInfo
{
    TrackScene scene = TrackScene::Straight;
    float deviation = 0.0f;// pure pursuit 角度误差，单位：度，负=左，正=右
};

extern TrackInfo g_track_info;
extern float base_start_speed;

extern float k_speed_straight; // aaa
extern float k_speed_curve;
extern float k_speed_sharp;
extern float k_speed_circle;
extern float k_speed_lost;

// 基础速度计算接口
float calc_base_speed(const TrackInfo &info);

#endif
