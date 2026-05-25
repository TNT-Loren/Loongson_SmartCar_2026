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
    float deviation = 0.0f;// 横向偏差，范围 [-1, 1]，0=居中，负=左偏，正=右偏
};

extern TrackInfo g_track_info;

// 基础速度计算接口
float calc_base_speed(const TrackInfo &info);

#endif
