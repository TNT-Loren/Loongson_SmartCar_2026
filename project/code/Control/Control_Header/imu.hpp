#ifndef __IMU_HPP__
#define __IMU_HPP__

#include "zf_common_headfile.hpp"
#include <chrono>

extern float yaw;

class YawTracker
{
private:
       // 内部常量 (封装，不暴露给外部)
       static constexpr float GYRO_SCALE_FACTOR = 16.4f;
       static constexpr float GYRO_Z_DEADBAND = 0.1f;
       static constexpr float YAW_SCALE_RATIO = 0.99756f; // 5.72; -1.4  11.17  2.089 20.24  16.17  17.7

       // 状态变量
       float gyro_z_bias = 0.0f;
       float last_gyro_z_dps = 0.0f;
       float unbounded_yaw = 0.0f;

       std::chrono::high_resolution_clock::time_point last_time;

       // 内部角度约束函数
       float wrap_to_180(float angle);

public:
       YawTracker();

       // 接口函数
       void init();   // 初始化与校准
       void update(float dt); // 任务执行 (供调度器调用)
       void reset();  // 航向角手动归零

       // 数据读取 (供 PID 等模块使用)
       float get_yaw() const;
};

// 声明全局实例，方便调度器直接访问
extern YawTracker yaw_tracker;

// 为了兼容你之前的调用习惯，保留 C 风格的接口封装
void imu_init();
void imu_update_task(float dt);

#endif
   