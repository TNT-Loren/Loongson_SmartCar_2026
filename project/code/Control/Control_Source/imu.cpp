#include "imu.hpp"
#include <cmath>
#include <cstdio>

// 实例化全局对象
YawTracker yaw_tracker;
zf_device_imu imu_dev;
float yaw = 0.0f; // 依然保留这个全局变量，方便示波器监控

YawTracker::YawTracker()
{
    reset();
}

void YawTracker::reset()
{
    unbounded_yaw = 0.0f;
    yaw = 0.0f;
    last_gyro_z_dps = 0.0f;
}

float YawTracker::wrap_to_180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

void YawTracker::init()
{
    imu_dev.init();
    printf("IMU 正在进行 Yaw 轴...\n");
    printf("请保持小车绝对静止约 5 秒钟，千万不要触碰车体！\n");
    // 1. 【预热与稳定】：给传感器 1 秒钟的电平稳定和温度预热时间
    system_delay_ms(1000);
    // 2. 【排空脏数据】：连续读取 50 次丢弃，防止寄存器里有上电瞬间的毛刺残存
    for (int i = 0; i < 50; i++)
    {
        imu_dev.get_gyro_z();
        system_delay_ms(2);
    }
    // 2000 次 * 2ms = 4000ms (加上 Linux 调度开销，实际约 4.5 秒)
    long sum_z = 0;
    const int sample_count = 2000;

    for (int i = 0; i < sample_count; i++)
    {
        sum_z += imu_dev.get_gyro_z();
        system_delay_ms(2);
    }

    // 4. 【计算零偏】：求出高精度的均值
    gyro_z_bias = ((float)sum_z / sample_count) / GYRO_SCALE_FACTOR;

    reset();
    last_time = std::chrono::high_resolution_clock::now();

    // 打印时多保留一位小数，方便你观察它的精细度
    printf("校准完美结束！共采集 %d 帧，Z轴高精度零偏: %.4f DPS\n", sample_count, gyro_z_bias);
}

    void YawTracker::update(float dt)
{
    // 获取原始数据并转换
    float raw_z = (float)imu_dev.get_gyro_z();
    float current_dps = (raw_z / GYRO_SCALE_FACTOR) - gyro_z_bias;

    // 死区处理
    if (std::abs(current_dps) < GYRO_Z_DEADBAND)
    {
        current_dps = 0.0f;
    }

    // 梯形积分算法 (直接使用外部喂进来的 dt)
    float delta_angle = (last_gyro_z_dps + current_dps) * 0.5f * dt;

    // 比例补偿与累加
    unbounded_yaw += (delta_angle * YAW_SCALE_RATIO);
    yaw = wrap_to_180(unbounded_yaw);

    // 更新历史
    last_gyro_z_dps = current_dps;
}

// =================== C 风格包装函数实现 ===================
void imu_init()
{
    yaw_tracker.init();
}

void imu_update_task(float dt)
{
    yaw_tracker.update(dt);
}
