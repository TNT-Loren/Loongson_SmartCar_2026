#include "zf_common_headfile.hpp"
#include "encoder.hpp"
//=============================
// 全局速度变量
float speed1 = 0.0;
float speed2 = 0.0;
//=====================================================================
#define ENCODER_QUAD_1_PATH ZF_ENCODER_QUAD_1
#define ENCODER_QUAD_2_PATH ZF_ENCODER_QUAD_2

// 创建编码器对象，传入文件路径
zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH);
zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH);

//========================================================================滤波
// 低通滤波参数结构（与原风格保持一致）
struct RC_Para Encoder_R_Para = {0, 0, 0.25f}; // value, temp, RC
struct RC_Para Encoder_L_Para = {0, 0, 0.25f};

static RC_Filter_pt RC_Encoder_L_temp = &Encoder_L_Para;
static RC_Filter_pt RC_Encoder_R_temp = &Encoder_R_Para;

// 低通滤波函数（与原实现等价）
float RCFilter(float value, RC_Filter_pt Filter)
{
    Filter->temp = value;
    Filter->value = (1.0f - Filter->RC) * Filter->value + Filter->RC * Filter->temp;
    return Filter->value;
}
//===============================================================================
void Encoder_Get(float dt)
{
    // 保持原有左右符号约定。先以真实 dt 换算速度，避免滤波脉冲数受周期抖动影响。
    const int32_t encoder_l_count = static_cast<int32_t>(encoder_quad_1.get_count());
    const int32_t encoder_r_count = -static_cast<int32_t>(encoder_quad_2.get_count());

    // 速度单位与原公式保持一致。
    //每圈CNT增加约4635.6，采样间隔原为0.005s，最后乘10放大
    //    /4632     9256     13905     18585   23178   ==>>每转一圈CNT+4635.6
    //     4629      9303   13943      18552   23220

    // 实际速度 (cm/s) = speed × 2.05
    // 实际速度 (m/s)  = speed × 0.0205
    // speed 100 10 rps	2.05 m/s
    const float raw_speed_l = static_cast<float>(encoder_l_count) * 10.0f / (4635.6f * dt);
    const float raw_speed_r = static_cast<float>(encoder_r_count) * 10.0f / (4635.6f * dt);
    speed1 = RCFilter(raw_speed_l, RC_Encoder_L_temp);
    speed2 = RCFilter(raw_speed_r, RC_Encoder_R_temp);

    encoder_clear_counts();
}
void encoder_update_task(float dt)
{
    Encoder_Get(dt);
}

void encoder_clear_counts(void)
{
    encoder_quad_1.clear_count();
    encoder_quad_2.clear_count();
}

void encoder_reset(void)
{
    encoder_clear_counts();
    Encoder_L_Para.temp = 0.0f;
    Encoder_L_Para.value = 0.0f;
    Encoder_R_Para.temp = 0.0f;
    Encoder_R_Para.value = 0.0f;
    speed1 = 0.0f;
    speed2 = 0.0f;
}

void Encoder_Init(void)
{
    encoder_reset();
}
