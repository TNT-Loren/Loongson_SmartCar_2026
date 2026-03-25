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

// 编码器计数变量
int16_t Encoder_L = 0;
int16_t Encoder_R = 0;
int16_t RC_Encoder_L = 0;
int16_t RC_Encoder_R = 0;

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
    // 读取计数（保持原代码左右符号约定）
    Encoder_L = (int16_t)encoder_quad_1.get_count();
    Encoder_R = -(int16_t)encoder_quad_2.get_count();

   // 低通滤波
    RC_Encoder_L = (int16_t)RCFilter((float)Encoder_L, RC_Encoder_L_temp);
    RC_Encoder_R = (int16_t)RCFilter((float)Encoder_R, RC_Encoder_R_temp);

    // speed1 = Encoder_L;
    // speed2 = Encoder_R;
    // 速度计算：与原公式保持一致
    //每圈CNT增加约4635.6，采样间隔原为0.005s，最后乘10放大
    //    /4632     9256     13905     18585   23178   ==>>每转一圈CNT+4635.6
    //     4629      9303   13943      18552   23220
    speed1 = (float)((RC_Encoder_L / 4635.6f / dt) * 10.0f);
    speed2 = (float)((RC_Encoder_R / 4635.6f / dt) * 10.0f);

    encoder_quad_1.clear_count();
    encoder_quad_2.clear_count();
}
void encoder_update_task(float dt)
{
    Encoder_Get(dt);
    // 定时器清零
    encoder_quad_1.clear_count();
    encoder_quad_2.clear_count();
}

void Encoder_Init(void)
{
    // 清零计数，确保初始状态一致
    encoder_quad_1.clear_count();
    encoder_quad_2.clear_count();

}
