#ifndef __encoder_hpp__
#define __encoder_hpp__

struct RC_Para
{
    float temp;  // 暂存值,存储RC_baro
    float value; // 滤波值
    float RC;    // 低通滤波参数
};

extern float speed1, speed2;

typedef struct RC_Para *RC_Filter_pt;
// extern int16 RC_Encoder_L, RC_Encoder_R;  // 低通滤波后

void Encoder_Init(void);
void encoder_update_task(float dt);

#endif