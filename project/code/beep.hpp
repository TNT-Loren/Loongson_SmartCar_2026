#ifndef __BEEP_HPP__
#define __BEEP_HPP__

#include "zf_common_headfile.hpp"

typedef enum
{
    On,
    Off,
} Beep_Handle;

extern uint8 Beep_Count;

void Beep_Init(void);
void Beep(Beep_Handle mode);
void Beep_On(void);
// 单次响铃，单位 ms；若当前正在响铃，则忽略本次设置，避免打断已有提示音。
void Set_Beeptime(uint16 set_time_ms);
// 设置连续响铃次数；每次默认响 100ms，间隔 100ms。
void Beep_Freq(void);
void Set_Beepfreq(uint8 count);
// 由 5ms 调度器周期调用，推进响铃时间和次数状态机。
void Beep_Task_5ms(void);

#endif
