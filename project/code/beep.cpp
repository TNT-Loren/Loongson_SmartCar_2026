#include "beep.hpp"

namespace
{
// 蜂鸣器任务挂在 scheduler 的 5ms 主定时器上，下面时间都按 5ms tick 换算。
constexpr uint16 k_beep_task_period_ms = 5;
constexpr uint16 k_beep_pulse_ms = 100;
constexpr uint16 k_beep_gap_ms = 100;

// 龙芯板载蜂鸣器设备节点，若实测为低电平有效，只需要反转 Beep() 内的电平。
zf_driver_gpio beep_gpio(ZF_GPIO_BEEP, O_RDWR);

// beep_time_ticks 控制当前响多久，beep_gap_ticks 控制多次响铃之间的静音间隔。
uint16 beep_time_ticks = 0;
uint16 beep_gap_ticks = 0;

// 向上取整，避免小于 5ms 的请求被换算成 0 tick。
uint16 ms_to_ticks(uint16 time_ms)
{
    return (time_ms + k_beep_task_period_ms - 1) / k_beep_task_period_ms;
}
}

uint8 Beep_Count = 0;

void Beep_Init(void)
{
    Beep(Off);
    beep_time_ticks = 0;
    beep_gap_ticks = 0;
    Beep_Count = 0;
}

void Beep(Beep_Handle mode)
{
    beep_gpio.set_level(mode == On ? 1 : 0);
}

void Beep_On(void)
{
    if (beep_time_ticks > 0)
    {
        --beep_time_ticks;
        Beep(On);
    }
    else
    {
        Beep(Off);
    }
}

// 旧代码接口：设置一次持续响铃，不抢占正在进行的响铃。
void Set_Beeptime(uint16 set_time_ms)
{
    if (beep_time_ticks == 0)
    {
        beep_time_ticks = ms_to_ticks(set_time_ms);
    }
}

void Beep_Freq(void)
{
    if (beep_time_ticks > 0)
    {
        return;
    }

    if (beep_gap_ticks > 0)
    {
        --beep_gap_ticks;
        return;
    }

    if (Beep_Count == 0)
    {
        return;
    }

    --Beep_Count;
    Set_Beeptime(k_beep_pulse_ms);
    beep_gap_ticks = ms_to_ticks(k_beep_gap_ms);
}

// 旧代码接口：设置响铃次数，新的次数会覆盖未完成的次数。
void Set_Beepfreq(uint8 count)
{
    Beep_Count = count;
    beep_gap_ticks = 0;
}

void Beep_Task_5ms(void)
{
    Beep_Freq();
    Beep_On();
}
