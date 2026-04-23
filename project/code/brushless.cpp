#include "brushless.hpp"
#include "zf_common_headfile.hpp"

#define ESC_PATH ZF_PWM_ESC_1

static struct pwm_info esc_info;
static zf_driver_pwm esc_pwm(ESC_PATH);

// 该主板，两个电调接口共用同一个PWM引脚GPIO89
// 该主板，两个电调接口共用同一个PWM引脚GPIO89
// 该主板，两个电调接口共用同一个PWM引脚GPIO89

// 电调PWM频率默认为50HZ
// 50HZ的周期为20ms

// 计算无刷电调转速   （1ms - 2ms）/20ms * 10000（10000是PWM的满占空比时候的值）
// 无刷电调转速 0%   为 500
// 无刷电调转速 20%  为 600
// 无刷电调转速 40%  为 700
// 无刷电调转速 60%  为 800
// 无刷电调转速 80%  为 900
// 无刷电调转速 100% 为 1000

namespace
{
    constexpr float ESC_PERCENT_MIN = 0.0f;
    constexpr float ESC_PERCENT_MAX = 100.0f;
    constexpr uint32 ESC_MIN_PULSE_NS = 1000000U;
    constexpr uint32 ESC_MAX_PULSE_NS = 2000000U;

    void esc_load_info()
    {
        esc_pwm.get_dev_info(&esc_info);
    }

    float esc_limit_percent(float percent)
    {
        if (percent < ESC_PERCENT_MIN)
        {
            return ESC_PERCENT_MIN;
        }

        if (percent > ESC_PERCENT_MAX)
        {
            return ESC_PERCENT_MAX;
        }

        return percent;
    }

    uint16 esc_percent_to_duty(float percent)
    {
        if (0 == esc_info.duty_max || 0 == esc_info.period_ns)
        {
            esc_load_info();
        }

        if (0 == esc_info.duty_max || 0 == esc_info.period_ns)
        {
            return 0;
        }

        const float limited_percent = esc_limit_percent(percent);
        const float pulse_ns = static_cast<float>(ESC_MIN_PULSE_NS) +
                               static_cast<float>(ESC_MAX_PULSE_NS - ESC_MIN_PULSE_NS) * (limited_percent / ESC_PERCENT_MAX);
        float duty = pulse_ns * static_cast<float>(esc_info.duty_max) / static_cast<float>(esc_info.period_ns);

        if (duty < 0.0f)
        {
            duty = 0.0f;
        }

        if (duty > static_cast<float>(esc_info.duty_max))
        {
            duty = static_cast<float>(esc_info.duty_max);
        }

        return static_cast<uint16>(duty + 0.5f);
    }
}

void esc_init()
{
    esc_load_info();
    // 打印PWM频率和duty最大值
    printf("esc pwm freq = %d Hz\r\n", esc_info.freq);
    printf("esc pwm duty_max = %d\r\n", esc_info.duty_max);
}

void esc_set_speed_percent(float percent)
{
    esc_pwm.set_duty(esc_percent_to_duty(percent));
}
