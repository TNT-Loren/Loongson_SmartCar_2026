#include "stop.hpp"

#include "image_test.hpp"
#include "scheduler.hpp"
#include "smartcar_params.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>

namespace
{
std::atomic<std::uint32_t> g_red_stop_count{0};
std::atomic<std::uint32_t> g_zebra_stop_count{0};
std::atomic<bool> g_stop_requested{false};
std::atomic<bool> g_zebra_event_pending{false};

using ZebraClock = std::chrono::steady_clock;
const ZebraClock::time_point g_invalid_time = ZebraClock::time_point::min();
ZebraClock::time_point g_zebra_start_time = g_invalid_time;
ZebraClock::time_point g_zebra_cooldown_until = g_invalid_time;
int g_zebra_confirmed_frames = 0;
bool g_zebra_armed = true;

int zebra_transition_count(bool frame_valid)
{
    if (!frame_valid ||
        Image_Flag.Left_Circle || Image_Flag.Right_Circle ||
        Left_Lost_Time > MT9V03X_H / 3 || Right_Lost_Time > MT9V03X_H / 3 ||
        obstacle_avoid_active())
    {
        return 0;
    }

    int start_row = std::clamp(smartcar::params::zebra::scan_start_row,
                               0, MT9V03X_H - 1);
    int end_row = std::clamp(smartcar::params::zebra::scan_end_row,
                             0, MT9V03X_H - 1);
    if (start_row < end_row)
    {
        std::swap(start_row, end_row);
    }

    int change_count = 0;
    for (int row = start_row; row >= end_row; --row)
    {
        // j+1 会被访问，因此左右边界都限制到图像倒数第二列。
        const int left = std::clamp(static_cast<int>(Left_Line[row]),
                                    0, MT9V03X_W - 2);
        const int right = std::clamp(static_cast<int>(Right_Line[row]),
                                     left, MT9V03X_W - 2);
        for (int col = left; col <= right; ++col)
        {
            if (bin_image[row][col] != bin_image[row][col + 1])
            {
                ++change_count;
            }
        }
    }
    return change_count;
}
}

bool red_stop(int stop_on_count)
{
    if (stop_on_count < 1)
    {
        return false;
    }
    if (g_stop_requested.load(std::memory_order_acquire))
    {
        return true;
    }

    const std::uint32_t current_count =
        g_red_stop_count.fetch_add(1, std::memory_order_acq_rel) + 1U;
    if (current_count >= static_cast<std::uint32_t>(stop_on_count))
    {
        g_stop_requested.store(true, std::memory_order_release);
        return true;
    }
    return false;
}

bool red_stop_requested()
{
    return g_stop_requested.load(std::memory_order_acquire);
}

std::uint32_t red_stop_count()
{
    return g_red_stop_count.load(std::memory_order_acquire);
}

void zebra_update_frame(bool frame_valid)
{
    const auto now = ZebraClock::now();
    if (g_zebra_start_time == g_invalid_time)
    {
        g_zebra_start_time = now;
    }

    if (now - g_zebra_start_time < smartcar::params::zebra::detect_start_delay)
    {
        // 启动延时期间完全关闭底层扫描，避免曝光稳定前的画面污染确认计数。
        Image_Flag.Zerba = false;
        g_zebra_confirmed_frames = 0;
        return;
    }

    const int change_count = zebra_transition_count(frame_valid);
    const bool raw_detected =
        change_count >= smartcar::params::zebra::transition_threshold;

    // 复用现有调试打印开关；主循环打印后会清零，因此不会持续刷屏。
    if (need_print.load() == 1)
    {
        std::printf("[ZEBRA] transitions=%d threshold=%d %s\r\n",
                    change_count,
                    smartcar::params::zebra::transition_threshold,
                    raw_detected ? "DETECTED" : "clear");
    }
    Image_Flag.Zerba = raw_detected;

    if (!raw_detected)
    {
        g_zebra_confirmed_frames = 0;
        // 必须看到斑马线消失，并且冷却时间已到，才允许下一条斑马线触发事件。
        if (now >= g_zebra_cooldown_until)
        {
            g_zebra_armed = true;
        }
        return;
    }

    if (!g_zebra_armed || now < g_zebra_cooldown_until)
    {
        g_zebra_confirmed_frames = 0;
        return;
    }

    ++g_zebra_confirmed_frames;
    if (g_zebra_confirmed_frames >= smartcar::params::zebra::confirm_frames)
    {
        g_zebra_confirmed_frames = 0;
        g_zebra_armed = false;
        g_zebra_cooldown_until = now + smartcar::params::zebra::detection_cooldown;
        g_zebra_event_pending.store(true, std::memory_order_release);
    }
}

bool zebra_consume_event()
{
    return g_zebra_event_pending.exchange(false, std::memory_order_acq_rel);
}

bool zebra_stop(int stop_on_count)
{
    if (stop_on_count < 1)
    {
        return false;
    }
    if (g_stop_requested.load(std::memory_order_acquire))
    {
        return true;
    }

    const std::uint32_t current_count =
        g_zebra_stop_count.fetch_add(1, std::memory_order_acq_rel) + 1U;
    if (current_count >= static_cast<std::uint32_t>(stop_on_count))
    {
        g_stop_requested.store(true, std::memory_order_release);
        return true;
    }
    return false;
}

std::uint32_t zebra_stop_count()
{
    return g_zebra_stop_count.load(std::memory_order_acquire);
}
