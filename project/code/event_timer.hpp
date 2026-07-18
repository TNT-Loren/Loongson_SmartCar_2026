#ifndef EVENT_TIMER_HPP
#define EVENT_TIMER_HPP

#include <chrono>

// 基于单调时钟的被动事件秒表；不创建线程，由业务代码主动查询是否超时。
class MonotonicEventTimer
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Duration = std::chrono::milliseconds;

    void start(TimePoint now = Clock::now());
    void reset();
    bool running() const;
    Duration elapsed(TimePoint now = Clock::now()) const;
    bool expired(Duration timeout, TimePoint now = Clock::now()) const;

private:
    TimePoint start_time_{};
    bool running_ = false;
};

#endif
