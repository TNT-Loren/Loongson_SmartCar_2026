#include "event_timer.hpp"

void MonotonicEventTimer::start(TimePoint now)
{
    start_time_ = now;
    running_ = true;
}

void MonotonicEventTimer::reset()
{
    start_time_ = TimePoint{};
    running_ = false;
}

bool MonotonicEventTimer::running() const
{
    return running_;
}

MonotonicEventTimer::Duration MonotonicEventTimer::elapsed(TimePoint now) const
{
    if (!running_ || now <= start_time_)
    {
        return Duration::zero();
    }

    return std::chrono::duration_cast<Duration>(now - start_time_);
}

bool MonotonicEventTimer::expired(Duration timeout, TimePoint now) const
{
    return running_ && elapsed(now) >= timeout;
}
