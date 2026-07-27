#ifndef SMARTCAR_STOP_HPP
#define SMARTCAR_STOP_HPP

#include <cstdint>

// 每检测到一个新的红色预警时调用一次。
// stop_on_count=-1 表示关闭；N>=1 表示第 N 次预警返回 true 并永久锁停。
bool red_stop(int stop_on_count);

// 供控制线程读取锁停状态；锁停只会在程序重启后清除。
bool red_stop_requested();

// 返回本次程序运行以来、启用 red_stop 后累计的预警次数。
std::uint32_t red_stop_count();

// 由图像线程每帧更新一次；底层判据和一次性事件状态均放在 stop.cpp。
void zebra_update_frame(bool frame_valid);
// 读取并清除一条已经完成确认的斑马线事件。
bool zebra_consume_event();

// 只在 zebra_consume_event() 返回 true 时调用一次。
// stop_on_count=-1 表示关闭；N>=1 表示第 N 条斑马线触发同一套整车锁停。
bool zebra_stop(int stop_on_count);
std::uint32_t zebra_stop_count();

#endif
