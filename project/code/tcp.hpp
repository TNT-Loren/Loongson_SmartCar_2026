#ifndef __TCP_HPP__
#define __TCP_HPP__

#include "zf_common_headfile.hpp"

// 初始化
bool tcp_debug_init(const char *ip, int port);

// 绑定需要监控的浮点数变量地址 (最多4路)
void tcp_bind_variables(float *ch0_ptr, float *ch1_ptr = nullptr, float *ch2_ptr = nullptr, float *ch3_ptr = nullptr);

// 中央调度器任务接口 (不带计时器，纯逻辑)
void tcp_update_task();

// 获取在线参数接口
float get_online_param(uint8_t index);

#endif

