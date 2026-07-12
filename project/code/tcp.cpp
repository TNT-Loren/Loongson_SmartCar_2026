#include "tcp.hpp"
#include "image_test.hpp"
#include "vs_inference.hpp"
#include <atomic>
#include <cstring>

#define TCP_IMAGE_SOURCE_TRACK 0
#define TCP_IMAGE_SOURCE_VS    1   //lgy

// 图传矩阵选择：巡线使用 TRACK，队友推理使用 VS。
#define TCP_IMAGE_SOURCE 1

#if TCP_IMAGE_SOURCE != TCP_IMAGE_SOURCE_TRACK && TCP_IMAGE_SOURCE != TCP_IMAGE_SOURCE_VS
#error "TCP_IMAGE_SOURCE must select TRACK or VS"
#endif

// 静态对象，仅在当前文件内可见，符合封装原则
zf_driver_tcp_client tcp_client_dev;
static std::atomic<bool> g_tcp_camera_send_failed(false);
static uint16 g_tcp_camera_image[image_height][image_width] = {{0}};

// 指针池：用于保存外部需要监控的变量地址
static float *monitor_ch[4] = {nullptr, nullptr, nullptr, nullptr};

// =========================================================
// 逐飞库底层适配函数 (Static 保护，防止外部误调)
// =========================================================
static uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    const uint32 sent = tcp_client_dev.send_data(buf, len);
    if (sent != len)
    {
        g_tcp_camera_send_failed.store(true);
    }
    return sent;
}
static uint32 tcp_read_wrap(uint8 *buf, uint32 len) { return tcp_client_dev.read_data(buf, len); }

/**
 * @brief  TCP 调试初始化
 * @param  ip   电脑 IP 地址
 * @param  port 端口号 (通常是 8086)
 * @return true 成功, false 失败
 */
bool tcp_debug_init(const char *ip, int port)
{
    if (tcp_client_dev.init(ip, port) == 0)
    {
        tcp_client_dev.set_retry_param(2, 2);
        printf("TCP Client OK. 成功连接逐飞助手!\n");
        // 初始化逐飞助手的底层收发接口
        seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);
        return true;
    }
    else
    {
        printf("TCP Client Error. 检查 IP 或网络!\n");
        return false;
    }
}



/**
 * @brief  图传初始化
 * @param  ip   电脑 IP 地址
 * @param  port 端口号 (通常是 8086)
 * @return true 成功, false 失败
 */
bool tcp_image_transmission_init(const char *ip, int port)
{
    if (tcp_client_dev.init(ip, port) == 0)
    {
        tcp_client_dev.set_retry_param(2, 2);
        printf("TCP Client OK. 成功连接逐飞助手!\n");
        // 初始化逐飞助手的底层收发接口
        seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

#if TCP_IMAGE_SOURCE == TCP_IMAGE_SOURCE_VS
        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660,
                                                     g_vs.image_copy[0],
                                                     UVC_WIDTH,
                                                     UVC_HEIGHT);
#else
        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660,
                                                     g_tcp_camera_image[0],
                                                     image_width,
                                                     image_height);
#endif
        return true;
    }
    else
    {
        printf("TCP Client Error. 检查 IP 或网络!\n");
        return false;
    }

}

// VS模式生成彩色ROI；没有有效ROI时生成完整彩色调试图。巡线模式下为空操作。
void tcp_camera_update_vs_image(void)
{
#if TCP_IMAGE_SOURCE == TCP_IMAGE_SOURCE_VS
    if (!g_vs.build_color_block_roi_image())
    {
        g_vs.build_color_debug_image();
    }
#endif
}

// 将当前调试图复制到 TCP 专用缓冲区，避免网络发送期间占用图像锁。
void tcp_camera_snapshot_debug_image(void)
{
#if TCP_IMAGE_SOURCE == TCP_IMAGE_SOURCE_TRACK
    std::lock_guard<std::mutex> lock(g_image_mutex);
    std::memcpy(g_tcp_camera_image, debug_image, sizeof(g_tcp_camera_image));
#endif
}

//读取故障状态的同时把它清零
bool tcp_camera_send_failed(void)
{
    return g_tcp_camera_send_failed.exchange(false);
}

/**
 * @brief  绑定要监控的变量地址 (原 tcp_start_oscilloscope_timer)
 * @note   调用此函数仅仅是“登记”变量，并不开启定时器，心跳由调度器控制
 */
void tcp_bind_variables(float *ch0_ptr, float *ch1_ptr, float *ch2_ptr, float *ch3_ptr)
{
    monitor_ch[0] = ch0_ptr;
    monitor_ch[1] = ch1_ptr;
    monitor_ch[2] = ch2_ptr;
    monitor_ch[3] = ch3_ptr;
    printf("TCP 示波器变量绑定完成，等待调度器触发任务...\n");
}

/**
 * @brief  TCP 数据收发任务 (由 scheduler.cpp 定时调用)
 * @note   建议调度周期：20ms
 */
void tcp_update_task()
{
    uint8_t count = 0;

    // 1. 采集数据并压入发送队列
    for (int i = 0; i < 4; i++)
    {
        if (monitor_ch[i]) // 如果该通道绑定了变量
        {
            // 必须按顺序填入 data[0], data[1]...
            seekfree_assistant_oscilloscope_data.data[count] = *(monitor_ch[i]);
            count++;
        }
    }

    // 2. 如果有绑定的通道，执行数据发送
    if (count > 0)
    {
        seekfree_assistant_oscilloscope_data.channel_num = count;
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    }

    // 3. 关键：处理上位机下发的滑块参数 (保证在线调参响应)
    // 这个函数会读取 TCP 接收缓冲区的数据并更新 seekfree_assistant_parameter 数组
    seekfree_assistant_data_analysis();
}

/**
 * @brief  获取在线调参滑块的值
 * @param  index 通道索引 (0-7)
 * @return 滑块对应的浮点数值
 */
float get_online_param(uint8_t index)
{
    if (index > 7)
        return 0.0f;
    return seekfree_assistant_parameter[index];
}
