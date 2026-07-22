#include "vs_ai_stream.hpp"

#if VS_AI_STREAM_FEATURE_ENABLE

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/resource.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr std::uint8_t k_protocol_version = 1; // VSAI 协议版本
constexpr std::uint8_t k_message_image = 1;    // 图片消息类型
constexpr std::uint8_t k_pixel_rgb888 = 2;     // RGB888 像素格式编号
constexpr std::size_t k_header_size = 36;      // VSAI 固定包头字节数
constexpr std::size_t k_image_width = 64;      // 模型输入宽度
constexpr std::size_t k_image_height = 64;     // 模型输入高度
constexpr std::size_t k_image_channels = 3;    // RGB 通道数
constexpr std::size_t k_image_bytes = k_image_width * k_image_height * k_image_channels;
constexpr auto k_reconnect_delay = std::chrono::milliseconds(1000); // 重连间隔
constexpr int k_connect_timeout_ms = 500; // 单次 TCP 连接超时
constexpr int k_send_timeout_ms = 250;    // 单次 TCP 发送超时

// 图传服务总开关。视觉线程和网络线程通过原子变量同步，避免加锁读取。
std::atomic<bool> g_stream_enabled{false};
#if VS_AI_STREAM_MODE == 0
// 手动模式的一次性截图请求：U 键置 true，下一张有效 ROI 消费后恢复 false。
std::atomic<bool> g_manual_image_requested{false};
#endif

// 一条等待封包发送的模型输入图片及其定位信息。
struct ImageMessage
{
    std::uint32_t sequence = 0;  // 图片序号
    std::uint64_t timestamp_ms = 0; // 单调时钟时间戳，单位毫秒
    std::uint16_t center_x = 0;  // 目标中心在原始图像中的 X 坐标
    std::uint16_t bottom_y = 0;  // 目标底部在原始图像中的 Y 坐标
    std::array<std::uint8_t, k_image_bytes> pixels{}; // 64x64 RGB888 载荷
};

// 返回当前单调时钟的毫秒值，用于图片时间戳。
std::uint64_t monotonic_ms()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

// 按网络大端序追加一个 16 位无符号整数。
void append_u16(std::vector<std::uint8_t> &out, std::uint16_t value)
{
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
    out.push_back(static_cast<std::uint8_t>(value & 0xff));
}

// 按网络大端序追加一个 32 位无符号整数。
void append_u32(std::vector<std::uint8_t> &out, std::uint32_t value)
{
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xff));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xff));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
    out.push_back(static_cast<std::uint8_t>(value & 0xff));
}

// 按网络大端序追加一个 64 位无符号整数。
void append_u64(std::vector<std::uint8_t> &out, std::uint64_t value)
{
    append_u32(out, static_cast<std::uint32_t>(value >> 32));
    append_u32(out, static_cast<std::uint32_t>(value & 0xffffffffULL));
}

// 将图片元数据和 RGB888 像素组装成一条完整 VSAI 消息。
std::vector<std::uint8_t> make_image_packet(const ImageMessage &message)
{
    std::vector<std::uint8_t> packet;
    packet.reserve(k_header_size + message.pixels.size());
    packet.insert(packet.end(), {'V', 'S', 'A', 'I'});
    packet.push_back(k_protocol_version);
    packet.push_back(k_message_image);
    append_u16(packet, static_cast<std::uint16_t>(k_header_size));
    append_u32(packet, message.sequence);
    append_u64(packet, message.timestamp_ms);
    append_u32(packet, static_cast<std::uint32_t>(message.pixels.size()));
    append_u16(packet, k_image_width);
    append_u16(packet, k_image_height);
    packet.push_back(k_image_channels);
    packet.push_back(k_pixel_rgb888);
    append_u16(packet, message.center_x);
    append_u16(packet, message.bottom_y);
    append_u16(packet, 0);
    packet.insert(packet.end(), message.pixels.begin(), message.pixels.end());
    return packet;
}

// 在限定时间内连接上位机；成功返回 socket，失败返回 -1。
int connect_with_timeout(const std::string &ip, std::uint16_t port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;

    const int original_flags = fcntl(fd, F_GETFL, 0);
    if (original_flags < 0 || fcntl(fd, F_SETFL, original_flags | O_NONBLOCK) < 0)
    {
        close(fd);
        return -1;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1)
    {
        close(fd);
        return -1;
    }

    int result = connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address));
    if (result < 0 && errno != EINPROGRESS)
    {
        close(fd);
        return -1;
    }

    if (result < 0)
    {
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(fd, &write_set);
        timeval timeout{};
        timeout.tv_sec = k_connect_timeout_ms / 1000;
        timeout.tv_usec = (k_connect_timeout_ms % 1000) * 1000;
        result = select(fd + 1, nullptr, &write_set, nullptr, &timeout);
        if (result <= 0)
        {
            close(fd);
            return -1;
        }

        int socket_error = 0;
        socklen_t error_size = sizeof(socket_error);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &error_size) < 0 ||
            socket_error != 0)
        {
            close(fd);
            return -1;
        }
    }

    if (fcntl(fd, F_SETFL, original_flags) < 0)
    {
        close(fd);
        return -1;
    }
    timeval send_timeout{};
    send_timeout.tv_sec = k_send_timeout_ms / 1000;
    send_timeout.tv_usec = (k_send_timeout_ms % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
    return fd;
}

// 循环发送完整消息，处理 TCP 短写；全部成功时返回 true。
bool send_packet(int fd, const std::vector<std::uint8_t> &packet)
{
    std::size_t offset = 0;
    while (offset < packet.size())
    {
        const ssize_t sent = send(fd, packet.data() + offset,
                                  packet.size() - offset, MSG_NOSIGNAL);
        if (sent <= 0)
            return false;
        offset += static_cast<std::size_t>(sent);
    }
    return true;
}

// 异步图片发送服务。视觉线程只更新最新图片，网络线程负责连接和发送。
class VSAiImageStreamService
{
public:
    // 创建服务并启动低优先级网络线程。
    VSAiImageStreamService()
        : image_thread_(&VSAiImageStreamService::image_loop, this)
    {
    }

    // 通知网络线程退出并等待线程回收。
    ~VSAiImageStreamService()
    {
        stopping_.store(true, std::memory_order_release);
        state_cv_.notify_all();
        image_cv_.notify_all();
        if (image_thread_.joinable())
            image_thread_.join();
    }

    // 更新上位机地址和端口；本函数不会主动建立连接。
    void configure(const VSAiStreamConfig &config)
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
    }

    // 开启或关闭后台服务；关闭时丢弃尚未发送的图片。
    void set_enabled(bool enabled)
    {
        enabled_.store(enabled, std::memory_order_release);
        if (!enabled)
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            image_pending_ = false;
        }
        state_cv_.notify_all();
        image_cv_.notify_all();
    }

    // 返回后台服务当前是否开启。
    bool enabled() const
    {
        return enabled_.load(std::memory_order_acquire);
    }

    // 复制一张 BGR ROI，按模型预处理转为 RGB888 后交给网络线程。
    std::uint32_t publish_image(const std::uint8_t *image,
                                std::size_t row_stride,
                                std::uint16_t center_x,
                                std::uint16_t bottom_y)
    {
        if (!enabled() || image == nullptr ||
            row_stride < k_image_width * k_image_channels)
        {
            return 0;
        }

        ImageMessage message;
        message.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
        const std::uint32_t sequence = message.sequence;
        message.timestamp_ms = monotonic_ms();
        message.center_x = center_x;
        message.bottom_y = bottom_y;
        for (std::size_t row = 0; row < k_image_height; ++row)
        {
            const std::uint8_t *source = image + row * row_stride;
            std::uint8_t *destination =
                message.pixels.data() + row * k_image_width * k_image_channels;
            for (std::size_t column = 0; column < k_image_width; ++column)
            {
                // OpenCV ROI 是 BGR；模型在 Infer() 中使用的输入顺序是 RGB。
                destination[column * 3] = source[column * 3 + 2];
                destination[column * 3 + 1] = source[column * 3 + 1];
                destination[column * 3 + 2] = source[column * 3];
            }
        }

        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = std::move(message);
            image_pending_ = true;
        }
        image_cv_.notify_one();
        return sequence;
    }

private:
    // 获取线程安全的连接配置副本。
    VSAiStreamConfig config_snapshot()
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return config_;
    }

    // 连接失败后等待重试；关闭服务时提前结束等待。
    void wait_reconnect()
    {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_cv_.wait_for(lock, k_reconnect_delay, [this] {
            return stopping_.load(std::memory_order_acquire) || !enabled();
        });
    }

    // 网络线程主循环：等待图片、按需连接、发送并处理重连。
    void image_loop()
    {
        setpriority(PRIO_PROCESS, 0, 19);
        int socket_fd = -1;
        while (!stopping_.load(std::memory_order_acquire))
        {
            ImageMessage message;
            {
                std::unique_lock<std::mutex> lock(image_mutex_);
                image_cv_.wait(lock, [this] {
                    return stopping_.load(std::memory_order_acquire) ||
                           !enabled() || image_pending_;
                });
                if (stopping_.load(std::memory_order_acquire))
                    break;
                if (!enabled())
                {
                    if (socket_fd >= 0)
                    {
                        close(socket_fd);
                        socket_fd = -1;
                    }
                    image_cv_.wait(lock, [this] {
                        return stopping_.load(std::memory_order_acquire) || enabled();
                    });
                    continue;
                }
                if (!image_pending_)
                    continue;
                message = latest_image_;
                image_pending_ = false;
            }

            if (!enabled())
                continue;
            if (socket_fd < 0)
            {
                const VSAiStreamConfig config = config_snapshot();
                socket_fd = connect_with_timeout(config.server_ip, config.image_port);
            }
            if (socket_fd < 0 || !send_packet(socket_fd, make_image_packet(message)))
            {
                if (socket_fd >= 0)
                    close(socket_fd);
                socket_fd = -1;
                wait_reconnect();
            }
        }
        if (socket_fd >= 0)
            close(socket_fd);
    }

    std::atomic<bool> enabled_{false};  // 后台服务运行开关
    std::atomic<bool> stopping_{false}; // 对象析构时的线程退出标志
    std::atomic<std::uint32_t> next_sequence_{1}; // 下一张图片序号

    std::mutex config_mutex_; // 保护连接配置
    VSAiStreamConfig config_; // 当前上位机地址和端口

    std::mutex state_mutex_;              // 配合重连等待条件变量
    std::condition_variable state_cv_;    // 服务状态变化通知

    std::mutex image_mutex_;           // 保护待发送图片
    std::condition_variable image_cv_; // 新图片或服务状态变化通知
    ImageMessage latest_image_;        // 队列中保留的最新图片
    bool image_pending_ = false;       // 是否存在待发送图片

    std::thread image_thread_; // 后台 TCP 发送线程
};

// 返回进程内唯一的图传服务实例，首次使用时创建。
VSAiImageStreamService &service()
{
    static VSAiImageStreamService instance;
    return instance;
}
} // namespace

// 保存连接配置，实际连接由第一次待发送图片触发。
void vs_ai_stream_configure(const VSAiStreamConfig &config)
{
    service().configure(config);
}

// 控制图传服务生命周期；关闭服务时同步清理手动请求和待发送图片。
void vs_ai_stream_set_enabled(bool enabled)
{
    if (enabled)
    {
        if (g_stream_enabled.load(std::memory_order_acquire))
            return;
        service().set_enabled(true);
        g_stream_enabled.store(true, std::memory_order_release);
    }
    else
    {
#if VS_AI_STREAM_MODE == 0
        g_manual_image_requested.store(false, std::memory_order_release);
#endif
        if (!g_stream_enabled.exchange(false, std::memory_order_acq_rel))
            return;
        service().set_enabled(false);
    }
}

// 响应 U 键：手动模式请求一张，自动模式切换连续发送开关。
bool vs_ai_stream_toggle_enabled()
{
#if VS_AI_STREAM_MODE == 0
    // main.cpp 的 U 键调用此接口。手动模式只登记一次发送请求，
    // 请求会一直保留到视觉线程产生下一张有效 ROI。
    if (!g_stream_enabled.load(std::memory_order_acquire))
        vs_ai_stream_set_enabled(true);
    g_manual_image_requested.store(true, std::memory_order_release);
    return true;
#else
    const bool enabled = !g_stream_enabled.load(std::memory_order_acquire);
    vs_ai_stream_set_enabled(enabled);
    return enabled;
#endif
}

// 返回图传服务总开关状态，不代表手动模式当前已有截图请求。
bool vs_ai_stream_is_enabled()
{
    return g_stream_enabled.load(std::memory_order_acquire);
}

// 根据当前模式决定是否放行 ROI，然后交给后台服务复制和发送。
std::uint32_t vs_ai_stream_publish_image(const std::uint8_t *bgr64,
                                         std::size_t row_stride,
                                         std::uint16_t center_x,
                                         std::uint16_t bottom_y)
{
    if (!g_stream_enabled.load(std::memory_order_acquire))
        return 0;
#if VS_AI_STREAM_MODE == 0
    if (bgr64 == nullptr || row_stride < k_image_width * k_image_channels)
        return 0;
    if (!g_manual_image_requested.exchange(false, std::memory_order_acq_rel))
        return 0;
#endif
    return service().publish_image(bgr64, row_stride, center_x, bottom_y);
}

#endif // VS_AI_STREAM_FEATURE_ENABLE
