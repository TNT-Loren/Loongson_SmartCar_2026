#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// VS AI 图片上传总开关。最终识别结果仍使用原有终端输出，不走此端口。
#ifndef VS_AI_STREAM_FEATURE_ENABLE
#define VS_AI_STREAM_FEATURE_ENABLE (1) // 0=完全编译移除，1=保留运行时开关
#endif

// AI ROI 图传模式：0=手动模式（每按一次 U 发送下一张有效 ROI）；
//                   1=自动模式（按 U 启停，开启后连续发送 ROI）。
#ifndef VS_AI_STREAM_MODE
#define VS_AI_STREAM_MODE (1)
#endif

#if VS_AI_STREAM_MODE != 0 && VS_AI_STREAM_MODE != 1
#error "VS_AI_STREAM_MODE must be 0 (manual) or 1 (automatic)"
#endif

#define VS_AI_STREAM_SERVER_IP "192.168.31.10" // 默认上位机 IP
#define VS_AI_STREAM_IMAGE_PORT (8091)          // VSAI 图片端口
#define VS_AI_STREAM_DEFAULT_ENABLED (VS_AI_STREAM_MODE) // 自动模式随程序启动，手动模式等待 U 键

/// 上位机连接配置。
struct VSAiStreamConfig
{
    std::string server_ip = VS_AI_STREAM_SERVER_IP;       ///< 上位机 IPv4 地址
    std::uint16_t image_port = VS_AI_STREAM_IMAGE_PORT;   ///< 上位机 TCP 监听端口
};

/// 更新上位机地址和端口，不会立即建立连接。
/// @param config 新的 TCP 连接配置。
void vs_ai_stream_configure(const VSAiStreamConfig &config);

/// 设置图传服务总开关；关闭时会取消手动请求并丢弃待发送图片。
/// @param enabled true 开启服务，false 关闭服务。
void vs_ai_stream_set_enabled(bool enabled);

/// 处理现有 U 键动作。
/// 手动模式下登记一次截图请求并返回 true；自动模式下切换服务开关并返回新状态。
/// @return 调用后的服务开启状态。
bool vs_ai_stream_toggle_enabled();

/// 查询图传服务是否开启。手动模式开启但尚未按 U 时也会返回 true。
/// @return true 表示服务允许接收截图，false 表示服务关闭。
bool vs_ai_stream_is_enabled();

/// 发布与 Infer() 共用的 64x64 BGR 原始 ROI。
/// 线上载荷会按模型预处理转换为 RGB888；队列只保留最新图片，不阻塞视觉线程。
/// @param bgr64 64x64、三通道、uint8 的 BGR ROI 首地址。
/// @param row_stride 相邻两行首地址之间的字节数，必须至少为 64*3。
/// @param center_x 目标中心在原始 320x240 图像中的 X 坐标。
/// @param bottom_y 目标底部在原始 320x240 图像中的 Y 坐标。
/// @return 成功入队时的图片序号；服务关闭、手动模式未触发或参数无效时返回 0。
std::uint32_t vs_ai_stream_publish_image(const std::uint8_t *bgr64,
                                         std::size_t row_stride,
                                         std::uint16_t center_x,
                                         std::uint16_t bottom_y);

/// 发布红色预警触发帧中的 320x240 BGR 全景图。
/// 仅自动模式会发送；手动模式直接返回 0，且不会消费 U 键登记的截图请求。
/// 线上使用 VSAI version=2、type=2 的全景图消息，像素载荷为 RGB888。
/// @return 自动模式下成功入队时的图片序号，其他情况返回 0。
std::uint32_t vs_ai_stream_publish_warning_image(const std::uint8_t *bgr320x240,
                                                 std::size_t row_stride,
                                                 std::uint16_t center_x,
                                                 std::uint16_t bottom_y);
