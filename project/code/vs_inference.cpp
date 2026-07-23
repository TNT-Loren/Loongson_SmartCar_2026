#include "vs_inference.hpp"
#include "vs_ai_stream.hpp"
#include "image_test.hpp"
#include <chrono>
#include <cstring>
#include <cmath>
#include <iostream>

VSInference g_vs;

static_assert(VS_TRACK_EDGE_X_MIN >= 0 &&
                  VS_TRACK_EDGE_X_MIN < VS_TRACK_EDGE_X_MAX &&
                  VS_TRACK_EDGE_X_MAX < UVC_WIDTH,
              "VS track edge X limits must be within the camera frame");

#if VS_AI_STREAM_FEATURE_ENABLE
static_assert(VS_BOX_SIZE == 64,
              "VS AI stream protocol currently requires a 64x64 model ROI");
#endif

// ===================================================================
// 内部类型别名（隐藏逐飞 SDK 具体类型）
// ===================================================================
using UVCDev = zf_device_uvc;
class LQ_NCNN;

// ===================================================================
// 辅助函数：Bresenham 直线（cv::Mat 版本）
// VS添加：移植自 zgc_draw_tool.cpp 的 dbg_line，适配 cv::Mat BGR 图像
// 纯整数 Bresenham 算法，无浮点、无除法，与 dbg_line 逻辑一致
// ===================================================================
static inline void bres_line(cv::Mat &img, int x1, int y1, int x2, int y2, const cv::Scalar &color)
{
    // VS添加：Bresenham 算法核心，与 zgc_draw_tool.cpp dbg_line 保持一致
    int dx = abs(x2 - x1);
    int sx = (x1 < x2) ? 1 : -1;
    int dy = -abs(y2 - y1);
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx + dy;

    cv::Vec3b c((uchar)color[0], (uchar)color[1], (uchar)color[2]);

    while (true)
    {
        if (x1 >= 0 && x1 < img.cols && y1 >= 0 && y1 < img.rows)
            img.at<cv::Vec3b>(y1, x1) = c;

        if (x1 == x2 && y1 == y2)
            break;

        int e2 = err << 1;
        if (e2 >= dy)
        {
            err += dy;
            x1 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

// ===================================================================
// 辅助函数：画水平线
// VS修改：原使用 cv::line，现改为调用 bres_line（Bresenham 算法，与 zgc_draw_tool.cpp dbg_line 一致）
// ===================================================================
// VS原始代码：
// static inline void h_line(cv::Mat& img, int y, const cv::Scalar& c) {
//     cv::line(img, cv::Point(0, y), cv::Point(img.cols - 1, y), c, 1);
// }
// VS修改后：
static inline void h_line(cv::Mat &img, int y, const cv::Scalar &c)
{
    bres_line(img, 0, y, img.cols - 1, y, c);
}

// ===================================================================
// classify_label — 标签到分类的映射（唯一维护点）
// ===================================================================
std::string VSInference::classify_label(const std::string &label)
{
    if (label == "急救包" || label == "望远镜")
        return "物资";
    if (label == "救护车" || label == "装甲车")
        return "载具";
    if (label == "爆炸物" || label == "枪械")
        return "武器";
    return "错误"; // 不应该出现的标签
}

// ===================================================================
// init — 初始化摄像头 + NCNN 模型 + 权重 LUT
//   不涉及任何 TCP/图传通信，由上层负责
// ===================================================================
bool VSInference::init(const std::vector<std::string> &_labels,
                       float mean_vals[3], float norm_vals[3])
{
    labels = _labels;
    initialized = false;
    red_warning.store(false, std::memory_order_relaxed);
    warning_armed = true;

    if (cfg.color_detect_y_max < 0 || cfg.color_detect_y_max >= UVC_HEIGHT ||
        cfg.finalize_y > cfg.color_detect_y_max)
    {
        printf("VS color detect Y must satisfy finalize_y <= max < %d, current finalize=%d max=%d\r\n",
               UVC_HEIGHT, cfg.finalize_y, cfg.color_detect_y_max);
        return false;
    }
    if (cfg.by_min < 0 || cfg.by_max <= cfg.by_min)
    {
        printf("VS Y range must satisfy 0 <= by_min < by_max, current=[%d, %d]\r\n",
               cfg.by_min, cfg.by_max);
        return false;
    }
#if VS_ENABLE_RED_MASK_CLOSE
    if (cfg.red_close_kernel_size_px <= 0 || cfg.red_close_iterations <= 0)
    {
        printf("VS red close config requires positive kernel px and iterations, current kernel_px=%d iterations=%d\r\n",
               cfg.red_close_kernel_size_px, cfg.red_close_iterations);
        return false;
    }
#endif
    if (cfg.finalize_y < cfg.by_max || cfg.finalize_y >= UVC_HEIGHT)
    {
        printf("VS finalize_y must be within [%d, %d], current=%d\r\n",
               cfg.by_max, UVC_HEIGHT - 1, cfg.finalize_y);
        return false;
    }

    // ---- 1. UVC 摄像头 ----
    if (ext_uvc_dev == nullptr)
    {
        // 未设置外部摄像头时，自己打开
        auto *uvc = new UVCDev();
        if (uvc->init(UVC_PATH) < 0)
        {
            printf("uvc init error\r\n");
            delete uvc;
            return false;
        }
#if VS_ENABLE_TERMINAL_OUTPUT
        printf("uvc init ok\r\n");
#endif
        uvc_dev = uvc;
    }
    else
    {
        // 使用外部共享摄像头，不重复 open /dev/video0
        uvc_dev = ext_uvc_dev;
#if VS_ENABLE_TERMINAL_OUTPUT
        printf("vs use external uvc ok\r\n");
#endif
    }

    // ---- 2. NCNN 模型 ----
    auto *net = new LQ_NCNN();
    net->SetModelPath(cfg.model_param.c_str(), cfg.model_bin.c_str());
    net->SetInputSize(cfg.box_size, cfg.box_size);
    net->SetLabels(labels);
    net->SetNormalize(mean_vals, norm_vals);
#if VS_ENABLE_TERMINAL_OUTPUT
    printf("正在加载NCNN模型...\n");
#endif
    if (!net->Init())
    {
        printf("NCNN模型加载失败!\n");
        delete net;
        if (ext_uvc_dev == nullptr)
            delete static_cast<UVCDev *>(uvc_dev);
        return false;
    }
#if VS_ENABLE_TERMINAL_OUTPUT
    printf("NCNN模型加载成功!\n");
#endif
    ncnn = net;

    // ---- 3. 预分配图像缓冲区（复用，避免每帧分配） ----
    src = cv::Mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC3);
    roi = cv::Mat(cfg.box_size, cfg.box_size, CV_8UC3);
    tx_frame = cv::Mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC3);

    // HSV 只处理 Y=0..color_detect_y_max，先裁剪再降采样以减少色域计算量。
    detect_h = cfg.color_detect_y_max + 1;
    if (cfg.hsv_scale > 1)
    {
        hsv_w = UVC_WIDTH / cfg.hsv_scale;
        hsv_h = (detect_h + cfg.hsv_scale - 1) / cfg.hsv_scale;
        src_small = cv::Mat(hsv_h, hsv_w, CV_8UC3);
    }
    else
    {
        hsv_w = UVC_WIDTH;
        hsv_h = detect_h;
    }

    // 缩小图尺寸不一定能被原图整除。预先记录每个缩小像素在320x240坐标中
    // 实际覆盖的宽高，后续面积阈值始终使用原图像素数，而不是缩小图像素数。
    hsv_x_pixel_weights.resize(hsv_w);
    for (int x = 0; x < hsv_w; ++x)
    {
        hsv_x_pixel_weights[x] =
            ((x + 1) * UVC_WIDTH / hsv_w) - (x * UVC_WIDTH / hsv_w);
    }
    hsv_y_pixel_weights.resize(hsv_h);
    for (int y = 0; y < hsv_h; ++y)
    {
        hsv_y_pixel_weights[y] =
            ((y + 1) * detect_h / hsv_h) - (y * detect_h / hsv_h);
    }

#if VS_ENABLE_RED_MASK_CLOSE
    // 调参值使用320x240原图像素，内部按实际缩放比例换算并保持奇数核。
    int close_w = std::max(1, (cfg.red_close_kernel_size_px * hsv_w + UVC_WIDTH / 2) / UVC_WIDTH);
    int close_h = std::max(1, (cfg.red_close_kernel_size_px * hsv_h + detect_h / 2) / detect_h);
    if ((close_w & 1) == 0)
        ++close_w;
    if ((close_h & 1) == 0)
        ++close_h;
    red_close_kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(close_w, close_h));
    // 闭运算包含膨胀和腐蚀，两阶段的总依赖半径用于局部ROI外扩。
    red_close_margin_x = (close_w / 2) * cfg.red_close_iterations * 2;
    red_close_margin_y = (close_h / 2) * cfg.red_close_iterations * 2;
#endif

    // ---- 4. 预计算 Y 方向指数权重 LUT ----
    lut_ofs = cfg.by_min;
    lut_size = cfg.by_max - cfg.by_min;
    for (int y = 0; y <= lut_size; y++)
    {
        int by = y + lut_ofs;
        float yn = (float)(by - cfg.by_min) / (cfg.by_max - cfg.by_min);
        if (yn < 0)
            yn = 0;
        if (yn > 1)
            yn = 1;
        weight_lut[y] = expf(cfg.exp_alpha * yn);
    }
#if VS_ENABLE_TERMINAL_OUTPUT
    printf("权重LUT初始化完成 (%d entries)\n", lut_size + 1);
#endif

    initialized = true;
    return true;
}

bool VSInference::init_smartcar_defaults(void *ext_uvc)
{
    set_external_camera(ext_uvc);

    // 默认参数统一来自 vs_inference.hpp 的“VS 调参区”宏。
    // 此处保留赋值，便于后续如果 main 侧先改 cfg，也能用默认配置一键覆盖。
    cfg.color_detect_y_max = VS_COLOR_DETECT_Y_MAX;
    cfg.red_close_kernel_size_px = VS_RED_CLOSE_KERNEL_SIZE_PX;
    cfg.red_close_iterations = VS_RED_CLOSE_ITERATIONS;
    cfg.box_size = VS_BOX_SIZE;
    cfg.box_y_offset = VS_BOX_Y_OFFSET_PX;
    cfg.area_min = VS_AREA_MIN;
    cfg.area_max = VS_AREA_MAX;
    cfg.hsv_scale = VS_HSV_SCALE;
    cfg.by_min = VS_BY_MIN;
    cfg.by_max = VS_BY_MAX;
    cfg.finalize_y = VS_FINALIZE_Y;
    cfg.lost_frames = VS_LOST_FRAMES;
    cfg.min_track = VS_MIN_TRACK;
    cfg.result_cooldown_ms = VS_RESULT_COOLDOWN_MS;
    cfg.exp_alpha = VS_EXP_ALPHA;

    // 标签顺序必须与模型输出顺序一致；mean/norm 必须与训练/导出预处理一致。
    static const std::vector<std::string> k_labels = {
        "爆炸物", "急救包", "救护车", "枪械", "望远镜", "装甲车"};
    float mean_vals[3] = {VS_NORM_MEAN};
    float norm_vals[3] = {VS_NORM_VAL};

    return init(k_labels, mean_vals, norm_vals);
}

// ===================================================================
// set_external_camera — 设置外部共享摄像头（避免重复 open /dev/video0）
//   调用时机：在 init() 之前调用
// ===================================================================
void VSInference::set_external_camera(void *ext_uvc)
{
    ext_uvc_dev = ext_uvc;
}

bool VSInference::set_cx_limit_mode(int mode)
{
    if (mode < VS_CX_LIMIT_BOTH || mode > VS_CX_LIMIT_RIGHT_ONLY)
    {
        return false;
    }
    cx_limit_mode.store(mode, std::memory_order_relaxed);
    return true;
}

int VSInference::get_cx_limit_mode() const
{
    return cx_limit_mode.load(std::memory_order_relaxed);
}

namespace
{
// 用相邻有效边线插值填补内部丢线；顶部/底部缺口沿用最近有效值。
// 若整侧都没有有效数据则保持 invalid，由目标筛选执行失败关闭。
void fill_track_edge_gaps(int *limits, bool *valid, int count)
{
    int first = 0;
    while (first < count && !valid[first])
    {
        ++first;
    }
    if (first == count)
    {
        return;
    }

    for (int y = 0; y < first; ++y)
    {
        limits[y] = limits[first];
        valid[y] = true;
    }

    int previous = first;
    for (int next = first + 1; next < count; ++next)
    {
        if (!valid[next])
        {
            continue;
        }

        const int span = next - previous;
        const int delta = limits[next] - limits[previous];
        for (int y = previous + 1; y < next; ++y)
        {
            limits[y] = limits[previous] + delta * (y - previous) / span;
            valid[y] = true;
        }
        previous = next;
    }

    for (int y = previous + 1; y < count; ++y)
    {
        limits[y] = limits[previous];
        valid[y] = true;
    }
}
} // namespace

// 将巡线模块发布的左右边线只读映射到 VS 有效检测区 Y=0..color_detect_y_max。
// 下半幅不参与映射；快照与正常巡线绘图共用数据源，但不回写边线数组。
void VSInference::update_track_edge_limits()
{
    {
        std::lock_guard<std::mutex> lock(g_image_mutex);
        for (int y = 0; y < detect_h; ++y)
        {
            const int track_y = std::clamp(y * image_height / UVC_HEIGHT,
                                           0, static_cast<int>(image_height) - 1);
            const int track_left = left_edge_line[track_y];
            const int track_right = right_edge_line[track_y];

            track_left_limit[y] = std::clamp(
                track_left * UVC_WIDTH / image_width,
                VS_TRACK_EDGE_X_MIN, VS_TRACK_EDGE_X_MAX);
            track_right_limit[y] = std::clamp(
                track_right * UVC_WIDTH / image_width,
                VS_TRACK_EDGE_X_MIN, VS_TRACK_EDGE_X_MAX);

            // 巡线以贴图像边缘的值表示丢线，不能把它当作 VS 的全幅有效区域。
            track_left_valid[y] =
                track_left > Border_Min && track_left < Border_Max;
            track_right_valid[y] =
                track_right > Border_Min && track_right < Border_Max;
        }
    }

    fill_track_edge_gaps(track_left_limit, track_left_valid, detect_h);
    fill_track_edge_gaps(track_right_limit, track_right_valid, detect_h);
}

bool VSInference::cx_is_valid(int cx, int bottom_y, int mode) const
{
    const int y = std::clamp(bottom_y, 0, detect_h - 1);
    if (mode == VS_CX_LIMIT_LEFT_ONLY)
        return track_left_valid[y] && cx >= track_left_limit[y];
    if (mode == VS_CX_LIMIT_RIGHT_ONLY)
        return track_right_valid[y] && cx <= track_right_limit[y];
    return track_left_valid[y] && track_right_valid[y] &&
           track_right_limit[y] > track_left_limit[y] &&
           cx >= track_left_limit[y] && cx <= track_right_limit[y];
}

// ===================================================================
// tick — 单帧完整处理周期（内部 wait + RGB565 解码）
// ===================================================================
bool VSInference::tick()
{
    if (!initialized || uvc_dev == nullptr)
    {
        printf("VS tick called before successful init\r\n");
        return false;
    }

    auto *uvc = static_cast<UVCDev *>(uvc_dev);

    // ---- 6.1 阻塞等待新帧 ----
    if (uvc->wait_image_refresh() < 0)
    {
        printf("camera refresh error\r\n");
        return false;
    }

    // ---- 6.2 获取 RGB565 指针 ----
    rgb_image = uvc->get_rgb_image_ptr();
    if (!rgb_image)
        return true;

    // ---- 6.3 RGB565 → BGR ----
    for (int y = 0; y < UVC_HEIGHT; y++)
    {
        const uint16_t *s = rgb_image + y * UVC_WIDTH;
        uint8_t *d = src.ptr<uint8_t>(y);
        for (int x = 0; x < UVC_WIDTH; x++)
        {
            uint16_t v = s[x];
            d[3 * x] = (v & 0x1F) << 3;
            d[3 * x + 1] = ((v >> 5) & 0x3F) << 2;
            d[3 * x + 2] = ((v >> 11) & 0x1F) << 3;
        }
    }

    // ---- 6.4~6.8 核心处理 ----
    process_frame(src);

    // 绘制统一延迟到 build_color_block_roi_image()/build_color_debug_image()。
    // TCP 关闭时上层不会构建图传画面，因此矩形和辅助线都不会执行。

    // ---- 6.9 BGR → RGB565 写入 image_copy（不发送，由上层决定） ----
    bgr_to_rgb565(src);

    fps_count++;
    return true;
}

// ===================================================================
// tick_bgr — 处理外部传入的 BGR 帧（共享巡线同一帧，不调用 wait）
// ===================================================================
bool VSInference::tick_bgr(const cv::Mat &bgr)
{
    if (!initialized)
    {
        printf("VS tick_bgr called before successful init\r\n");
        return false;
    }
    if (bgr.empty() || bgr.type() != CV_8UC3 ||
        bgr.cols != UVC_WIDTH || bgr.rows != UVC_HEIGHT)
    {
        printf("VS invalid BGR frame: type=%d size=%dx%d, expected CV_8UC3 %dx%d\r\n",
               bgr.empty() ? -1 : bgr.type(), bgr.cols, bgr.rows,
               UVC_WIDTH, UVC_HEIGHT);
        return false;
    }

    // 直接拷贝外部 BGR 帧到内部工作缓冲区
    bgr.copyTo(src);

    // ---- 核心处理（色块检测 + NCNN 推理 + 跟踪状态机）----
    process_frame(src);

    // 图传输出统一由 build_color_block_roi_image()/build_color_debug_image() 按需生成。
    // 这里不执行任何绘制，也不做 RGB565 转换；TCP 关闭时没有绘制开销。
    fps_count++;
    return true;
}

// ===================================================================
// process_frame — 色块检测 → 轮廓筛选 → 跟踪状态机 → NCNN 推理
// ===================================================================
void VSInference::process_frame(cv::Mat &src)
{
    update_track_edge_limits();

    // ---- 先裁剪 Y=0..color_detect_y_max，再降采样；下半幅不进入色域计算 ----
    cv::Mat detect_source = src(cv::Rect(0, 0, UVC_WIDTH, detect_h));
    if (cfg.hsv_scale > 1)
    {
        // 最近邻缩放计算量更低，也不会把红色与背景线性混色造成新的掩膜断裂。
        cv::resize(detect_source, src_small, cv::Size(hsv_w, hsv_h),
                   0, 0, cv::INTER_NEAREST);
    }
    cv::Mat &detect = (cfg.hsv_scale > 1) ? src_small : detect_source;

    // ---- HSV + 红色双区间提取 ----
    cv::cvtColor(detect, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cfg.hsv1_low, cfg.hsv1_high, mask1);
    cv::inRange(hsv, cfg.hsv2_low, cfg.hsv2_high, mask2);
    cv::bitwise_or(mask1, mask2, mask);

    red_component_count = 1;
    red_component_areas.clear();
    red_component_roi = cv::boundingRect(mask);
    // boundingRect同时完成非零检查；空画面直接跳过补色和连通域扫描。
    if (red_component_roi.area() > 0)
    {
#if VS_ENABLE_RED_MASK_CLOSE
        // 只处理红色像素附近区域；外扩覆盖闭运算全部依赖范围，效果等同整图处理。
        const int x0 = std::max(0, red_component_roi.x - red_close_margin_x);
        const int y0 = std::max(0, red_component_roi.y - red_close_margin_y);
        const int x1 = std::min(mask.cols,
                                red_component_roi.x + red_component_roi.width + red_close_margin_x);
        const int y1 = std::min(mask.rows,
                                red_component_roi.y + red_component_roi.height + red_close_margin_y);
        red_component_roi = cv::Rect(x0, y0, x1 - x0, y1 - y0);
        cv::Mat close_roi = mask(red_component_roi);
        cv::morphologyEx(close_roi, close_roi, cv::MORPH_CLOSE, red_close_kernel,
                         cv::Point(-1, -1), cfg.red_close_iterations,
                         cv::BORDER_CONSTANT | cv::BORDER_ISOLATED,
                         cv::morphologyDefaultBorderValue());
#endif

        cv::Mat component_mask = mask(red_component_roi);
        red_component_count = cv::connectedComponentsWithStats(
            component_mask, red_component_labels, red_component_stats,
            red_component_centroids, 8, CV_32S);

        // 按缩放网格实际覆盖范围累计，结果单位严格对应320x240原图像素。
        red_component_areas.assign(red_component_count, 0);
        for (int y = 0; y < red_component_labels.rows; ++y)
        {
            const int *labels_row = red_component_labels.ptr<int>(y);
            const int source_y = y + red_component_roi.y;
            const int row_weight = hsv_y_pixel_weights[source_y];
            for (int x = 0; x < red_component_labels.cols; ++x)
            {
                const int label = labels_row[x];
                if (label > 0)
                {
                    const int source_x = x + red_component_roi.x;
                    red_component_areas[label] +=
                        hsv_x_pixel_weights[source_x] * row_weight;
                }
            }
        }
    }

    has_best = false;
    roi_valid = false;
    best_cx = -1;
    best_by = -1;
#if VS_ENABLE_TERMINAL_OUTPUT
    best_area = 0;
#endif

    int half = cfg.box_size / 2;
    const int current_cx_limit_mode =
        cx_limit_mode.load(std::memory_order_relaxed);

    // ---- 筛选最优目标 ----
    for (int i = 1; i < red_component_count; ++i)
    {
        const int sx = red_component_stats.at<int>(i, cv::CC_STAT_LEFT) + red_component_roi.x;
        const int sy = red_component_stats.at<int>(i, cv::CC_STAT_TOP) + red_component_roi.y;
        const int sw = red_component_stats.at<int>(i, cv::CC_STAT_WIDTH);
        const int sh = red_component_stats.at<int>(i, cv::CC_STAT_HEIGHT);

        // 按缩放网格边界映射，坐标单位始终是320x240原图像素。
        const int left = sx * UVC_WIDTH / hsv_w;
        const int right = (sx + sw) * UVC_WIDTH / hsv_w - 1;
        const int cx = (left + right) / 2;
        const int by = (sy + sh) * detect_h / hsv_h - 1;

        // 正式检测区使用巡线边线限幅；范围外轮廓不再计算面积和后续条件。
        if (!cx_is_valid(cx, by, current_cx_limit_mode))
            continue;

        const int area = red_component_areas[i];

        // 过滤噪点（面积阈值在原始分辨率空间，不受 hsv_scale 影响）
        if (area < cfg.area_min)
            continue;

        // 所有正式检测区域统一使用同一个面积上限。
        if (area > cfg.area_max)
            continue;

        cv::Point tl(cx - half, by - cfg.box_size + 1 + cfg.box_y_offset);
        cv::Point br(cx + half - 1, by + cfg.box_y_offset);

        // 只允许已经建立的目标越过 LUT 上限继续靠近结算线；
        // IDLE/LOST 状态仍必须先在正常有效区内建立或恢复跟踪。
        const bool extending_track = (state == TRACKING || state == WAIT_CLEAR);
        const int y_max = extending_track
                              ? std::min(UVC_HEIGHT - 1, cfg.finalize_y + cfg.hsv_scale)
                              : cfg.by_max;
        bool yok = (by >= cfg.by_min && by <= y_max);
        if (!yok)
            continue;

        if (!has_best || by > best_by)
        {
            has_best = true;
            best_cx = cx;
            best_by = by;
#if VS_ENABLE_TERMINAL_OUTPUT
            best_area = area;
#endif
            best_tl = tl;
            best_br = br;
        }
    }

#if VS_ENABLE_TERMINAL_OUTPUT
    if (has_best)
        printf("[AREA] %d px^2 (320x240)\r\n", best_area);
#endif

    // 提前结算后锁定本目标，直到检测区连续空闲若干帧再允许下一轮识别。
    // 这样急救包上的红色区域不会被当成新色块继续推理或重复投票。
    if (state == WAIT_CLEAR)
    {
        if (has_best)
        {
            lost_cnt = 0;
        }
        else if (++lost_cnt >= cfg.lost_frames)
        {
            lost_cnt = 0;
            state = IDLE;
#if VS_ENABLE_TERMINAL_OUTPUT
            printf("[REARM] red target cleared, ready for next target\r\n");
#endif
        }
        return;
    }

    // ---- 跟踪状态机 ----
    int obj_by = best_by;

    // 建立跟踪后允许越过 LUT 上限到结算线；LUT 索引会钳制在末端。
    bool in_zone;
    if (state == TRACKING)
    {
        int m = cfg.hsv_scale; // 量化容差 (hsv_scale=4 → ±4px)
        in_zone = (has_best && obj_by >= cfg.by_min - m &&
                   obj_by <= cfg.finalize_y + m);
    }
    else
    {
        in_zone = (has_best && obj_by >= cfg.by_min && obj_by <= cfg.by_max);
    }

    if (in_zone)
    {
        int xs = std::max(0, best_tl.x), xe = std::min(UVC_WIDTH - 1, best_br.x);
        int ys = std::max(0, best_tl.y), ye = std::min(UVC_HEIGHT - 1, best_br.y);
        const cv::Mat cropped_roi = src(cv::Rect(xs, ys, xe - xs + 1, ye - ys + 1));
        if (cropped_roi.cols == cfg.box_size && cropped_roi.rows == cfg.box_size)
        {
            cropped_roi.copyTo(roi);
        }
        else
        {
            // 靠近图像边界时原始框会被裁短。这里提前复用 Infer() 原有的线性缩放，
            // 使模型和 AI 图传实际读取的始终是同一张固定尺寸 ROI。
            cv::resize(cropped_roi, roi,
                       cv::Size(cfg.box_size, cfg.box_size),
                       0, 0, cv::INTER_LINEAR);
        }
        roi_valid = !roi.empty();

        // 每轮目标在第一次得到有效 ROI 时立即预警，不再等待顶部预警区命中。
        if (roi_valid && warning_armed)
        {
            warning_armed = false;
            red_warning.store(true, std::memory_order_release);
#if VS_AI_STREAM_FEATURE_ENABLE && VS_AI_STREAM_MODE == 1
            // 发布触发本次预警的 320x240 原始全景帧；不裁剪、不缩放。
            if (vs_ai_stream_is_enabled())
            {
                vs_ai_stream_publish_warning_image(
                    src.ptr<std::uint8_t>(0), src.step,
                    static_cast<std::uint16_t>(best_cx),
                    static_cast<std::uint16_t>(obj_by));
            }
#endif
#if VS_ENABLE_TERMINAL_OUTPUT
            printf("[WARNING] first valid ROI center=(%d,%d)\r\n",
                   best_cx, obj_by);
#endif
        }

        try
        {
            auto *net = static_cast<LQ_NCNN *>(ncnn);
#if VS_ENABLE_TERMINAL_OUTPUT
            auto t0 = std::chrono::steady_clock::now();
#endif
            std::string r = net->Infer(roi);
#if VS_ENABLE_TERMINAL_OUTPUT
            auto t1 = std::chrono::steady_clock::now();
            long long us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
#endif
#if VS_AI_STREAM_FEATURE_ENABLE
            // Infer() 成功后才发布，并且必须直接使用本次推理的同一份 64x64 ROI。
            // 禁止在图传分支单独缩放，否则截图可能与模型实际输入不一致。
            if (vs_ai_stream_is_enabled() &&
                roi.type() == CV_8UC3 &&
                roi.cols == VS_BOX_SIZE && roi.rows == VS_BOX_SIZE)
            {
                vs_ai_stream_publish_image(
                    roi.ptr<std::uint8_t>(0), roi.step,
                    static_cast<std::uint16_t>(best_cx),
                    static_cast<std::uint16_t>(obj_by));
            }
#endif

            int yo = obj_by - lut_ofs;
            if (yo < 0)
                yo = 0;
            if (yo > lut_size)
                yo = lut_size;
            float w = weight_lut[yo];

            votes[r] += w;
            if (votes[r] > best_w)
            {
                best_w = votes[r];
                best_label = r;
            }

            if (state == LOST)
                lost_cnt = 0;
            state = TRACKING;
            tracked_by = obj_by;
            track_cnt++;

#if VS_ENABLE_TERMINAL_OUTPUT
            printf("[TRACK #%d] center=(%d,%d) w=%.2f -> %s | %s (%lld us)\r\n",
                   track_cnt, best_cx, obj_by, w,
                   r.c_str(), classify_label(r).c_str(), us);
#endif

            // 投票从进入有效区时就开始累计；到达结算线后立即使用现有票数输出，
            // 不再等待红色目标完全离开，也不再执行后续帧推理。
            if (obj_by >= cfg.finalize_y && track_cnt >= cfg.min_track)
            {
#if VS_ENABLE_TERMINAL_OUTPUT
                printf("[FINALIZE] target reached y=%d (line=%d)\r\n",
                       obj_by, cfg.finalize_y);
#endif
                output_final(true);
                state = WAIT_CLEAR;
                lost_cnt = 0;
                return;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[NCNN异常] " << e.what() << std::endl;
        }
    }
    else
    {
        if (state == TRACKING)
        {
            if (lost_cnt == 0)
            {
                lost_since = std::chrono::steady_clock::now();
            }
            lost_cnt++;
            if (lost_cnt >= cfg.lost_frames)
            {
                // 跟踪帧数不足 → 直接丢弃本次统计，复位到 IDLE
                if (track_cnt < cfg.min_track)
                {
#if VS_ENABLE_TERMINAL_OUTPUT
                    printf("[DROP] track_cnt=%d < min_track=%d, reset\n",
                           track_cnt, cfg.min_track);
#endif
                    track_cnt = 0;
                    lost_cnt = 0;
                    votes.clear();
                    best_w = 0;
                    best_label = "";
                    // 未形成有效投票结果，不重新布防；本轮预警仍只允许触发一次。
                    state = IDLE;
                }
                else
                {
                    state = LOST;
                }
            }
        }
        else if (state == LOST)
        {
            lost_cnt++;
        }
    }

    if (state == LOST && lost_cnt >= cfg.lost_frames && track_cnt >= cfg.min_track)
    {
        output_final(false);
    }
}

// ===================================================================
// build_color_block_roi_image — 将绿色检测框内 ROI 居中输出到 image_copy
//   调用时机：tick()/tick_bgr() 之后。
//   用途：黑色背景 + 原尺寸 ROI 居中 + 保留绿色框，不做放缩。
// ===================================================================
bool VSInference::build_color_block_roi_image()
{
    if (!initialized || !roi_valid || roi.empty())
    {
        return false;
    }

    tx_frame.setTo(cv::Scalar(0, 0, 0));
    const int roi_w = std::min(roi.cols, UVC_WIDTH);
    const int roi_h = std::min(roi.rows, UVC_HEIGHT);
    const int dst_x = (UVC_WIDTH - roi_w) / 2;
    const int dst_y = (UVC_HEIGHT - roi_h) / 2;

    cv::Rect src_rect(0, 0, roi_w, roi_h);
    cv::Rect dst_rect(dst_x, dst_y, roi_w, roi_h);
    roi(src_rect).copyTo(tx_frame(dst_rect));
    rectangle(tx_frame,
              cv::Point(dst_rect.x, dst_rect.y),
              cv::Point(dst_rect.x + dst_rect.width - 1,
                        dst_rect.y + dst_rect.height - 1),
              cv::Scalar(0, 255, 0),
              1);

    bgr_to_rgb565(tx_frame);
    return true;
}

// ===================================================================
// build_color_debug_image — 输出 VS 整帧彩色调试界面
//   调用时机：tick()/tick_bgr() 之后。
//   只在真正需要图传 VS 调试界面时才画绿色框和辅助线，降低普通推理路径开销。
// ===================================================================
bool VSInference::build_color_debug_image()
{
    if (!initialized || src.empty())
    {
        return false;
    }

    // 红色调参图只在上层实际请求构建图传画面时执行。
    if (red_tuning_view)
    {
        return build_red_tuning_debug_image();
    }

    const bool need_box = roi_valid;
#if !VS_ENABLE_GUIDELINES
    if (!need_box)
    {
        bgr_to_rgb565(src);
        return true;
    }
#endif

    src.copyTo(tx_frame);
    if (need_box)
    {
        const int xs = std::max(0, best_tl.x);
        const int xe = std::min(UVC_WIDTH - 1, best_br.x);
        const int ys = std::max(0, best_tl.y);
        const int ye = std::min(UVC_HEIGHT - 1, best_br.y);
        if (xe >= xs && ye >= ys)
        {
            rectangle(tx_frame,
                      cv::Point(xs, ys),
                      cv::Point(xe, ye),
                      cv::Scalar(0, 255, 0),
                      1);
        }
    }
#if VS_ENABLE_GUIDELINES
    draw_guidelines(tx_frame);
#endif
    bgr_to_rgb565(tx_frame);
    return true;
}

// ===================================================================
// build_red_tuning_debug_image — 红色阈值调参视图
//   暗红：仅通过 HSV；绿色：能够形成有效 ROI 的正式检测区域。
//   仅在调参视图被选择且上层请求图传画面时运行，不进入普通推理路径。
// ===================================================================
bool VSInference::build_red_tuning_debug_image()
{
    if (mask.empty())
    {
        return false;
    }

    red_debug_normal_mask.create(mask.rows, mask.cols, CV_8UC1);
    red_debug_normal_mask.setTo(cv::Scalar(0));
    red_debug_full_mask.create(UVC_HEIGHT, UVC_WIDTH, CV_8UC1);
    red_debug_full_mask.setTo(cv::Scalar(0));

    const int scale = std::max(1, cfg.hsv_scale);
    const bool extending_track = (state == TRACKING || state == WAIT_CLEAR);
    const int normal_y_max = extending_track
                                 ? std::min(UVC_HEIGHT - 1, cfg.finalize_y + scale)
                                 : cfg.by_max;
    const int current_cx_limit_mode =
        cx_limit_mode.load(std::memory_order_relaxed);

    std::vector<uint8_t> component_types(red_component_count, 0);
    for (int i = 1; i < red_component_count; ++i)
    {
        const int sx = red_component_stats.at<int>(i, cv::CC_STAT_LEFT) + red_component_roi.x;
        const int sy = red_component_stats.at<int>(i, cv::CC_STAT_TOP) + red_component_roi.y;
        const int sw = red_component_stats.at<int>(i, cv::CC_STAT_WIDTH);
        const int sh = red_component_stats.at<int>(i, cv::CC_STAT_HEIGHT);
        const int left = sx * UVC_WIDTH / hsv_w;
        const int right = (sx + sw) * UVC_WIDTH / hsv_w - 1;
        const int cx = (left + right) / 2;
        const int by = (sy + sh) * detect_h / hsv_h - 1;

        if (!cx_is_valid(cx, by, current_cx_limit_mode))
            continue;

        const int area = red_component_areas[i];

        const int normal_area_max = cfg.area_max;
        const bool normal_valid =
            by >= cfg.by_min && by <= normal_y_max &&
            area >= cfg.area_min && area <= normal_area_max;

        if (normal_valid)
            component_types[i] = 1;
    }

    // 根据主检测已生成的标签着色，不再重复提取或绘制轮廓。
    if (red_component_count > 1)
    {
        for (int y = 0; y < red_component_labels.rows; ++y)
        {
            const int *labels_row = red_component_labels.ptr<int>(y);
            uint8_t *normal_row = red_debug_normal_mask.ptr<uint8_t>(y + red_component_roi.y);
            for (int x = 0; x < red_component_labels.cols; ++x)
            {
                const uint8_t type = component_types[labels_row[x]];
                if (type != 0)
                    normal_row[x + red_component_roi.x] = 255;
            }
        }
    }

    tx_frame.setTo(cv::Scalar(0, 0, 0));

    // 所有 HSV 命中显示为暗红，便于观察颜色阈值是否过宽。
    cv::Mat full_detect_roi =
        red_debug_full_mask(cv::Rect(0, 0, UVC_WIDTH, detect_h));
    cv::resize(mask, full_detect_roi, cv::Size(UVC_WIDTH, detect_h),
               0, 0, cv::INTER_NEAREST);
    tx_frame.setTo(cv::Scalar(0, 0, 96), red_debug_full_mask);

    // 通过正式色块检测区筛选的区域覆盖为绿色。
    red_debug_full_mask.setTo(cv::Scalar(0));
    cv::resize(red_debug_normal_mask, full_detect_roi,
               cv::Size(UVC_WIDTH, detect_h), 0, 0, cv::INTER_NEAREST);
    tx_frame.setTo(cv::Scalar(0, 255, 0), red_debug_full_mask);

#if VS_ENABLE_GUIDELINES
    draw_guidelines(tx_frame);
#endif
    bgr_to_rgb565(tx_frame);
    return true;
}

void VSInference::cycle_color_debug_view()
{
    red_tuning_view = !red_tuning_view;
}

const char *VSInference::get_color_debug_view_name() const
{
    return red_tuning_view ? "RED_TUNING" : "COLOR";
}

// ===================================================================
// output_final — 汇总投票并输出最终结果
// ===================================================================
void VSInference::output_final(bool immediate)
{
    const auto now = std::chrono::steady_clock::now();
    if (last_result_time != std::chrono::steady_clock::time_point::min())
    {
        const long long cooldown_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_result_time).count();
        if (cooldown_ms < cfg.result_cooldown_ms)
        {
#if VS_ENABLE_TERMINAL_OUTPUT
            printf("[COOLDOWN] drop final result, remain=%lld ms\n",
                   static_cast<long long>(cfg.result_cooldown_ms) - cooldown_ms);
#endif
            state = IDLE;
            track_cnt = 0;
            lost_cnt = 0;
            votes.clear();
            best_w = 0;
            best_label = "";
            // 冷却期丢弃的结果未对外发布，不重新布防红色预警。
            return;
        }
    }

    final_cat = classify_label(best_label);
    final_label = best_label;
    final_weight = best_w;
    final_frames = track_cnt;
    final_lost_ms = immediate
                        ? 0
                        : std::chrono::duration_cast<std::chrono::milliseconds>(now - lost_since).count();
    result_ready = true;
    last_result_time = now;

    // 只有成功产生投票结果后，才允许下一个目标再次触发一次预警。
    // 已发布的事件保留到 consume_red_warning()，保证同帧结算时不会丢失预警。
    warning_armed = true;

#if VS_ENABLE_TERMINAL_OUTPUT
    printf("\n========================================\n");
    printf("[FINAL] label: %s | category: %s | weight: %.2f | frames: %d\n",
           final_label.c_str(), final_cat.c_str(), final_weight, final_frames);
    printf("[FINAL] vote breakdown:\n");
    for (auto &kv : votes)
        printf("  %s: %.2f\n", kv.first.c_str(), kv.second);
    printf("========================================\n\n");
#endif

    state = IDLE;
    track_cnt = 0;
    lost_cnt = 0;
    votes.clear();
    best_w = 0;
    best_label = "";
}

// ===================================================================
// draw_guidelines — 辅助线叠加
// ===================================================================
void VSInference::draw_guidelines(cv::Mat &src)
{
    h_line(src, cfg.by_min, cv::Scalar(0, 0, 255));      // 红色: 有效 ROI 区 Y 下界
    h_line(src, cfg.by_max, cv::Scalar(0, 255, 255));     // 黄色: 有效区 Y 上界
    h_line(src, cfg.finalize_y, cv::Scalar(255, 0, 255)); // 紫红色: 提前结算线
    const int mode = cx_limit_mode.load(std::memory_order_relaxed);
    const int edge_rows = std::min(src.rows, detect_h);
    if (mode != VS_CX_LIMIT_RIGHT_ONLY)
    {
        for (int y = 1; y < edge_rows; ++y)
        {
            if (!track_left_valid[y - 1] || !track_left_valid[y])
                continue;
            bres_line(src,
                      track_left_limit[y - 1], y - 1,
                      track_left_limit[y], y,
                      cv::Scalar(0, 255, 255));
        }
    }
    if (mode != VS_CX_LIMIT_LEFT_ONLY)
    {
        for (int y = 1; y < edge_rows; ++y)
        {
            if (!track_right_valid[y - 1] || !track_right_valid[y])
                continue;
            bres_line(src,
                      track_right_limit[y - 1], y - 1,
                      track_right_limit[y], y,
                      cv::Scalar(0, 255, 255));
        }
    }
}

// ===================================================================
// bgr_to_rgb565 — BGR 图像转换为 RGB565 写入 image_copy
//   上层调用 seekfree_assistant_camera_send() 将 image_copy 发出
// ===================================================================
void VSInference::bgr_to_rgb565(cv::Mat &src)
{
    for (int y = 0; y < src.rows; ++y)
    {
        const uint8_t *row = src.ptr<uint8_t>(y);
        uint16_t *dst = image_copy[y];
        for (int x = 0; x < src.cols; ++x)
        {
            uint8_t b = row[3 * x], g = row[3 * x + 1], r = row[3 * x + 2];
            uint16_t v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            dst[x] = ((v & 0xFF) << 8) | ((v & 0xFF00) >> 8);
        }
    }
}

// ===================================================================
// 结果查询接口
// ===================================================================
bool VSInference::has_new_result() const { return result_ready; }
bool VSInference::has_red_warning() const
{
    return red_warning.load(std::memory_order_acquire);
}
bool VSInference::consume_red_warning()
{
    return red_warning.exchange(false, std::memory_order_acq_rel);
}
bool VSInference::consume_new_result(std::string &result)
{
    if (!result_ready)
    {
        return false;
    }

    result = final_cat;
    printf("Final Result: %s\r\n", result.c_str());
    printf("物体离开等待: %lld ms\r\n", final_lost_ms);
    result_ready = false;
    return true;
}
std::string VSInference::get_label() const { return final_label; }
std::string VSInference::get_result() const { return final_cat; }
std::string VSInference::get_category() const { return final_cat; }
float VSInference::get_weight() const { return final_weight; }
int VSInference::get_frames() const { return final_frames; }
void VSInference::clear_result() { result_ready = false; }
int VSInference::take_fps_count()
{
    const int count = fps_count;
    fps_count = 0;
    return count;
}

bool VSInference::is_tracking() const { return track_cnt >= cfg.min_track; }
std::string VSInference::get_tracking_label() const { return best_label; }
std::string VSInference::get_tracking_result() const { return classify_label(best_label); }
long long VSInference::get_lost_ms() const
{
    if (state != LOST)
        return 0;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - lost_since)
        .count();
}
