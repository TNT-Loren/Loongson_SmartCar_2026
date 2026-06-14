#include "vs_inference.hpp"
#include <chrono>
#include <cstring>
#include <cmath>
#include <iostream>

// ===================================================================
// 内部类型别名（隐藏逐飞 SDK 具体类型）
// ===================================================================
using UVCDev = zf_device_uvc;
class LQ_NCNN;

// ===================================================================
// 辅助函数：画水平线
// ===================================================================
static inline void h_line(cv::Mat& img, int y, const cv::Scalar& c) {
    cv::line(img, cv::Point(0, y), cv::Point(img.cols - 1, y), c, 1);
}
        
// ===================================================================
// classify_label — 标签到分类的映射（唯一维护点）
// ===================================================================
std::string VSInference::classify_label(const std::string& label) {
    if (label == "急救包" || label == "望远镜")      return "物资";
    if (label == "救护车" || label == "装甲车")      return "载具";
    if (label == "爆炸物" || label == "枪械")        return "武器";
    return "错误"; // 不应该出现的标签
}

// ===================================================================
// init — 初始化摄像头 + NCNN 模型 + 权重 LUT
//   不涉及任何 TCP/图传通信，由上层负责
// ===================================================================
bool VSInference::init(const std::vector<std::string>& _labels,
                       float mean_vals[3], float norm_vals[3])
{
    labels = _labels;

    // ---- 1. UVC 摄像头 ----
    if (ext_uvc_dev == nullptr) {
        // 未设置外部摄像头时，自己打开
        auto* uvc = new UVCDev();
        if (uvc->init(UVC_PATH) < 0) {
            printf("uvc init error\r\n");
            delete uvc; return false;
        }
        if (cfg.en_terminal_output) printf("uvc init ok\r\n");
        uvc_dev = uvc;
    } else {
        // 使用外部共享摄像头，不重复 open /dev/video0
        uvc_dev = ext_uvc_dev;
        if (cfg.en_terminal_output) printf("vs use external uvc ok\r\n");
    }

    // ---- 2. NCNN 模型 ----
    auto* net = new LQ_NCNN();
    net->SetModelPath(cfg.model_param.c_str(), cfg.model_bin.c_str());
    net->SetInputSize(cfg.box_size, cfg.box_size);
    net->SetLabels(labels);
    net->SetNormalize(mean_vals, norm_vals);
    if (cfg.en_terminal_output) printf("正在加载NCNN模型...\n");
    if (!net->Init()) {
        printf("NCNN模型加载失败!\n");
        delete net;
        if (ext_uvc_dev == nullptr) delete static_cast<UVCDev*>(uvc_dev);
        return false;
    }
    if (cfg.en_terminal_output) printf("NCNN模型加载成功!\n");
    ncnn = net;

    // ---- 3. 预分配图像缓冲区（复用，避免每帧分配） ----
    src = cv::Mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC3);
    roi = cv::Mat(cfg.box_size, cfg.box_size, CV_8UC3);

    // HSV 降采样缓冲区
    if (cfg.hsv_scale > 1) {
        hsv_w = UVC_WIDTH  / cfg.hsv_scale;
        hsv_h = UVC_HEIGHT / cfg.hsv_scale;
        src_small = cv::Mat(hsv_h, hsv_w, CV_8UC3);
    } else {
        hsv_w = UVC_WIDTH;
        hsv_h = UVC_HEIGHT;
    }

    // ---- 4. 预计算 Y 方向指数权重 LUT ----
    lut_ofs  = cfg.by_min;
    lut_size = cfg.by_max - cfg.by_min;
    for (int y = 0; y <= lut_size; y++) {
        int by = y + lut_ofs;
        float yn = (float)(by - cfg.by_min) / (cfg.by_max - cfg.by_min);
        if (yn < 0) yn = 0;
        if (yn > 1) yn = 1;
        weight_lut[y] = expf(cfg.exp_alpha * yn);
    }
    if (cfg.en_terminal_output) printf("权重LUT初始化完成 (%d entries)\n", lut_size + 1);

    return true;
}

// ===================================================================
// set_external_camera — 设置外部共享摄像头（避免重复 open /dev/video0）
//   调用时机：在 init() 之前调用
// ===================================================================
void VSInference::set_external_camera(void* ext_uvc)
{
    ext_uvc_dev = ext_uvc;
}

// ===================================================================
// tick — 单帧完整处理周期（内部 wait + RGB565 解码）
// ===================================================================
bool VSInference::tick()
{
    auto* uvc = static_cast<UVCDev*>(uvc_dev);

    // ---- 6.1 阻塞等待新帧 ----
    if (uvc->wait_image_refresh() < 0) {
        printf("camera refresh error\r\n");
        return false;
    }

    // ---- 6.2 获取 RGB565 指针 ----
    rgb_image = uvc->get_rgb_image_ptr();
    if (!rgb_image) return true;

    // ---- 6.3 RGB565 → BGR ----
    for (int y = 0; y < UVC_HEIGHT; y++) {
        const uint16_t* s = rgb_image + y * UVC_WIDTH;
        uint8_t* d = src.ptr<uint8_t>(y);
        for (int x = 0; x < UVC_WIDTH; x++) {
            uint16_t v = s[x];
            d[3*x]     = (v & 0x1F) << 3;
            d[3*x+1]   = ((v >> 5) & 0x3F) << 2;
            d[3*x+2]   = ((v >> 11) & 0x1F) << 3;
        }
    }

    // ---- 6.4~6.8 核心处理 ----
    process_frame(src);

    // ---- 6.9 辅助线 ----
    if (cfg.en_guidelines) draw_guidelines(src);

    // ---- 6.10 BGR → RGB565 写入 image_copy（不发送，由上层决定） ----
    bgr_to_rgb565(src);

    return true;
}

// ===================================================================
// tick_bgr — 处理外部传入的 BGR 帧（共享巡线同一帧，不调用 wait）
// ===================================================================
bool VSInference::tick_bgr(const cv::Mat& bgr)
{
    // 直接拷贝外部 BGR 帧到内部工作缓冲区
    bgr.copyTo(src);

    // ---- 核心处理（色块检测 + NCNN 推理 + 跟踪状态机）----
    process_frame(src);

    // ---- 图传输出（仅图传开启时执行，关图传时跳过省算力）----
    if (cfg.en_guidelines) {
        draw_guidelines(src);
        bgr_to_rgb565(src);
    }

    return true;
}

// ===================================================================
// process_frame — 色块检测 → 轮廓筛选 → 跟踪状态机 → NCNN 推理
// ===================================================================
void VSInference::process_frame(cv::Mat& src)
{
    // ---- 选择检测图像（降采样以降低算力）----
    cv::Mat& detect = (cfg.hsv_scale > 1) ? src_small : src;
    if (cfg.hsv_scale > 1)
        cv::resize(src, src_small, cv::Size(hsv_w, hsv_h));

    // ---- HSV + 红色双区间提取 ----
    cv::cvtColor(detect, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cfg.hsv1_low, cfg.hsv1_high, mask1);
    cv::inRange(hsv, cfg.hsv2_low, cfg.hsv2_high, mask2);
    mask = mask1 | mask2;

    // ---- 查找轮廓 ----
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    has_best  = false;
    best_by   = -1;
    best_area = 0;

    int half = cfg.box_size / 2;

    // ---- 筛选最优目标 ----
    for (size_t i = 0; i < contours.size(); i++) {
        int area = (int)contourArea(contours[i]);

        cv::RotatedRect rr = minAreaRect(contours[i]);
        cv::Point2f v[4]; rr.points(v);
        int by = (int)v[0].y;
        for (int j = 1; j < 4; j++)
            if (v[j].y > by) by = v[j].y;

        int cx = (int)rr.center.x;

        // ---- 坐标从降采样空间映射回原始分辨率（320×240）----
        if (cfg.hsv_scale > 1) {
            cx   *= cfg.hsv_scale;
            by   *= cfg.hsv_scale;
            area *= (cfg.hsv_scale * cfg.hsv_scale);
        }

        // 过滤噪点（面积阈值在原始分辨率空间，不受 hsv_scale 影响）
        if (area < cfg.area_min) continue;

        // 两段分区面积上限
        int amax = (by < cfg.zone_end_y) ? cfg.zone_area_max : cfg.area_max;
        if (area > amax) continue;

        cv::Point tl(cx - half, by - cfg.box_size + 1);
        cv::Point br(cx + half - 1, by);

        bool xok = (cx >= cfg.cx_min && cx <= cfg.cx_max);
        bool yok = (by >= cfg.by_min && by <= cfg.by_max);
        if (!xok || !yok) continue;

        if (!has_best || by > best_by) {
            has_best  = true;
            best_by   = by;
            best_area = area;
            best_tl   = tl;
            best_br   = br;
        }
    }

    if (cfg.en_terminal_output && has_best)
        printf("[AREA] %d px\r\n", best_area);

    // ---- 跟踪状态机 ----
    int obj_cx  = (best_tl.x + best_br.x) / 2;
    int obj_by  = best_by;

    // 有效区判定：跟踪中放宽边界，补偿 hsv 降采样量化抖动
    bool in_zone;
    if (state == TRACKING) {
        int m = cfg.hsv_scale;  // 量化容差 (hsv_scale=4 → ±4px)
        in_zone = (has_best && obj_by >= cfg.by_min - m && obj_by <= cfg.by_max + m);
    } else {
        in_zone = (has_best && obj_by >= cfg.by_min && obj_by <= cfg.by_max);
    }

    if (in_zone) {
        rectangle(src, best_tl, best_br, cv::Scalar(0, 255, 0), 1);

        int xs = std::max(0, best_tl.x), xe = std::min(UVC_WIDTH - 1, best_br.x);
        int ys = std::max(0, best_tl.y), ye = std::min(UVC_HEIGHT - 1, best_br.y);
        src(cv::Rect(xs, ys, xe - xs + 1, ye - ys + 1)).copyTo(roi);

        try {
            auto* net = static_cast<LQ_NCNN*>(ncnn);
            auto t0 = std::chrono::steady_clock::now();
            std::string r = net->Infer(roi);
            auto t1 = std::chrono::steady_clock::now();
            long long us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

            int yo = obj_by - lut_ofs;
            if (yo < 0) yo = 0;
            if (yo > lut_size) yo = lut_size;
            float w = weight_lut[yo];

            votes[r] += w;
            if (votes[r] > best_w) { best_w = votes[r]; best_label = r; }

            if (state == LOST) lost_cnt = 0;
            state = TRACKING;
            tracked_by = obj_by;
            track_cnt++;

            if (cfg.en_terminal_output) {
                printf("[TRACK #%d] center=(%d,%d) w=%.2f -> %s | %s (%lld us)\r\n",
                       track_cnt, obj_cx, obj_by, w,
                       r.c_str(), classify_label(r).c_str(), us);
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[NCNN异常] " << e.what() << std::endl;
        }
    }
    else {
        if (state == TRACKING) {
            lost_cnt++;
            if (lost_cnt >= cfg.lost_frames) {
                // 跟踪帧数不足 → 直接丢弃本次统计，复位到 IDLE
                if (track_cnt < cfg.min_track) {
                    if (cfg.en_terminal_output)
                        printf("[DROP] track_cnt=%d < min_track=%d, reset\n",
                               track_cnt, cfg.min_track);
                    track_cnt = 0;
                    lost_cnt = 0;
                    votes.clear();
                    best_w = 0;
                    best_label = "";
                    state = IDLE;
                } else {
                    state = LOST;
                    lost_since = std::chrono::steady_clock::now();
                }
            }
        }
        else if (state == LOST) {
            lost_cnt++;
        }
    }

    if (state == LOST && lost_cnt >= cfg.lost_frames && track_cnt >= cfg.min_track) {
        output_final();
    }
}

// ===================================================================
// output_final — 物体消失后汇总投票，输出最终结果
// ===================================================================
void VSInference::output_final()
{
    final_cat   = classify_label(best_label);
    final_label = best_label;
    final_weight = best_w;
    final_frames = track_cnt;
    result_ready = true;

    if (cfg.en_terminal_output) {
        printf("\n========================================\n");
        printf("[FINAL] label: %s | category: %s | weight: %.2f | frames: %d\n",
               final_label.c_str(), final_cat.c_str(), final_weight, final_frames);
        printf("[FINAL] vote breakdown:\n");
        for (auto& kv : votes) printf("  %s: %.2f\n", kv.first.c_str(), kv.second);
        printf("========================================\n\n");
    }

    state = IDLE; track_cnt = 0; lost_cnt = 0;
    votes.clear(); best_w = 0; best_label = "";
}

// ===================================================================
// draw_guidelines — 辅助线叠加
// ===================================================================
void VSInference::draw_guidelines(cv::Mat& src)
{
    h_line(src, cfg.zone_end_y, cv::Scalar(0, 0, 255));     // 红色: 分区边界
    h_line(src, cfg.by_min,     cv::Scalar(0, 255, 255));    // 黄色: 有效区 Y 下界
    h_line(src, cfg.by_max,     cv::Scalar(0, 255, 255));    // 黄色: 有效区 Y 上界
    cv::line(src, cv::Point(cfg.cx_min, 0), cv::Point(cfg.cx_min, src.rows - 1),
             cv::Scalar(0, 255, 255), 1);
    cv::line(src, cv::Point(cfg.cx_max, 0), cv::Point(cfg.cx_max, src.rows - 1),
             cv::Scalar(0, 255, 255), 1);
}

// ===================================================================
// bgr_to_rgb565 — BGR 图像转换为 RGB565 写入 image_copy
//   上层调用 seekfree_assistant_camera_send() 将 image_copy 发出
// ===================================================================
void VSInference::bgr_to_rgb565(cv::Mat& src)
{
    for (int y = 0; y < src.rows; ++y) {
        const uint8_t* row = src.ptr<uint8_t>(y);
        uint16_t* dst = image_copy[y];
        for (int x = 0; x < src.cols; ++x) {
            uint8_t b = row[3*x], g = row[3*x+1], r = row[3*x+2];
            uint16_t v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            dst[x] = ((v & 0xFF) << 8) | ((v & 0xFF00) >> 8);
        }
    }
}

// ===================================================================
// 结果查询接口
// ===================================================================
bool        VSInference::has_new_result() const { return result_ready; }
std::string VSInference::get_label()    const { return final_label; }
std::string VSInference::get_result()   const { return final_cat; }
std::string VSInference::get_category() const { return final_cat; }
float       VSInference::get_weight()   const { return final_weight; }
int         VSInference::get_frames()   const { return final_frames; }
void        VSInference::clear_result() { result_ready = false; }

bool        VSInference::is_tracking()        const { return track_cnt >= cfg.min_track; }
std::string VSInference::get_tracking_label() const { return best_label; }
std::string VSInference::get_tracking_result() const { return classify_label(best_label); }
long long   VSInference::get_lost_ms()    const {
    if (state != LOST) return 0;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - lost_since).count();
}
