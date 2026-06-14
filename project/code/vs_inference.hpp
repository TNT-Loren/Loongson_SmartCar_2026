#pragma once

#include <string>
#include <vector>
#include <map>
#include "zf_common_headfile.hpp"

// ===================================================================
// VSConfig — 所有可调参数集中在此结构体，init() 之前修改即生效
// ===================================================================
struct VSConfig {
    // ---- 使能开关 ----
    bool en_terminal_output = false; // 终端输出使能（默认关闭，检测期间不发送任何信息）
    bool en_guidelines     = true;  // 辅助线显示使能（叠加到输出画面）

    // ---- 色块检测参数 ----
    int hsv_scale     = 4;       // HSV降采样（1=320×240, 2=160×120, 4=80×60）
    int box_size      = 64;      // 绿框大小（像素），须与模型输入尺寸一致
    int area_min      = 30;      // 色块最小面积（过滤噪点，单位 px²）
    int area_max      = 600;     // 色块最大面积（下半区/有效区使用）
    int zone_end_y    = 120;     // 两段分区边界 Y 坐标
    int zone_area_max = 300;     // 上半区面积上限（比下半区更严格，抑制顶端干扰）

    // ---- 有效区域（绿框底部中心坐标范围，在此范围内才参与检测和跟踪） ----
    int cx_min = 80;             // 底部中心 X 下限
    int cx_max = 240;            // 底部中心 X 上限
    int by_min = 100;            // 底部中心 Y 下限（也是权重 LUT 归一化基准）
    int by_max = 180;            // 底部中心 Y 上限

    // ---- HSV 红色阈值（用于提取红色色块） ----
    cv::Scalar hsv1_low{0,   150, 100};   // 区间1 下限 (H=0~10)
    cv::Scalar hsv1_high{10,  255, 255};  // 区间1 上限
    cv::Scalar hsv2_low{160, 150, 100};   // 区间2 下限 (H=160~179)
    cv::Scalar hsv2_high{179, 255, 255};  // 区间2 上限

    // ---- 跟踪参数 ----
    int   lost_frames = 20;      // 连续丢失帧数阈值（达到后确认物体已离开）
    int   min_track   = 8;       // 最少跟踪帧数（低于此值不输出最终结果）
    float exp_alpha   = 2.5f;    // Y 指数权重陡峭度（越大底层帧权重越高）

    // ---- 模型文件路径 ----
    std::string model_param = "tiny_classifier_fp32.ncnn.param";
    std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
};

// ===================================================================
// VSInference — 色块检测 + NCNN 推理 + 跟踪状态机（下层模块）
//
// 不负责任何通信/TCP/图传发送，只做图像处理与推理。
// 上层调用流程：
//   1. 创建对象，通过 cfg 设置参数
//   2. 调用 init() 初始化摄像头/模型
//   3. 循环调用 tick() 处理每一帧
//   4. tick() 后将 image_copy[] 发送到上位机（由上层负责）
//   5. 通过 has_new_result() / get_result() 获取最终检测结果
// ===================================================================
class VSInference {
public:
    VSConfig cfg;                       // 所有可调参数（init 前修改）

    // ---- 输出缓冲区（tick() 后更新，上层负责发送） ----
    uint16_t image_copy[UVC_HEIGHT][UVC_WIDTH];

    // ---- 生命周期 ----
    // 初始化摄像头和 NCNN 模型，返回 true 成功
    bool init(const std::vector<std::string>& labels,
              float mean_vals[3], float norm_vals[3]);

    // 设置外部摄像头（共享已有 uvc_dev，避免重复 open /dev/video0）
    void set_external_camera(void* ext_uvc);

    // tick() — 单帧处理（内部 wait + RGB565 解码），返回 false 表示摄像头采集失败
    bool tick();

    // tick_bgr() — 处理外部传入的 BGR 帧（不调用 wait，共享巡线的同一帧）
    bool tick_bgr(const cv::Mat& bgr);

    // ---- 上层获取最终推理结果（物体离开有效区后输出一次） ----
    bool        has_new_result() const;  // 是否有新的最终结果待读取
    std::string get_label()      const;  // 原始推理标签，如 "急救包" / "救护车" / "枪械"
    std::string get_result()     const;  // 分类结果，如 "物资" / "载具" / "武器"
    std::string get_category()   const;  // 分类结果（同 get_result）

    // ---- 实时追踪结果（跟踪中即时可用，不等物体离开）----
    bool        is_tracking()        const;  // 是否正在跟踪目标（track_cnt >= min_track）
    std::string get_tracking_label() const;  // 当前跟踪中最高票标签
    std::string get_tracking_result() const; // 当前跟踪中分类结果
    float       get_weight()     const;  // 累积权重值
    int         get_frames()     const;  // 累计跟踪帧数
    long long   get_lost_ms()    const;  // 物体离开后已等待的毫秒数（仅 LOST 状态有效）
    void        clear_result()        ;  // 清除结果标志，准备下一次

private:
    // ===== 设备句柄 =====
    void* uvc_dev     = nullptr;        // UVC 摄像头（内部创建，ext_uvc_dev 为空时使用）
    void* ext_uvc_dev = nullptr;        // 外部传入的摄像头（共享巡线的 uvc_dev）
    void* ncnn        = nullptr;        // NCNN 推理引擎

    // ===== 图像缓冲区（预分配复用，避免每帧 malloc/free） =====
    uint16_t* rgb_image = nullptr;      // 摄像头原始 RGB565 指针
    cv::Mat   src;                      // BGR 工作图像 (UVC_WIDTH × UVC_HEIGHT × 3)
    cv::Mat   roi;                      // NCNN 推理 ROI  (box_size × box_size × 3)
    cv::Mat   src_small;                // HSV 降采样缓冲区（预分配复用）
    int       hsv_w = 0, hsv_h = 0;     // 降采样后宽高

    // ===== 色块检测中间变量 =====
    cv::Mat hsv, mask1, mask2, mask;
    bool    has_best  = false;
    int     best_by   = -1;
    int     best_area = 0;
    cv::Point best_tl, best_br;

    // ===== 模型标签 =====
    std::vector<std::string> labels;

    // ===== 指数权重 LUT（启动时预计算，运行时 O(1) 查表） =====
    float weight_lut[200];
    int   lut_ofs = 0, lut_size = 0;

    // ===== 跟踪状态机 =====
    enum { IDLE, TRACKING, LOST } state = IDLE;
    int     tracked_cx = 0, tracked_by = 0;
    int     track_cnt  = 0, lost_cnt = 0;
    float   best_w     = 0;
    std::string best_label;
    std::map<std::string, float> votes;
    std::chrono::steady_clock::time_point lost_since;  // TRACKING→LOST 时刻

    // ===== 最终结果缓存 =====
    std::string final_label, final_cat;
    float final_weight = 0;
    int   final_frames = 0;
    bool  result_ready = false;

    // ===== 内部方法 =====
    void process_frame(cv::Mat& src);
    void output_final();
    void draw_guidelines(cv::Mat& src);
    void bgr_to_rgb565(cv::Mat& src);

    static std::string classify_label(const std::string& label);
};
