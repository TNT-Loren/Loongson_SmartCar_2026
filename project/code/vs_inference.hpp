#pragma once

#include <atomic>
#include <string>
#include <vector>
#include <map>
#include "zf_common_headfile.hpp"

// ===================================================================
// VS 调参区：只改这里。坐标基于 320x240 彩色图。
// ===================================================================
// 开关：0=关闭，1=开启
#define VS_ENABLE_TERMINAL_OUTPUT (0) // 终端打印详细识别日志
#define VS_ENABLE_GUIDELINES (1)      // VS 彩色调试图中绘制辅助线

// 色块检测：先 HSV 找红色目标，再用 box 框出模型输入 ROI。
#define VS_COLOR_DETECT_Y_MAX (120) // 色域计算最大Y坐标（含），下方区域不参与HSV/轮廓计算
#define VS_HSV_SCALE (4)            // HSV降采样倍率；默认裁剪后约为80x31
#define VS_ENABLE_RED_MASK_CLOSE (1) // 连接降采样后相邻的红色色块，避免缩放造成轮廓断裂
#define VS_RED_CLOSE_KERNEL_SIZE_PX (12) // 闭运算核边长，单位为320x240原图px；内部自动换算
#define VS_RED_CLOSE_ITERATIONS (1)  // 闭运算次数；过大会把原本独立的目标合并
#define VS_BOX_SIZE (64)       // 绿色检测框大小；必须与 NCNN 输入尺寸一致
#define VS_BOX_Y_OFFSET_PX (5) // 检测框Y偏移：正=下移，负=上移，单位px
#define VS_AREA_MIN (24)       // 色块最小面积，单位为320x240原图px^2
#define VS_AREA_MAX (320)      // 有效区色块最大面积，单位为320x240原图px^2

// VS 左右边线 X 限制调参区：越界后强制锁定到对应边界。
#define VS_TRACK_EDGE_X_MIN (10)
#define VS_TRACK_EDGE_X_MAX (310)

// 起投与 LUT 区域：目标先在该范围内建立跟踪，越过 BY_MAX 后继续跟踪到结算线。
#define VS_CX_LIMIT_BOTH (0)       // 同时启用巡线左右边线限制
#define VS_CX_LIMIT_LEFT_ONLY (1)  // 只启用巡线左边线限制
#define VS_CX_LIMIT_RIGHT_ONLY (2) // 只启用巡线右边线限制
#define VS_CX_LIMIT_MODE (0)       // 初始模式：0=两侧，1=仅左侧，2=仅右侧
#if VS_CX_LIMIT_MODE < VS_CX_LIMIT_BOTH || VS_CX_LIMIT_MODE > VS_CX_LIMIT_RIGHT_ONLY
#error "VS_CX_LIMIT_MODE must be 0, 1 or 2"
#endif
// X 限制不再使用固定值，而是逐行读取巡线模块输出的左右边线。
#define VS_BY_MIN (50)      // 底部中心Y下限；也是权重LUT归一化起点
#define VS_BY_MAX (80)     // LUT归一化终点；超过后继续使用最大权重
#define VS_FINALIZE_Y (90) // 提前结算线

// HSV 红色双区间阈值：OpenCV HSV中红色跨 0/179，需要两段合并。
#define VS_HSV1_LOW 0, 150, 100
#define VS_HSV1_HIGH 10, 255, 255
#define VS_HSV2_LOW 160, 150, 100
#define VS_HSV2_HIGH 179, 255, 255

// 跟踪与输出：目标触及提前结算线或离开有效区后，输出一次累计投票结果。
#define VS_LOST_FRAMES (2)           // 连续丢失多少帧后确认目标离开
#define VS_MIN_TRACK (2)             // 至少跟踪多少帧才认为结果有效
#define VS_RESULT_COOLDOWN_MS (2000) // 两次最终结果输出之间的冷却时间，单位ms
#define VS_EXP_ALPHA (2.5f)          // Y方向指数权重；越大越偏向近处目标

// NCNN 模型与归一化：必须与训练/导出模型时保持一致。
// 龙邱原模型训练结果
#define VS_MODEL_PARAM_PATH        "tiny_classifier_fp32.ncnn.param"
#define VS_MODEL_BIN_PATH          "tiny_classifier_fp32.ncnn.bin"
#define VS_NORM_MEAN               123.675f, 116.28f, 103.53f
#define VS_NORM_VAL                0.01712475f, 0.017507f, 0.01742919f

// 激进模型1，速度未对比，测试的时候效果比第1次的好
// #define VS_MODEL_PARAM_PATH        "falsh_tiny_classifier_fp32.ncnn.param"
// #define VS_MODEL_BIN_PATH          "falsh_tiny_classifier_fp32.ncnn.bin"
// #define VS_NORM_MEAN               123.675f, 116.28f, 103.53f
// #define VS_NORM_VAL                0.01712475f, 0.017507f, 0.01742919f

// 激进模型2，速度未对比，测试的时候效果比第2次的好
// #define VS_MODEL_PARAM_PATH "v3_tiny_classifier_fp32.ncnn.param"
// #define VS_MODEL_BIN_PATH "v3_tiny_classifier_fp32.ncnn.bin"
// #define VS_NORM_MEAN 151.602920f, 144.057952f, 147.296495f
// #define VS_NORM_VAL 0.025728557f, 0.019134327f, 0.027740937f

// ===================================================================
// VSConfig：运行时配置镜像。默认值全部来自上方宏，通常只改“VS 调参区”。
// ===================================================================
struct VSConfig
{
    int color_detect_y_max = VS_COLOR_DETECT_Y_MAX;
    int hsv_scale = VS_HSV_SCALE;
    int red_close_kernel_size_px = VS_RED_CLOSE_KERNEL_SIZE_PX;
    int red_close_iterations = VS_RED_CLOSE_ITERATIONS;
    int box_size = VS_BOX_SIZE;
    int box_y_offset = VS_BOX_Y_OFFSET_PX;
    int area_min = VS_AREA_MIN;
    int area_max = VS_AREA_MAX;

    int by_min = VS_BY_MIN;
    int by_max = VS_BY_MAX;
    int finalize_y = VS_FINALIZE_Y;

    cv::Scalar hsv1_low{VS_HSV1_LOW};
    cv::Scalar hsv1_high{VS_HSV1_HIGH};
    cv::Scalar hsv2_low{VS_HSV2_LOW};
    cv::Scalar hsv2_high{VS_HSV2_HIGH};

    int lost_frames = VS_LOST_FRAMES;
    int min_track = VS_MIN_TRACK;
    int result_cooldown_ms = VS_RESULT_COOLDOWN_MS;
    float exp_alpha = VS_EXP_ALPHA;

    std::string model_param = VS_MODEL_PARAM_PATH;
    std::string model_bin = VS_MODEL_BIN_PATH;
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
class VSInference
{
public:
    VSConfig cfg; // 所有可调参数（init 前修改）

    // ---- 输出缓冲区（tick() 后更新，上层负责发送） ----
    uint16_t image_copy[UVC_HEIGHT][UVC_WIDTH] = {};

    // ---- 生命周期 ----
    // 初始化摄像头和 NCNN 模型，返回 true 成功
    bool init(const std::vector<std::string> &labels,
              float mean_vals[3], float norm_vals[3]);

    // 智能车当前默认 VS 配置：共享外部摄像头，设置色块检测/跟踪/模型参数并初始化。
    bool init_smartcar_defaults(void *ext_uvc);

    // 设置外部摄像头（共享已有 uvc_dev，避免重复 open /dev/video0）
    void set_external_camera(void *ext_uvc);

    // 运行时动态边线限幅模式：0=左右开启，1=仅左开启，2=仅右开启。
    // 可在其他cpp中通过 g_vs.set_cx_limit_mode(mode) 随时切换。
    bool set_cx_limit_mode(int mode);
    int get_cx_limit_mode() const;

    // tick() — 单帧处理（内部 wait + RGB565 解码），返回 false 表示摄像头采集失败
    bool tick();

    // tick_bgr() — 处理外部传入的 BGR 帧（不调用 wait，共享巡线的同一帧）
    bool tick_bgr(const cv::Mat &bgr);

    // 将最近一帧绿色检测框内的内容按原尺寸居中写入 image_copy，供上层原分辨率图传发送。
    // 返回 false 表示当前没有有效色块 ROI，image_copy 不更新。
    bool build_color_block_roi_image();

    // 将最近一帧 VS 彩色调试画面写入 image_copy，供上层原分辨率图传发送。
    // 包含色块绿色框和辅助线；返回 false 表示当前没有可用彩色帧。
    bool build_color_debug_image();

    // 键盘调参视图：完整彩色图 <-> 红色阈值有效区。
    void cycle_color_debug_view();
    const char *get_color_debug_view_name() const;

    // ---- 上层获取最终推理结果（触及结算线或离开有效区后输出一次） ----
    bool has_new_result() const;                  // 是否有新的最终结果待读取
    bool consume_new_result(std::string &result); // 读取、打印并清除一次最终结果
    bool has_red_warning() const;                 // 首个有效 ROI 是否产生了预警事件
    bool consume_red_warning();                   // 读取并清除一次预警事件
    std::string get_label() const;                // 原始推理标签，如 "急救包" / "救护车" / "枪械"
    std::string get_result() const;               // 分类结果，如 "物资" / "载具" / "武器"
    std::string get_category() const;             // 分类结果（同 get_result）

    // ---- 实时追踪结果（跟踪中即时可用，不等物体离开）----
    bool is_tracking() const;                // 是否正在跟踪目标（track_cnt >= min_track）
    std::string get_tracking_label() const;  // 当前跟踪中最高票标签
    std::string get_tracking_result() const; // 当前跟踪中分类结果
    float get_weight() const;                // 累积权重值
    int get_frames() const;                  // 累计跟踪帧数
    long long get_lost_ms() const;           // 物体离开后已等待的毫秒数（仅 LOST 状态有效）
    void clear_result();                     // 清除结果标志，准备下一次
    int take_fps_count();                    // 读取并清零成功处理的帧数

private:
    // ===== 设备句柄 =====
    void *uvc_dev = nullptr;     // UVC 摄像头（内部创建，ext_uvc_dev 为空时使用）
    void *ext_uvc_dev = nullptr; // 外部传入的摄像头（共享巡线的 uvc_dev）
    void *ncnn = nullptr;        // NCNN 推理引擎
    bool initialized = false;    // init() 完整成功后才允许处理图像

    // ===== 图像缓冲区（预分配复用，避免每帧 malloc/free） =====
    uint16_t *rgb_image = nullptr;  // 摄像头原始 RGB565 指针
    cv::Mat src;                    // BGR 工作图像 (UVC_WIDTH × UVC_HEIGHT × 3)
    cv::Mat roi;                    // NCNN 推理 ROI  (box_size × box_size × 3)
    cv::Mat src_small;              // HSV 降采样缓冲区（预分配复用）
    cv::Mat tx_frame;               // 图传输出工作缓冲，避免每帧 clone/zeros 反复分配
    cv::Mat red_debug_normal_mask;  // 调参视图中的有效检测掩码（按需分配）
    cv::Mat red_debug_full_mask;    // 调参视图的原分辨率掩码（按需分配）
    int detect_h = 0;               // 参与色域计算的原图高度
    int hsv_w = 0, hsv_h = 0;       // 降采样后宽高
    bool red_tuning_view = false;   // false=完整彩色图，true=红色阈值调参图

    // ===== 色块检测中间变量 =====
    cv::Mat hsv, mask1, mask2, mask;
    cv::Mat red_component_labels;    // 缩小掩膜的连通域标签（CV_32S）
    cv::Mat red_component_stats;     // 连通域包围框等统计结果
    cv::Mat red_component_centroids; // OpenCV连通域接口输出，主逻辑不使用浮点中心
    std::vector<int> red_component_areas; // 每个连通域映射到320x240后的实际像素面积
    std::vector<int> hsv_x_pixel_weights; // 缩小图每列代表的原图像素宽度
    std::vector<int> hsv_y_pixel_weights; // 缩小图每行代表的原图像素高度
    cv::Rect red_component_roi;       // 当前帧局部补色/连通域区域（缩小图坐标）
    int red_component_count = 1;     // 包含0号背景标签
#if VS_ENABLE_RED_MASK_CLOSE
    cv::Mat red_close_kernel;       // init时创建，运行时复用
    int red_close_margin_x = 0;     // 保证局部闭运算结果与整图处理一致的外扩量
    int red_close_margin_y = 0;
#endif
    bool has_best = false;
    bool roi_valid = false;
    int best_cx = -1;
    int best_by = -1;
#if VS_ENABLE_TERMINAL_OUTPUT
    int best_area = 0;
#endif
    cv::Point best_tl, best_br;

    // ===== 模型标签 =====
    std::vector<std::string> labels;

    // ===== 指数权重 LUT（启动时预计算，运行时 O(1) 查表） =====
    float weight_lut[200];
    int lut_ofs = 0, lut_size = 0;

    // ===== 跟踪状态机 =====
    enum
    {
        IDLE,
        TRACKING,
        LOST,
        WAIT_CLEAR
    } state = IDLE;
    int tracked_cx = 0, tracked_by = 0;
    int track_cnt = 0, lost_cnt = 0;
    float best_w = 0;
    std::string best_label;
    std::map<std::string, float> votes;
    std::chrono::steady_clock::time_point lost_since; // 连续丢失的第一帧时刻

    // ===== 最终结果缓存 =====
    std::string final_label, final_cat;
    float final_weight = 0;
    int final_frames = 0;
    bool result_ready = false;
    long long final_lost_ms = 0;
    int fps_count = 0;
    std::atomic<bool> red_warning{false};
    std::atomic<int> cx_limit_mode{VS_CX_LIMIT_MODE};
    int track_left_limit[UVC_HEIGHT] = {};  // 巡线左边线映射到 VS 有效检测区的逐行限制
    int track_right_limit[UVC_HEIGHT] = {}; // 巡线右边线映射到 VS 有效检测区的逐行限制
    bool track_left_valid[UVC_HEIGHT] = {};  // 对应行是否有可用于限幅的左边线
    bool track_right_valid[UVC_HEIGHT] = {}; // 对应行是否有可用于限幅的右边线
    bool warning_armed = true; // 首次有效 ROI 后锁定，仅在成功输出投票结果后重新布防
    std::chrono::steady_clock::time_point last_result_time = std::chrono::steady_clock::time_point::min();

    // ===== 内部方法 =====
    void process_frame(cv::Mat &src);
    void output_final(bool immediate = false);
    void draw_guidelines(cv::Mat &src);
    void bgr_to_rgb565(cv::Mat &src);
    bool build_red_tuning_debug_image();
    void update_track_edge_limits();
    bool cx_is_valid(int cx, int bottom_y, int mode) const;

    static std::string classify_label(const std::string &label);
};

extern VSInference g_vs;
