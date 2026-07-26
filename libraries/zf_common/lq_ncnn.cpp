/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-03
@修改内容：整理代码风格，与其他驱动库保持一致
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "lq_ncnn.hpp"

#include <limits>
#include <stdexcept>
#include <cmath>

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：LQ_NCNN::LQ_NCNN()
 * @功能说明：NCNN类的构造函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：LQ_NCNN ncnn;
 * @备注说明：初始化默认参数
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
LQ_NCNN::LQ_NCNN()
    : m_initialized(false)
    , m_input_width(96)
    , m_input_height(96)
    , m_input_name("in0")
    , m_output_name("out0")
    , m_last_confidence(0.0f)
{
    // 默认使用ImageNet标准归一化
    m_mean_vals[0] = 123.675f;
    m_mean_vals[1] = 116.28f;
    m_mean_vals[2] = 103.53f;
    m_norm_vals[0] = 0.01712475f;
    m_norm_vals[1] = 0.017507f;
    m_norm_vals[2] = 0.01742919f;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：bool LQ_NCNN::Init()
 * @功能说明：初始化NCNN模型
 * @参数说明：无
 * @函数返回：true:成功 false:失败
 * @调用方法：ncnn.Init();
 * @备注说明：加载模型文件和参数
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
bool LQ_NCNN::Init()
{
    // 设置NCNN选项
    m_net.opt.use_vulkan_compute = false;
    m_net.opt.num_threads = 1;

    // 加载模型参数
    if (m_param_path.empty() || m_net.load_param(m_param_path.c_str()) != 0) {
        printf("NCNN: 加载参数文件失败: %s\n", m_param_path.c_str());
        return false;
    }

    // 加载模型权重
    if (m_bin_path.empty() || m_net.load_model(m_bin_path.c_str()) != 0) {
        printf("NCNN: 加载模型文件失败: %s\n", m_bin_path.c_str());
        return false;
    }

    m_initialized = true;
    printf("NCNN: 模型加载成功 (%s)\n", m_param_path.c_str());

    return true;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：std::string LQ_NCNN::Infer(const cv::Mat& bgr_image)
 * @功能说明：推理函数
 * @参数说明：bgr_image : 输入BGR图像
 * @函数返回：推理结果（类别名称或编号）
 * @调用方法：std::string result = ncnn.Infer(image);
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
std::string LQ_NCNN::Infer(const cv::Mat& bgr_image)
{
    if (!m_initialized) {
        throw std::runtime_error("NCNN not initialized. Call Init() first.");
    }
    if (bgr_image.empty()) {
        throw std::invalid_argument("Input image is empty.");
    }
    if (bgr_image.type() != CV_8UC3) {
        throw std::invalid_argument("Input image must be CV_8UC3 BGR.");
    }

    // NCNN 直接读取 BGR 并输出 RGB 张量，避免 OpenCV resize/cvtColor
    // 产生两张临时图。VS 输入已是 64x64，正常路径不再执行缩放。
    ncnn::Mat input;
    const int stride = static_cast<int>(bgr_image.step);
    if (bgr_image.cols == m_input_width && bgr_image.rows == m_input_height) {
        input = ncnn::Mat::from_pixels(
            bgr_image.data, ncnn::Mat::PIXEL_BGR2RGB,
            bgr_image.cols, bgr_image.rows, stride);
    } else {
        input = ncnn::Mat::from_pixels_resize(
            bgr_image.data, ncnn::Mat::PIXEL_BGR2RGB,
            bgr_image.cols, bgr_image.rows, stride,
            m_input_width, m_input_height);
    }
    input.substract_mean_normalize(m_mean_vals, m_norm_vals);

    // 创建提取器并推理
    ncnn::Extractor ex = m_net.create_extractor();

    // 检查输入blob名称是否有效（调试用：首次打印blob名称）
    int ret_input = ex.input(m_input_name.c_str(), input);
    if (ret_input != 0)
    {
        printf("[NCNN WARN] input blob '%s' 设置失败(ret=%d)，模型可能使用不同的输入名称\n",
               m_input_name.c_str(), ret_input);
    }

    ncnn::Mat logits;
    int ret_extract = ex.extract(m_output_name.c_str(), logits);
    if (ret_extract != 0)
    {
        printf("[NCNN WARN] extract blob '%s' 失败(ret=%d)，模型可能使用不同的输出名称\n",
               m_output_name.c_str(), ret_extract);
    }

    // 缓存最近一次推理的原始logits分值（用于外部调试）
    m_last_logits.clear();
    m_last_logits.reserve(logits.w);
    for (int i = 0; i < logits.w; ++i)
    {
        m_last_logits.push_back(logits[i]);
    }

    // 获取最大概率类别
    int class_id = Argmax(logits);
    if (class_id < 0) {
        throw std::runtime_error("Failed to get class id from output logits.");
    }

    // 有些模型最后一层已输出概率，有些模型输出 logits；两种情况统一得到概率置信度。
    bool is_probability = true;
    double probability_sum = 0.0;
    for (int i = 0; i < logits.w; ++i) {
        const float value = logits[i];
        if (!std::isfinite(value) || value < 0.0f || value > 1.0f) {
            is_probability = false;
            break;
        }
        probability_sum += value;
    }

    if (is_probability && std::fabs(probability_sum - 1.0) <= 0.01) {
        m_last_confidence = logits[class_id];
    } else {
        const float max_logit = logits[class_id];
        double exp_sum = 0.0;
        for (int i = 0; i < logits.w; ++i) {
            exp_sum += std::exp(static_cast<double>(logits[i] - max_logit));
        }
        m_last_confidence = exp_sum > 0.0
                                ? static_cast<float>(1.0 / exp_sum)
                                : 0.0f;
    }

    // 返回类别名称或编号
    if (class_id >= 0 && class_id < static_cast<int>(m_labels.size())) {
        return m_labels[class_id];
    }
    return std::to_string(class_id);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetModelPath(const std::string& param_path, const std::string& bin_path)
 * @功能说明：设置模型路径
 * @参数说明：param_path : 模型参数文件路径
 * @参数说明：bin_path   : 模型权重文件路径
 * @函数返回：无
 * @调用方法：ncnn.SetModelPath("model.ncnn.param", "model.ncnn.bin");
 * @备注说明：应在Init()之前调用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetModelPath(const std::string& param_path, const std::string& bin_path)
{
    m_param_path = param_path;
    m_bin_path = bin_path;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetInputSize(int width, int height)
 * @功能说明：设置输入尺寸
 * @参数说明：width  : 输入宽度
 * @参数说明：height : 输入高度
 * @函数返回：无
 * @调用方法：ncnn.SetInputSize(224, 224);
 * @备注说明：应在Init()之前调用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetInputSize(int width, int height)
{
    m_input_width = width;
    m_input_height = height;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetLabels(const std::vector<std::string>& labels)
 * @功能说明：设置类别标签
 * @参数说明：labels : 类别名称列表
 * @函数返回：无
 * @调用方法：ncnn.SetLabels({"cat", "dog", "bird"});
 * @备注说明：应在Init()之前调用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetLabels(const std::vector<std::string>& labels)
{
    m_labels = labels;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3])
 * @功能说明：设置归一化参数
 * @参数说明：mean_vals  : 均值
 * @参数说明：norm_vals : 标准差
 * @函数返回：无
 * @调用方法：ncnn.SetNormalize(mean, norm);
 * @备注说明：应在Init()之前调用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3])
{
    m_mean_vals[0] = mean_vals[0];
    m_mean_vals[1] = mean_vals[1];
    m_mean_vals[2] = mean_vals[2];
    m_norm_vals[0] = norm_vals[0];
    m_norm_vals[1] = norm_vals[1];
    m_norm_vals[2] = norm_vals[2];
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：std::vector<float> LQ_NCNN::GetLastScores()
 * @功能说明：获取最近一次推理的原始输出分值(logits)
 * @参数说明：无
 * @函数返回：各类别的原始logit值向量
 * @调用方法：std::vector<float> scores = ncnn.GetLastScores();
 * @备注说明：用于调试，查看模型对各分类的原始置信度
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
std::vector<float> LQ_NCNN::GetLastScores() const
{
    return m_last_logits;
}

float LQ_NCNN::GetLastConfidence() const
{
    return m_last_confidence;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：int LQ_NCNN::Argmax(const ncnn::Mat& logits)
 * @功能说明：获取最大概率类别索引
 * @参数说明：logits : 模型输出
 * @函数返回：类别索引
 * @调用方法：内部调用
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LQ_NCNN::Argmax(const ncnn::Mat& logits)
{
    if (logits.w <= 0) {
        return -1;
    }

    int best_index = 0;
    float best_value = -std::numeric_limits<float>::infinity();

    for (int i = 0; i < logits.w; ++i) {
        const float value = logits[i];
        if (value > best_value) {
            best_value = value;
            best_index = i;
        }
    }
    return best_index;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：LQ_NCNN::~LQ_NCNN()
 * @功能说明：NCNN类的析构函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：创建的对象生命周期结束后系统自动调用
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
LQ_NCNN::~LQ_NCNN()
{
    // NCNN会自动释放资源
}
