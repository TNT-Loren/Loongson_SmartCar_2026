#ifndef __IMAGE_TEST_HPP__
#define __IMAGE_TEST_HPP__

#include "zf_common_headfile.hpp"
#include "image_size.hpp"
#include <atomic>
#include <mutex>

// ============================================================
//  图像尺寸宏定义
// ============================================================
#define MT9V03X_H image_height       // 处理图像高度 120
#define MT9V03X_W image_width        // 处理图像宽度 160
#define image_w 150                  // IPM图像宽度
#define image_h 100                  // IPM图像高度

// ============================================================
//  像素值常量
// ============================================================
#define IMG_BLACK 0                  // 黑色像素（边界/黑线）
#define IMG_WHITE 255                // 白色像素（赛道/可通行区域）

// ============================================================
//  边界搜索常量
// ============================================================
#define Border_Max (image_width - 2) // 右边界最大允许值 158
#define Border_Min 1                 // 左边界最小允许值 1
#define USE_num (MT9V03X_H * 3)     // 八邻域搜索最大步数 360

// ============================================================
//  外部设备与互斥锁
// ============================================================
extern zf_device_uvc uvc_dev;              // UVC摄像头设备
extern std::mutex g_image_mutex;           // 图像数据互斥锁（保护图像采集/处理）
extern std::mutex g_vision_result_mutex;   // 视觉结果互斥锁（保护偏差/场景输出）

// ============================================================
//  全局标志位结构体
// ============================================================
typedef struct
{
    uint8 Cross_Fill;          // 十字路口标志
    uint8 Lose_Line_R;         // 右边界丢失标志
    uint8 Lose_Line_L;         // 左边界丢失标志
    bool Left_Circle;          // 左环岛标志
    bool Right_Circle;         // 右环岛标志
    bool Zerba;                // 斑马线标志
    bool Roadblock;            // 路障标志
    uint8 Ramp;                // 坡道标志
    bool Get_Start_Point;      // 是否找到边界搜索起始点
    bool L_Find;               // 是否找到左边界起点
    bool R_Find;               // 是否找到右边界起点
} Flag_Handle;

// ============================================================
//  图像数据数组（全局）
// ============================================================
extern uint8 *rgay_image;                          // 指向摄像头原始灰度图数据的指针
extern uint8 image_copy[image_height][image_width]; // 缩放后的灰度图副本 120×160
extern uint16 debug_image[image_height][image_width]; // RGB565 彩色调试图，供 SCC8660 图传使用
extern bool g_debug_show_binary_image;                // false=灰度底图，true=二值底图
extern bool g_debug_show_ipm_lines;                    // false=普通图传，true=白底IPM边线图
extern uint8 bin_image[MT9V03X_H][MT9V03X_W];      // 二值化图像 120×160（0=黑/边界, 255=白/赛道）
extern uint8 sobel_image[MT9V03X_H][MT9V03X_W];    // Sobel边缘检测图像
extern uint8 bin_image_ipm[image_h][image_w];       // IPM逆透视变换后的二值图 100×150
extern int16 Ipm_Left_Line[MT9V03X_H];              // 原图左边线投影到 160x120 IPM 后的行数组，-1=无效
extern int16 Ipm_Right_Line[MT9V03X_H];             // 原图右边线投影到 160x120 IPM 后的行数组，-1=无效
extern uint16 Ipm_Left_Points[MT9V03X_H][2];        // 补线后左边线 IPM 投影、平滑、等距采样后的有序点集 [x,y]
extern uint16 Ipm_Right_Points[MT9V03X_H][2];       // 补线后右边线 IPM 投影、平滑、等距采样后的有序点集 [x,y]
extern uint16 Ipm_Mid_Points[MT9V03X_H][2];         // 当前可靠边法向偏移生成的 IPM 中线点集 [x,y]
extern uint16 Ipm_Bilateral_Mid_Points[MT9V03X_H][2]; // 丢线兜底：左右 IPM 行数组相加除以2生成的中线点集 [x,y]
extern uint16 Ipm_Left_Point_Count;                 // IPM左边线点数
extern uint16 Ipm_Right_Point_Count;                // IPM右边线点数
extern uint16 Ipm_Mid_Point_Count;                  // IPM中线点数
extern uint16 Ipm_Bilateral_Mid_Point_Count;         // 丢线兜底 IPM 双边平均中线点数

// ============================================================
//  赛道边线数据（从下到上，行为索引）
// ============================================================
extern uint8 Left_Line[MT9V03X_H];       // 左边界每行的X坐标
extern uint8 Right_Line[MT9V03X_H];      // 右边界每行的X坐标
extern uint8 Mid_Line[MT9V03X_H];        // 原始中线每行的X坐标 = (左+右)/2
extern uint8 End_Mid_Line[MT9V03X_H];    // 平滑滤波后的最终中线
extern uint8 Test_Mid_Line[MT9V03X_H];   // 可靠边偏移生成的测试中线，仅用于图传对比
extern uint8 Road_Wide[MT9V03X_H];       // 每行的道路宽度 = 右-左


// ============================================================
//  输出给控制器的数据（像素坐标映射后的结果）
// ============================================================
extern uint8 left_edge_line[image_height];   // 限幅后的左边界X像素坐标
extern uint8 right_edge_line[image_height];  // 限幅后的右边界X像素坐标
extern uint8 mid_line[image_height];         // 限幅后的中线X像素坐标
extern uint8 boundary_y_line[image_height];  // 行号标记

// ============================================================
//  阈值与输出变量
// ============================================================
extern int Threshold;             // OTSU计算的全局二值化阈值
extern int Threshold_IPM;         // IPM图像的二值化阈值
extern float Left_K;              // 左边界斜率
extern float Right_K;             // 右边界斜率

extern Flag_Handle Image_Flag;    // 全局标志位实例

// ============================================================
//  丢线与标志变量
// ============================================================
extern int Right_Lost_Time;               // 右边界丢失行数
extern int Left_Lost_Time;                // 左边界丢失行数
extern int Both_Lost_Time;                // 双边界同时丢失的行数
extern uint8 flag_starting_line;          // 斑马线检测标志
extern uint8 right_cusp_flag;             // 右侧尖点存在标志
extern int16 right_cusp_row;              // 右侧尖点行坐标
extern int16 right_cusp_col;              // 右侧尖点列坐标
extern uint8 left_cusp_flag;              // 左侧尖点存在标志
extern int16 left_cusp_row;               // 左侧尖点行坐标
extern int16 left_cusp_col;               // 左侧尖点列坐标
extern int continuity_change_right_flag;  // 右边界连续性变化位置（行号，0表示无变化）
extern int continuity_change_left_flag;   // 左边界连续性变化位置（行号，0表示无变化）
extern int monotonicity_change_left_flag; // 左边界单调性突变位置（拐点判断）
extern int monotonicity_change_right_flag;// 右边界单调性突变位置（拐点判断）
extern int left_down_guai[2];             // 左下拐点坐标 [0]=行, [1]=列
extern int right_down_guai[2];            // 右下拐点坐标 [0]=行, [1]=列
extern int monotonicity_change_line[2];   // 单调性变化位置的完整坐标 [0]=行, [1]=列

// ============================================================
//  最长白列与边界跟踪数据
// ============================================================
/* 旧版最长白列起点参考变量，当前已停用但保留 cpp 中旧实现。
extern int Longest_White_Column_Left[2];   // 左侧最长白列 [0]=长度, [1]=列号
extern int Longest_White_Column_Right[2];  // 右侧最长白列 [0]=长度, [1]=列号
*/
extern uint16 points_l[(uint16)USE_num][2];// 左边界八邻域跟踪点集 [i][0]=X, [i][1]=Y
extern uint16 points_r[(uint16)USE_num][2];// 右边界八邻域跟踪点集 [i][0]=X, [i][1]=Y
extern uint16 dir_r[(uint16)USE_num];      // 右边界每个跟踪点的出界方向（0~7）
extern uint16 dir_l[(uint16)USE_num];      // 左边界每个跟踪点的出界方向（0~7）
extern uint16 data_stastics_l;             // 左边界实际跟踪到的点数
extern uint16 data_stastics_r;             // 右边界实际跟踪到的点数
extern int point_mode;                     // 拐点检测模式：0=突变找点，1=向量法找点
extern uint8 start_point_l[2];             // 左边界搜索起始点 [0]=X, [1]=Y
extern uint8 start_point_r[2];             // 右边界搜索起始点 [0]=X, [1]=Y
extern uint8 hightest;                     // 左右边线汇合的最高行号
extern uint8_t turn_point_num;             // 检测到的拐点总数（0~4）
extern uint8_t island_state;               // 环岛状态机当前状态（0~6）

enum class TestMidlineMode : uint8_t
{
    Auto = 0,       // 自动按左右边可靠性选择
    ForceLeft = 1,  // 手动强制认为左边可靠
    ForceRight = 2  // 手动强制认为右边可靠
};

enum class ReliableEdgeMode : uint8_t
{
    Auto = 0,       // 自动按可靠边评分选择
    ForceLeft = 1,  // 手动强制左边为可靠边
    ForceRight = 2  // 手动强制右边为可靠边
};

enum class ObstacleAvoidDirection : uint8_t
{
    None = 0,
    Left = 1,  // 强制左边线作为临时中线参考
    Right = 2  // 强制右边线作为临时中线参考
};

enum class IpmMidlineScene : uint8_t
{
    Invalid = 0,     // 点数或可见长度不足，暂不可信
    Straight = 1,    // 长且整体平直
    GentleCurve = 2, // 长且单方向弯曲，包含缓弯
    SCurve = 3,      // 长且存在明显左右换向
    SharpCurve = 4   // 可见长度短，或横向变化相对纵向变化很大
};

struct IpmMidlineSceneResult
{
    IpmMidlineScene scene = IpmMidlineScene::Invalid;
    uint16 count = 0;
    uint8 highest = 0;
    int x_min = 0;
    int x_max = 0;
    int y_min = 0;
    int y_max = 0;
    int x_span = 0;
    int y_span = 0;
    int dx_total = 0;
    int turn_count = 0;
    bool has_reversal = false;
    float x_y_ratio = 0.0f;
    float path_length = 0.0f;
    float chord_length = 0.0f;
    float straightness = 0.0f;      // 首尾直线距离 / 折线长度，越接近1越直
    float mean_abs_curv = 0.0f;     // 平均绝对曲率，单位约为 1/IPM像素
    float max_abs_curv = 0.0f;      // 最大局部曲率，单位约为 1/IPM像素
    float signed_curv_sum = 0.0f;   // 曲率带符号累计，正负表示总体弯向
    float straight_score = 0.0f;
    float gentle_score = 0.0f;
    float s_score = 0.0f;
    float sharp_score = 0.0f;
};

extern TestMidlineMode g_test_midline_mode;
extern ReliableEdgeMode g_ipm_reliable_edge_mode;
extern std::atomic<float> g_ipm_midline_offset_px;

// ============================================================
//  核心视觉处理函数
// ============================================================
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);   // 大津法自动计算二值化阈值
void turn_to_bin(void);                                       // 将灰度图二值化为bin_image
void image_filter(uint8 (*image)[MT9V03X_W]);                 // 3×3形态学滤波（膨胀+腐蚀）
void image_draw_rectan(uint8 (*image)[MT9V03X_W]);            // 在图像左右边和顶部绘制黑框（防止搜索越界）
void update_track_lines(void);                                // 将Left/Right_Line映射到边缘/中线像素坐标数组
void build_debug_image(bool show_binary = false);                // 构建 RGB565 彩色调试图
void Image_Process(void);                                     // 图像处理总入口（二值化→搜索边线→拐点→元素识别→最终中线）
bool image_test(void);                                        // 视觉主循环函数（成功返回true，采集异常返回false）
bool is_lost_line(void);                                      // 基于补线后的当前左右边线判断是否最终丢线
void build_test_midline(TestMidlineMode mode);                 // 生成可靠边偏移测试中线，不参与控制
void transform_lines_to_ipm(const uint8 left_line[MT9V03X_H],
                            const uint8 right_line[MT9V03X_H],
                            int16 ipm_left_line[MT9V03X_H],
                            int16 ipm_right_line[MT9V03X_H]);     // 将原图左右边线投影成 160x120 IPM 行数组
void build_ipm_midline(ReliableEdgeMode mode);                  // 按自动/手动可靠边生成 IPM 中线点集
void cycle_test_midline_mode(void);                            // 键盘调试：切换测试中线可靠边模式
const char *test_midline_mode_name(TestMidlineMode mode);      // 返回测试中线模式名
void cycle_ipm_reliable_edge_mode(void);                       // 键盘调试：切换IPM可靠边选择模式
const char *reliable_edge_mode_name(ReliableEdgeMode mode);    // 返回IPM可靠边模式名
void trigger_obstacle_avoid(ObstacleAvoidDirection direction);  // 触发定时绕行：Left=左边线为临时中线，Right=右边线为临时中线
void obstacle_avoid_timer_task(void);                           // 查询单调事件计时器，到期后恢复正常 IPM 偏移
bool obstacle_avoid_active(void);                               // 当前是否处于绕行状态
ObstacleAvoidDirection current_obstacle_avoid_direction(void);  // 当前绕行方向
const char *obstacle_avoid_direction_name(ObstacleAvoidDirection direction);
void cycle_debug_view_mode(void);                              // 键盘调试：切换普通图传/IPM边线图
const char *debug_view_mode_name(void);                        // 返回当前图传显示模式名
IpmMidlineSceneResult classify_ipm_midline_scene(const uint16 points[MT9V03X_H][2],
                                                 uint16 count,
                                                 uint8 highest); // 只根据IPM中线点集做路径形状粗分类，不参与控制
const char *ipm_midline_scene_name(IpmMidlineScene scene);       // 返回IPM中线路径形状名

#endif
