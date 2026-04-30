// #ifndef __TRACK_ELEMENT_HPP__
// #define __TRACK_ELEMENT_HPP__

// #include "image_size.hpp"
// #include "zf_common_typedef.hpp"

// // 赛道元素状态枚举，按简单→复杂排列，后期状态机只会从简单往复杂跳
// typedef enum
// {
//     TRACK_ELEMENT_STRAIGHT = 0, // 直线 / 缓弯
//     TRACK_ELEMENT_CURVE,        // 急弯
//     TRACK_ELEMENT_CROSS,        // 十字路口
//     TRACK_ELEMENT_LEFT_RING,    // 左圆环（入口在左）
//     TRACK_ELEMENT_RIGHT_RING    // 右圆环（入口在右）
// } Track_Element_State_Enum;

// // 每帧从搜线结果提取的原始特征，纯数据，不做判断
// typedef struct
// {
//     uint16 left_point_count;   // 左边界搜到点数
//     uint16 right_point_count;  // 右边界搜到点数
//     uint16 left_top_row;       // 左边界最高搜到行
//     uint16 right_top_row;      // 右边界最高搜到行
//     uint16 left_dir_change;    // 左边界方向跳变次数（八邻域 dir 翻转计数）
//     uint16 right_dir_change;   // 右边界方向跳变次数
//     uint8  line_lost;          // 丢线状态：0=不丢线, 1=左丢线, 2=右丢线, 3=双丢线
//     uint8  longest_side;       // 0=左边线更长, 1=右边线更长
//     int16  bottom_width;       // 近端赛道宽度（底部若干行均值）
//     int16  middle_width;       // 中间赛道宽度
//     int16  top_width;          // 远端赛道宽度（最高搜到行附近）
//     int16  near_offset;        // 近端中线偏离图像中心的偏移量
//     int16  far_offset;         // 远端中线偏离图像中心的偏移量
// } Track_Element_Feature_TypeDef;

// // 状态机本体，跨帧保持
// typedef struct
// {
//     uint8 state;                    // 当前元素状态
//     uint8 last_state;               // 上一帧状态，用于检测状态切换沿
//     uint8 target_boundary;          // 当前巡线依赖边：0=左边, 1=右边
//     uint8 element_processing_flage; // 元素处理进度标志，各处理函数自行读写
//     uint8 state_locking;            // 状态锁计数器，>0 时禁止状态切换
// } Track_Element_Machine_TypeDef;

// extern Track_Element_Feature_TypeDef track_element_feature;// 全局实例，保存当前帧特征
// extern Track_Element_Machine_TypeDef track_element_machine;// 全局实例，保存状态机变量

// // 上电时调用一次，清零特征和状态机，初始化为直线状态
// void track_element_init(void);

// // 每帧主入口：采集特征 → 判断状态 → dispatch 对应处理函数
// void track_element_update(void);

// // 从 IPM 搜线结果（points_l/r、dir_l/r、left/right_edge_line、mid_line）中提取当前帧特征
// void track_element_collect_feature(void);

// // 根据特征判断当前属于哪种赛道元素，更新状态机 state 字段
// void track_element_judge_state(void);

// // 各元素的处理函数，内部负责切换 target_boundary、更新 element_processing_flage 等
// void track_straight_process(void);
// void track_curve_process(void);
// void track_cross_process(void);
// void track_left_ring_process(void);
// void track_right_ring_process(void);

// // 调试辅助：把状态枚举值转成可读字符串
// const char *track_element_get_state_name(uint8 state);

// #endif
