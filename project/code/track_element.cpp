// #include "track_element.hpp"

// #include "IPM_image.hpp"
// #include "main.hpp"

// #include <cstring>

// // --- 全局实例 -----------------------------------------------------------
// // track_element_feature：
// //   只保存“这一帧”从 IPM 搜线结果中提出来的特征。
// //   用法是：每帧进入 track_element_collect_feature() 后重新覆盖填写。
// //
// // track_element_machine：
// //   保存“跨帧延续”的状态机变量。
// //   用法是：不要每帧清零，只在状态切换或流程推进时修改。
// Track_Element_Feature_TypeDef track_element_feature = {0};
// Track_Element_Machine_TypeDef track_element_machine = {0};

// static void lost_line(void);
// static constexpr uint16 k_lost_line_ratio_num = 1; // 丢线阈值分子，默认 1/3
// static constexpr uint16 k_lost_line_ratio_den = 3; // 丢线阈值分母，默认 1/3

// // ============================================================================
// // 上电初始化：清零所有特征和状态机，默认直线状态、巡左边
// // ============================================================================
// void track_element_init(void)
// {
//     std::memset(&track_element_feature, 0, sizeof(track_element_feature));
//     std::memset(&track_element_machine, 0, sizeof(track_element_machine));

//     track_element_machine.state = TRACK_ELEMENT_STRAIGHT;
//     track_element_machine.last_state = TRACK_ELEMENT_STRAIGHT;
//     track_element_machine.target_boundary = 0; // 默认巡左边
// }

// // ============================================================================
// // 特征采集：从 IPM 搜线结果中提取本帧原始数据，填入 track_element_feature
// //
// // 可用的输入数据（全部来自 IPM_image.cpp，extern 可见）：
// //   g_left_point_count  / g_right_point_count        — 左右搜到点数
// //   points_l[][] / points_r[][]                      — 离散边界点坐标
// //   dir_l[]      / dir_r[]                           — 每个点的八邻域方向
// //   left_edge_line[] / right_edge_line[] / mid_line[] — 每行边界/中线 x 坐标
// //   start_point_l[] / start_point_r[]                 — 起点坐标
// //   valid_l_bound[] / valid_r_bound[]                 — 有效区边界
// //
// // 建议填写顺序：
// //   1. left_point_count / right_point_count — 直接取 g_left_point_count / g_right_point_count
// //   2. left_top_row / right_top_row         — 遍历 points 找最小 y
// //   3. left_dir_change / right_dir_change   — 遍历 dir 统计相邻方向跳变次数
// //   4. line_lost                            — 判断左右边是否丢线，写成 0/1/2/3
// //   5. bottom_width / middle_width / top_width — 在底部/中间/顶部取 right-left
// //   6. near_offset / far_offset             — 取对应行 mid_line 与 image_width/2 的偏差
// //   7. longest_side                         — 比较左右点数
// //
// // 字段对照建议：
// //   track_element_feature.left_point_count  = g_left_point_count;
// //   track_element_feature.right_point_count = g_right_point_count;
// //
// //   track_element_feature.left_top_row：
// //     遍历 points_l[i][1]，取最小 y。
// //     如果左边一个点都没有，建议填 image_height - 1，表示几乎整条左边都没搜到。
// //
// //   track_element_feature.right_top_row：
// //     遍历 points_r[i][1]，取最小 y。
// //
// //   track_element_feature.left_dir_change：
// //     遍历 dir_l，统计相邻两个方向差值较大的次数。
// //     这个量后面适合用来判断：拐点多不多、边线是否突然弯折。
// //
// //   track_element_feature.right_dir_change：
// //     与左边同理，遍历 dir_r。
// //
// //   track_element_feature.line_lost：
// //     建议统一写成离散状态值：
// //       0 -> 两边都不丢线
// //       1 -> 左边丢线
// //       2 -> 右边丢线
// //       3 -> 两边都丢线
// //     这样后面判元素时，直接 switch / if 判断就够了。
// //
// //   track_element_feature.longest_side：
// //     比较左右点数或者比较左右 top_row / row_span。
// //     第一版最简单的写法就是：
// //       左边点数多 -> 0
// //       右边点数多 -> 1
// //
// //   track_element_feature.bottom_width：
// //     取图像下方某一行或几行平均宽度：
// //       right_edge_line[row] - left_edge_line[row]
// //
// //   track_element_feature.middle_width：
// //     取图像中部宽度。
// //
// //   track_element_feature.top_width：
// //     取图像上部宽度。
// //     后面十字判断通常会很依赖“top_width 是否突然变大”。
// //
// //   track_element_feature.near_offset：
// //     取近端某一行中线偏差：
// //       mid_line[row] - image_width / 2
// //
// //   track_element_feature.far_offset：
// //     取远端某一行中线偏差。
// //     后面直线/曲线判断一般会用 near_offset、far_offset 的差值。
// // ============================================================================
// void track_element_collect_feature(void)
// {
//     // TODO: 按上方建议顺序逐项填写
// }

// // ============================================================================
// // 状态判断：读取 track_element_feature，决定本帧 state
// //
// // 判据层级（由简单到复杂，命中即退出）：
// //   1. 直线/曲线 — 根据 far_offset 和赛道宽度变化判定
// //   2. 十字      — 赛道宽度突然放大、左右同时丢线、宽度恢复后的特征
// //   3. 左圆环    — 左侧出现大面积白区、左边界外扩/丢失、右边界保持
// //   4. 右圆环    — 镜像
// //
// // 状态切换规则（写在 judge 里，不要散落到各个 process 中）：
// //   - 从简单状态切到复杂状态：需要连续确认 N 帧（用 state_locking 计数）
// //   - 从复杂状态退回简单状态：特征消失后立即退回（或也加连续确认）
// //   - 切状态时把 last_state 记下来，方便 process 里做沿触发
// //
// // 这一层只做“判状态”，不要在这里直接补线、改 mid_line、改 points_l/r。
// // 这里最适合做的是：
// //   if (...) track_element_machine.state = TRACK_ELEMENT_CROSS;
// //   if (...) track_element_machine.state = TRACK_ELEMENT_LEFT_RING;
// //
// // track_element_machine 常用字段建议：
// //   state：
// //     当前最终状态。process 函数的 switch 就靠它分发。
// //
// //   last_state：
// //     上一帧稳定状态。适合检测“刚进入十字”“刚进入左环”这种沿触发。
// //
// //   target_boundary：
// //     处理函数最终输出给后级巡线逻辑的“优先信哪一边”。
// //     约定：
// //       0 -> 优先左边
// //       1 -> 优先右边
// //
// //   element_processing_flage：
// //     这个标志建议用来表示：当前是否正处于元素处理流程中。
// //     比如：
// //       0 -> 普通巡线
// //       1 -> 十字/圆环处理中
// //
// //   state_locking：
// //     这里可以直接当成“状态锁计数器”用。
// //     常见写法：
// //       进入复杂元素后先置一个计数；
// //       计数没清零前，不允许被新判据抢状态。
// // ============================================================================
// void track_element_judge_state(void)
// {
//     // TODO: 按上方层级逐项判断，更新 track_element_machine.state
// }

// // ============================================================================
// // 直线/缓弯处理：巡两条边线的中线，加权方式偏近端
// // ============================================================================
// void track_straight_process(void)
// {
//     // TODO
// }

// // ============================================================================
// // 急弯处理：巡外侧边线（或切换为单边巡线），近端权重提高
// // ============================================================================
// void track_curve_process(void)
// {
//     // TODO
// }

// // ============================================================================
// // 十字处理：十字路口赛道宽度突变，需要锁定直行方向、短暂补线
// // 典型做法：十字入口锁定中线方向 → 十字内部直线补线 → 十字出口恢复巡线
// // ============================================================================
// void track_cross_process(void)
// {
//     // TODO
// }

// // ============================================================================
// // 左圆环处理：圆环入口在赛道左侧，左边界外扩形成环形
// // 典型做法：入口识别→锁定 target_boundary 为右边界→绕过圆环→出口恢复
// // ============================================================================
// void track_left_ring_process(void)
// {
//     // TODO
// }

// // ============================================================================
// // 右圆环处理：圆环入口在赛道右侧，镜像左圆环逻辑
// // ============================================================================
// void track_right_ring_process(void)
// {
//     // TODO
// }

// // ============================================================================
// // 每帧主入口
// // ============================================================================
// void track_element_update(void)
// {
//     // 用法说明：
//     // 1. 先保存上一帧状态
//     // 2. 再采集这一帧特征
//     // 3. 再根据特征判断当前 state
//     // 4. 最后按 state 分发到对应 process
//     //
//     // 所以你后面使用时，一般只需要在主流程里每帧调用一次：
//     //   track_element_update();
//     //
//     // 调完之后，其他模块就可以直接读取：
//     //   track_element_feature.xxx
//     //   track_element_machine.state
//     //   track_element_machine.target_boundary
//     track_element_machine.last_state = track_element_machine.state;

//     track_element_collect_feature();// 从 IPM 搜线结果提取特征，填入 track_element_feature
//     track_element_judge_state();
//     lost_line();

//     switch (track_element_machine.state)
//     {
//     case TRACK_ELEMENT_STRAIGHT:
//         track_straight_process();
//         break;

//     case TRACK_ELEMENT_CURVE:
//         track_curve_process();
//         break;

//     case TRACK_ELEMENT_CROSS:
//         track_cross_process();
//         break;

//     case TRACK_ELEMENT_LEFT_RING:
//         track_left_ring_process();
//         break;

//     case TRACK_ELEMENT_RIGHT_RING:
//         track_right_ring_process();
//         break;

//     default:
//         track_straight_process();
//         break;
//     }
// }

// // ============================================================================
// // 调试辅助
// // ============================================================================
// const char *track_element_get_state_name(uint8 state)
// {
//     switch (state)
//     {
//     case TRACK_ELEMENT_STRAIGHT:  return "straight";
//     case TRACK_ELEMENT_CURVE:     return "curve";
//     case TRACK_ELEMENT_CROSS:     return "cross";
//     case TRACK_ELEMENT_LEFT_RING: return "left_ring";
//     case TRACK_ELEMENT_RIGHT_RING:return "right_ring";
//     default:                      return "unknown";
//     }
// }

// //=================================================
// // 下面是一些 IPM 搜线结果处理的辅助函数，track_element_collect_feature() 里可能会用到
// //=================================================

// //0两边不丢线，1左边丢线，2右边丢线，3两边都丢线
// static void lost_line(void)
// {
//     uint16 left_overlap_count = 0;// 左边丢线点数
//     uint16 right_overlap_count = 0;
//     float left_overlap_ratio = 0.0f;//
//     float right_overlap_ratio = 0.0f;

//     uint8 left_lost = 0;
//     uint8 right_lost = 0;

//     track_element_feature.line_lost = 0;

//     if (g_left_point_count == 0)
//     {
//         left_lost = 1;
//     }
//     else
//     {
//         for (uint16 i = 0; i < g_left_point_count && i < k_max_search_points; ++i)
//         {
//             const int x = points_l[i][0];
//             const int y = points_l[i][1];

//             if (y < 0 || y >= image_height)
//             {
//                 continue;
//             }

//             if (valid_l_bound[y] > valid_r_bound[y])
//             {
//                 continue;
//             }

//             if (x == valid_l_bound[y])
//             {
//                 ++left_overlap_count;
//             }
//         }

//         left_overlap_ratio = static_cast<float>(left_overlap_count) /
//                              static_cast<float>(g_left_point_count);

//         if (left_overlap_count * k_lost_line_ratio_den >
//             g_left_point_count * k_lost_line_ratio_num)
//         {
//             left_lost = 1;
//         }
//     }

//     if (g_right_point_count == 0)
//     {
//         right_lost = 1;
//     }
//     else
//     {
//         for (uint16 i = 0; i < g_right_point_count && i < k_max_search_points; ++i)
//         {
//             const int x = points_r[i][0];
//             const int y = points_r[i][1];

//             if (y < 0 || y >= image_height)
//             {
//                 continue;
//             }

//             if (valid_l_bound[y] > valid_r_bound[y])
//             {
//                 continue;
//             }

//             if (x == valid_r_bound[y])
//             {
//                 ++right_overlap_count;
//             }
//         }

//         right_overlap_ratio = static_cast<float>(right_overlap_count) /
//                               static_cast<float>(g_right_point_count);

//         if (right_overlap_count * k_lost_line_ratio_den >
//             g_right_point_count * k_lost_line_ratio_num)
//         {
//             right_lost = 1;
//         }
//     }

//     test3 = left_overlap_ratio;
//     test4 = right_overlap_ratio;

//     if (left_lost && right_lost)
//     {
//         track_element_feature.line_lost = 3;
//     }
//     else if (left_lost)
//     {
//         track_element_feature.line_lost = 1;
//     }
//     else if (right_lost)
//     {
//         track_element_feature.line_lost = 2;
//     }
// }
