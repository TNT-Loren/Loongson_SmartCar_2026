// #include "image_ring.hpp"

// #include "IPM_image.hpp"
// #include "image.hpp"
// #include "imu.hpp"

// #include <algorithm>
// #include <cmath>

// namespace
// {
// static constexpr uint8 k_front_ring_phase_threshold = 3;
// static constexpr uint8 k_front_ring_blank_reset_threshold = 6;
// static constexpr uint16 k_front_ring_min_points = 20;
// static constexpr uint16 k_front_ring_half_step = 20;
// static constexpr float k_front_ring_state4_min_yaw = 45.0f;
// static constexpr float k_front_ring_out_min_yaw = 200.0f;
// static constexpr float k_front_ring_done_min_yaw = 300.0f;

// static float front_ring_yaw_progress()
// {
//     if (g_front_ring_ctrl.kind == FrontRingNone)
//     {
//         return 0.0f;
//     }

//     const float delta = yaw_tracker.get_unbounded_yaw() - g_front_ring_ctrl.enter_yaw;
//     return std::fabs(delta);
// }

// static void sync_public_ring_state()
// {
//     g_front_ring_kind = g_front_ring_ctrl.kind;
//     g_front_ring_state = (g_front_ring_ctrl.kind == FrontRingNone) ? 0 : g_front_ring_ctrl.state;
// }

// static void front_ring_init(uint8 ring_kind)
// {
//     g_front_ring_ctrl = {};
//     g_front_ring_ctrl.kind = ring_kind;
//     g_front_ring_ctrl.state = FrontRingState1;
//     g_front_ring_ctrl.enter_yaw = yaw_tracker.get_unbounded_yaw();
//     mode_element = 2;
//     sync_public_ring_state();
// }

// static void front_ring_switch_next()
// {
//     if (g_front_ring_ctrl.state < FrontRingState9)
//     {
//         ++g_front_ring_ctrl.state;
//     }
//     g_front_ring_ctrl.phase_counter = 0;
//     sync_public_ring_state();
// }

// static void ring_count_up_or_down(bool condition)
// {
//     if (condition)
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     else if (g_front_ring_ctrl.phase_counter > 0)
//     {
//         --g_front_ring_ctrl.phase_counter;
//     }
// }

// static void ring_switch_if_ready(bool use_strict_greater = false)
// {
//     const bool ready = use_strict_greater
//                            ? (g_front_ring_ctrl.phase_counter > k_front_ring_phase_threshold)
//                            : (g_front_ring_ctrl.phase_counter >= k_front_ring_phase_threshold);
//     if (ready)
//     {
//         front_ring_switch_next();
//     }
// }

// static void front_ring_update_state1()
// {
//     ring_count_up_or_down(front_ring_is_current_lost(g_front_ring_ctrl.kind));
//     ring_switch_if_ready(true);
// }

// static void front_ring_update_state2()
// {
//     if (front_ring_is_current_good(g_front_ring_ctrl.kind))
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     ring_switch_if_ready(false);
// }

// static void front_ring_update_state3()
// {
//     const bool opposite_usable = front_ring_opposite_point_count(g_front_ring_ctrl.kind) >= k_front_ring_min_points;
//     const bool current_short = front_ring_current_point_count(g_front_ring_ctrl.kind) < k_front_ring_half_step;
//     if (opposite_usable || current_short)
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     else if (g_front_ring_ctrl.phase_counter > 0)
//     {
//         --g_front_ring_ctrl.phase_counter;
//     }
//     ring_switch_if_ready(true);
// }

// static void front_ring_update_state4()
// {
//     const bool opposite_long = front_ring_opposite_point_count(g_front_ring_ctrl.kind) > k_front_ring_half_step;
//     if (opposite_long && g_front_ring_ctrl.sum_yaw > k_front_ring_state4_min_yaw)
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     else if (g_front_ring_ctrl.phase_counter > 0)
//     {
//         --g_front_ring_ctrl.phase_counter;
//     }
//     ring_switch_if_ready(true);
// }

// static void front_ring_update_state5()
// {
//     const bool yaw_ready = g_front_ring_ctrl.sum_yaw > k_front_ring_out_min_yaw;
//     const bool opposite_ready = front_ring_is_opposite_good(g_front_ring_ctrl.kind);
//     const bool current_lost = front_ring_is_current_lost(g_front_ring_ctrl.kind);
//     if (yaw_ready && (opposite_ready || current_lost))
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     else if (g_front_ring_ctrl.phase_counter > 0)
//     {
//         --g_front_ring_ctrl.phase_counter;
//     }
//     ring_switch_if_ready(true);
// }

// static void front_ring_update_state6()
// {
//     if (front_ring_is_opposite_good(g_front_ring_ctrl.kind) ||
//         front_ring_opposite_point_count(g_front_ring_ctrl.kind) < k_front_ring_min_points)
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     ring_switch_if_ready(false);
// }

// static void front_ring_update_state7()
// {
//     if (front_ring_is_opposite_good(g_front_ring_ctrl.kind))
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     else if (g_front_ring_ctrl.phase_counter > 0)
//     {
//         --g_front_ring_ctrl.phase_counter;
//     }
//     ring_switch_if_ready(false);
// }

// static void front_ring_update_state8()
// {
//     const bool current_lost = front_ring_is_current_lost(g_front_ring_ctrl.kind);
//     const bool yaw_ready_with_opposite =
//         g_front_ring_ctrl.sum_yaw > k_front_ring_done_min_yaw &&
//         front_ring_is_opposite_good(g_front_ring_ctrl.kind);
//     ring_count_up_or_down(current_lost || yaw_ready_with_opposite);
//     ring_switch_if_ready(true);
// }

// static void front_ring_update_state9()
// {
//     const bool exit_ready = g_front_ring_ctrl.sum_yaw > k_front_ring_done_min_yaw &&
//                             front_ring_are_boundaries_good() &&
//                             front_ring_is_current_good(g_front_ring_ctrl.kind) &&
//                             !front_ring_has_any_entry_corner();
//     if (exit_ready)
//     {
//         ++g_front_ring_ctrl.phase_counter;
//     }
//     else if (g_front_ring_ctrl.phase_counter > 0)
//     {
//         --g_front_ring_ctrl.phase_counter;
//     }

//     if (g_front_ring_ctrl.phase_counter > k_front_ring_phase_threshold)
//     {
//         front_ring_reset();
//     }
// }
// } // namespace

// FrontRingControl g_front_ring_ctrl = {};
// uint8 g_front_ring_kind = FrontRingNone;
// uint8 g_front_ring_state = 0;

// void front_ring_reset(void)
// {
//     g_front_ring_ctrl = {};
//     g_front_ring_ctrl.kind = FrontRingNone;
//     g_front_ring_ctrl.state = FrontRingState1;
//     g_front_ring_kind = FrontRingNone;
//     g_front_ring_state = 0;
//     front_ring_release_boundary_takeover();
//     if (g_front_cross_kind == FrontCrossNone)
//     {
//         mode_element = 0;
//     }
// }

// void front_ring_process(void)
// {
//     if (g_front_ring_ctrl.kind == FrontRingNone)
//     {
//         if (front_ring_left_entry_condition())
//         {
//             front_ring_init(FrontRingLeft);
//         }
//         else if (front_ring_right_entry_condition())
//         {
//             front_ring_init(FrontRingRight);
//         }
//         else
//         {
//             g_front_ring_ctrl.flag = 0;
//             sync_public_ring_state();
//             return;
//         }
//     }
//     else
//     {
//         g_front_ring_ctrl.sum_yaw = front_ring_yaw_progress();
//         mode_element = 2;

//         static uint8 blank_counter = 0;
//         if (g_front_line_lost == 3)
//         {
//             ++blank_counter;
//         }
//         else
//         {
//             blank_counter = 0;
//         }
//         if (blank_counter >= k_front_ring_blank_reset_threshold)
//         {
//             blank_counter = 0;
//             front_ring_reset();
//             return;
//         }

//         switch (g_front_ring_ctrl.state)
//         {
//             case FrontRingState1:
//                 front_ring_update_state1();
//                 break;
//             case FrontRingState2:
//                 front_ring_update_state2();
//                 break;
//             case FrontRingState3:
//                 front_ring_update_state3();
//                 break;
//             case FrontRingState4:
//                 front_ring_update_state4();
//                 break;
//             case FrontRingState5:
//                 front_ring_update_state5();
//                 break;
//             case FrontRingState6:
//                 front_ring_update_state6();
//                 break;
//             case FrontRingState7:
//                 front_ring_update_state7();
//                 break;
//             case FrontRingState8:
//                 front_ring_update_state8();
//                 break;
//             case FrontRingState9:
//                 front_ring_update_state9();
//                 break;
//             default:
//                 front_ring_reset();
//                 return;
//         }
//     }

//     if (g_front_ring_ctrl.kind != FrontRingNone)
//     {
//         g_front_cross_kind = FrontCrossNone;
//         g_front_cross_state = FrontCrossState1;
//         front_ring_apply_boundary_takeover(g_front_ring_ctrl.kind, g_front_ring_ctrl.state);
//     }
//     sync_public_ring_state();
// }

// const char *front_ring_kind_name(void)
// {
//     switch (g_front_ring_ctrl.kind)
//     {
//         case FrontRingLeft:
//             return "L";
//         case FrontRingRight:
//             return "R";
//         default:
//             return "N";
//     }
// }

// const char *front_ring_state_name(void)
// {
//     if (g_front_ring_ctrl.kind == FrontRingNone)
//     {
//         return "N";
//     }

//     switch (g_front_ring_ctrl.state)
//     {
//         case FrontRingState1:
//             return "S1";
//         case FrontRingState2:
//             return "S2";
//         case FrontRingState3:
//             return "S3";
//         case FrontRingState4:
//             return "S4";
//         case FrontRingState5:
//             return "S5";
//         case FrontRingState6:
//             return "S6";
//         case FrontRingState7:
//             return "S7";
//         case FrontRingState8:
//             return "S8";
//         case FrontRingState9:
//             return "S9";
//         default:
//             return "?";
//     }
// }
