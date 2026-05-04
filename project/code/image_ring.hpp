// #ifndef __IMAGE_RING_HPP__
// #define __IMAGE_RING_HPP__

// #include "zf_common_headfile.hpp"

// enum FrontRingKind : uint8
// {
//     FrontRingNone = 0,
//     FrontRingLeft = 1,
//     FrontRingRight = 2,
// };

// enum FrontRingState : uint8
// {
//     FrontRingState1 = 1,
//     FrontRingState2 = 2,
//     FrontRingState3 = 3,
//     FrontRingState4 = 4,
//     FrontRingState5 = 5,
//     FrontRingState6 = 6,
//     FrontRingState7 = 7,
//     FrontRingState8 = 8,
//     FrontRingState9 = 9,
// };

// struct FrontRingControl
// {
//     uint8 state = FrontRingState1;
//     uint8 kind = FrontRingNone;
//     uint8 phase_counter = 0;
//     float sum_yaw = 0.0f;
//     float enter_yaw = 0.0f;
//     uint8 last_high = 0;
//     uint8 flag = 0;
// };

// extern FrontRingControl g_front_ring_ctrl;
// extern uint8 g_front_ring_kind;
// extern uint8 g_front_ring_state;

// void front_ring_process(void);
// void front_ring_reset(void);
// const char *front_ring_kind_name(void);
// const char *front_ring_state_name(void);

// // image.cpp provides the frame features and boundary edit primitives used by the ring state machine.
// bool front_ring_left_entry_condition(void);
// bool front_ring_right_entry_condition(void);
// bool front_ring_is_current_lost(uint8 ring_kind);
// bool front_ring_is_current_good(uint8 ring_kind);
// bool front_ring_is_opposite_good(uint8 ring_kind);
// bool front_ring_are_boundaries_good(void);
// bool front_ring_has_current_corner(uint8 ring_kind);
// bool front_ring_has_any_entry_corner(void);
// uint16 front_ring_current_point_count(uint8 ring_kind);
// uint16 front_ring_opposite_point_count(uint8 ring_kind);
// void front_ring_apply_boundary_takeover(uint8 ring_kind, uint8 ring_state);
// void front_ring_release_boundary_takeover(void);

// #endif
