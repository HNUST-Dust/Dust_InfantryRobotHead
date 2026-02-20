#pragma once

#include <cstdint>

namespace motor_ids {

// DJI wheel motors on CAN1 (0x201~0x203)
static constexpr uint32_t kLeftWheel    = 0x201;
static constexpr uint32_t kRightWheel   = 0x202;
static constexpr uint32_t kPoke         = 0x203;

} // namespace motor_ids
