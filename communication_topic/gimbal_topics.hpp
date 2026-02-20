#pragma once
#include "topic.hpp"

namespace orb {

struct GimbalInfo {
    float yaw_angle;
    float yaw_omega;
    float pitch_angle;
    float pitch_omega;
};

inline Topic<GimbalInfo> gimbal_info;

} // namespace orb
