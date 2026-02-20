#pragma once
#include <cstdint>

#include "topic.hpp"

namespace orb {

enum class ChassisSpin : uint8_t {
    ChassisSpinNone = 0,
    ChassisSpinCW = 1,
    ChassisSpinCCW = 2,
};

enum class SuperCap : uint8_t {
    SuperCapCharge = 0,
    SuperCapDischarge = 1,
};

enum AutoAim : uint8_t {
    AutoAimOff = 0,
    AutoAimOn = 1,
};

enum class GimbalSetZero : uint8_t {
    GimbalSetZeroOff = 0,
    GimbalSetZeroOn = 1,
};

enum class FrictionWheel : uint8_t {
    FrictionWheelOff = 0,
    FrictionWheelOn = 1,
};

enum class Shoot : uint8_t {
    ShootOff = 0,
    ShootOn = 1,
};

enum class Eject : uint8_t {
    EjectOff = 0,
    EjectOn = 1,
};

struct RcControl {
    uint8_t yaw_angle;                // 偏移角度
    uint8_t pitch_angle;              // 俯仰角度
    uint8_t chassis_speed_x;          // 平移方向：前、后、左、右
    uint8_t chassis_speed_y;          // 底盘移动总速度
    uint8_t chassis_rotation;         // 自转：不转、顺时针转、逆时针转
    ChassisSpin chassis_spin;       // 小陀螺：不转、顺时针转、逆时针转
    SuperCap super_cap;             // 超级电容：充电、放电
    AutoAim auto_aim;               // 自瞄开关
    GimbalSetZero gimbal_set_zero;  // 云台归零
    FrictionWheel friction_wheel;   // 摩擦轮控制
    Shoot shoot;                    // 发射控制
    Eject eject;                    // 退弹控制
};


inline Topic<RcControl> rc_control;
}// namespace orb