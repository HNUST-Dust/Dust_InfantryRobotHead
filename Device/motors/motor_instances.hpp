#pragma once
#include "dji_c6xx.hpp"
#include "dm_mit.hpp"
#include "cubemars_mit.hpp"

namespace actuator::instances {
    inline actuator::drivers::DjiC6xxMin g_left_wheel;
    inline actuator::drivers::DjiC6xxMin g_right_wheel;
    inline actuator::drivers::DjiC6xxMin g_poke;
}
