#pragma once

#include "../Device/debug_tools.h"
#include "../Algorithm/control/alg_pid.h"
#include "cmsis_os2.h"

class Booster 
{
public:
    static Booster& Instance() {
        static Booster instance;
        return instance;
    }

    void Init();
    void Task();
    void Exit();

private:
    DebugTools  debug_tools_;
    float left_wheel_omega_ = 0.0f;
    float right_wheel_omega_ = 0.0f;
    float poke_omega_ = 0.0f;

    // 速度环 PID
    alg::Pid left_wheel_pid_;
    alg::Pid right_wheel_pid_;
    alg::Pid poke_pid_;

    bool started_ = false;
    osThreadId_t thread_ = nullptr;

    inline void SetLeftWheelOmega(float omega){
        left_wheel_omega_ = omega;
    };
    inline void SetRightWheelOmega(float omega){
        right_wheel_omega_ = omega;
    };
    inline void SetPokeOmega(float omega){
        poke_omega_ = omega;
    };

    static void TaskEntry(void *param);
};