#include "app_booster.h"
#include "../communication_topic/rc_control_topics.hpp"
#include "../communication_topic/pc_control_topics.hpp"
#include "../communication_topic/actuator_cmd_topics.hpp"
#include "../Device/motors/motor_instances.hpp"

#include "cmsis_os2.h"
#include "arm_math.h"
extern "C" {
#include "FreeRTOS.h"
#include "task.h" 
}

namespace {
using actuator::instances::g_left_wheel;
using actuator::instances::g_right_wheel;
using actuator::instances::g_poke;
}
void Booster::Init()
{
    if (started_) {
        configASSERT(false);
        return;
    }

    // 左摩擦轮速度环 PID
    left_wheel_pid_.configure({
        .kp           = 1.0f,
        .ki           = 0.0f,
        .kd           = 0.0f,
        .i_out_max    = 5000.0f,
        .out_max      = 16384.0f,
        .dt           = 0.001f,
    });

    // 右摩擦轮速度环 PID
    right_wheel_pid_.configure({
        .kp           = 1.0f,
        .ki           = 0.0f,
        .kd           = 0.0f,
        .i_out_max    = 5000.0f,
        .out_max      = 16384.0f,
        .dt           = 0.001f,
    });

    // 拨弹轮速度环 PID
    poke_pid_.configure({
        .kp           = 10.0f,
        .ki           = 2.0f,
        .kd           = 0.0f,
        .i_out_max    = 3000.0f,
        .out_max      = 16384.0f,
        .dt           = 0.001f,
    });

    started_ = true;

    static const osThreadAttr_t kBoosterTaskAttr = {
        .name = "booster_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(Booster::TaskEntry, this, &kBoosterTaskAttr);
    if (!thread_) {
        configASSERT(false);
    }
}

void Booster::Exit()
{

}

void Booster::TaskEntry(void *param)
{
    Booster *self = static_cast<Booster *>(param);
    self->Task();
}

void Booster::Task()
{
    Subscription<orb::RcControl> rc_control_sub(orb::rc_control);
    Subscription<orb::PcControl> pc_control_sub(orb::pc_control);
    orb::RcControl rc_control{};
    orb::PcControl pc_control{};

    for (;;) 
    {
        (void)rc_control_sub.copy(rc_control);
        (void)pc_control_sub.copy(pc_control);

        if (rc_control.friction_wheel == orb::FrictionWheel::FrictionWheelOn){
            SetLeftWheelOmega(45); //40
            SetRightWheelOmega(-45);
        }else if(rc_control.friction_wheel == orb::FrictionWheel::FrictionWheelOff){
            SetLeftWheelOmega(0);
            SetRightWheelOmega(0);
        }

        // 没穷举出所有情况

        // 开火逻辑
        if ((rc_control.shoot == orb::Shoot::ShootOn) 
        && (rc_control.eject == orb::Eject::EjectOff)
        ) {
            if (((rc_control.auto_aim == orb::AutoAim::AutoAimOn)
            && (pc_control.shoot_switch == 1))
            || (rc_control.auto_aim == orb::AutoAim::AutoAimOff)
            ) {
                SetPokeOmega(8.0);//20
            } 
        }
        // 停止逻辑
        if ((rc_control.shoot == orb::Shoot::ShootOff) 
            && (rc_control.eject == orb::Eject::EjectOff)){
            SetPokeOmega(0);
        }
        // 退弹逻辑
        if ((rc_control.shoot == orb::Shoot::ShootOff) 
            && (rc_control.eject == orb::Eject::EjectOn)){
            SetPokeOmega(-5.0);
        }

        // 速度环 PID 计算 & 电流输出
        g_left_wheel.SetTargetCurrent(
            left_wheel_pid_.update(left_wheel_omega_, g_left_wheel.now_omega_rad_s()));
        g_right_wheel.SetTargetCurrent(
            right_wheel_pid_.update(right_wheel_omega_, g_right_wheel.now_omega_rad_s()));
        g_poke.SetTargetCurrent(
            poke_pid_.update(poke_omega_, g_poke.now_omega_rad_s()));

        // 打包发送一帧（left/right/poke 同组 0x200）
        actuator::drivers::DjiC6xxMin::SendGroup(&g_left_wheel, &g_right_wheel, &g_poke);

        osDelay(pdMS_TO_TICKS(10));
    }
}