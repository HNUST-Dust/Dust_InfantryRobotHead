#include "app_booster.h"
#include "../communication_topic/rc_control_topics.hpp"
#include "../communication_topic/pc_control_topics.hpp"
#include "../communication_topic/actuator_cmd_topics.hpp"

#include "cmsis_os2.h"
#include "arm_math.h"
#include "motor_ids.hpp"
extern "C" {
#include "FreeRTOS.h"
#include "task.h" 
}
void Booster::Init()
{
    if (started_) {
        configASSERT(false);
        return;
    }

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
    static constexpr uint16_t kIds[3] = {
        static_cast<uint16_t>(motor_ids::kLeftWheel),
        static_cast<uint16_t>(motor_ids::kRightWheel),
        static_cast<uint16_t>(motor_ids::kPoke),
    };

    for (uint8_t i = 0; i < 3; ++i) {
        orb::DjiC6xxOmegaCmd c{};
        c.bus = orb::CanBus::CAN1;
        c.rx_std_id = kIds[i];
        c.omega = 0.0f;
        orb::dji_c6xx_omega_cmd.publish(c);
    }
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

        static constexpr uint16_t kIds[3] = {
            static_cast<uint16_t>(motor_ids::kLeftWheel),
            static_cast<uint16_t>(motor_ids::kRightWheel),
            static_cast<uint16_t>(motor_ids::kPoke),
        };

        // 摩擦轮电机
        for (uint8_t i = 0; i < 3; ++i) {
            orb::DjiC6xxOmegaCmd c{};
            c.bus = orb::CanBus::CAN1;
            c.rx_std_id = kIds[i];
            c.omega = 0.0f;
            if (i == 0) {
                c.omega = left_wheel_omega_;
            } else if (i == 1) {
                c.omega = right_wheel_omega_;
            } else if (i == 2) {
                c.omega = poke_omega_;
            }
            orb::dji_c6xx_omega_cmd.publish(c);
        }
        osDelay(pdMS_TO_TICKS(10));
    }
}