#include "FreeRTOS.h"

#include "cmsis_os2.h"

#include "booster.h"

#include "drv_can.h"


void Class_Booster::Init()
{
    // 外置 PID 初始化（速度环输出电流），周期与任务一致：10ms
    booster_1_pid_omega_.Init(2.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.002f);
    booster_2_pid_omega_.Init(2.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.002f);
    booster_3_pid_omega_.Init(8.0f, 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.002f);

    // 电机驱动层只做“电流输出”（协议/解析留在 dvc_motor_dji），控制环放在 Booster 任务
    Motor_Booster_1.Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_CURRENT); // 202为右摩擦轮
    Motor_Booster_2.Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_CURRENT); // 203为左摩擦轮
    Motor_Booster_3.Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_CURRENT); // 201为拨弹盘

    Motor_Booster_1.Set_Target_Current(0.0f);
    Motor_Booster_2.Set_Target_Current(0.0f);
    Motor_Booster_3.Set_Target_Current(0.0f);

    booster_1_omega_filter_.configure(300,0.002f);
    booster_2_omega_filter_.configure(300,0.002f);
    booster_3_omega_filter_.configure(300,0.002f);
    static const osThreadAttr_t BoosterTaskAttr = {
        .name = "BoosterTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Class_Booster::TaskEntry, this, &BoosterTaskAttr);
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Class_Booster::TaskEntry(void *argument)
{
    Class_Booster *self = static_cast<Class_Booster *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

// 实际任务逻辑
void Class_Booster::Task()
{
    for (;;) {

        // 摩擦轮目标速度（兼容旧逻辑：默认 45 rad/s；若外部设置了 Target_Velocity 则用外部值）
        if (start_switch == 1)
        {
            const float friction_omega = 45.0f;
            target_omega_1 = friction_omega;
            target_omega_2 = -(friction_omega + 5.0f);
        }else if(start_switch == 0){
            target_omega_1 = 0.0f;
            target_omega_2 = 0.0f;
        }


        // 开火/停止/退弹逻辑（拨弹盘目标速度）
        if ((shoot_switch == 1) && (reverse_switch == 0))
        {
            const bool fire_allowed = (((auto_aim_flag == 1) && (auto_aim_fire_switch == 1)) || (auto_aim_flag == 0));
            if (fire_allowed)
            {
                target_omega_3 = -5.0f; // 20
            } else {
                target_omega_3 = 0.0f;
            }
        }
        if ((shoot_switch == 0) && (reverse_switch == 1))
        {
            target_omega_3 = 5.0f;
        }
        if(shoot_switch == 0 && reverse_switch == 0)
        {
            target_omega_3 = 0.0f;
        }

        // 外置 PID + 滤波（速度→电流），输出交给电机驱动层做限幅/打包
        const float omega_1_f = booster_1_omega_filter_.update(Motor_Booster_1.Get_Now_Omega());
        booster_1_pid_omega_.SetTarget(target_omega_1);
        booster_1_pid_omega_.SetNow(Motor_Booster_1.Get_Now_Omega());
        booster_1_pid_omega_.CalculatePeriodElapsedCallback();
        Motor_Booster_1.Set_Target_Current(booster_1_pid_omega_.GetOut()); // 下

        const float omega_2_f = booster_2_omega_filter_.update(Motor_Booster_2.Get_Now_Omega());
        booster_2_pid_omega_.SetTarget(target_omega_2);
        booster_2_pid_omega_.SetNow(Motor_Booster_2.Get_Now_Omega());
        booster_2_pid_omega_.CalculatePeriodElapsedCallback();
        Motor_Booster_2.Set_Target_Current(booster_2_pid_omega_.GetOut());

        const float omega_3_f = booster_3_omega_filter_.update(Motor_Booster_3.Get_Now_Omega());
        booster_3_pid_omega_.SetTarget(target_omega_3);
        booster_3_pid_omega_.SetNow(Motor_Booster_3.Get_Now_Omega());
        booster_3_pid_omega_.CalculatePeriodElapsedCallback();
        Motor_Booster_3.Set_Target_Current(booster_3_pid_omega_.GetOut());

        Motor_Booster_1.Calculate_PeriodElapsedCallback();
        Motor_Booster_2.Calculate_PeriodElapsedCallback();
        Motor_Booster_3.Calculate_PeriodElapsedCallback();

        // 摩擦轮电机
        CAN_Send_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, 8);
        osDelay(pdMS_TO_TICKS(2));
    }
}