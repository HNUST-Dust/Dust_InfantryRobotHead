#include "FreeRTOS.h"

#include "cmsis_os2.h"

#include "booster.h"

#include "drv_can.h"


void Class_Booster::Init()
{
    // 3508电机初始化
    Motor_Booster_1.PID_Omega.Init(1.0f,0.0f,0.0f);
    Motor_Booster_2.PID_Omega.Init(1.0f,0.0f,0.0f);
    Motor_Booster_3.PID_Omega.Init(1.0f,0.0f,0.0f);

    Motor_Booster_1.Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA); // 202为右摩擦轮
    Motor_Booster_2.Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA); // 203为左摩擦轮
    Motor_Booster_3.Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA); // 201为拨弹盘

    Motor_Booster_1.Set_Target_Omega(0.0f);
    Motor_Booster_2.Set_Target_Omega(0.0f);
    Motor_Booster_3.Set_Target_Omega(0.0f);

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
    for (;;)
    {
        if (start_switch == 1){
            Motor_Booster_1.Set_Target_Omega(40); //40
            Motor_Booster_2.Set_Target_Omega(-40);
        }else if(start_switch == 0){
            Motor_Booster_1.Set_Target_Omega(0);
            Motor_Booster_2.Set_Target_Omega(0);
        }

        // 没穷举出所有情况
        if (((shoot_switch == 1) || (auto_aim_fire_switch == 1)) 
            && (reverse_switch == 0)
            && (start_switch == 1))
        {
            Motor_Booster_3.Set_Target_Omega(18);//20
        }else if ((shoot_switch == 0) && (auto_aim_fire_switch == 0) &&(reverse_switch == 0)){
            Motor_Booster_3.Set_Target_Omega(0);
        }else if ((shoot_switch == 0) && (auto_aim_fire_switch == 0) && (reverse_switch == 1)){
            Motor_Booster_3.Set_Target_Omega(-18);
        }

        Motor_Booster_1.Calculate_PeriodElapsedCallback();
        Motor_Booster_2.Calculate_PeriodElapsedCallback();
        Motor_Booster_3.Calculate_PeriodElapsedCallback();

        // 摩擦轮电机
        CAN_Send_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, 8);
        osDelay(pdMS_TO_TICKS(10));
    }
}