#include "booster.h"
#include "mcu_comm.h"
#include "pc_comm.h"

#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "VT03.h"
#include "commander.h"
#include "spi.h"
#include "debug_tools.h"

void Class_Commander::Init()
{
    // dwt初始化
    dwt_init(480);
    // IMU初始化
    imu_.Init();
    // 图传接收机初始化
    VT03.Init(&huart1);
    // 与下板通讯服务初始化
    MCU_Comm.Init(&hfdcan2,0x00,0x01);
    // 与上位机通讯初始化
    PC_Comm.Init();
    // 发射机构初始化
    Booster.Init();

    static const osThreadAttr_t CommanderTaskAttr = {
        .name = "CommanderTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Class_Commander::TaskEntry, this, &CommanderTaskAttr);
}


void Class_Commander::TaskEntry(void *argument)
{
    Class_Commander *self = static_cast<Class_Commander *>(argument);
    self->Task();
}

void Class_Commander::Task()
{
    for (;;)
    {
        // 左侧自定义按键逻辑
        if (VT03.Data.Left_Key == VT03_Key_Status_TRIG_PRESSED_FREE 
            || VT03.Data.Keyboard_Key[9] == VT03_Key_Status_TRIG_PRESSED_FREE){
            if (Booster.Get_Switch_Statue() == 1){
                Booster.Set_Switch_Statue(0);
            }else if(Booster.Get_Switch_Statue() == 0){
                Booster.Set_Switch_Statue(1);
            }
        }
        //遥控器扳机键逻辑
        if (VT03.Data.Right_Key == VT03_Key_Status_PRESSED || VT03.Data.Keyboard_Key[8] == VT03_Key_Status_PRESSED){
            Booster.Set_Reverse_Statue(1);
        }else if (VT03.Data.Right_Key == VT03_Key_Status_FREE || VT03.Data.Keyboard_Key[8] == VT03_Key_Status_FREE){
            Booster.Set_Reverse_Statue(0);
        }
        if (VT03.Data.Trigger == VT03_Key_Status_PRESSED || VT03.Data.Mouse_Left_Key == VT03_Key_Status_PRESSED){
            Booster.Set_Shoot_Statue(1);
        }else if (VT03.Data.Trigger == VT03_Key_Status_FREE || VT03.Data.Mouse_Left_Key == VT03_Key_Status_FREE){
            Booster.Set_Shoot_Statue(0);
        }

        // 将遥控器数据发给下板
        MCU_Comm.CAN_Send_Command();

        // 将下板传回的数据发送给上位机
        PC_Comm.PC_Send_Data.Armor = 0x00;
        memcpy(PC_Comm.PC_Send_Data.Yaw, MCU_Comm.MCU_Recv_Data.Yaw, 4);
        memcpy(PC_Comm.PC_Send_Data.Pitch, MCU_Comm.MCU_Recv_Data.Pitch, 4);
        PC_Comm.Send_Message();

        // 将上位机传回的数据发送给下板
        memcpy(MCU_Comm.MCU_AutoAim_Data.Yaw,&PC_Comm.PC_Recv_Data.Yaw,4 * sizeof(uint8_t));
        memcpy(MCU_Comm.MCU_AutoAim_Data.Pitch,&PC_Comm.PC_Recv_Data.Pitch,4 * sizeof(uint8_t));
        MCU_Comm.CAN_Send_AutoAim();

        // 将陀螺仪数据发送给下板
        MCU_Comm.CanSendImu();
        debugtools_.VofaSendFloat(g_yaw);
        debugtools_.VofaSendTail();
        osDelay(pdMS_TO_TICKS(1));
    }
}

