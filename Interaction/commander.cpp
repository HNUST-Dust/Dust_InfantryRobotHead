#include "booster.h"
#include "imu_temp_ctrl.h"
#include "mcu_comm.h"
#include "pc_comm.h"

#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "VT03.h"
#include "commander.h"
#include "spi.h"
#include "debug_tools.h"
#include <cstring>

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
        if (VT03.Data.Right_Key == VT03_Key_Status_PRESSED 
            || VT03.Data.Keyboard_Key[8] == VT03_Key_Status_PRESSED){
            Booster.Set_Reverse_Statue(1);
        }else if (VT03.Data.Right_Key == VT03_Key_Status_FREE 
            || VT03.Data.Keyboard_Key[8] == VT03_Key_Status_FREE){
            Booster.Set_Reverse_Statue(0);
        }
        if (VT03.Data.Trigger == VT03_Key_Status_PRESSED 
            || VT03.Data.Mouse_Left_Key == VT03_Key_Status_PRESSED){
            Booster.Set_Shoot_Statue(1);
        }else if (VT03.Data.Trigger == VT03_Key_Status_FREE 
            || VT03.Data.Mouse_Left_Key == VT03_Key_Status_FREE){
            Booster.Set_Shoot_Statue(0);
        }

        // 将遥控器数据发给下板
        MCU_Comm.CAN_Send_Command();

        // 将下板传回的数据发送给上位机
        PC_Comm.PC_Send_Data.mode = 1; // 自瞄模式
        memcpy(PC_Comm.PC_Send_Data.q,
            g_q_vision,
            4 * sizeof(float));
        memcpy(&PC_Comm.PC_Send_Data.yaw.yaw_ang, 
            &g_yaw, 
            4);
        memcpy(&PC_Comm.PC_Send_Data.yaw.yaw_vel, 
            MCU_Comm.MCU_Recv_Data.Yaw_Omega, 
            4);
        memcpy(&PC_Comm.PC_Send_Data.pitch.pitch_ang, 
            &g_pitch_vision, 
            4);
        memcpy(&PC_Comm.PC_Send_Data.pitch.pitch_vel, 
            MCU_Comm.MCU_Recv_Data.Pitch_Omega, 
            4);
        PC_Comm.PC_Send_Data.bullet.bullet_speed = 20.0f; // 子弹速度20m/s
        PC_Comm.PC_Send_Data.bullet.bullet_count = 1; // 子弹累计发送次数
        PC_Comm.PC_Send_Data.crc16 = 0; // TODO: 计算CRC16校验码
        PC_Comm.Send_Message();

        // 将上位机传回的数据发送给下板
        memcpy(MCU_Comm.MCU_AutoAim_Data.yaw_angle,
            &PC_Comm.PC_Recv_Data.yaw.yaw_ang,
            4 * sizeof(uint8_t));
        memcpy(MCU_Comm.MCU_AutoAim_Data.yaw_omega,
            &PC_Comm.PC_Recv_Data.yaw.yaw_vel,
            4 * sizeof(uint8_t));
        memcpy(MCU_Comm.MCU_AutoAim_Data.yaw_torque,
            &PC_Comm.PC_Recv_Data.yaw.yaw_acc,
            4 * sizeof(uint8_t));
        memcpy(MCU_Comm.MCU_AutoAim_Data.pitch_angle,
            &PC_Comm.PC_Recv_Data.pitch.pitch_ang,
            4 * sizeof(uint8_t));
        memcpy(MCU_Comm.MCU_AutoAim_Data.pitch_omega,
            &PC_Comm.PC_Recv_Data.pitch.pitch_vel,
            4 * sizeof(uint8_t));
        memcpy(MCU_Comm.MCU_AutoAim_Data.pitch_torque,
            &PC_Comm.PC_Recv_Data.pitch.pitch_acc,
            4 * sizeof(uint8_t));
        MCU_Comm.CAN_Send_AutoAim();
        
        // 自瞄火控
        if (PC_Comm.PC_Recv_Data.mode == 2){
            Booster.Set_AutoAIM_Fire_Statue(1);
        }else{
            Booster.Set_AutoAIM_Fire_Statue(0);
        }

        // 将陀螺仪数据发送给下板
        MCU_Comm.CanSendImu();
        // debugtools_.VofaSendFloat(g_yaw_omega);
        // debugtools_.VofaSendTail();
        osDelay(pdMS_TO_TICKS(1));
    }
}

