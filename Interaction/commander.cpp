#include "VT02.h"
#include "booster.h"
#include "imu_temp_ctrl.h"
#include "mcu_comm.h"
#include "pc_comm.h"

#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "VT03.h"
#include "dr16.h"
#include "commander.h"
#include "debug_tools.h"
#include <cstring>

void Commander::Init()
{
    // dwt初始化
    dwt_init(480);
    // IMU初始化
    imu_.Init();
    // 图传接收机初始化
    VT03.Init(&huart1);
    dr16_.Init();
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
    osThreadNew(Commander::TaskEntry, this, &CommanderTaskAttr);
}


void Commander::TaskEntry(void *argument)
{
    Commander *self = static_cast<Commander *>(argument);
    self->Task();
}
void Commander::control_data_process()
{
    // --- 计算目标速度 ---
#ifdef USE_VT02
    if (vt02_.GetData()->keyboard[VT02_KEY_W].current_status == VT02::PRESSED) { // W
        target_speed_x = 255.0f;
    } else if (vt02_.GetData()->keyboard[VT02_KEY_S].current_status == VT02::PRESSED) { // S
        target_speed_x = 0.0f;
    } else {
        target_speed_x = 127.0f; // 无输入时保持静止
    }

    if (vt02_.GetData()->keyboard[VT02_KEY_D].current_status == VT02::PRESSED) { // D
        target_speed_y = 255.0f;
    } else if (vt02_.GetData()->keyboard[VT02_KEY_A].current_status == VT02::PRESSED) { // A
        target_speed_y = 0.0f;
    } else {
        target_speed_y = 127.0f; // 无输入时保持静止
    }
#endif
#ifndef USE_VT02
    if (VT03.Data.Keyboard_Key[0] == VT03_KEY_PRESSED) { // W
        target_speed_x = 255.0f;
    } else if (VT03.Data.Keyboard_Key[1] == VT03_KEY_PRESSED) { // S
        target_speed_x = 0.0f;
    } else {
        target_speed_x = 127.0f; // 无输入时保持静止
    }

    if (VT03.Data.Keyboard_Key[3] == VT03_KEY_PRESSED) { // D
        target_speed_y = 255.0f;
    } else if (VT03.Data.Keyboard_Key[2] == VT03_KEY_PRESSED) { // A
        target_speed_y = 0.0f;
    } else {
        target_speed_y = 127.0f; // 无输入时保持静止
    }
#endif
    // --- 缓启动逼近 ---
    float accel = 1.0f;
    if (chassis_speed_x < target_speed_x)
        chassis_speed_x = fminf(chassis_speed_x + accel, target_speed_x);
    else if (chassis_speed_x > target_speed_x)
        chassis_speed_x = fmaxf(chassis_speed_x - accel, target_speed_x);

    if (chassis_speed_y < target_speed_y)
        chassis_speed_y = fminf(chassis_speed_y + accel, target_speed_y);
    else if (chassis_speed_y > target_speed_y)
        chassis_speed_y = fmaxf(chassis_speed_y - accel, target_speed_y);
}

void Commander::publish_control_info()
{
#ifdef USE_VT02
    MCU_Comm.MCU_Comm_Data.Yaw_Angle            = (uint8_t)((vt02_.GetData()->mouse.x + 1)*127);
    MCU_Comm.MCU_Comm_Data.Pitch_Angle          = (uint8_t)((vt02_.GetData()->mouse.y + 1)*127);

    MCU_Comm.MCU_Comm_Data.Chassis_Speed_X      = chassis_speed_x;
    MCU_Comm.MCU_Comm_Data.Chassis_Speed_Y      = chassis_speed_y;
    
    if(vt02_.GetData()->keyboard[VT02_KEY_SHIFT].current_status == VT02::PRESSED)//shift
    {
        MCU_Comm.MCU_Comm_Data.Chassis_Spin = 0; //顺时针转
    }else{
        MCU_Comm.MCU_Comm_Data.Chassis_Spin = 1;
    }

    if(vt02_.GetData()->keyboard[VT02_KEY_E].key_status == VT02::TRIG_PRESSED_FREE)//E键
    {
        MCU_Comm.MCU_Comm_Data.Supercap = 1; //放电
    }else{
        MCU_Comm.MCU_Comm_Data.Supercap = 0; //充电
    }

    if(vt02_.GetData()->mouse.r == VT02::PRESSED)
    {
        MCU_Comm.MCU_Comm_Data.AutoAim = 1; //自瞄开启
        Booster.Set_AutoAim_Fire_Statue(1);
    }else {
        MCU_Comm.MCU_Comm_Data.AutoAim = 0; //自瞄关闭
        Booster.Set_AutoAim_Fire_Statue(0);
    }

    // 摩擦轮逻辑
    if (vt02_.GetData()->keyboard[VT02_KEY_F].key_status == VT02::TRIG_PRESSED_FREE){ //F
        // ★ 消费事件，防止重复触发
        vt02_.GetData()->keyboard[VT02_KEY_F].key_status
            = VT02::FREE;
        if (Booster.Get_Switch_Statue() == 1){
            Booster.Set_Switch_Statue(0);
        }else if(Booster.Get_Switch_Statue() == 0){
            Booster.Set_Switch_Statue(1);
        }
    }

    // 云台归零逻辑
    if (vt02_.GetData()->keyboard[VT02_KEY_B].key_status == VT02::TRIG_PRESSED_FREE){ //B
        // ★ 消费事件，防止重复触发
        vt02_.GetData()->keyboard[VT02_KEY_B].key_status
            = VT02::FREE;
        MCU_Comm.MCU_Comm_Data.Gimbal_SetZero = 1;
    }

    //拨弹盘逻辑
    if (vt02_.GetData()->keyboard[VT02_KEY_R].current_status == VT02::PRESSED){
        Booster.Set_Reverse_Statue(1);
    }else {
        Booster.Set_Reverse_Statue(0);
    }   
    if (vt02_.GetData()->mouse.l == VT02::PRESSED){
        Booster.Set_Shoot_Statue(1);
    }else if (vt02_.GetData()->mouse.l == VT02::FREE){
        Booster.Set_Shoot_Statue(0);
    }

#endif
#ifndef USE_VT02
    MCU_Comm.MCU_Comm_Data.Yaw_Angle            = (uint8_t)((VT03.Data.Right_X 
        + VT03.Data.Mouse_X)*255);
    MCU_Comm.MCU_Comm_Data.Pitch_Angle          = (uint8_t)((VT03.Data.Right_Y 
        + VT03.Data.Mouse_Y)*255);

    if(target_speed_x == 127){
        MCU_Comm.MCU_Comm_Data.Chassis_Speed_X      = (uint8_t)(VT03.Data.Left_X * 255);
    }else if(target_speed_x != 127){
        MCU_Comm.MCU_Comm_Data.Chassis_Speed_X      = chassis_speed_x;
    }
    if(target_speed_y == 127){
        MCU_Comm.MCU_Comm_Data.Chassis_Speed_Y      = (uint8_t)(VT03.Data.Left_Y * 255);
    }else if(target_speed_y != 127){
        MCU_Comm.MCU_Comm_Data.Chassis_Speed_Y      = chassis_speed_y;
    }

    MCU_Comm.MCU_Comm_Data.Chassis_Rotation         = (uint8_t)(VT03.Data.Wheel*255);

    if((uint8_t)(VT03.Data.Mode_Switch) == 0 
        || VT03.Data.Keyboard_Key[4] == VT03_Key_Status_PRESSED)//shift
    {
        MCU_Comm.MCU_Comm_Data.Chassis_Spin = 0; //顺时针转
    }else{
        MCU_Comm.MCU_Comm_Data.Chassis_Spin         = (uint8_t)(VT03.Data.Mode_Switch); //并允许遥控器命令进行覆盖
    }

    if((uint8_t)(VT03.Data.Pause) == 1 
        || VT03.Data.Keyboard_Key[7] == VT03_Key_Status_PRESSED)//E键
    {
        MCU_Comm.MCU_Comm_Data.Supercap = 1; //放电
    }else{
        MCU_Comm.MCU_Comm_Data.Supercap = 0; //充电
    }
    
    if((uint8_t)(VT03.Data.Mouse_Right_Key) == 1)
    {
        MCU_Comm.MCU_Comm_Data.AutoAim = 1; //自瞄开启
        Booster.Set_AutoAim_Fire_Statue(1);
    }else {
        MCU_Comm.MCU_Comm_Data.AutoAim = 0; //自瞄关闭
        Booster.Set_AutoAim_Fire_Statue(0);
    }

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
#endif
    // 将遥控器数据发给下板
    MCU_Comm.CANSendCommandAndUI();
    if(MCU_Comm.MCU_Comm_Data.Gimbal_SetZero == 1)
    {
        // 发送完成后归零
        MCU_Comm.MCU_Comm_Data.Gimbal_SetZero = 0;
    }
}

void Commander::transfer_info_to_pc()
{
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

}

void Commander::transfer_info_to_bottomboard()
{
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
    
}

void Commander::subscribe_info_from_pc()
{
    // 自瞄火控
#ifdef USE_VT02 
    if ((PC_Comm.PC_Recv_Data.mode == 2) 
    && (vt02_.GetData()->mouse.r == VT02::PRESSED))
    {
        Booster.Set_AutoAim_Fire_Statue(1);
    }else{
        Booster.Set_AutoAim_Fire_Statue(0);
    }
#endif
#ifndef USE_VT02
    
    if ((PC_Comm.PC_Recv_Data.mode == 2) 
        && (VT03.Data.Mouse_Right_Key == 1))
    {
        Booster.Set_AutoAim_Fire_Statue(1);
    }else{
        Booster.Set_AutoAim_Fire_Statue(0);
    }
#endif
}

void Commander::publish_posture_info_to_bottomboard()
{
    // 将陀螺仪数据发送给下板
    MCU_Comm.CanSendImu();
}
void Commander::Task()
{
    for (;;)
    {
        control_data_process();
        publish_control_info();
        transfer_info_to_pc();
        transfer_info_to_bottomboard();
        subscribe_info_from_pc();
        publish_posture_info_to_bottomboard();

        // debugtools_.VofaSendFloat((float)vt02_.GetData()->mouse.x);
        // debugtools_.VofaSendFloat((float)vt02_.GetData()->mouse.y);
        // debugtools_.VofaSendFloat((float)MCU_Comm.MCU_Comm_Data.Yaw_Angle);
        // debugtools_.VofaSendFloat((float)MCU_Comm.MCU_Comm_Data.Pitch_Angle);
        // debugtools_.VofaSendFloat((float)vt02_.GetData()->mouse.z);
        // debugtools_.VofaSendTail();
        osDelay(pdMS_TO_TICKS(1));
    }
}

