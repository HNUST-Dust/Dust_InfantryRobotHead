//
// Created by noe on 25-8-15.
//

#include "bsp_usb.h"
#include "bsp_usart.h"
#include "commander.h"
#include "mcu_comm.h"
#include "stm32h7xx_hal_gpio.h"
#include <cstdint>
#include <sys/types.h>

#include "Init.h"

// global variable
uint8_t* USB_RxBuf;
uint8_t usb_tx_cmplt_flag = 0;

Class_Commander Commander;

uint8_t chassis_speed_x = 127;
uint8_t chassis_speed_y = 127;
uint8_t target_speed_x = 127;
uint8_t target_speed_y = 127;
/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x201):
        {
            Commander.Booster.Motor_Booster_3.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x202):
        {
            Commander.Booster.Motor_Booster_1.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x203):
        {
            Commander.Booster.Motor_Booster_2.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    }
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (GIMBAL_INFO_ID):
        {
            Commander.MCU_Comm.CAN_Gimbal_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        default:
            break;
    }
}

void VT03_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    Commander.VT03.UART_RxCpltCallback(Buffer,Length);
    Commander.MCU_Comm.MCU_Comm_Data.Yaw_Angle            = (uint8_t)((Commander.VT03.Data.Right_X 
                                                                        + Commander.VT03.Data.Mouse_X)*255);
    Commander.MCU_Comm.MCU_Comm_Data.Pitch_Angle          = (uint8_t)((Commander.VT03.Data.Right_Y 
                                                                        + Commander.VT03.Data.Mouse_Y)*255);

    // --- 计算目标速度 ---
    if (Commander.VT03.Data.Keyboard_Key[0] == VT03_KEY_PRESSED) { // W
        target_speed_x = 255.0f;
    } else if (Commander.VT03.Data.Keyboard_Key[1] == VT03_KEY_PRESSED) { // S
        target_speed_x = 0.0f;
    } else {
        target_speed_x = 127.0f; // 无输入时保持静止
    }

    if (Commander.VT03.Data.Keyboard_Key[3] == VT03_KEY_PRESSED) { // D
        target_speed_y = 255.0f;
    } else if (Commander.VT03.Data.Keyboard_Key[2] == VT03_KEY_PRESSED) { // A
        target_speed_y = 0.0f;
    } else {
        target_speed_y = 127.0f; // 无输入时保持静止
    }

    // --- 缓启动逼近 ---
    float accel = 5.0f;
    if (chassis_speed_x < target_speed_x)
        chassis_speed_x = fminf(chassis_speed_x + accel, target_speed_x);
    else if (chassis_speed_x > target_speed_x)
        chassis_speed_x = fmaxf(chassis_speed_x - accel, target_speed_x);

    if (chassis_speed_y < target_speed_y)
        chassis_speed_y = fminf(chassis_speed_y + accel, target_speed_y);
    else if (chassis_speed_y > target_speed_y)
        chassis_speed_y = fmaxf(chassis_speed_y - accel, target_speed_y);

    if(target_speed_x == 127){
        Commander.MCU_Comm.MCU_Comm_Data.Chassis_Speed_X      = (uint8_t)(Commander.VT03.Data.Left_X * 255);
    }else if(target_speed_x != 127){
        Commander.MCU_Comm.MCU_Comm_Data.Chassis_Speed_X      = chassis_speed_x;
    }
    if(target_speed_y == 127){
        Commander.MCU_Comm.MCU_Comm_Data.Chassis_Speed_Y      = (uint8_t)(Commander.VT03.Data.Left_Y * 255);
    }else if(target_speed_y != 127){
        Commander.MCU_Comm.MCU_Comm_Data.Chassis_Speed_Y      = chassis_speed_y;
    }

    Commander.MCU_Comm.MCU_Comm_Data.Chassis_Rotation         = (uint8_t)(Commander.VT03.Data.Wheel*255);

    if((uint8_t)(Commander.VT03.Data.Mode_Switch) == 0 
        || Commander.VT03.Data.Keyboard_Key[4] == VT03_Key_Status_PRESSED)//shift
    {
        Commander.MCU_Comm.MCU_Comm_Data.Chassis_Spin = 0; //顺时针转
    }else{
        Commander.MCU_Comm.MCU_Comm_Data.Chassis_Spin         = (uint8_t)(Commander.VT03.Data.Mode_Switch); //并允许遥控器命令进行覆盖
    }

    if((uint8_t)(Commander.VT03.Data.Pause) == 1 
        || Commander.VT03.Data.Keyboard_Key[7] == VT03_Key_Status_PRESSED)//E键
    {
        Commander.MCU_Comm.MCU_Comm_Data.Supercap = 1; //放电
    }else{
        Commander.MCU_Comm.MCU_Comm_Data.Supercap = 0; //充电
    }

    if((uint8_t)(Commander.VT03.Data.Mouse_Right_Key) == 1)
    {
        Commander.MCU_Comm.MCU_Comm_Data.AutoAim = 1; //自瞄开启
    }else {
        Commander.MCU_Comm.MCU_Comm_Data.AutoAim = 0; //自瞄关闭
    }
}

/**
 * @bief USB接收完成回调函数
 *
 * @param len 接收到的数据长度
 */
void usb_rx_callback(uint16_t len)
{
    Commander.PC_Comm.RxCpltCallback();
}

/**
 * @bief USB发送完成回调函数
 *
 * @param len 发送的数据长度
 */
void usb_tx_callback(uint16_t len)
{

}

void Init()
{
    osDelay(10000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    // USB初始化
    USB_Init(usb_tx_callback,usb_rx_callback);
    // UART1 初始化，新图传通讯
    UART_Init(&huart1,VT03_UART1_Callback,512);
    // CAN1 初始化，控制发射
    CAN_Init(&hfdcan1,Device_CAN1_Callback);
    // CAN2 初始化，与下板通讯
    CAN_Init(&hfdcan2,Device_CAN2_Callback);

    Commander.Init();
}
