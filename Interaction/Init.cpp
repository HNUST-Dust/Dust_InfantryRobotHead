//
// Created by noe on 25-8-15.
//

#include "bsp_usb.h"
#include "bsp_usart.h"
#include "commander.h"
#include "mcu_comm.h"
#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_gpio.h"
#include "usart.h"
#include <cstdint>
#include <sys/types.h>

#include "Init.h"

// global variable
uint8_t* USB_RxBuf;
uint8_t usb_tx_cmplt_flag = 0;

Commander Commander;


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
}


void vt02_uart1_callback(uint8_t *buffer, uint16_t length)
{
    Commander.vt02_.RxCpltCallback(buffer, length);
}

void hipnuc_uart10_callback(uint8_t *buffer, uint16_t length)
{
    Commander.hipnuc_imu_.RxCpltCallback(buffer, length);
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
    // UART_Init(&huart1,dr16_uart1_callback,1024);
    // UART_Init(&huart1,vt02_uart1_callback,512);
    // UART10
    UART_Init(&huart10,hipnuc_uart10_callback,512);
    // CAN1 初始化，控制发射
    CAN_Init(&hfdcan1,Device_CAN1_Callback);
    // CAN2 初始化，与下板通讯
    CAN_Init(&hfdcan2,Device_CAN2_Callback);

    Commander.Init();
}
