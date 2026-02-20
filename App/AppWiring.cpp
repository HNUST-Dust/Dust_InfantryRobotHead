/**
 * @file AppWiring.cpp
 * @brief 平台 IO 装配实现：UART/CAN RX → 对应模块回调
 *
 * 核心逻辑：
 * =========
 * - 通过 `bsp_uart_init()` 注册 UART RX 回调。
 * - 通过 `bsp_can_add_rx_callback()` 注册 CAN RX 回调（支持多个订阅者扇出）。
 * - 回调内部只做“ID 分发/转发”，将数据交给模块实例处理。
 *
 * 数据流（以当前实现为准）：
 * ========================
 * - UART7: Debug/VOFA → `DebugTools::Instance().VofaReceiveCallback()`
 * - UART1: Referee → `Referee::Instance().RxCpltCallback()`
 * - CAN1/2/3: Motors → 按电机 ID 分发到各电机实例 `CanRxCpltCallback()`
 * - CAN2(特定 ID): 外部 MCU 数据 → `McuComm::Instance().RxCpltCallback()`
 * - CAN3(0x100): Supercap → `Supercap::Instance().CanRxCpltCallback()`
 *
 * 注意事项：
 * =========
 * - 该文件不做 CAN/UART 发送；发送必须走统一的 TxTask/Topic 出口。
 * - 回调可能在中断上下文执行：必须避免阻塞与复杂计算。
 */

#include "AppWiring.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"
#include "bsp_usb_port.h"

#include "../Communication/dvc_mcu_comm.h"  // AUTOAIM_INFO_ID / REMOTE_CONTROL_ID / IMU_INFO_ID
#include "../Communication/dvc_pc_comm.h"
#include "../Device/debug_tools.h"

#include "../Device/motor_ids.hpp"

#include "motors/dji_c6xx.hpp"
#include "dvc_vt03.h"

// USART7 VOFA debug
static void uart7_debug_callback(uint8_t *buffer, uint16_t length)
{
    DebugTools::Instance().VofaReceiveCallback(buffer, length);
}

// USART1 裁判系统
static void uart1_referee_callback(uint8_t *buffer, uint16_t length)
{
    VT03::Instance().RxCpltCallback(buffer, length);
}

// 底盘电机
static void can1_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }

    switch (frame->id) {
    case motor_ids::kLeftWheel:
        actuator::instances::dji_201.CanRxCpltCallback(frame);
        break;
    case motor_ids::kRightWheel:
        actuator::instances::dji_202.CanRxCpltCallback(frame);
        break;
    case motor_ids::kPoke:
        actuator::instances::dji_203.CanRxCpltCallback(frame);
        break;
    default:
        break;
    }
}

// 上下板通讯
static void can2_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }
    switch (frame->id) {
    case GIMBAL_INFO_ID:
        McuComm::Instance().RxCpltCallback(frame);
        break;

    default:
        break;
    }
}

static void usb_tx_callback(uint16_t len)
{
    // 目前不需要处理发送完成事件
}

static void usb_rx_callback(uint16_t len)
{
    PcComm::Instance().RxCpltCallback(len);
}

void App_WirePlatformIo(void)
{
    constexpr uint16_t kUartRxBufferSize = 512;

    bsp_usb_init(bsp_usb_get(), usb_tx_callback, usb_rx_callback);

    // UART
    bsp_uart_init(bsp_uart_get(BSP_UART7), uart7_debug_callback, kUartRxBufferSize);
    bsp_uart_init(bsp_uart_get(BSP_UART1), uart1_referee_callback, kUartRxBufferSize);

    // CAN
    auto* can1 = bsp_can_get(BSP_CAN_BUS1);
    auto* can2 = bsp_can_get(BSP_CAN_BUS2);

    (void)bsp_can_add_rx_callback(can1, can1_rx_callback);
    (void)bsp_can_add_rx_callback(can2, can2_rx_callback);
}
