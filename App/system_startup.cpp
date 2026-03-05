/**
 * @file system_startup.cpp
 * @brief 工程统一启动序列实现（System_Boot + staged bring-up）
 *
 * 核心逻辑：
 * =========
 * - `System_Boot()` 按固定顺序执行：BSP -> Board -> Modules -> App。
 * - 将“模块拉起顺序”集中管理，避免分散在各处导致依赖关系难以审查。
 *
 * 关键依赖：
 * =========
 * - DWT 时间基在 Board_BringUp() 早期初始化，供 daemon/控制环路使用。
 * - daemon_supervisor 必须在各模块注册 client 前 init。
 * - CAN/UART 发送统一经由 TxTask：这里负责启动 `CanTxTask`/`UartTxTask` 作为唯一发送出口。
 *
 * 故障策略：
 * =========
 * - 通过 `DaemonSupervisor::set_system_fault_hook()` 设置系统级故障回调。
 *   当前实现为 best-effort：请求底盘/云台退出，并发布执行器层的管理指令。
 */

#include "system_startup.h"
#include "app_booster.h"
#include "bsp_can_port.h"
#include "bsp_uart_port.h"
#include "bsp_spi_port.h"
#include "bsp_usb_port.h"
#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"
#include "../Communication/dvc_mcu_comm.h"
#include "../Communication/dvc_pc_comm.h"
#include "../Device/debug_tools.h"
#include "../Device/motors/motor_instances.hpp"
#include "dvc_vt03.h"

#include "bmi088.h"


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

    switch (frame->id) {
    case 0x201:
        actuator::instances::g_left_wheel.CanRxCpltCallback(frame);
        break;
    case 0x202:
        actuator::instances::g_right_wheel.CanRxCpltCallback(frame);
        break;
    case 0x203:
        actuator::instances::g_poke.CanRxCpltCallback(frame);
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



namespace {
static void daemon_system_fault(DaemonClient&)
{

}
} // namespace

void Bsp_BringUp(void)
{
    dwt_init(480);

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

    // SPI 初始化（阻塞模式，无需回调）
    bsp_spi_init(bsp_spi_get(BSP_SPI_DEV_BMI088_ACCEL), nullptr, nullptr);
    bsp_spi_init(bsp_spi_get(BSP_SPI_DEV_BMI088_GYRO),  nullptr, nullptr);

}

void Board_BringUp(void)
{
    // BMI088 模块：采样 + 解算 + 发布 orb::imu_data
    static Bmi088 s_bmi088;
    s_bmi088.Start();
}

void Modules_BringUp(void)
{
    // 守护系统初始化
    DaemonSupervisor::Start(daemon_system_fault);

    // 大疆电机初始化（CAN1）
    auto* can1 = bsp_can_get(BSP_CAN_BUS1);

    actuator::instances::g_left_wheel.Init(can1, {
        .bus          = 1,
        .rx_std_id    = 0x201,
        .tx_std_id    = 0x200,
        .gearbox_ratio = 1.0f,
        .current_limit = 20.0f,
    });

    actuator::instances::g_right_wheel.Init(can1, {
        .bus          = 1,
        .rx_std_id    = 0x202,
        .tx_std_id    = 0x200,
        .gearbox_ratio = 1.0f,
        .current_limit = 20.0f,
    });

    actuator::instances::g_poke.Init(can1, {
        .bus          = 1,
        .rx_std_id    = 0x203,
        .tx_std_id    = 0x200,
        .gearbox_ratio = 1.0f,
        .current_limit = 20.0f,
    });

    // 上下板通讯组件初始化（CAN2）
    McuComm::Instance().Init(orb::CanBus::CAN2,bsp_can_get(BSP_CAN_BUS2));

    // // 裁判系统初始化（UART1 RX already started in Bsp_BringUp）
    // Referee::Instance().Init(bsp_uart_get(BSP_UART1), orb::UartPort::U1);

    // 接收机初始化 
    VT03::Instance().Init(bsp_uart_get(BSP_UART1), orb::UartPort::U1);

    // VOFA 初始化（UART7 RX already started in Bsp_BringUp）
    DebugTools::Instance().Init(bsp_uart_get(BSP_UART7));
}

void App_Start(void)
{
    Booster::Instance().Init();
}

void startup_thread(void *argument)
{
    Bsp_BringUp();
    Board_BringUp();
    Modules_BringUp();
    App_Start();
    vTaskDelete(NULL);
}