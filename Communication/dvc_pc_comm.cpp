#include "dvc_pc_comm.h"
#include "../Platform/bsp_usb_port.h"
#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "../communication_topic/pc_comm_topics.hpp"
#include <cassert>

extern uint8_t g_recived_flag;

void PcComm::Init(const Config& cfg) {
    if (started_) {
        configASSERT(false);
        return;
    }
    cfg_ = cfg;

    started_ = true;

    static const osThreadAttr_t kPcCommTaskAttr = {
        .name = "pc_comm_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(PcComm::TaskEntry, this, &kPcCommTaskAttr);
    if (!thread_) {
        configASSERT(false);
    }
}

void PcComm::TaskEntry(void *param) {
    PcComm *self = static_cast<PcComm *>(param);
    self->Task();
}

void PcComm::RxCpltCallback(uint16_t len) {
    uint8_t* rx = bsp_usb_get_rx_buffer();
    if (!rx || len == 0) return;

    uint16_t copy_len = len;
    if (copy_len >= 2 && rx[0] == 'S' && rx[1] == 'P' && copy_len >= 29) {
        orb::PcRecvAutoAimData parsed{};
        // deserializePcRecv expects at least 29 bytes available
        orb::deserializePcRecv(rx, parsed);
        orb::pc_recv.publish(parsed);
        return;
    }
}

void PcComm::Task() {
    for (;;) {
        if (g_recived_flag) {
            g_recived_flag = 0;
            // 这里可以处理接收数据（如有需要）
        }
        // 发送数据直接调用 bsp_usb_transmit
        if (pc_send_data_pending_) {
            BspUsbHandle h = bsp_usb_get();
            bsp_usb_transmit(h, reinterpret_cast<const uint8_t*>(&pc_send_data_), sizeof(pc_send_data_));
            pc_send_data_pending_ = false;
        }
        osDelay(1);
    }
}

void PcComm::Send(const orb::PcSendAutoAimData& data) {
    pc_send_data_ = data;
    pc_send_data_pending_ = true;
}
