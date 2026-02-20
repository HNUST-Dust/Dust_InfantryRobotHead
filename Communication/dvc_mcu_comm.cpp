#include "dvc_mcu_comm.h"
#include "../communication_topic/pc_control_topics.hpp"
#include "../communication_topic/rc_control_topics.hpp"
#include "../communication_topic/imu_topics.hpp"
#include "../communication_topic/can_topics.hpp"
#include "../communication_topic/gimbal_topics.hpp"
#include "FreeRTOS.h"
#include "bsp_can_port.h"
#include "task.h"
#include "cmsis_os2.h"
#include "topic.hpp"
#include <cstdint>

void McuComm::Init(orb::CanBus bus, BspCanHandle can) {
    if (started_) {
        configASSERT(false);
        return;
    }
    tx_bus_ = bus;
    can_ = can;

    started_ = true;

    static const osThreadAttr_t kMcuCommTaskAttr = {
        .name = "mcu_comm_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(McuComm::TaskEntry, this, &kMcuCommTaskAttr);
    if (!thread_) {
        configASSERT(false);
    }
}

void McuComm::TaskEntry(void *param) {
    McuComm *self = static_cast<McuComm *>(param);
    self->Task();
}

void McuComm::PublishFrame(uint16_t std_id, const uint8_t data[], uint8_t len) {
    if (!can_) {
        return;
    }

    orb::CanTxFrame f{};
    f.bus = tx_bus_;
    f.id = std_id;
    f.id_type = orb::CanIdType::Std;
    f.frame_type = orb::CanFrameType::Data;
    f.is_fd = true;
    f.brs = true;
    f.len = len;
    std::memset(f.data, 0, sizeof(f.data));
    if (len > 0) {
        const uint8_t n = (len <= (int)sizeof(f.data)) ? len : (int)sizeof(f.data);
        std::memcpy(f.data, data, n);
    }
    orb::can_tx.publish(f);
}

void McuComm::SendCommandAndUI(const orb::RcControl& rc)
{
    uint8_t can_tx[12] = {0};
    can_tx[0] = rc.yaw_angle;
    can_tx[1] = rc.pitch_angle;
    can_tx[2] = rc.chassis_speed_x;
    can_tx[3] = rc.chassis_speed_y;
    can_tx[4] = rc.chassis_rotation;
    can_tx[5] = static_cast<uint8_t>(rc.chassis_spin);
    can_tx[6] = static_cast<uint8_t>(rc.super_cap);
    can_tx[7] = rc.auto_aim;
    can_tx[8] = static_cast<uint8_t>(rc.gimbal_set_zero);

    PublishFrame(REMOTE_CONTRL_ID, can_tx, 12);
}

void McuComm::SendAutoAim(const orb::PcControl& pc)
{
    uint8_t can_tx_autoaim[8] = {0};
    union { float f; uint8_t b[4]; } conv;
    conv.f = pc.yaw_angle;
    can_tx_autoaim[0] = conv.b[0];
    can_tx_autoaim[1] = conv.b[1];
    can_tx_autoaim[2] = conv.b[2];
    can_tx_autoaim[3] = conv.b[3];
    conv.f = pc.pitch_angle;
    can_tx_autoaim[4] = conv.b[0];
    can_tx_autoaim[5] = conv.b[1];
    can_tx_autoaim[6] = conv.b[2];
    can_tx_autoaim[7] = conv.b[3];

    PublishFrame(AUTOAIM_INFO_ID, can_tx_autoaim, 8);
}

void McuComm::SendImu(const orb::ImuData& imu)
{
    uint8_t can_tx_imu[16] = {0};
    union { float f; uint8_t b[4]; } conv;
    conv.f = imu.yaw_angle;
    can_tx_imu[0] = conv.b[0];
    can_tx_imu[1] = conv.b[1];
    can_tx_imu[2] = conv.b[2];
    can_tx_imu[3] = conv.b[3];
    conv.f = imu.pitch_angle;
    can_tx_imu[4] = conv.b[0];
    can_tx_imu[5] = conv.b[1];
    can_tx_imu[6] = conv.b[2];
    can_tx_imu[7] = conv.b[3];
    conv.f = imu.yaw_omega;
    can_tx_imu[8] = conv.b[0];
    can_tx_imu[9] = conv.b[1];
    can_tx_imu[10] = conv.b[2];
    can_tx_imu[11] = conv.b[3];
    conv.f = imu.pitch_omega;
    can_tx_imu[12] = conv.b[0];
    can_tx_imu[13] = conv.b[1];
    can_tx_imu[14] = conv.b[2];
    can_tx_imu[15] = conv.b[3];

    PublishFrame(IMU_INFO_ID, can_tx_imu, 16);
}

void McuComm::RxCpltCallback(const BspCanFrame* frame) {
    orb::GimbalInfo info{};
    union { float f; uint8_t b[4]; } conv;
    conv.b[0] = frame->data[0]; conv.b[1] = frame->data[1]; conv.b[2] = frame->data[2]; conv.b[3] = frame->data[3];
    info.yaw_angle = conv.f;
    conv.b[0] = frame->data[4]; conv.b[1] = frame->data[5]; conv.b[2] = frame->data[6]; conv.b[3] = frame->data[7];
    info.yaw_omega = conv.f;
    conv.b[0] = frame->data[8]; conv.b[1] = frame->data[9]; conv.b[2] = frame->data[10]; conv.b[3] = frame->data[11];
    info.pitch_angle = conv.f;
    conv.b[0] = frame->data[12]; conv.b[1] = frame->data[13]; conv.b[2] = frame->data[14]; conv.b[3] = frame->data[15];
    info.pitch_omega = conv.f;

    orb::gimbal_info.publish(info);
}

void McuComm::Task() {
    Subscription<orb::RcControl> rc_control_sub(orb::rc_control);
    Subscription<orb::PcControl> pc_control_sub(orb::pc_control);
    Subscription<orb::ImuData> imu_data_sub(orb::imu_data);
    orb::RcControl rc_control{};
    orb::PcControl pc_control{};
    orb::ImuData imu_data{};
    for (;;) 
    {
        (void)rc_control_sub.copy(rc_control);
        (void)pc_control_sub.copy(pc_control);
        (void)imu_data_sub.copy(imu_data);

        SendCommandAndUI(rc_control);
        SendAutoAim(pc_control);
        SendImu(imu_data);

        osDelay(1);
    }
}