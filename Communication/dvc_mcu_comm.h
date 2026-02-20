#pragma once
#include "debug_tools.h"
#include "bsp_can_port.h"
#include "can_topics.hpp"
#include "../communication_topic/pc_control_topics.hpp"
#include "../communication_topic/rc_control_topics.hpp"
#include "../communication_topic/imu_topics.hpp"
#include "../communication_topic/can_topics.hpp"


struct McuControlData
{
    uint8_t yaw_angle;          // 偏移角度
    uint8_t pitch_angle;        // 俯仰角度
    uint8_t chassis_speed_x;    // 平移方向：前、后、左、右
    uint8_t chassis_speed_y;    // 底盘移动总速度
    uint8_t chassis_rotation;   // 自转：不转、顺时针转、逆时针转
    uint8_t chassis_spin;       // 小陀螺：不转、顺时针转、逆时针转
    uint8_t supercap;           // 超级电容：充电、放电
    uint8_t auto_aim;            // 自瞄开关
    uint8_t gimbal_set_zero;     // 云台归零
};
constexpr uint8_t REMOTE_CONTRL_ID = 0xAB;

struct McuReciveData
{
    uint8_t yaw_angle[4];
    uint8_t yaw_omega[4];
    uint8_t pitch_angle[4];
    uint8_t pitch_omega[4];
};
constexpr uint16_t GIMBAL_INFO_ID      = 0x0A;

struct McuAutoaimData
{
    uint8_t yaw_angle[4];
    uint8_t yaw_omega[4];
    uint8_t yaw_torque[4];
    uint8_t pitch_angle[4];
    uint8_t pitch_omega[4];
    uint8_t pitch_torque[4];
};
constexpr uint8_t AUTOAIM_INFO_ID    = 0xFA;

struct McuImuData
{
    uint8_t yaw[4];
    uint8_t pitch[4];
};
constexpr uint8_t IMU_INFO_ID    = 0xAE;

class McuComm {
public:
    static McuComm& Instance() {
        static McuComm instance;
        return instance;
    }
    struct Config {
        orb::CanBus bus = orb::CanBus::CAN2;
    };

    void Init(orb::CanBus bus,BspCanHandle can);
    void Task();
    void RxCpltCallback(const BspCanFrame* frame);

private:
    DebugTools debug_tools_;
    bool started_ = false;
    osThreadId_t thread_ = nullptr;

    // TX publish target bus (unified CanTxTask)
    orb::CanBus tx_bus_ = orb::CanBus::CAN1;
    BspCanHandle can_ = nullptr;

    void PublishFrame(uint16_t std_id, const uint8_t data[], uint8_t len);
    void SendCommandAndUI(const orb::RcControl& rc);
    void SendAutoAim(const orb::PcControl& pc);
    void SendImu(const orb::ImuData& imu);


    static void TaskEntry(void *param);
};