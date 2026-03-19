//
// Created by noe on 25-8-15.
//

#ifndef COMMANDER_H
#define COMMANDER_H

#include "FreeRTOS.h"
// app
#include "booster.h"
#include "IMU.hpp"
#include "hipnuc_imu.hpp"
// module
#include "debug_tools.h"
#include "mcu_comm.h"
#include "pc_comm.h"
#include "low_pass_filter.hpp"
#include "VT03.h"
#include "VT02.h"

// #define USE_VT02 
class Commander
{
public:
    // IMU
    Imu imu_;
    // hipnuc IMU
    HipnucIMU hipnuc_imu_;
    // 与下板通讯服务
    Class_MCU_Comm MCU_Comm;
    // 与接收机通讯服务
    Class_VT03 VT03;
    VT02 vt02_;
    // 与上位机通讯
    Class_PC_Comm PC_Comm;
    // 发射机构
    Class_Booster Booster;
    // 调试工具
    DebugTools debugtools_;
    /**
     * @brief 控制台初始化
     */
    void Init();

    /**
     * @brief 控制台上层调度任务
     */
    void Task();

private:
    uint8_t chassis_speed_x = 127;
    uint8_t chassis_speed_y = 127;
    uint8_t target_speed_x = 127;
    uint8_t target_speed_y = 127;
    LowPassFilter mouse_x_lpf_;
    LowPassFilter mouse_y_lpf_;
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
    void control_data_process();
    void publish_control_info();
    void transfer_info_to_pc();
    void transfer_info_to_bottomboard();
    void subscribe_info_from_pc();
    void publish_posture_info_to_bottomboard();
};


#endif //COMMANDER_H
