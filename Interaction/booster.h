#ifndef BOOSTER_H
#define BOOSTER_H

#include "dvc_motor_dji.h"
#include <cstdint>
#include "alg_pid.h"
#include "low_pass_filter.hpp"
class Class_Booster
{
public:

    // 发射机构2个3508， 控制摩擦轮
    Class_Motor_DJI_C620 Motor_Booster_1,
                         Motor_Booster_2;

    // 发射机构1个2006，控制拨弹盘
    Class_Motor_DJI_C610 Motor_Booster_3;

    alg::LowPassFilter booster_1_omega_filter_;
    alg::LowPassFilter booster_2_omega_filter_;
    alg::LowPassFilter booster_3_omega_filter_;

    // 外置 PID：由 Booster 任务自行计算（速度环输出电流）
    Pid booster_1_pid_omega_;
    Pid booster_2_pid_omega_;
    Pid booster_3_pid_omega_;
    
    void Init();
    void Task();
    inline void Set_Target_Velocity(float __Target_Velocity);
    inline void Set_Switch_Statue(uint8_t __switch);
    inline void Set_Shoot_Statue(uint8_t __switch);
    inline void Set_Reverse_Statue(uint8_t __switch);
    inline void Set_AutoAim_Statue(uint8_t __switch);
    inline void Set_AutoAim_Fire_Statue(uint8_t __switch);
    inline uint8_t Get_Switch_Statue();
    inline uint8_t Get_Shoot_Statue();
    inline uint8_t Get_Reverse_Statue();
    // 目标速度
    float target_omega_1 = 0.0f;
    float target_omega_2 = 0.0f;
    float target_omega_3 = 0.0f;
    float Target_Velocity = 0.0f;
protected:
    uint8_t start_switch = 0; // 摩擦轮开关状态
    uint8_t shoot_switch = 0; // 拨弹盘开关状态
    uint8_t auto_aim_flag = 0; // 自瞄模式状态位
    uint8_t auto_aim_fire_switch = 0; // 自瞄火控开关状态
    uint8_t reverse_switch = 0; // 退弹开关状态


    static void TaskEntry(void *param); 
};

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Booster::Set_Target_Velocity(float __Target_Velocity)
{
    Target_Velocity = __Target_Velocity;
}

inline void Class_Booster::Set_Switch_Statue(uint8_t __switch)
{
    start_switch = __switch;
}

inline void Class_Booster::Set_Shoot_Statue(uint8_t __switch)
{
    shoot_switch = __switch;
}

inline void Class_Booster::Set_Reverse_Statue(uint8_t __switch)
{
    reverse_switch = __switch;
}

inline void Class_Booster::Set_AutoAim_Statue(uint8_t __switch)
{
    auto_aim_flag = __switch;
}

inline void Class_Booster::Set_AutoAim_Fire_Statue(uint8_t __switch)
{
    auto_aim_fire_switch = __switch;
}

inline uint8_t Class_Booster::Get_Switch_Statue()
{
    return start_switch;
}
inline uint8_t Class_Booster::Get_Shoot_Statue()
{
    return shoot_switch;
}
inline uint8_t Class_Booster::Get_Reverse_Statue()
{
    return reverse_switch;
}
#endif //BOOSTER_H
