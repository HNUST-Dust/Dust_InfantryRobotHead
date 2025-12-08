#ifndef PC_COMM_H
#define PC_COMM_H

#include "bsp_usb.h"
#include "cmsis_os2.h"

extern uint8_t g_recived_flag;

/**
 * @brief 转换联合体
 * 
 */
union PcConv
{
    uint8_t b[4];
    float f;
};

#pragma pack(1)
/**
 * @brief 自瞄发送结构体
 * 
 */
struct PCSendAutoAimData
{
    uint8_t head[2] = {'S','P'};

    uint8_t mode = 0;               // 0-空闲 1-自瞄

    float q[4];                     // 四元数姿态[w,x,y,z]

    struct
    {
        float yaw_ang;              // yaw轴角度
        float yaw_vel;              // yaw轴角速度
    } yaw;
    
    struct
    {
        float pitch_ang;            // pitch轴角度
        float pitch_vel;            // pitch轴角速度
    } pitch;
    
    struct
    {
        float bullet_speed;         // 子弹速度
        uint16_t bullet_count;      // 子弹累计发送次数
    } bullet;
    
    uint16_t crc16;                 // 校验位
};

/**
 * @brief 自瞄接收结构体
 * 
 */
struct PCRecvAutoAimData
{
    uint8_t head[2] = {'S','P'};
    uint8_t mode = 0;           // 0-空闲 1-自瞄不开火 2-自瞄开火  

    struct
    {
        float yaw_ang;          // yaw轴角度
        float yaw_vel;          // yaw轴角速度
        float yaw_acc;          // yaw轴角加速度
    } yaw;
    
    struct
    {
        float pitch_ang;        // pitch轴角度
        float pitch_vel;        // pitch轴角速度
        float pitch_acc;        // pitch轴角加速度
    } pitch;
    
    uint16_t crc16;             // 校验位
};

#pragma pack()

class Class_PC_Comm
{
public:
    PCSendAutoAimData PC_Send_Data;
    PCRecvAutoAimData PC_Recv_Data;
    void Init();
    void Send_Message();
    void RxCpltCallback();
private:

};



#endif //PC_COMM_H
