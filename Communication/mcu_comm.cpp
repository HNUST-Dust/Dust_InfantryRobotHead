#include "mcu_comm.h"
#include "VT03.h"
#include "drv_can.h"
// #include "ins_task.h"
#include "commander.h"
#include "pc_comm.h"
#include "imu_temp_ctrl.h"

void Class_MCU_Comm::Init(
     FDCAN_HandleTypeDef* hcan,
     uint8_t __CAN_Rx_ID,
     uint8_t __CAN_Tx_ID)
{
     if (hcan->Instance == FDCAN1)
     {
          CAN_Manage_Object = &CAN1_Manage_Object;
     }
     else if (hcan->Instance == FDCAN2)
     {
          CAN_Manage_Object = &CAN2_Manage_Object;
     }

     CAN_Rx_ID = __CAN_Rx_ID;
     CAN_Tx_ID = __CAN_Tx_ID;
}

void Class_MCU_Comm::CANSendCommandAndUI()
{
     static uint8_t CAN_Tx_Frame[12]; 
     CAN_Tx_Frame[0] = MCU_Comm_Data.Yaw_Angle;
     CAN_Tx_Frame[1] = MCU_Comm_Data.Pitch_Angle;
     CAN_Tx_Frame[2] = MCU_Comm_Data.Chassis_Speed_X;
     CAN_Tx_Frame[3] = MCU_Comm_Data.Chassis_Speed_Y;
     CAN_Tx_Frame[4] = MCU_Comm_Data.Chassis_Rotation;
     CAN_Tx_Frame[5] = MCU_Comm_Data.Chassis_Spin;
     CAN_Tx_Frame[6] = MCU_Comm_Data.Supercap;
     CAN_Tx_Frame[7] = MCU_Comm_Data.AutoAim;
     CAN_Tx_Frame[8] = MCU_Comm_Data.Gimbal_SetZero;
     
     FDCAN_Send_Data(CAN_Manage_Object->CAN_Handler, REMOTE_CONTRL_ID, CAN_Tx_Frame, 12);
}

void Class_MCU_Comm::CAN_Send_AutoAim()
{
     static uint8_t CAN_Tx_Frame[8];

     // yaw and pitch angle
     CAN_Tx_Frame[0] = MCU_AutoAim_Data.yaw_angle[0];
     CAN_Tx_Frame[1] = MCU_AutoAim_Data.yaw_angle[1];
     CAN_Tx_Frame[2] = MCU_AutoAim_Data.yaw_angle[2];
     CAN_Tx_Frame[3] = MCU_AutoAim_Data.yaw_angle[3];
     CAN_Tx_Frame[4] = MCU_AutoAim_Data.pitch_angle[0];
     CAN_Tx_Frame[5] = MCU_AutoAim_Data.pitch_angle[1];
     CAN_Tx_Frame[6] = MCU_AutoAim_Data.pitch_angle[2];
     CAN_Tx_Frame[7] = MCU_AutoAim_Data.pitch_angle[3];

     // //yaw and pitch omega
     // CAN_Tx_Frame[8] = MCU_AutoAim_Data.yaw_omega[0];
     // CAN_Tx_Frame[9] = MCU_AutoAim_Data.yaw_omega[1];
     // CAN_Tx_Frame[10] = MCU_AutoAim_Data.yaw_omega[2];
     // CAN_Tx_Frame[11] = MCU_AutoAim_Data.yaw_omega[3];
     // CAN_Tx_Frame[12] = MCU_AutoAim_Data.pitch_omega[0];
     // CAN_Tx_Frame[13] = MCU_AutoAim_Data.pitch_omega[1];
     // CAN_Tx_Frame[14] = MCU_AutoAim_Data.pitch_omega[2];
     // CAN_Tx_Frame[15] = MCU_AutoAim_Data.pitch_omega[3];

     // // yaw and pitch torque
     // CAN_Tx_Frame[16] = MCU_AutoAim_Data.yaw_torque[0];
     // CAN_Tx_Frame[17] = MCU_AutoAim_Data.yaw_torque[1];
     // CAN_Tx_Frame[18] = MCU_AutoAim_Data.yaw_torque[2];
     // CAN_Tx_Frame[19] = MCU_AutoAim_Data.yaw_torque[3];
     // CAN_Tx_Frame[20] = MCU_AutoAim_Data.pitch_torque[0];
     // CAN_Tx_Frame[21] = MCU_AutoAim_Data.pitch_torque[1];
     // CAN_Tx_Frame[22] = MCU_AutoAim_Data.pitch_torque[2];
     // CAN_Tx_Frame[23] = MCU_AutoAim_Data.pitch_torque[3];

     FDCAN_Send_Data(CAN_Manage_Object->CAN_Handler, AUTOAIM_INFO_ID, CAN_Tx_Frame, 8);
}

void Class_MCU_Comm::CanSendImu()
{
     static uint8_t can_tx_frame[16];
     // 把float转换成字节
     union { float f; uint8_t b[4]; } conv;
     conv.f = g_total_yaw;
     can_tx_frame[0] = conv.b[0];
     can_tx_frame[1] = conv.b[1];
     can_tx_frame[2] = conv.b[2];
     can_tx_frame[3] = conv.b[3];

     conv.f = g_pitch;
     can_tx_frame[4] = conv.b[0];
     can_tx_frame[5] = conv.b[1];
     can_tx_frame[6] = conv.b[2];
     can_tx_frame[7] = conv.b[3];

     conv.f = g_yaw_omega;
     can_tx_frame[8] = conv.b[0];
     can_tx_frame[9] = conv.b[1];
     can_tx_frame[10] = conv.b[2];
     can_tx_frame[11] = conv.b[3];

     conv.f = g_pitch_omega;
     can_tx_frame[12] = conv.b[0];
     can_tx_frame[13] = conv.b[1];
     can_tx_frame[14] = conv.b[2];
     can_tx_frame[15] = conv.b[3];

     FDCAN_Send_Data(CAN_Manage_Object->CAN_Handler, IMU_INFO_ID, can_tx_frame, 16);
}
void Class_MCU_Comm::Data_Process()
{

}

void Class_MCU_Comm::CAN_Gimbal_RxCpltCallback(uint8_t* Rx_Data) {
     //yaw
     // angle 
     memcpy(MCU_Recv_Data.Yaw_Angle, &Rx_Data[0], 4);
     // omega
     memcpy(MCU_Recv_Data.Yaw_Omega, &Rx_Data[4], 4);
     
     // pitch
     // angle 
     memcpy(MCU_Recv_Data.Pitch_Angle, &Rx_Data[8], 4);
     // omega
     memcpy(MCU_Recv_Data.Pitch_Omega, &Rx_Data[12], 4);
}
