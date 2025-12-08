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

void Class_MCU_Comm::CAN_Send_Command()
{
     static uint8_t CAN_Tx_Frame[8];
     CAN_Tx_Frame[0] = MCU_Comm_Data.Start_Of_Frame;
     CAN_Tx_Frame[1] = MCU_Comm_Data.Yaw_Angle;
     CAN_Tx_Frame[2] = MCU_Comm_Data.Pitch_Angle;
     CAN_Tx_Frame[3] = MCU_Comm_Data.Chassis_Speed_X;
     CAN_Tx_Frame[4] = MCU_Comm_Data.Chassis_Speed_Y;
     CAN_Tx_Frame[5] = MCU_Comm_Data.Chassis_Rotation;
     CAN_Tx_Frame[6] = MCU_Comm_Data.Chassis_Spin;
     CAN_Tx_Frame[7] = MCU_Comm_Data.Supercap;

     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);
}

void Class_MCU_Comm::CAN_Send_AutoAim()
{
     static uint8_t CAN_Tx_Frame[8];

     // yaw angle
     CAN_Tx_Frame[0] = MCU_AutoAim_Data.SOF1;
     CAN_Tx_Frame[1] = MCU_AutoAim_Data.yaw_angle[0];
     CAN_Tx_Frame[2] = MCU_AutoAim_Data.yaw_angle[1];
     CAN_Tx_Frame[3] = MCU_AutoAim_Data.yaw_angle[2];
     CAN_Tx_Frame[4] = MCU_AutoAim_Data.yaw_angle[3];
     CAN_Tx_Frame[5] = 0x00;
     CAN_Tx_Frame[6] = 0x00;
     CAN_Tx_Frame[7] = 0x00;

     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);

     //yaw omega
     CAN_Tx_Frame[0] = MCU_AutoAim_Data.SOF2;
     CAN_Tx_Frame[1] = MCU_AutoAim_Data.yaw_omega[0];
     CAN_Tx_Frame[2] = MCU_AutoAim_Data.yaw_omega[1];
     CAN_Tx_Frame[3] = MCU_AutoAim_Data.yaw_omega[2];
     CAN_Tx_Frame[4] = MCU_AutoAim_Data.yaw_omega[3];
     CAN_Tx_Frame[5] = 0x00;
     CAN_Tx_Frame[6] = 0x00;
     CAN_Tx_Frame[7] = 0x00;

     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);

     // yaw torque
     CAN_Tx_Frame[0] = MCU_AutoAim_Data.SOF2;
     CAN_Tx_Frame[1] = MCU_AutoAim_Data.yaw_torque[0];
     CAN_Tx_Frame[2] = MCU_AutoAim_Data.yaw_torque[1];
     CAN_Tx_Frame[3] = MCU_AutoAim_Data.yaw_torque[2];
     CAN_Tx_Frame[4] = MCU_AutoAim_Data.yaw_torque[3];
     CAN_Tx_Frame[5] = 0x00;
     CAN_Tx_Frame[6] = 0x00;
     CAN_Tx_Frame[7] = 0x00;

     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);
   
     // pitch angle
     CAN_Tx_Frame[0] = MCU_AutoAim_Data.SOF4;
     CAN_Tx_Frame[1] = MCU_AutoAim_Data.pitch_angle[0];
     CAN_Tx_Frame[2] = MCU_AutoAim_Data.pitch_angle[1];
     CAN_Tx_Frame[3] = MCU_AutoAim_Data.pitch_angle[2];
     CAN_Tx_Frame[4] = MCU_AutoAim_Data.pitch_angle[3];
     CAN_Tx_Frame[5] = 0x00;
     CAN_Tx_Frame[6] = 0x00;
     CAN_Tx_Frame[7] = 0x00;

     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);

     // pitch omega
     CAN_Tx_Frame[0] = MCU_AutoAim_Data.SOF5;
     CAN_Tx_Frame[1] = MCU_AutoAim_Data.pitch_omega[0];
     CAN_Tx_Frame[2] = MCU_AutoAim_Data.pitch_omega[1];
     CAN_Tx_Frame[3] = MCU_AutoAim_Data.pitch_omega[2];
     CAN_Tx_Frame[4] = MCU_AutoAim_Data.pitch_omega[3];
     CAN_Tx_Frame[5] = 0x00;
     CAN_Tx_Frame[6] = 0x00;
     CAN_Tx_Frame[7] = 0x00;

     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);

     // pitch torque
     CAN_Tx_Frame[0] = MCU_AutoAim_Data.SOF6;
     CAN_Tx_Frame[1] = MCU_AutoAim_Data.pitch_torque[0];
     CAN_Tx_Frame[2] = MCU_AutoAim_Data.pitch_torque[1];
     CAN_Tx_Frame[3] = MCU_AutoAim_Data.pitch_torque[2];
     CAN_Tx_Frame[4] = MCU_AutoAim_Data.pitch_torque[3];
     CAN_Tx_Frame[5] = 0x00;
     CAN_Tx_Frame[6] = 0x00;
     CAN_Tx_Frame[7] = 0x00;

     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);
   
}

void Class_MCU_Comm::CanSendImu()
{
     static uint8_t can_tx_frame[8];
     can_tx_frame[0] = mcu_imu_data_.start_of_yaw_frame;
     // 把float转换成字节
     union { float f; uint8_t b[4]; } conv;
     // conv.f = INS.YawTotalAngle;
     conv.f = g_yaw;
     can_tx_frame[1] = conv.b[0];
     can_tx_frame[2] = conv.b[1];
     can_tx_frame[3] = conv.b[2];
     can_tx_frame[4] = conv.b[3];
     can_tx_frame[5] = 0x00;
     can_tx_frame[6] = 0x00;
     can_tx_frame[7] = 0x00;
     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, can_tx_frame, 8);

     can_tx_frame[0] = mcu_imu_data_.start_of_pitch_frame;
     // conv.f = INS.Pitch;
     conv.f = g_pitch;
     can_tx_frame[1] = conv.b[0];
     can_tx_frame[2] = conv.b[1];
     can_tx_frame[3] = conv.b[2];
     can_tx_frame[4] = conv.b[3];
     can_tx_frame[5] = 0x00;
     can_tx_frame[6] = 0x00;
     can_tx_frame[7] = 0x00; 
     CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, can_tx_frame, 8);
}
void Class_MCU_Comm::Data_Process()
{

}

void Class_MCU_Comm::CAN_RxCpltCallback(uint8_t* Rx_Data)
{
     // 判断是哪一帧
     switch (Rx_Data[0])
     {
          case 0x0A: //第一帧
               memcpy(MCU_Recv_Data.Yaw_Angle, &Rx_Data[1], 4 * sizeof(uint8_t));
               break;
          case 0x0B: //第二帧
               memcpy(MCU_Recv_Data.Yaw_Omega, &Rx_Data[1], 4 * sizeof(uint8_t));
               break;
          case 0x0C: //第三帧
               memcpy(MCU_Recv_Data.Pitch_Angle, &Rx_Data[1], 4 * sizeof(uint8_t));
               break;
          case 0x0D: //第四帧
               memcpy(MCU_Recv_Data.Pitch_Omega, &Rx_Data[1], 4 * sizeof(uint8_t));
               break;
          default:
               break;
     }
}
