#include "bsp_usb.h"
#include "pc_comm.h"
#include "cmsis_os2.h"

uint8_t g_recived_flag;

void Class_PC_Comm::Init()
{

}

void Class_PC_Comm::Send_Message()
{
    uint8_t buffer[43];
    memcpy(buffer, &PC_Send_Data, sizeof(PCSendAutoAimData));
    USB_Transmit(buffer,43);
}

void Class_PC_Comm::RxCpltCallback()
{
    if (PC_Recv_Data.head[0] == 'S' && PC_Recv_Data.head[1] == 'P'){
        g_recived_flag = 1;
        memcpy(&PC_Recv_Data,bsp_usb_rx_buffer,29 * sizeof(uint8_t));
    }
}
