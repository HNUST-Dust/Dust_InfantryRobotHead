#include "dr16.h"
#include "stdlib.h"
#include "string.h"
void DR16::Init()
{
    recived_raw_data_ = {
        0,
        0,
        0, 
        0,
        0, 
        0, 
        0,
        0,
        0
    };

}
void DR16::judge_key(KeyStatus *key, uint8_t status, uint8_t pre_status)
{
    if (status == 1 && pre_status == 0)
    {
        *key = TRIG_FREE_PRESSED;
    }
    else if (status == 0 && pre_status == 1)
    {
        *key = TRIG_PRESSED_FREE;
    }
    else if (status == 1 && pre_status == 1)
    {
        *key = PRESSED;
    }
    else
    {
        *key = FREE;
    }
}

void DR16::RxCpltCallback(uint8_t *rx_data, uint16_t length)
{
    RecivedRawData *tmp_buffer = (RecivedRawData *) rx_data;
    // 解析通道
    recived_raw_data_.channel0 = (rx_data[0] | rx_data[1] << 8) & 0x07FF;
    recived_raw_data_.channel0 -= 1024;
    recived_raw_data_.channel1 = (rx_data[1] >> 3 | rx_data[2] << 5) & 0x07FF;
    recived_raw_data_.channel1 -= 1024;
    recived_raw_data_.channel2 = (rx_data[2] >> 6 | rx_data[3] << 2 | rx_data[4] << 10) & 0x07FF;
    recived_raw_data_.channel2 -= 1024;
    recived_raw_data_.channel3 = (rx_data[4] >> 1 | rx_data[5] << 7) & 0x07FF;
    recived_raw_data_.channel3 -= 1024;

    recived_raw_data_.switch1 = ((rx_data[5] >> 4) & 0x000C) >> 2;
    recived_raw_data_.switch2 = (rx_data[5] >> 4) & 0x0003;

    recived_raw_data_.mouse.x = rx_data[6] | (rx_data[7] << 8); // x axis
    recived_raw_data_.mouse.y = rx_data[8] | (rx_data[9] << 8);
    recived_raw_data_.mouse.z = rx_data[10] | (rx_data[11] << 8);

    recived_raw_data_.mouse.l = rx_data[12];
    recived_raw_data_.mouse.r = rx_data[13];

    recived_raw_data_.keyboard.key_code = rx_data[14] | rx_data[15] << 8; // key borad code
    recived_raw_data_.pulley_wheel = -(rx_data[16] | rx_data[17] << 8) - 1024;

    // 数据异常处理
    if ((abs(recived_raw_data_.channel0) > 660) || \
    (abs(recived_raw_data_.channel1) > 660) || \
    (abs(recived_raw_data_.channel2) > 660) || \
    (abs(recived_raw_data_.channel3) > 660))
    {
        //memset(&recived_raw_data_, 0, sizeof(DR16::RecivedRawData));
    }

    // 归一化处理
    recived_processed_data_.right_stick_x = recived_raw_data_.channel0 / 660.0f;
    recived_processed_data_.right_stick_y = recived_raw_data_.channel1 / 660.0f;
    recived_processed_data_.left_stick_x  = recived_raw_data_.channel2 / 660.0f;
    recived_processed_data_.left_stick_y  = recived_raw_data_.channel3 / 660.0f;
    recived_processed_data_.wheel = recived_raw_data_.pulley_wheel / 660.0f;
    
    recived_processed_data_.mouse.x = recived_raw_data_.mouse.x / 32767.0f * MOUSE_SENSITIVITY_X;
    recived_processed_data_.mouse.y = recived_raw_data_.mouse.y / 32767.0f * MOUSE_SENSITIVITY_Y;
    recived_processed_data_.mouse.z = recived_raw_data_.mouse.z / 32767.0f;
    // 鼠标Y轴限制
    if(recived_processed_data_.mouse.y > 0.5f){
        recived_processed_data_.mouse.y = 0.5f;
    }else if(recived_processed_data_.mouse.y < -0.5f){
        recived_processed_data_.mouse.y = -0.5f;
    }

    recived_processed_data_.left_switch  = recived_raw_data_.switch1;
    recived_processed_data_.right_switch = recived_raw_data_.switch2;

    // 按键 （透传
    recived_processed_data_.mouse.l = recived_raw_data_.mouse.l;
    recived_processed_data_.mouse.r = recived_raw_data_.mouse.r;
    recived_processed_data_.keyboard.key_code = recived_raw_data_.keyboard.key_code;


}

