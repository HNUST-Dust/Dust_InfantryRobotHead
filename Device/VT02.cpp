#include "VT02.h"
#include "stdlib.h"
#include "string.h"
#include "CRC.h"
#include "string.h"

void VT02::Init()
{

}

void VT02::judge_key(Key *key, uint8_t status, uint8_t pre_status)
{
    if (status == 1 && pre_status == 0)
    {
        key->key_status = TRIG_FREE_PRESSED;
    }
    else if (status == 0 && pre_status == 1)
    {
        key->key_status = TRIG_PRESSED_FREE;
    }
    else if (status == 1 && pre_status == 1)
    {
        key->key_status = PRESSED;
    }
    else
    {
        key->key_status = FREE;
    }
}
// #define KEY_CONFIRM_CNT  3
// void VT02::judge_key(Key *key,
//     uint8_t status,
//     uint8_t pre_status)
// {
//     (void)pre_status;   // 不再依赖 pre_status

//     /* 触发态自动回落到稳定态 */
//     if (key->key_status == TRIG_FREE_PRESSED) {
//         key->key_status = PRESSED;
//     } else if (key->key_status == TRIG_PRESSED_FREE) {
//         key->key_status = FREE;
//     }

//     /* ---------- FREE → PRESSED ---------- */
//     if (key->key_status == FREE) {

//         if (status == PRESSED) {
//             if (key->count < KEY_CONFIRM_CNT) {
//                 key->count++;
//             }

//             if (key->count >= KEY_CONFIRM_CNT) {
//                 key->key_status = TRIG_FREE_PRESSED;
//                 key->count = 0;
//             }
//         } else {
//             key->count = 0;
//         }
//     }
//     /* ---------- PRESSED → FREE ---------- */
//     else if (key->key_status == PRESSED) {

//         if (status == FREE) {
//             if (key->count < KEY_CONFIRM_CNT) {
//                 key->count++;
//             }

//             if (key->count >= KEY_CONFIRM_CNT) {
//                 key->key_status = TRIG_PRESSED_FREE;
//                 key->count = 0;
//             }
//         } else {
//             key->count = 0;
//         }
//     }
// }

void VT02::RxCpltCallback(uint8_t *rx_data, uint16_t length)
{
    if (verify_crc16_check_sum(rx_data, (uint32_t)length))
    {
        // 解析通道
        recived_raw_data_.mouse.x = rx_data[8] | (rx_data[7] << 8); // x axis
        recived_raw_data_.mouse.y = rx_data[10] | (rx_data[9] << 8);
        recived_raw_data_.mouse.z = rx_data[12] | (rx_data[11] << 8);
    
        recived_raw_data_.mouse.l = rx_data[13];
        recived_raw_data_.mouse.r = rx_data[14];
    
        recived_raw_data_.keyboard.key_code = rx_data[15] | rx_data[16] << 8; // key borad code
    
        // 鼠标处理
        recived_processed_data_.mouse.x = recived_raw_data_.mouse.x / 32767.0f * 1.0f;
        recived_processed_data_.mouse.y -= recived_raw_data_.mouse.y / 32767.0f * 0.2f;
        recived_processed_data_.mouse.z = recived_raw_data_.mouse.z / 32767.0f;
        // 鼠标限制
        if(recived_processed_data_.mouse.y > 1.0f){
            recived_processed_data_.mouse.y = 1.0f;
        }else if(recived_processed_data_.mouse.y < -1.0f){
            recived_processed_data_.mouse.y = -1.0f;
        }
    
        // 按键 （透传
        recived_processed_data_.mouse.l = recived_raw_data_.mouse.l;
        recived_processed_data_.mouse.r = recived_raw_data_.mouse.r;
        recived_processed_data_.keyboard[VT02_KEY_W].current_status = recived_raw_data_.keyboard.bit.W;
        recived_processed_data_.keyboard[VT02_KEY_S].current_status = recived_raw_data_.keyboard.bit.S;
        recived_processed_data_.keyboard[VT02_KEY_A].current_status = recived_raw_data_.keyboard.bit.A;
        recived_processed_data_.keyboard[VT02_KEY_D].current_status = recived_raw_data_.keyboard.bit.D;
        recived_processed_data_.keyboard[VT02_KEY_R].current_status = recived_raw_data_.keyboard.bit.R;
        recived_processed_data_.keyboard[VT02_KEY_E].current_status = recived_raw_data_.keyboard.bit.E;
        recived_processed_data_.keyboard[VT02_KEY_F].current_status = recived_raw_data_.keyboard.bit.F;
        recived_processed_data_.keyboard[VT02_KEY_SHIFT].current_status = recived_raw_data_.keyboard.bit.SHIFT;

        judge_key(&recived_processed_data_.keyboard[VT02_KEY_F], recived_raw_data_.keyboard.bit.F, pre_recived_raw_data_.keyboard.bit.F);
        
        memcpy(&pre_recived_raw_data_, &recived_raw_data_, sizeof(RecivedRawData));
    }
}

