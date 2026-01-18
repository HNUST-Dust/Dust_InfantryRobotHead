#pragma once
#include <cstdint>
#define MOUSE_SENSITIVITY_X (30.0f)
#define MOUSE_SENSITIVITY_Y (-8.0f)

// 键位宏定义
#define VT02_KEY_W 0
#define VT02_KEY_S 1
#define VT02_KEY_A 2
#define VT02_KEY_D 3
#define VT02_KEY_SHIFT 4
#define VT02_KEY_CTRL 5
#define VT02_KEY_Q 6
#define VT02_KEY_E 7
#define VT02_KEY_R 8
#define VT02_KEY_F 9
#define VT02_KEY_G 10
#define VT02_KEY_Z 11
#define VT02_KEY_X 12
#define VT02_KEY_C 13
#define VT02_KEY_V 14
#define VT02_KEY_B 15


/**
 * ^ ch3       ^ ch1
 * |           |
 * + ——> ch2   + ——> ch0
 * 
 *      2           2            -----> +
 * sw1: 3      sw2: 3     wheel: |
 *      1           1            L----<
 */
class VT02
{
public:
    enum SwitchStatus {
        UP   = (uint8_t)2,
        MID  = (uint8_t)3,
        DOWN = (uint8_t)1,
    };
    enum KeyStatus {
        FREE   = (uint8_t)0,
        TRIG_FREE_PRESSED,
        TRIG_PRESSED_FREE,
        PRESSED = (uint8_t)1,
    };
    struct Key {
        KeyStatus key_status;
        uint8_t count;
        uint8_t current_status;
    };

    struct RecivedRawData {
        struct{
            uint8_t start_of_frame;
            uint16_t data_length;
            uint8_t seq;
            uint8_t crc_8;
        } frame_header;

        uint16_t cmd_id;
        /* mouse movement and button information */
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
    
            uint8_t l;
            uint8_t r;
        } mouse;
        /* keyboard key information */
        union
        {
            uint16_t key_code;
            struct
            {
                uint16_t W : 1;
                uint16_t S : 1;
                uint16_t A : 1;
                uint16_t D : 1;
                uint16_t SHIFT : 1;
                uint16_t CTRL : 1;
                uint16_t Q : 1;
                uint16_t E : 1;
                uint16_t R : 1;
                uint16_t F : 1;
                uint16_t G : 1;
                uint16_t Z : 1;
                uint16_t X : 1;
                uint16_t C : 1;
                uint16_t V : 1;
                uint16_t B : 1;
            } bit;
        } keyboard;
        int16_t pulley_wheel;
        uint16_t crc16;
    } __attribute__((packed));

    /**
     * ^ y
     * |
     * + ——> x
     */
    struct RecivedProcessedData {
        struct
        {
            float x;
            float y;
            float z;
    
            uint8_t l;
            uint8_t r;
        } mouse;
        // union
        // {
        //     uint16_t key_code;
        //     struct
        //     {
        //         uint16_t W : 1;
        //         uint16_t S : 1;
        //         uint16_t A : 1;
        //         uint16_t D : 1;
        //         uint16_t SHIFT : 1;
        //         uint16_t CTRL : 1;
        //         uint16_t Q : 1;
        //         uint16_t E : 1;
        //         uint16_t R : 1;
        //         uint16_t F : 1;
        //         uint16_t G : 1;
        //         uint16_t Z : 1;
        //         uint16_t X : 1;
        //         uint16_t C : 1;
        //         uint16_t V : 1;
        //         uint16_t B : 1;
        //     } bit;
        // } keyboard;
        Key keyboard[16];
        float wheel;
    };

    void Init();
    void RxCpltCallback(uint8_t *rx_data, uint16_t length);
    inline RecivedProcessedData* GetData() {
        return &recived_processed_data_;
    }

private:
    void judge_key(Key *key,uint8_t status, uint8_t pre_status);
    RecivedRawData recived_raw_data_;
    RecivedRawData pre_recived_raw_data_;
    RecivedProcessedData recived_processed_data_ = {
        {0,0,0,0,0},
        {FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE,FREE},0
    };
};
