#ifndef DEVICE_DR16_H
#define DEVICE_DR16_H

#include <cstdint>

/**
 * ^ ch3       ^ ch1
 * |           |
 * + ——> ch2   + ——> ch0
 * 
 *      2           2            -----> +
 * sw1: 3      sw2: 3     wheel: |
 *      1           1            L----<
 */
class DR16
{
public:
    enum SwitchStatus {
        UP   = (uint8_t)2,
        MID  = (uint8_t)3,
        DOWN = (uint8_t)1,
    };

    struct RecivedRawData {
        int16_t channel0;
        int16_t channel1;
        int16_t channel2;
        int16_t channel3;
        uint8_t switch1;
        uint8_t switch2;
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
    } __attribute__((packed));

    /**
     * ^ y
     * |
     * + ——> x
     */
    struct RecivedProcessedData {
        float right_stick_x;
        float right_stick_y;
        float left_stick_x;
        float left_stick_y;
        uint8_t left_switch;
        uint8_t right_switch;
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
    
            uint8_t l;
            uint8_t r;
        } mouse;
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
        } keyboard;
        float wheel;
    };

    void Init();
    void RxCpltCallback(uint8_t *rx_data, uint16_t length);
    inline RecivedProcessedData* GetData() {
        return &recived_processed_data_;
    }

private:
    RecivedRawData recive_raw_data_;
    RecivedProcessedData recived_processed_data_;
};

#endif
