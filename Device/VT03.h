#ifndef VT03_H
#define VT03_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_usart.h"

/* Exported macros -----------------------------------------------------------*/

// 拨动开关位置
#define VT03_SWITCH_UP (1)
#define VT03_SWITCH_DOWN (2)
#define VT03_SWITCH_MIDDLE (3)

// 按键开关位置
#define VT03_KEY_FREE (0)
#define VT03_KEY_PRESSED (1)

// 键位宏定义
#define VT03_KEY_W 0
#define VT03_KEY_S 1
#define VT03_KEY_A 2
#define VT03_KEY_D 3
#define VT03_KEY_SHIFT 4
#define VT03_KEY_CTRL 5
#define VT03_KEY_Q 6
#define VT03_KEY_E 7
#define VT03_KEY_R 8
#define VT03_KEY_F 9
#define VT03_KEY_G 10
#define VT03_KEY_Z 11
#define VT03_KEY_X 12
#define VT03_KEY_C 13
#define VT03_KEY_V 14
#define VT03_KEY_B 15

/* Exported types ------------------------------------------------------------*/

/**
 * @brief VT03源数据
 *
 */
struct Struct_Remote_Data
{
    uint8_t Start_Of_Frame_1;
    uint8_t Start_Of_Frame_2;
    uint64_t Channel_0 : 11;
    uint64_t Channel_1 : 11;
    uint64_t Channel_2 : 11;
    uint64_t Channel_3 : 11;
    uint64_t Mode_Switch : 2;
    uint64_t Pause : 1;
    uint64_t Fn_1 : 1; // 左侧开关
    uint64_t Fn_2 : 1; // 右侧开关
    uint64_t Wheel : 11;
    uint64_t Trigger : 1;
    uint64_t reserved_1 : 3;
    int16_t Mouse_X;
    int16_t Mouse_Y;
    int16_t Mouse_Z;
    uint8_t Mouse_Left_Key : 2;
    uint8_t Mouse_Right_Key : 2;
    uint8_t Mouse_Middle_Key : 2;
    uint8_t reserved_2 : 2;
    uint16_t Key;
    uint16_t CRC16;
} __attribute__((packed));
/**
 * @brief 遥控器VT03状态
 *
 */
enum Enum_VT03_Status
{
    VT03_Status_DISABLE = 0,
    VT03_Status_ENABLE,
};

/**
 * @brief 拨动开关状态
 *
 */
enum Enum_VT03_Switch_Status
{
    VT03_STATUS_LEFT = 0,
    VT03_STATUS_MIDDLE = 1,
    VT03_STATUS_RIGHT = 2,
};

/**
 * @brief 按键状态
 *
 */
enum Enum_VT03_Key_Status
{
    VT03_Key_Status_FREE = 0,
    VT03_Key_Status_TRIG_FREE_PRESSED,
    VT03_Key_Status_TRIG_PRESSED_FREE,
    VT03_Key_Status_PRESSED = 1,
};

/**
 * @brief VT03经过处理的的数据, 摇杆信息经过归一化到-1~1, 鼠标信息有待进一步标定
 *
 */
struct Struct_VT03_Data
{
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;
    Enum_VT03_Switch_Status Mode_Switch;
    float Mouse_X;
    float Mouse_Y;
    float Mouse_Z;
    Enum_VT03_Key_Status Left_Key;
    Enum_VT03_Key_Status Right_Key;
    Enum_VT03_Key_Status Middle_Key;
    Enum_VT03_Key_Status Pause;
    Enum_VT03_Key_Status Trigger;
    Enum_VT03_Key_Status Mouse_Left_Key;
    Enum_VT03_Key_Status Mouse_Right_Key;
    Enum_VT03_Key_Status Mouse_Middle_Key;
    Enum_VT03_Key_Status Keyboard_Key[16];
    float Wheel;
};

/**
 * @brief Specialized, 遥控器VT03
 *
 */
class Class_VT03
{
public:
    // 遥控器VT03对外接口信息
    Struct_VT03_Data Data = {
        0.0f,
        0.0f,
        0.5f,
        0.5f,
        VT03_STATUS_MIDDLE,
        0.0f,
        0.0f,
        0.0f,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        VT03_Key_Status_FREE,
        127,
    };

    void Init(UART_HandleTypeDef *huart);

    inline Enum_VT03_Status Get_Status();

    inline float Get_Right_X();

    inline float Get_Right_Y();

    inline float Get_Left_X();

    inline float Get_Left_Y();

    inline Enum_VT03_Switch_Status Get_Mode_Switch();

    inline Enum_VT03_Key_Status Get_Left_Key();

    inline Enum_VT03_Key_Status Get_Right_Key();

    inline Enum_VT03_Key_Status Get_Trigger();

    inline float Get_Mouse_X();

    inline float Get_Mouse_Y();

    inline float Get_Mouse_Z();

    inline Enum_VT03_Key_Status Get_Mouse_Left_Key();

    inline Enum_VT03_Key_Status Get_Mouse_Right_Key();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_W();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_S();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_A();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_D();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_Shift();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_Ctrl();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_Q();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_E();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_R();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_F();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_G();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_Z();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_X();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_C();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_V();

    inline Enum_VT03_Key_Status Get_Keyboard_Key_B();

    inline float Get_Wheel();

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    void TIM_1ms_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量
    uint16_t CRC16_Init = 0xffff;
    // 绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;

    // 常量

    // 摇杆偏移量
    float Rocker_Offset = 364.0f;
    // 摇杆总刻度
    float Rocker_Num = 1320.0f;

    // 内部变量
    // 前一时刻的遥控器VT03状态信息
    Struct_Remote_Data Pre_UART_Rx_Data;
    // 当前时刻的遥控器VT03接收flag
    uint32_t Flag = 0;
    // 前一时刻的遥控器VT03接收flag
    uint32_t Pre_Flag = 0;

    // 读变量

    // 遥控器VT03状态
    Enum_VT03_Status VT03_Status = VT03_Status_DISABLE;


    // 写变量

    // 读写变量

    // 内部函数

    uint16_t Get_CRC16_Check_Sum(uint8_t *P_Msg, uint16_t Length, uint16_t CRC16);

    bool Verify_CRC16_Check_Sum(uint8_t *P_Msg, uint16_t Length);

    void Data_Process(uint16_t Length);

    void _Judge_Switch(Enum_VT03_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);

    void _Judge_Key(Enum_VT03_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取遥控器VT03右侧x轴摇杆状态
 *
 * @return float 遥控器VT03右侧x轴摇杆状态
 */
inline float Class_VT03::Get_Right_X()
{
    return (Data.Right_X);
}

/**
 * @brief 获取遥控器VT03右侧y轴摇杆状态
 *
 * @return float 遥控器VT03右侧y轴摇杆状态
 */
inline float Class_VT03::Get_Right_Y()
{
    return (Data.Right_Y);
}

/**
 * @brief 获取遥控器VT03左侧x轴摇杆状态
 *
 * @return float 遥控器VT03左侧x轴摇杆状态
 */
inline float Class_VT03::Get_Left_X()
{
    return (Data.Left_X);
}

/**
 * @brief 获取遥控器VT03左侧y轴摇杆状态
 *
 * @return float 遥控器VT03左侧y轴摇杆状态
 */
inline float Class_VT03::Get_Left_Y()
{
    return (Data.Left_Y);
}

/**
 * @brief 获取遥控器VT03模式开关状态
 *
 * @return Enum_VT03_Switch_Status 遥控器VT03拨动开关状态
 */
inline Enum_VT03_Switch_Status Class_VT03::Get_Mode_Switch()
{
    return (Data.Mode_Switch);
}

/**
 * @brief 获取遥控器左侧开关状态
 *
 * @return Enum_VT03_Switch_Status 遥控器左侧自定义开关状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Left_Key()
{
    return (Data.Left_Key);
}

/**
 * @brief 获取遥控器右侧开关状态
 *
 * @return Enum_VT03_Switch_Status 遥控器右侧自定义开关状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Right_Key()
{
    return (Data.Right_Key);
}

/**
 * @brief 获取遥控器扳机键状态
 *
 * @return Enum_VT03_Switch_Status 遥控器扳机键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Trigger()
{
    return (Data.Trigger);
}
/**
 * @brief 获取鼠标x轴状态
 *
 * @return float 鼠标x轴状态
 */
inline float Class_VT03::Get_Mouse_X()
{
    return (Data.Mouse_X);
}

/**
 * @brief 获取鼠标y轴状态
 *
 * @return float 鼠标y轴状态
 */
inline float Class_VT03::Get_Mouse_Y()
{
    return (Data.Mouse_Y);
}

/**
 * @brief 获取鼠标z轴状态
 *
 * @return float 鼠标z轴状态
 */
inline float Class_VT03::Get_Mouse_Z()
{
    return (Data.Mouse_Z);
}

/**
 * @brief 获取鼠标左键状态
 *
 * @return Enum_VT03_Key_Status 鼠标左键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Mouse_Left_Key()
{
    return (Data.Mouse_Left_Key);
}

/**
 * @brief 获取鼠标右键状态
 *
 * @return Enum_VT03_Key_Status 鼠标右键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Mouse_Right_Key()
{
    return (Data.Mouse_Right_Key);
}

/**
 * @brief 获取键盘W键状态
 *
 * @return Enum_VT03_Key_Status 键盘W键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_W()
{
    return (Data.Keyboard_Key[VT03_KEY_W]);
}

/**
 * @brief 获取键盘S键状态
 *
 * @return Enum_VT03_Key_Status 键盘S键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_S()
{
    return (Data.Keyboard_Key[VT03_KEY_S]);
}

/**
 * @brief 获取键盘A键状态
 *
 * @return Enum_VT03_Key_Status 键盘A键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_A()
{
    return (Data.Keyboard_Key[VT03_KEY_A]);
}

/**
 * @brief 获取键盘D键状态
 *
 * @return Enum_VT03_Key_Status 键盘D键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_D()
{
    return (Data.Keyboard_Key[VT03_KEY_D]);
}

/**
 * @brief 获取键盘Shift键状态
 *
 * @return Enum_VT03_Key_Status 键盘Shift键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_Shift()
{
    return (Data.Keyboard_Key[VT03_KEY_SHIFT]);
}

/**
 * @brief 获取键盘Ctrl键状态
 *
 * @return Enum_VT03_Key_Status 键盘Ctrl键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_Ctrl()
{
    return (Data.Keyboard_Key[VT03_KEY_CTRL]);
}

/**
 * @brief 获取键盘Q键状态
 *
 * @return Enum_VT03_Key_Status 键盘Q键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_Q()
{
    return (Data.Keyboard_Key[VT03_KEY_Q]);
}

/**
 * @brief 获取键盘E键状态
 *
 * @return Enum_VT03_Key_Status 键盘E键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_E()
{
    return (Data.Keyboard_Key[VT03_KEY_E]);
}

/**
 * @brief 获取键盘R键状态
 *
 * @return Enum_VT03_Key_Status 键盘R键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_R()
{
    return (Data.Keyboard_Key[VT03_KEY_R]);
}

/**
 * @brief 获取键盘F键状态
 *
 * @return Enum_VT03_Key_Status 键盘F键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_F()
{
    return (Data.Keyboard_Key[VT03_KEY_F]);
}

/**
 * @brief 获取键盘G键状态
 *
 * @return Enum_VT03_Key_Status 键盘G键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_G()
{
    return (Data.Keyboard_Key[VT03_KEY_G]);
}

/**
 * @brief 获取键盘Z键状态
 *
 * @return Enum_VT03_Key_Status 键盘Z键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_Z()
{
    return (Data.Keyboard_Key[VT03_KEY_Z]);
}

/**
 * @brief 获取键盘X键状态
 *
 * @return Enum_VT03_Key_Status 键盘X键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_X()
{
    return (Data.Keyboard_Key[VT03_KEY_X]);
}

/**
 * @brief 获取键盘C键状态
 *
 * @return Enum_VT03_Key_Status 键盘C键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_C()
{
    return (Data.Keyboard_Key[VT03_KEY_C]);
}

/**
 * @brief 获取键盘V键状态
 *
 * @return Enum_VT03_Key_Status 键盘V键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_V()
{
    return (Data.Keyboard_Key[VT03_KEY_V]);
}

/**
 * @brief 获取键盘B键状态
 *
 * @return Enum_VT03_Key_Status 键盘B键状态
 */
inline Enum_VT03_Key_Status Class_VT03::Get_Keyboard_Key_B()
{
    return (Data.Keyboard_Key[VT03_KEY_B]);
}

/**
 * @brief 获取遥控器VT03拨轮状态
 *
 * @return float 遥控器VT03yaw轴状态
 */
inline float Class_VT03::Get_Wheel()
{
    return (Data.Wheel);
}

#endif

/************************ COPYRIGHT(C) HNUST-DUST **************************/
