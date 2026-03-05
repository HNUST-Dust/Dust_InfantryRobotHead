#include "debug_tools.h"

#include <cstdint>
#include <cstring>

#include "cmsis_os2.h"

DebugTools& DebugTools::Instance()
{
    static DebugTools inst;
    return inst;
}

void DebugTools::Init(BspUartHandle uart)
{
    if (started_) {
        return;
    }
    if (uart == nullptr) {
        return;
    }

    uart_ = uart;
    started_ = true;
}

void DebugTools::StartThread()
{
    if (!started_ || uart_ == nullptr) {
        return;
    }
    if (thread_ != nullptr) {
        return;
    }

    static const osThreadAttr_t kDebugToolsTaskAttr = {
        .name = "debug_tools",
        .stack_size = 512,
        .priority = (osPriority_t)osPriorityNormal,
    };
    thread_ = osThreadNew(DebugTools::TaskEntry, this, &kDebugToolsTaskAttr);
}

void DebugTools::FeedFloat(float v)
{
    // lock-free: 只允许单调增加计数；超过容量直接丢弃
    uint32_t idx = float_count_.load(std::memory_order_relaxed);
    if (idx >= kMaxFloats) {
        return;
    }
    floats_[idx] = v;
    float_count_.store(idx + 1, std::memory_order_release);
}

void DebugTools::TaskEntry(void* argument)
{
    auto* self = static_cast<DebugTools*>(argument);
    self->Task();
}

void DebugTools::Task()
{
    for (;;) {
        

        osDelay(10); // 100Hz → 每帧 68B，115200bps 约 5.9ms 发完，不积压
    }
}

void DebugTools::VofaSendFloat(float data)
{
    if (!started_ || uart_ == nullptr) {
        return;
    }

    static_assert(sizeof(float) == 4, "VOFA float must be 4 bytes");
    uint8_t buf[4]{};
    std::memcpy(buf, &data, 4);
    (void)bsp_uart_send(uart_, buf, 4);
}

void DebugTools::VofaSendTail()
{
    if (!started_ || uart_ == nullptr) {
        return;
    }

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    (void)bsp_uart_send(uart_, tail, 4);
}

void DebugTools::VofaReceiveCallback(uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
}