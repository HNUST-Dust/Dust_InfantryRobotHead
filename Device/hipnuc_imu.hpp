#pragma once
#include "bsp_usart.h"
#include "CRC.h"

#include <cstring>

class HipnucIMU {
public:
    void Init() {
        total_yaw_angle_rad_ = 0.0f;
        yaw_angle_rad_ = 0.0f;
        pitch_angle_rad_ = 0.0f;
        roll_angle_rad_ = 0.0f;
        yaw_omega_rad_ = 0.0f;
        pitch_omega_rad_ = 0.0f;
        roll_omega_rad_ = 0.0f;
        total_yaw_angle_deg_ = 0.0f;
        yaw_angle_deg_ = 0.0f;
        pitch_angle_deg_ = 0.0f;
        roll_angle_deg_ = 0.0f;
        yaw_omega_deg_ = 0.0f;
        pitch_omega_deg_ = 0.0f;

        q_[0] = 1.0f; // w
        q_[1] = 0.0f; // x 
        q_[2] = 0.0f; // y
        q_[3] = 0.0f; // z

        has_last_yaw_deg_ = false;
        last_yaw_deg_ = 0.0f;
    }
    void RxCpltCallback(uint8_t *buffer, uint16_t length) {
        if (buffer == nullptr || length < kMinFrameLen) {
            return;
        }

        // 允许一次回调里包含多个帧，做简单同步扫描。
        uint16_t i = 0;
        while (i + kMinFrameLen <= length) {
            // SOF: 0x5A 0xA5
            if (buffer[i] != kSof0 || buffer[i + 1] != kSof1) {
                ++i;
                continue;
            }

            const uint16_t data_len = ReadLE<uint16_t>(buffer + i + 2);
            const uint32_t frame_len = static_cast<uint32_t>(data_len) + kOverheadLen;
            if (data_len < 1 || data_len > 4096) {
                ++i;
                continue;
            }
            if (i + frame_len > length) {
                // 当前 buffer 不够完整一帧，等待下一次回调
                return;
            }

            uint8_t *frame = buffer + i;

            // 兼容两种常见帧布局：
            // A) SOF(2) LEN(2) CRC16(2) DATA(LEN)   —— 与你图中字段顺序一致
            // B) SOF(2) LEN(2) DATA(LEN) CRC16(2)   —— 许多工程常用
            // 通过 tag(0x91) 与 CRC 校验自动判定 payload 起点。
            const uint8_t *payload_a = frame + 6; // layout A
            const uint8_t *payload_b = frame + 4; // layout B
            const bool a_tag_ok = (data_len >= 1) && (payload_a[0] == kHi91Tag);
            const bool b_tag_ok = (data_len >= 1) && (payload_b[0] == kHi91Tag);

            bool parsed = false;
            // 暂时去掉 CRC 校验：仅根据 tag(0x91) 判断 payload 起点
            if (a_tag_ok) {
                if (data_len >= kHi91PayloadLen) {
                    ParseHI91(payload_a, data_len);
                }
                parsed = true;
            } else if (b_tag_ok) {
                if (data_len >= kHi91PayloadLen) {
                    ParseHI91(payload_b, data_len);
                }
                parsed = true;
            }

            // 同步策略：解析成功跳过整帧；否则逐字节滑动继续找 SOF
            if (parsed) {
                i = static_cast<uint16_t>(i + frame_len);
            } else {
                ++i;
            }
        }
    }

    float total_yaw_angle_rad_;
    float yaw_angle_rad_;
    float pitch_angle_rad_;
    float roll_angle_rad_;
    float yaw_omega_rad_;
    float pitch_omega_rad_;
    float roll_omega_rad_;

    float total_yaw_angle_deg_;
    float yaw_angle_deg_;
    float pitch_angle_deg_;
    float roll_angle_deg_;
    float yaw_omega_deg_;
    float pitch_omega_deg_;
    float q_[4];

private:
    static constexpr uint8_t kSof0 = 0x5A;
    static constexpr uint8_t kSof1 = 0xA5;
    static constexpr uint16_t kMinFrameLen = 6;      // SOF(2) + LEN(2) + CRC16(2)
    static constexpr uint16_t kOverheadLen = 6;      // SOF(2) + LEN(2) + CRC16(2)
    static constexpr uint8_t kHi91Tag = 0x91;
    static constexpr uint16_t kHi91PayloadLen = 76;
    static constexpr float kDeg2Rad = 0.01745329251994329577f;

    bool has_last_yaw_deg_ = false;
    float last_yaw_deg_ = 0.0f;

    template <typename T>
    static T ReadLE(const uint8_t *p) {
        T v{};
        std::memcpy(&v, p, sizeof(T));
        return v;
    }

    static bool VerifyCrcBeforeData(const uint8_t *frame, uint16_t data_len) {
        if (frame == nullptr) {
            return false;
        }
        // layout A: [0..1]=SOF, [2..3]=LEN(le), [4..5]=CRC16(le), [6..]=DATA
        const uint16_t expected = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5]) << 8);
        uint16_t crc = 0xFFFF;
        crc = get_crc16_check_sum(const_cast<uint8_t *>(frame), 4, crc);
        crc = get_crc16_check_sum(const_cast<uint8_t *>(frame + 6), data_len, crc);
        return crc == expected;
    }

    void ParseHI91(const uint8_t *payload, uint16_t payload_len) {
        if (payload == nullptr || payload_len < kHi91PayloadLen) {
            return;
        }

        // 字节偏移与协议表一致（小端）
        // 0: tag
        // 1: main_status (uint16)
        // 3: temperature (int8)
        // 4: air_pressure (float)
        // 8: system_time (uint32)
        // 12: acc_b (float[3])
        // 24: gyr_b (float[3])  deg/s
        // 36: mag_b (float[3])
        // 48: roll (float) deg
        // 52: pitch (float) deg
        // 56: yaw (float) deg
        // 60: quat (float[4]) WXYZ

        const float gyr_x_deg = -ReadLE<float>(payload + 24);
        const float gyr_y_deg = ReadLE<float>(payload + 28);
        const float gyr_z_deg = ReadLE<float>(payload + 32);

        const float roll_deg = ReadLE<float>(payload + 48);
        const float pitch_deg = -ReadLE<float>(payload + 52);
        const float yaw_deg = ReadLE<float>(payload + 56);

        const float qw = ReadLE<float>(payload + 60);
        const float qx = ReadLE<float>(payload + 64);
        const float qy = ReadLE<float>(payload + 68);
        const float qz = ReadLE<float>(payload + 72);

        roll_angle_deg_ = roll_deg;
        pitch_angle_deg_ = pitch_deg;
        yaw_angle_deg_ = yaw_deg;

        // gyro: X/Y/Z 近似对应 roll/pitch/yaw 角速度
        pitch_omega_deg_ = gyr_y_deg;
        yaw_omega_deg_ = gyr_z_deg;

        roll_angle_rad_ = roll_angle_deg_ * kDeg2Rad;
        pitch_angle_rad_ = pitch_angle_deg_ * kDeg2Rad;
        yaw_angle_rad_ = yaw_angle_deg_ * kDeg2Rad;

        roll_omega_rad_ = gyr_x_deg;
        pitch_omega_rad_ = pitch_omega_deg_;
        yaw_omega_rad_ = yaw_omega_deg_;

        // yaw 解包络，得到累计航向角
        if (!has_last_yaw_deg_) {
            has_last_yaw_deg_ = true;
            last_yaw_deg_ = yaw_angle_deg_;
            total_yaw_angle_deg_ = yaw_angle_deg_;
        } else {
            float delta = yaw_angle_deg_ - last_yaw_deg_;
            if (delta > 180.0f) {
                delta -= 360.0f;
            } else if (delta < -180.0f) {
                delta += 360.0f;
            }
            total_yaw_angle_deg_ += delta;
            last_yaw_deg_ = yaw_angle_deg_;
        }
        total_yaw_angle_rad_ = total_yaw_angle_deg_ * kDeg2Rad;

        q_[0] = qw;
        q_[1] = qx;
        q_[2] = qy;
        q_[3] = qz;
    }
};
