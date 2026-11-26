/**
 * @file htdwLegacyMotor.hpp
 * @author L2JF
 * @brief 高擎旧版 ht_can / LivelyBot 寄存器协议电机驱动（标准帧 0x100 回传）
 * @version 0.1
 * @date 2026-11-20
 *
 * @note
 * - 控制帧发 11-bit 标准帧 motor_id（如 0x001，不触发 0 字节空应答），查询帧发 29-bit 扩展帧 0x8000 | motor_id（如 0x8001）
 * - 电机反馈返回 11-bit 标准帧 (motor_id << 8)（如 ID=1 返回 0x100）
 * - 多圈位置查询（2 字节）：{0x19, 0x01} -> 返回 {0x29, 0x01, pos32[4]}（100000 count/圈，±21474.8 圈）
 * - 位置速度力矩混控帧（8 字节）：{0x07, 0x35, vel[2], tqe_limit[2], pos[2]}
 * - 纯力矩控制帧（4 字节）：{0x05, 0x13, tqe[2]}
 * - 停止失能帧（3 字节）：{0x01, 0x00, 0x00}
 * - id 范围 1~7，且同总线有大疆 3508 时千万别设为 2（2 << 8 = 0x200 会撞上 3508 控制帧）；
 * - 灵足电机和高擎电机不能同时位1（否则会串帧）
 * - 改 ID 时总线只能单接这一颗高擎电机
 */

#pragma once

#include <cmath>
#include <cstdint>
#include "CanBus.hpp"
#include "MotorBase.hpp"
#include "math_utils.hpp"

class HtdwLegacyMotor : public CanDevice, public MotorBase {
public:
    enum class CtrlMode : uint8_t {
        QUERY_ONLY  = 0,  ///< 仅按周期发送 {0x19, 0x01} 查询多圈位置，不驱动电机
        STOP        = 1,  ///< 发送 {0x01, 0x00, 0x00} 停止失能（并按分频穿插查询）
        TORQUE      = 2,  ///< 发送 {0x05, 0x13, tqe} 纯力矩模式（并按分频穿插查询）
        POS_VEL_TQE = 3   ///< 发送 {0x07, 0x35, vel, tqe, pos} 位置-速度-力矩模式（并按分频穿插查询）
    };

    //query_every_n: 每发n帧查询一次多圈位置（1ms任务下 5~10 即 5~10ms 查一次）
    HtdwLegacyMotor(CanBus *manager,
                    uint8_t motor_id,
                    float limit_min_deg = -180.0f,
                    float limit_max_deg = 180.0f,
                    float torque_scale = kDefaultTqeScale,
                    uint8_t send_div = 1U,
                    uint8_t query_every_n = 5U)
        : CanDevice(manager,
                    (static_cast<uint32_t>(motor_id & 0x7FU) << 8U),
                    false,
                    static_cast<uint32_t>(motor_id & 0x7FU),
                    false),
          motor_id_(static_cast<uint8_t>(motor_id & 0x7FU)),
          limit_min_deg_(limit_min_deg),
          limit_max_deg_(limit_max_deg),
          torque_scale_(torque_scale > 0.0f ? torque_scale : kDefaultTqeScale),
          send_div_(send_div == 0U ? 1U : send_div),
          query_every_n_(query_every_n < 2U ? 2U : query_every_n),
          frame_cnt_(query_every_n < 2U ? 2U : query_every_n) {
        output_type_ = OutputType::NONE;
        max_cmd_ = 50.0f;
        rx_timeout_ms_ = 50U;
    }

    /** @brief 仅查询位置/速度/力矩反馈，不下发控制 */
    void setQueryOnly() { ctrl_mode_ = CtrlMode::QUERY_ONLY; }

    /** @brief 停止电机（关闭三相输出） */
    void setStop() { ctrl_mode_ = CtrlMode::STOP; }

    /** @brief 纯力矩控制（N·m） */
    void setTorque(float torque_nm) {
        setMotorCmd(torque_nm);
        ctrl_mode_ = CtrlMode::TORQUE;
    }

    /**
     * @brief 位置-速度-最大力矩协同控制
     *
     * @param pos_deg       目标角度 (°)
     * @param speed_deg_s   速度上限 (°/s)，> 0 时按限制运行，<= 0 时不限制
     * @param max_torque_nm 最大力矩限制 (N·m)，> 0 时按限制输出
     */
    void setPosVelTqe(float pos_deg, float speed_deg_s, float max_torque_nm) {
        setMotorDeg(pos_deg);
        setMotorDegSpeed(speed_deg_s);
        setMotorCmd(max_torque_nm);
        ctrl_mode_ = CtrlMode::POS_VEL_TQE;
    }

    uint8_t getMotorID() const { return motor_id_; }
    uint8_t getMotorMode() const { return motor_mode_; }
    uint32_t getTxDlc() const { return tx_dlc_; }
    CanBus::Type getTxType() const {
        return tx_is_extid_ ? CanBus::Type::EXTENDED : CanBus::Type::STANDARD;
    }
    bool hasFeedback() const { return has_feedback_; }

    //这个现在必须是标准帧，因为早期掩码较宽，在与灵足同时跑的时候容易串帧
    bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header) const override {
        const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
        if (frame_is_ext != is_extid_) {
            return false;
        }
        return (rx_header.Identifier & kRxIdMask) == id_;
    }

    void onRx(const uint8_t data[8], const uint8_t len) override {
        if (len >= 6U && data[0] == kRespPos32 && data[1] == kRegPos) {
            const uint32_t raw_u32 =
                static_cast<uint32_t>(data[2]) |
                (static_cast<uint32_t>(data[3]) << 8U) |
                (static_cast<uint32_t>(data[4]) << 16U) |
                (static_cast<uint32_t>(data[5]) << 24U);
            if (raw_u32 != kI32NanBits) {
                const int32_t raw_pos32 = static_cast<int32_t>(raw_u32);
                sum_deg_ = (static_cast<float>(raw_pos32) / kPos32Scale) * kRevToDeg;
                single_deg_ = math_utils::WrapAngle(sum_deg_, -180.0f, 180.0f);
                has_feedback_ = true;
            }
        } else if (len >= 3U && data[0] == kRespMode && data[1] == kRegMode) {
            motor_mode_ = data[2];
        }
    }

    bool buildTx(uint8_t data[8]) override {
        if (++send_cnt_ < send_div_) {
            return false;
        }
        send_cnt_ = 0U;

        if (ctrl_mode_ == CtrlMode::QUERY_ONLY) {
            if (++frame_cnt_ < query_every_n_) {
                return false;
            }
            frame_cnt_ = 0U;
            packQueryFrame(data);
            return true;
        }

        // 控制帧走标准帧 motor_id（无回包），按 query_every_n_ 穿插扩展帧 0x8000|id 查询 32 位多圈位置
        if (++frame_cnt_ >= query_every_n_) {
            frame_cnt_ = 0U;
            packQueryFrame(data);
            return true;
        }

        switch (ctrl_mode_) {
            case CtrlMode::STOP:
                packStopFrame(data);
                return true;
            case CtrlMode::TORQUE:
                packTorqueFrame(data);
                return true;
            case CtrlMode::POS_VEL_TQE:
                packPosVelTqeFrame(data);
                return true;
            case CtrlMode::QUERY_ONLY:
            default:
                packQueryFrame(data);
                return true;
        }
    }

private:
    static constexpr uint32_t kHeaderQuery = 0x8000UL;
    static constexpr uint32_t kRxIdMask = 0x7FFUL;

    static constexpr uint8_t kCmdStop0 = 0x01U;
    static constexpr uint8_t kCmdStop1 = 0x00U;
    static constexpr uint8_t kCmdStop2 = 0x00U;

    static constexpr uint8_t kCmdTorque0 = 0x05U;
    static constexpr uint8_t kCmdTorque1 = 0x13U;

    static constexpr uint8_t kCmdPosVelTqe0 = 0x07U;
    static constexpr uint8_t kCmdPosVelTqe1 = 0x35U;

    static constexpr uint8_t kCmdQuery32 = 0x19U;
    static constexpr uint8_t kRegPos = 0x01U;
    static constexpr uint8_t kRegMode = 0x00U;
    static constexpr uint8_t kRespPos32 = 0x29U;
    static constexpr uint8_t kRespMode = 0x21U;

    static constexpr uint16_t kNoLimitRaw = 0x8000U;
    static constexpr uint32_t kI32NanBits = 0x80000000UL;
    static constexpr float kRevToDeg = 360.0f;
    static constexpr float kPosScale = 10000.0f;
    static constexpr float kPos32Scale = 100000.0f;
    static constexpr float kVelScale = 4000.0f;
    static constexpr float kDefaultTqeScale = 200.0f;

    uint8_t motor_id_{1U};
    float limit_min_deg_{-180.0f};
    float limit_max_deg_{180.0f};
    float torque_scale_{kDefaultTqeScale};
    uint8_t send_div_{1U};
    uint8_t send_cnt_{0U};
    uint8_t query_every_n_{5U};
    uint8_t frame_cnt_{5U};

    CtrlMode ctrl_mode_{CtrlMode::QUERY_ONLY};
    uint8_t motor_mode_{0U};
    bool has_feedback_{false};
    uint32_t tx_dlc_{FDCAN_DLC_BYTES_2};

    static int16_t clampInt16(float value) {
        // 避开协议保留字 0x8000 (-32768) 及 NaN
        if (!std::isfinite(value)) {
            return 0;
        }
        if (value > 32767.0f) {
            return 32767;
        }
        if (value < -32767.0f) {
            return -32767;
        }
        return static_cast<int16_t>(std::lroundf(value));
    }

    static void writeU16(uint8_t *dst, uint16_t val) {
        dst[0] = static_cast<uint8_t>(val & 0xFFU);
        dst[1] = static_cast<uint8_t>((val >> 8U) & 0xFFU);
    }

    void packQueryFrame(uint8_t data[8]) {
        tx_id_ = kHeaderQuery | static_cast<uint32_t>(motor_id_);
        tx_is_extid_ = true;
        data[0] = kCmdQuery32;
        data[1] = kRegPos;
        tx_dlc_ = FDCAN_DLC_BYTES_2;
    }

    void packStopFrame(uint8_t data[8]) {
        tx_id_ = static_cast<uint32_t>(motor_id_);
        tx_is_extid_ = false;
        data[0] = kCmdStop0;
        data[1] = kCmdStop1;
        data[2] = kCmdStop2;
        tx_dlc_ = FDCAN_DLC_BYTES_3;
    }

    void packTorqueFrame(uint8_t data[8]) {
        tx_id_ = static_cast<uint32_t>(motor_id_);
        tx_is_extid_ = false;
        math_utils::Clamp(cmd_, -max_cmd_, max_cmd_);
        const int16_t raw_tqe = clampInt16(cmd_ * torque_scale_);
        data[0] = kCmdTorque0;
        data[1] = kCmdTorque1;
        writeU16(&data[2], static_cast<uint16_t>(raw_tqe));
        tx_dlc_ = FDCAN_DLC_BYTES_4;
    }

    void packPosVelTqeFrame(uint8_t data[8]) {
        tx_id_ = static_cast<uint32_t>(motor_id_);
        tx_is_extid_ = false;
        math_utils::Clamp(ref_deg_, limit_min_deg_, limit_max_deg_);
        math_utils::Clamp(cmd_, 0.0f, max_cmd_);

        const float pos_rev = ref_deg_ / kRevToDeg;
        const int16_t raw_pos = clampInt16(pos_rev * kPosScale);

        uint16_t raw_vel_u16 = kNoLimitRaw;
        if (ref_deg_speed_ > 0.0f) {
            const int16_t raw_vel = clampInt16((ref_deg_speed_ / kRevToDeg) * kVelScale);
            raw_vel_u16 = static_cast<uint16_t>(raw_vel > 0 ? raw_vel : 1);
        }

        int16_t raw_tqe = clampInt16(cmd_ * torque_scale_);
        if (raw_tqe < 0) {
            raw_tqe = 0;
        }

        data[0] = kCmdPosVelTqe0;
        data[1] = kCmdPosVelTqe1;
        writeU16(&data[2], raw_vel_u16);
        writeU16(&data[4], static_cast<uint16_t>(raw_tqe));
        writeU16(&data[6], static_cast<uint16_t>(raw_pos));
        tx_dlc_ = FDCAN_DLC_BYTES_8;
    }
};

