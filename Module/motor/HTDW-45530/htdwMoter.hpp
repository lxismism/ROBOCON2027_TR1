/**
 * @file htdwMoter.hpp
 * @author L2JF
 * @brief HTDW-45530电机类
 * @version 0.1
 * @date 2026-11-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include "CanBus.hpp"
#include "MotorBase.hpp"
#include "math_utils.hpp"

/**
 * @note
 * - MIT 发送 CAN ID: 0x58000 | motor_id（8 字节位域打包，不含模式码/查询码）
 * - 系统命令 CAN ID: 0x18000 | motor_id（用于上电/掉线后配置 0x0E 周期返回）
 * - 电机返回 CAN ID: 0x10000 | (motor_id << 8)
 * - 协议底层物理单位为 圈 / 圈/s / N·m；本类对外接口与 MotorBase 保持一致（度 / 度/s / N·m）
 */
class HtdwMotor : public CanDevice, public MotorBase {
public:
    HtdwMotor(CanBus *manager,
              uint8_t motor_id,
              float kp = 0.0f,
              float kd = 0.0f,
              float max_cmd = 100.0f,
              float limit_min_deg = kPosMinDeg,
              float limit_max_deg = kPosMaxDeg,
              uint32_t feedback_period_us = 1000U,
              uint8_t send_div = 2U)
        : CanDevice(manager,
                    kHeaderRx | (static_cast<uint32_t>(motor_id & 0x7FU) << 8U),
                    true,
                    kHeaderMit | static_cast<uint32_t>(motor_id & 0x7FU),
                    true),
          motor_id_(static_cast<uint8_t>(motor_id & 0x7FU)),
          limit_min_deg_(limit_min_deg),
          limit_max_deg_(limit_max_deg),
          feedback_period_us_(feedback_period_us),
          send_div_(send_div == 0U ? 1U : send_div) {
        max_cmd_ = max_cmd;

        math_utils::Clamp(kp, 0.0f, static_cast<float>(kKpMax));
        math_utils::Clamp(kd, 0.0f, static_cast<float>(kKdMax));
        kp_ = kp;
        kd_ = kd;

        if (feedback_period_us_ >= kMinPeriodicReturnUs) {
            const uint32_t period_ms = (feedback_period_us_ + 999U) / 1000U;
            const uint32_t adaptive_timeout = period_ms * 5U;
            rx_timeout_ms_ = (adaptive_timeout > 20U) ? adaptive_timeout : 20U;
        }
    }

    /**
     * @brief 设置 MIT 运控目标参数（内部自动转换为协议要求的 圈 / 圈/s）
     *
     * @param pos_deg     目标角度 (°)
     * @param kp          位置刚度 [0, 800]
     * @param kd          速度阻尼 [0, 200]（位置控制时建议 > 0）
     * @param torque      前馈力矩 (N·m)
     * @param speed_deg_s 目标角速度 (°/s)
     */
    void setMIT(float pos_deg,
                float kp,
                float kd,
                float torque = 0.0f,
                float speed_deg_s = 0.0f) {
        setDeg(pos_deg);
        setDegSpeed(speed_deg_s);
        setCmd(torque);
        math_utils::Clamp(kp, 0.0f, static_cast<float>(kKpMax));
        math_utils::Clamp(kd, 0.0f, static_cast<float>(kKdMax));
        kp_ = kp;
        kd_ = kd;
    }

    uint8_t getMotorID() const { return motor_id_; }
    uint8_t getFaultState() const { return fault_state_; }
    bool isPeriodicFeedbackReady() const { return periodic_ready_; }

    bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header) const override {
        const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
        if (frame_is_ext != is_extid_) {
            return false;
        }
        return (rx_header.Identifier & kRxIdMask) == id_;
    }

    void onRx(const uint8_t data[8], const uint8_t len) override {
        if (len < 2U) {
            return;
        }

        if (data[0] == kQueryStdState) {
            if (len < 8U) {
                return;
            }
            fault_state_ = data[1];

            const int16_t raw_pos = static_cast<int16_t>(
                static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8U));
            const int16_t raw_vel = static_cast<int16_t>(
                static_cast<uint16_t>(data[4]) | (static_cast<uint16_t>(data[5]) << 8U));
            const int16_t raw_tqe = static_cast<int16_t>(
                static_cast<uint16_t>(data[6]) | (static_cast<uint16_t>(data[7]) << 8U));

            const float pos_rev = static_cast<float>(raw_pos) / kRxPosScale;
            const float vel_rev_s = static_cast<float>(raw_vel) / kRxVelScale;

            sum_deg_ = pos_rev * kRevToDeg;
            single_deg_ = math_utils::WrapAngle(sum_deg_, -180.0f, 180.0f);
            deg_speed_ = vel_rev_s * kRevToDeg;
            torque_ = static_cast<float>(raw_tqe) / kRxTqeScale;

            periodic_ready_ = true;
        } else if (data[0] == kCmdSystem) {
            if (data[1] == 0x00U) {
                periodic_ready_ = true;
            }
        }
    }

    /**
     * @brief 组装发送帧：调用方需先调用 buildTx()，返回 true 后再读取 getTxID()（29-bit 扩展帧，8 字节）
     */
    bool buildTx(uint8_t data[8]) override {
        if (++send_cnt_ < send_div_) {
            return false;
        }
        send_cnt_ = 0U;

        const bool offline = isOffline();
        if (offline) {
            periodic_ready_ = false;
        }

        if ((feedback_period_us_ >= kMinPeriodicReturnUs) && (offline || !periodic_ready_)) {
            const uint32_t now = HAL_GetTick();
            if ((now - last_cfg_ms_) < kConfigIntervalMs) {
                return false;
            }
            last_cfg_ms_ = now;
            packPeriodicReturnCmd(data, feedback_period_us_);
            return true;
        }

        packMitCmd(data);
        return true;
    }

private:
    static constexpr uint32_t kHeaderInt16 = 0x18000UL;
    static constexpr uint32_t kHeaderMit = 0x58000UL;
    static constexpr uint32_t kHeaderRx = 0x10000UL;
    static constexpr uint32_t kRxIdMask = 0x1FFFFF80UL;

    static constexpr uint8_t kCmdSystem = 0x03U;
    static constexpr uint8_t kQueryStdState = 0x0EU;

    static constexpr float kRevToDeg = 360.0f;

    // MIT 发送物理量程（协议固定，采用 double 避免 -3.2768f 在 float32 下的精度损失导致零点偏移）
    static constexpr double kPosMinRev = -3.2768;
    static constexpr double kPosMaxRev = 3.2767;
    static constexpr float kPosMinDeg = static_cast<float>(kPosMinRev) * kRevToDeg;
    static constexpr float kPosMaxDeg = static_cast<float>(kPosMaxRev) * kRevToDeg;
    static constexpr double kVelMinRevS = -10.0;
    static constexpr double kVelMaxRevS = 10.0;
    static constexpr double kTqeMinNm = -100.0;
    static constexpr double kTqeMaxNm = 100.0;
    static constexpr double kKpMin = -800.0;
    static constexpr double kKpMax = 800.0;
    static constexpr double kKdMin = -200.0;
    static constexpr double kKdMax = 200.0;

    // 0x0E 返回帧 int16 换算系数
    static constexpr float kRxPosScale = 10000.0f;
    static constexpr float kRxVelScale = 4000.0f;
    static constexpr float kRxTqeScale = 100.0f;

    static constexpr uint32_t kMinPeriodicReturnUs = 100U;
    static constexpr uint32_t kConfigIntervalMs = 20U;

    uint8_t motor_id_{1U};
    float kp_{0.0f};
    float kd_{0.0f};
    float limit_min_deg_{kPosMinDeg};
    float limit_max_deg_{kPosMaxDeg};
    uint32_t feedback_period_us_{1000U};
    uint8_t send_div_{2U};
    uint8_t send_cnt_{0U};

    uint8_t fault_state_{0U};
    bool periodic_ready_{false};
    uint32_t last_cfg_ms_{0U};

    static uint16_t floatToUint(double x, double x_min, double x_max, uint8_t bits) {
        if (x < x_min) {
            x = x_min;
        } else if (x > x_max) {
            x = x_max;
        }
        const double span = x_max - x_min;
        if (span <= 0.0) {
            return 0U;
        }
        const uint32_t max_int = (1UL << bits) - 1UL;
        double u = (x - x_min) * static_cast<double>(max_int) / span;
        if (u < 0.0) {
            u = 0.0;
        } else if (u > static_cast<double>(max_int)) {
            u = static_cast<double>(max_int);
        }
        return static_cast<uint16_t>(u);
    }

    void packPeriodicReturnCmd(uint8_t data[8], uint32_t period_us) {
        data[0] = kCmdSystem;
        data[1] = 0x00U;
        data[2] = 0x05U;
        data[3] = kQueryStdState;
        data[4] = static_cast<uint8_t>(period_us & 0xFFU);
        data[5] = static_cast<uint8_t>((period_us >> 8U) & 0xFFU);
        data[6] = static_cast<uint8_t>((period_us >> 16U) & 0xFFU);
        data[7] = static_cast<uint8_t>((period_us >> 24U) & 0xFFU);

        tx_id_ = kHeaderInt16 | static_cast<uint32_t>(motor_id_);
    }

    void packMitCmd(uint8_t data[8]) {
        math_utils::Clamp(ref_deg_, limit_min_deg_, limit_max_deg_);

        float kp = kp_;
        float kd = kd_;
        math_utils::Clamp(kp, 0.0f, static_cast<float>(kKpMax));
        math_utils::Clamp(kd, 0.0f, static_cast<float>(kKdMax));

        const double pos_rev = static_cast<double>(ref_deg_) / static_cast<double>(kRevToDeg);
        const double vel_rev_s = static_cast<double>(ref_deg_speed_) / static_cast<double>(kRevToDeg);

        const uint16_t raw_pos = floatToUint(pos_rev, kPosMinRev, kPosMaxRev, 16U);
        const uint16_t raw_vel = floatToUint(vel_rev_s, kVelMinRevS, kVelMaxRevS, 12U);
        const uint16_t raw_tqe = floatToUint(static_cast<double>(cmd_), kTqeMinNm, kTqeMaxNm, 12U);
        const uint16_t raw_kp = floatToUint(static_cast<double>(kp), kKpMin, kKpMax, 12U);
        const uint16_t raw_kd = floatToUint(static_cast<double>(kd), kKdMin, kKdMax, 12U);

        data[0] = static_cast<uint8_t>(raw_pos & 0xFFU);
        data[1] = static_cast<uint8_t>((raw_pos >> 8U) & 0xFFU);
        data[2] = static_cast<uint8_t>(raw_vel & 0xFFU);
        data[3] = static_cast<uint8_t>(((raw_vel >> 8U) & 0x0FU) | ((raw_tqe & 0x0FU) << 4U));
        data[4] = static_cast<uint8_t>((raw_tqe >> 4U) & 0xFFU);
        data[5] = static_cast<uint8_t>(raw_kp & 0xFFU);
        data[6] = static_cast<uint8_t>(((raw_kp >> 8U) & 0x0FU) | ((raw_kd & 0x0FU) << 4U));
        data[7] = static_cast<uint8_t>((raw_kd >> 4U) & 0xFFU);

        tx_id_ = kHeaderMit | static_cast<uint32_t>(motor_id_);
    }
};
