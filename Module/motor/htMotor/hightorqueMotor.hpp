/**
 * @file hightorqueMotor.hpp
 * @author YLX
 * @brief 高擎关节模组 8 字节经典 CAN 驱动
 * 帧头: 0x001 标准帧=控制, 0x8001 扩展帧=查询(bit15 要求回包), 0x0100=回包
 * 控制字: 07 07 普通 / 07 35 协同 / 17 01、19 01 读状态 / 01 00 00 停机
 * 量纲: 位置 10000 count/圈, 速度 4000 count/(转/秒), 0x8000 表示"不限"
 *
  */
#pragma once

#include "CanBus.hpp"
#include "MotorBase.hpp"
#include <cstdint>
#include <cstring>

class HightorqueMotor : public CanDevice, public MotorBase {
public:
    enum class CtrlMode : uint8_t { Disable = 0, Speed, Position };

    //构造函数
    HightorqueMotor(CanBus *bus, uint32_t motor_id, float reduction = 1.0f)
        : CanDevice(bus, motor_id, false, motor_id, false) {
        reduction_ratio_ = reduction;   
        max_cmd_ = 2800.0f;             /* 速度模式限幅, 度/秒 */
        setRxTimeoutMs(kFeedbackTimeoutMs);  /* 基类 isOffline() 的超时, 反馈周期 20ms 取 3 倍 */
    }

    void setTargetSpeed(float deg_per_sec) {
        mode_ = CtrlMode::Speed;
        setMotorDegSpeed(deg_per_sec);
    }

    void setTargetPosition(float deg) {
        mode_ = CtrlMode::Position;
        setMotorDeg(deg);
    }

    void setPositionSpeedLimit(float deg_per_sec) {
        pos_speed_limit_ = (deg_per_sec > 0.0f) ? deg_per_sec : kDefaultPosSpeedLimit;
    }

    void setAccelLimit(float deg_per_sec2) {
        accel_limit_ = (deg_per_sec2 > 0.0f) ? deg_per_sec2 : kAccelOff;
    }

    void Disable(void) {
        mode_ = CtrlMode::Disable;
        setMotorCmd(0.0f);
        dis_tick_ = 0U;
    }

    /* 回零是为了校准: 保持 -> 停机 -> 设零点 -> 等待 -> 可选写 flash, 不阻塞任务 */
    void requestRezero(bool save_to_flash = false) {
        if (svc_ != Svc::None) return;      /* 回零途中禁止重入 */
        svc_ = Svc::Hold;
        svc_tick_ = 0U;
        svc_save_ = save_to_flash;
        mode_ = CtrlMode::Disable;
        setMotorCmd(0.0f);
        dis_tick_ = kStopFrameTicks;
    }

    /* ---------------- 每次调用, 须在 buildTx 之前 ---------------- */
    void update(void) {
        if (mode_ == CtrlMode::Disable) {
            setMotorCmd(0.0f);
            last_cmd_ms_ = HAL_GetTick();   
            return;
        }

        const uint32_t now = HAL_GetTick();
        uint32_t dt_ms = (last_cmd_ms_ == 0U) ? 1U : (now - last_cmd_ms_);
        last_cmd_ms_ = now;
        if (dt_ms == 0U) {     
            dt_ms = 1U;
        } else if (dt_ms > 30U) {
            dt_ms = 30U;
        }

        if (mode_ == CtrlMode::Speed) {
            const float target = getRefDegSpeed();
            float cmd = cmd_;
            if (accel_limit_ > 0.0f) {
                const float step = accel_limit_ * (0.001f * static_cast<float>(dt_ms));
                const float diff = target - cmd;
                if (diff > step)        cmd += step;
                else if (diff < -step) cmd -= step;
                else                    cmd = target;
            } else {
                cmd = target;
            }
            setMotorCmd(cmd);
        } else {
            cmd_deg_ = getRefDeg(); 
        }
    }

    /* ---------------- 组帧: 每次最多一帧 ---------------- */
    bool buildTx(uint8_t data[8]) override {
        /* 上次发送失败，就重发*/
        if (tx_retry_) {                    
            tx_retry_ = false;
            std::memcpy(data, tx_last_, 8U);
            return true;
        }
        const bool send = compose(data);
        if (send) std::memcpy(tx_last_, data, 8U);
        return send;
    }

    /* 队列满时由调用方通知, 就重发*/
    void notifyTxFail(void) { tx_retry_ = true; }


    CanBus::Type getFrameType(void) const {
        return tx_is_extid_ ? CanBus::Type::EXTENDED : CanBus::Type::STANDARD;
    }

    /* ---------------- 反馈 ---------------- */
    bool matchRx(const FDCAN_RxHeaderTypeDef &header) const override {
        const uint32_t id = header.Identifier;
        /* 0x01xx 回包: 高字节=源地址(电机 ID), 低字节=目的地址。*/
        if ((id >> 8) == (id_ & 0x7FU)) return true;
        /* 扩展帧其余 ID 一概不收, 只认 0x8001 查询帧回显 */
        if (header.IdType == FDCAN_EXTENDED_ID) return id == readId();
        return id == ctrlId();              /* 标准帧 0x001 同 ID 应答 */
    }

    void onRx(const uint8_t data[8], uint8_t len) override {
        /* 27 01 <pos:2> <vel:2> <tqe:2>: int16 小端, 0x8000 = 无效/不限 */
        if ((len >= 8U) && (data[0] == kRspRead) && (data[1] == kRegStatus)) {
            const int16_t p = getI16(&data[2]);
            const int16_t v = getI16(&data[4]);
            const int16_t t = getI16(&data[6]);
            /* 无效值保持上次读数; 单圈位置写 single_deg_(基类语义) */
            if (p != kI16Nan) single_deg_ = static_cast<float>(p) / kPosCountsPerRev * 360.0f;
            if (v != kI16Nan) deg_speed_  = static_cast<float>(v) / kVelCountsPerRps * 360.0f;
            /* torque_ 是驱动器原始扭矩 count, 不是 N·m */
            if (t != kI16Nan) torque_     = static_cast<float>(t);
            markFeedback();
            return;
        }
        /* 29 01 <pos:int32>: 多圈绝对位置, LSB 0.00001 圈, 写基类累计角度 */
        if ((len >= 6U) && (data[0] == kRspRead32) && (data[1] == kRegStatus)) {
            const int32_t p32 = static_cast<int32_t>(
                static_cast<uint32_t>(data[2])        | (static_cast<uint32_t>(data[3]) << 8)  |
                (static_cast<uint32_t>(data[4]) << 16) | (static_cast<uint32_t>(data[5]) << 24));
            sum_deg_ = static_cast<float>(p32) * kPos32Count2Deg;
            markFeedback();
        }
    }

private:
    enum class Svc : uint8_t { None = 0, Hold, Stop, Zero, Wait, Save };

    /* ---------------- 协议常量 ---------------- */
    static constexpr uint8_t kPad        = 0x00;
    static constexpr uint8_t kCmdNormal = 0x07;
    static constexpr uint8_t kCmdCoop   = 0x35;
    static constexpr uint8_t kCmdStop    = 0x01;
    static constexpr uint8_t kCmdRead    = 0x17;
    static constexpr uint8_t kCmdRead32 = 0x19;
    static constexpr uint8_t kRegStatus  = 0x01;
    static constexpr uint8_t kRspRead   = 0x27;
    static constexpr uint8_t kRspRead32 = 0x29;
    static constexpr int16_t kI16Nan    = static_cast<int16_t>(0x8000);

    static constexpr float kPosCountsPerRev   = 10000.0f;
    static constexpr float kPos32CountsPerRev  = 100000.0f;
    static constexpr float kVelCountsPerRps    = 4000.0f;
    static constexpr float kDeg2Pos            = kPosCountsPerRev / 360.0f;
    static constexpr float kDegS2Vel          = kVelCountsPerRps / 360.0f;
    static constexpr float kPos32Count2Deg   = 360.0f / kPos32CountsPerRev;

    static constexpr float kDefaultPosSpeedLimit = 120.0f;  /* deg/s, 非正参数兜底 */
    static constexpr float kAccelOff             = 0.0f;    /* 关闭斜坡 */

    /* 每拍 1ms: 读帧 20 拍(20ms), 离线超时 60ms = 3 倍余量 */
    static constexpr uint16_t kReadPeriodTicks   = 20U;
    static constexpr uint32_t kFeedbackTimeoutMs = 60U;
    static constexpr uint16_t kStopFrameTicks  = 10U;   /* 停机帧连发, 防单帧丢失 */
    static constexpr uint16_t kSvcHoldTicks    = 200U;  /* 零速保持, 等机构停稳 */
    static constexpr uint16_t kSvcStopTicks    = 20U;   /* 回零必须在停机态 */
    static constexpr uint16_t kSvcZeroTicks    = 10U;   /* 设零帧重复确保送达 */
    static constexpr uint16_t kSvcWaitTicks    = 200U;  /* 等电机内部落盘 */
    static constexpr uint16_t kSvcSaveTicks    = 10U;  /* 写 flash 重复确保送达 */

    /* 官方 ht_can 服务帧, 8 字节原样下发(尾部补 0) */
    static constexpr uint8_t kFrameHold[8]      = {0x07,0x07,0x00,0x80,0x00,0x00,0x00,0x80};
    static constexpr uint8_t kFrameStop[8]      = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    static constexpr uint8_t kFrameSetZero[8]   = {0x40,0x01,0x04,0x64,0x20,0x63,0x0A,0x00};
    static constexpr uint8_t kFrameSaveFlash[8] = {0x05,0xB3,0x02,0x00,0x00,0x00,0x00,0x00};

    /* ---------------- 帧头 ID ---------------- */
    uint32_t ctrlId(void) const { return id_; }                        /* 标准 0x001 */
    uint32_t readId(void) const { return 0x8000U | (id_ & 0x7FU); }   /* 扩展 0x8001 */

    /* CanBus 对任何 matchRx 帧都会 markRx(), 基类 isOffline() 会被无关帧刷成"假在线"。
       这里只认有效回帧; 且上电未收到过反馈时不判离线(反馈链路没接通也能正常控制)。 */
    bool linkLost(void) const {
        return fb_seen_ && ((HAL_GetTick() - last_fb_ms_) > kFeedbackTimeoutMs);
    }
    void markFeedback(void) {
        last_fb_ms_ = HAL_GetTick();
        fb_seen_    = true;
    }

    /* 只在确定发帧时写帧头, 返回 false 时 tx_id_/tx_is_extid_ 保持上一帧 */
    bool compose(uint8_t data[8]) {
        std::memset(data, kPad, 8U);

        ++read_phase_;
        const bool probe_due = (read_phase_ >= kReadPeriodTicks);
        if (probe_due) read_phase_ = 0U;

        uint32_t id = ctrlId();      /* 默认标准控制帧 0x001 */
        bool ext = false;
        bool send = true;

        if (svc_ != Svc::None) {
            send = buildService(data, id, ext);
        } else if (mode_ == CtrlMode::Disable) {
            if (dis_tick_ < kStopFrameTicks) {
                ++dis_tick_;
                data[0] = kCmdStop;    /* 01 00 00 停机 */
            } else {
                send = false;
            }
        } else if (probe_due) {
            buildRead(data, id, ext);        /* 读帧兼作链路探测, 掉线时也照发 */
        } else if (linkLost()) {
            send = false;                   /* 曾在线又掉线: 只暂停控制指令, 等回包恢复 */
        } else if (mode_ == CtrlMode::Position) {
            buildPosition(data);
        } else {
            buildSpeed(data);
        }

        if (!send) return false;
        tx_id_ = id;
        tx_is_extid_ = ext;
        return true;
    }

    void buildRead(uint8_t data[8], uint32_t &id, bool &ext) {
        ++read_alt_;
        data[0] = (read_alt_ & 1U) ? kCmdRead : kCmdRead32;   /* 17 01 / 19 01 交替 */
        data[1] = kRegStatus;
        id  = readId();
        ext = true;
    }

    void buildPosition(uint8_t data[8]) {
        /* 07 35 <限速> <扭矩不限> <目标位置>, 协同模式 */
        data[0] = kCmdNormal;
        data[1] = kCmdCoop;
        putI16(&data[2], toI16(pos_speed_limit_ * kDegS2Vel));
        putI16(&data[4], kI16Nan);
        putI16(&data[6], toI16(cmd_deg_ * kDeg2Pos));
    }

    void buildSpeed(uint8_t data[8]) {
        /* 07 07 <位置不限> <速度> <扭矩不限> */
        data[0] = kCmdNormal;
        data[1] = kCmdNormal;
        putI16(&data[2], kI16Nan);
        putI16(&data[4], toI16(cmd_ * kDegS2Vel));
        putI16(&data[6], kI16Nan);
    }

    bool buildService(uint8_t data[8], uint32_t &id, bool &ext) {
        ++svc_tick_;
        id  = readId();      /* 服务帧默认扩展 0x8001 */
        ext = true;

        switch (svc_) {
            case Svc::Hold:               /* 零速保持, 先让电机停稳 */
                id  = ctrlId();
                ext = false;
                std::memcpy(data, kFrameHold, 8U);
                if (svc_tick_ >= kSvcHoldTicks) { svc_ = Svc::Stop; svc_tick_ = 0U; }
                return true;

            case Svc::Stop:               /* 停机态才允许设零点 */
                std::memcpy(data, kFrameStop, 8U);
                if (svc_tick_ >= kSvcStopTicks) { svc_ = Svc::Zero; svc_tick_ = 0U; }
                return true;

            case Svc::Zero:               /* 设当前位置为零点(只改 RAM) */
                std::memcpy(data, kFrameSetZero, 8U);
                if (svc_tick_ >= kSvcZeroTicks) { svc_ = Svc::Wait; svc_tick_ = 0U; }
                return true;

            case Svc::Wait:               /* 等电机内部落盘, 期间不发帧 */
                if (svc_tick_ < kSvcWaitTicks) return false;
                svc_ = svc_save_ ? Svc::Save : Svc::None;
                svc_tick_ = 0U;
                return false;

            case Svc::Save:               /* 写 flash, 掉电不丢 */
                std::memcpy(data, kFrameSaveFlash, 8U);
                if (svc_tick_ >= kSvcSaveTicks) svc_ = Svc::None;
                return true;

            default:
                svc_ = Svc::None;
                return false;
        }
    }

    static int16_t toI16(float v) {
        /* 饱和到 ±32767, 必须避开 0x8000(协议中表示"不限"); NaN 按 0 处理 */
        if (v >= 32767.0f)  return 32767;
        if (v <= -32767.0f) return -32767;
        if (v != v)         return 0;
        return static_cast<int16_t>(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
    }
    static void putI16(uint8_t *p, int16_t v) {
        const uint16_t u = static_cast<uint16_t>(v);   
        p[0] = static_cast<uint8_t>(u & 0xFFU);
        p[1] = static_cast<uint8_t>((u >> 8U) & 0xFFU);
    }
    static int16_t getI16(const uint8_t *p) {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                    (static_cast<uint16_t>(p[1]) << 8));
    }

    /* ---------------- 状态 ---------------- */
    CtrlMode mode_{CtrlMode::Disable};
    float cmd_deg_{0.0f};
    float accel_limit_{1800.0f};
    float pos_speed_limit_{240.0f};

    Svc svc_{Svc::None};
    uint16_t svc_tick_{0U};
    bool svc_save_{false};

    uint8_t  read_alt_{0U};
    uint16_t read_phase_{0U};
    uint32_t last_cmd_ms_{0U};
    uint32_t dis_tick_{kStopFrameTicks};

    uint32_t last_fb_ms_{0U};
    bool fb_seen_{false};

    uint8_t tx_last_[8]{};
    bool tx_retry_{false};
};
