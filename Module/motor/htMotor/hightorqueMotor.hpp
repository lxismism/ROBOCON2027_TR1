/**
 * @file hightorqueMotor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 高擎（Hightorque）关节模组 —— 8 字节经典 CAN 驱动（官方 ht_can 协议）
 * @version 0.3
 * @date 2026-11-11
 *
 * @copyright Copyright (c) 2026
 *
 * ================= 位置模式为什么「一直在转」（v0.3 定位） =================
 * 现象：发 p50，串口里控制帧已经是对的 07 07 83 06 00 80 00 80（pos=1667=60°），
 *       但电机一直朝负方向转，反馈 position 恒等于 0x7FFF（32767 = 3.2767 圈）。
 *
 * 根因：**int16 位置量程只有 ±3.2767 圈**（LSB 0.0001 圈）。
 *       之前的测速跑了几十秒 240 度/秒，输出轴已经卷出去几十圈，早就超出量程，
 *       反馈被限幅钉死在 0x7FFF；目标 60° 在「几十圈之外」，所以电机以驱动器
 *       自己的最高速（实测约 420 度/秒）一路往回赶 —— 看上去就是「一直在转」，
 *       而且反馈值一动不动，看不出还剩多远。
 *
 * 解决办法（v0.3）：
 *   1. 位置模式改发协同模式 07 35 <限速> <扭矩> <目标位置>，用明确的限速值
 *      而不是 0x8000（NaN=不限速），避免驱动器用最高速狂飙；
 *   2. 增加 int32 绝对位置读取（19 01 → 29 01），能看到超出 ±3.27 圈的真实
 *      位置，据此算出「还差多少度」，转不转得完一眼可见；
 *   3. 增加回零指令：串口 z → 停机 → 以当前位置为 0 点（40 01 04 64 20 63 0A）；
 *      串口 w → 把零点写进 flash（05 B3 02 00 00）。回零后 p50 立刻可用。
 *
 * ================= 官方 ht_can 协议要点（8 字节经典帧） =================
 * 帧 ID：16 位 —— bit15 = 1 表示要求对方回包，bit14..8 源地址，bit7..0 目的地址
 *     控制帧：0x0001（标准帧）
 *     读状态：0x8001（扩展帧，必须带 bit15 电机才回包）
 *     回包帧：0x0100（高字节 = 电机 ID，低字节 = 目的地址 0，标准帧）
 * 控制字：
 *     07 07 <pos:2> <val:2> <tqe:2>   正常模式（位置或速度二选一，另一项填 0x8000）
 *     07 35 <val:2> <tqe:2> <pos:2>   协同模式（以 val 速度走到 pos，限 tqe 扭矩）
 *     05 13 <tqe:2>                   纯扭矩
 *     17 01                           读 3×int16（位置/速度/扭矩）
 *     19 01                           读 1×int32（绝对位置，LSB 0.00001 圈）
 *     01 00 00                        停机
 *     40 01 04 64 20 63 0A            以当前位置为 0 点（只改 RAM）
 *     05 B3 02 00 00                  保存配置到 flash
 * 量纲：
 *     int16 位置：LSB 0.0001 圈   → 1 圈 = 10000，量程 ±3.2767 圈（±32767 count）
 *     int32 位置：LSB 0.00001 圈  → 1 圈 = 100000，量程 ±21474 圈
 *     int16 速度：LSB 0.00025 转/秒 → 1 转/秒 = 4000，量程 ±8.19 转/秒
 *     int16 扭矩：与型号相关，0x8000 表示不限
 */

#pragma once

#include "Canbus.hpp"
#include "MotorBase.hpp"
#include <cstdint>

class HightorqueMotor : public CanDevice, public MotorBase {
public:
    enum class CtrlMode : uint8_t {
        Disable  = 0,
        Speed    = 1,   /* 07 07 00 80 <vel> <tqe>，闭环由电机内部做 */
        Position = 2,   /* 07 35 <vel限速> <tqe> <pos>，绝对位置，闭环由电机内部做 */
    };

    /* ------------------------- 协议常量 ------------------------- */
    static constexpr uint8_t  kPad        = 0x00;   /* 未用字节填 0 */
    static constexpr uint8_t  kCmdNormal  = 0x07;   /* 07 07 正常模式 / 07 35 协同模式 */
    static constexpr uint8_t  kCmdCoop    = 0x35;   /* 07 35 协同模式 */
    static constexpr uint8_t  kCmdStop    = 0x01;   /* 01 00 00 停机 */
    static constexpr uint8_t  kCmdRead    = 0x17;   /* 17 01 读 3×int16 */
    static constexpr uint8_t  kCmdRead32  = 0x19;   /* 19 01 读 1×int32（位置） */
    static constexpr uint8_t  kRegStatus  = 0x01;   /* 位置/速度/扭矩 起始寄存器 */
    static constexpr uint8_t  kRspRead    = 0x27;   /* 回 27 01 */
    static constexpr uint8_t  kRspRead32  = 0x29;   /* 回 29 01 */
    static constexpr int16_t  kI16Nan     = static_cast<int16_t>(0x8000);  /* 不限制 */

    /* 量纲：counts per revolution / counts per revolution-per-second */
    static constexpr float kPosCountsPerRev  = 10000.0f;   /* int16：0.0001 圈/count */
    static constexpr float kPos32CountsPerRev = 100000.0f; /* int32：0.00001 圈/count */
    static constexpr float kVelCountsPerRps  = 4000.0f;    /* 0.00025 转/秒/count */
    static constexpr float kDeg2Pos          = kPosCountsPerRev / 360.0f;   /* 度 → int16 count */
    static constexpr float kDegS2Vel         = kVelCountsPerRps / 360.0f;   /* 度/秒 → count */
    static constexpr float kPos32Count2Deg   = 360.0f / kPos32CountsPerRev; /* int32 count → 度 */

    /* 位置命令字的两种写法：
     *   true  → 07 35 <限速> <扭矩> <位置>：协同模式，**用 pos_speed_limit_ 限速**，
     *           位置指令带一个明确的行走速度，不会像「不限速」那样一路狂奔。
     *   false → 07 07 <位置> 00 80 00 80：官方示例的写法（val 填 0x8000 = 不限速），
     *           万一某个固件不认 07 35，把它改成 false 即可回退。 */
    static constexpr bool kPosUseCoop = true;

    /* ------------------------- 帧调度（can3SendTask 5ms 调用） ------------------------- */
    static constexpr uint16_t kReadPeriodTicks = 20;   /* 每 20 拍读一次状态（约 100ms） */
    static constexpr uint16_t kStopFrameTicks  = 10;   /* 失能时补发 10 帧停机 */
    static constexpr uint16_t kSvcHoldTicks    = 200;  /* 回零前先「零速保持」1s，把电机刹停 */
    static constexpr uint16_t kSvcStopTicks    = 20;   /* 再停机 100ms（回零必须在停机状态） */
    static constexpr uint16_t kSvcZeroTicks    = 10;   /* 回零帧重复 50ms */
    static constexpr uint16_t kSvcWaitTicks    = 200;  /* 官方建议回零后等 1s 再写 flash */
    static constexpr uint16_t kSvcSaveTicks    = 10;   /* 保存帧重复 50ms */

    /* 控制帧用哪种 ID：false → 标准帧 0x001（本车实测可行，和达妙/大疆不冲突） */
    static constexpr bool kCtrlUseExtended = false;

    HightorqueMotor(CanBus *bus, uint32_t motor_id, float reduction = 1.0f)
        : CanDevice(bus, motor_id, false, motor_id, false)
    {
        (void)reduction;
        output_type_ = OutputType::SPEED;
        max_cmd_     = 2800.0f;              /* int16 速度量程 ±8.19 转/秒 ≈ ±2949 度/秒 */
        mode_        = CtrlMode::Disable;    /* 默认不转，必须显式给目标 */
    }

    void init(void) {}

    /* ------------------------- 对外接口 ------------------------- */
    /* 目标输出轴角速度，单位 度/秒（正负均可）。不依赖反馈，上电即可运行 */
    void setTargetSpeed(float deg_per_sec) {
        mode_       = CtrlMode::Speed;
        ref_deg_s_  = deg_per_sec;
    }

    /* 目标位置：绝对角度（度，相对编码器零点）。p0 是零点，p50 是零点 +50 度。
       位置环在电机内部闭合，主控只负责把目标角度发下去。
       注意：int16 位置量程 ±3.2767 圈（±1179 度），超范围会被裁剪；
       输出轴被卷到量程之外时必须先回零（requestRezero）。 */
    void setTargetPosition(float deg) {
        mode_     = CtrlMode::Position;
        ref_deg_  = deg;
    }

    /* 位置模式行走速度上限，度/秒（协同模式的 val）。<=0 时按 120 度/秒 */
    void setPositionSpeedLimit(float deg_per_sec) {
        pos_speed_limit_ = (deg_per_sec > 0.0f) ? deg_per_sec : 120.0f;
    }

    /* 加速度限幅，单位 度/秒²；<=0 表示关闭平滑（速度模式用） */
    void setAccelLimit(float deg_per_sec2) {
        accel_limit_ = (deg_per_sec2 > 0.0f) ? deg_per_sec2 : 0.0f;
    }

    void Disable(void) {
        mode_     = CtrlMode::Disable;
        cmd_deg_s_ = 0.0f;
        dis_tick_ = 0;
    }

    /* 回零：停机 → 以当前位置为 0 点（40 01 04 64 20 63 0A）→ 可选写 flash。
       走 buildTx 的帧序列，不阻塞任务。回零后 p0 就是当前位置。 */
    void requestRezero(bool save_to_flash = false) {
        svc_        = Svc::Hold;
        svc_tick_   = 0;
        svc_save_   = save_to_flash;
        mode_       = CtrlMode::Disable;
        cmd_deg_s_  = 0.0f;
        dis_tick_   = kStopFrameTicks;       /* 序列结束后不要再补停机帧 */
    }

    bool isBusy(void) const { return svc_ != Svc::None; }

    /* ------------------------- 周期调用（can3SendTask，5ms） ------------------------- */
    void update(void) {
        if (start_tick_ == 0U) {
            start_tick_ = HAL_GetTick();
        }

        if (mode_ == CtrlMode::Disable) {
            cmd_deg_s_ = 0.0f;
            return;
        }

        /* 用真实经过的时间做限幅，任务被长时间打断也不会一次跳太多 */
        const uint32_t now = HAL_GetTick();
        uint32_t dt_ms = now - last_cmd_tick_;
        last_cmd_tick_ = now;
        if (dt_ms == 0U) {
            dt_ms = 1U;
        } else if (dt_ms > 30U) {
            dt_ms = 30U;
        }

        if (mode_ == CtrlMode::Speed) {
            const float target = ref_deg_s_;
            if (accel_limit_ > 0.0f) {
                const float max_step = accel_limit_ * (0.001f * static_cast<float>(dt_ms));
                const float diff     = target - cmd_deg_s_;
                if (diff > max_step) {
                    cmd_deg_s_ += max_step;
                } else if (diff < -max_step) {
                    cmd_deg_s_ -= max_step;
                } else {
                    cmd_deg_s_ = target;
                }
            } else {
                cmd_deg_s_ = target;
            }
            /* 两个工程的 math_utils::Clamp 签名不同：
               midterm 是 float Clamp(float,...) 返回限幅值，
               TR1     是 void  Clamp(float&,...) 原地改。
               这里直接内联限幅，驱动对两种签名都成立，方便后续合并代码。 */
            if (cmd_deg_s_ > max_cmd_) {
                cmd_deg_s_ = max_cmd_;
            } else if (cmd_deg_s_ < -max_cmd_) {
                cmd_deg_s_ = -max_cmd_;
            }
        } else {
            /* 位置模式：直接把绝对目标交给驱动器，加减速曲线由电机内部按
               pos_speed_limit_ 自己规划，主控不做斜坡。 */
            cmd_deg_ = ref_deg_;
        }
    }

    /* ------------------------- 组帧（每拍最多一帧 8 字节） ------------------------- */
    bool buildTx(uint8_t data[8]) override {
        for (uint8_t i = 0; i < 8U; ++i) {
            data[i] = kPad;
        }
        frame_id_  = id_;
        frame_ext_ = false;

        ++tick_;

        /* 0) 回零 / 保存序列优先，期间其它帧一概不发 */
        if (svc_ != Svc::None) {
            return buildService(data);
        }

        /* 1) 失能：补几帧停机，之后彻底静默（不刷总线） */
        if (mode_ == CtrlMode::Disable) {
            if (dis_tick_ < kStopFrameTicks) {
                ++dis_tick_;
                setCtrlAddr();
                data[0] = kCmdStop;
                data[1] = 0x00;
                data[2] = 0x00;
                return snapTx(data);
            }
            return false;
        }

        /* 2) 周期读状态：读帧必须带 bit15（0x8000|id）电机才会回包
              17 01 → 27 01（int16 位置/速度/扭矩）
              19 01 → 29 01（int32 绝对位置，能看到超出 ±3.27 圈的真实位置） */
        if ((tick_ % kReadPeriodTicks) == 0U) {
            ++read_alt_;
            if ((read_alt_ & 1U) != 0U) {
                data[0] = kCmdRead;
                data[1] = kRegStatus;
            } else {
                data[0] = kCmdRead32;
                data[1] = kRegStatus;
            }
            frame_id_  = 0x8000U | (id_ & 0x7FU);
            frame_ext_ = true;
            return snapTx(data, false);      /* 读帧不覆盖「最近控制帧」快照 */
        }

        /* 3) 设定值帧 */
        setCtrlAddr();

        if (mode_ == CtrlMode::Position) {
            const int16_t pos_cnt = toI16(cmd_deg_ * kDeg2Pos);
            if (kPosUseCoop) {
                /* 07 35 <val:限速> <tqe:不限> <pos:目标> —— 协同模式，
                   电机按 val 的速度走到 pos，到位后保持。字节顺序和 07 07 不同！ */
                data[0] = kCmdNormal;                                      /* 0x07 */
                data[1] = kCmdCoop;                                        /* 0x35 */
                putI16(&data[2], toI16(pos_speed_limit_ * kDegS2Vel));     /* 行走速度 */
                putI16(&data[4], kI16Nan);                                 /* 扭矩不限 */
                putI16(&data[6], pos_cnt);                                 /* 目标位置 */
            } else {
                /* 07 07 <pos> 00 80 <tqe>：官方位置控制示例写法 */
                data[0] = kCmdNormal;
                data[1] = kCmdNormal;
                putI16(&data[2], pos_cnt);
                putI16(&data[4], kI16Nan);
                putI16(&data[6], kI16Nan);
            }
        } else {
            /* 07 07 00 80 <vel> <tqe>：速度模式 */
            data[0] = kCmdNormal;
            data[1] = kCmdNormal;
            putI16(&data[2], kI16Nan);                          /* 位置不限 */
            putI16(&data[4], toI16(cmd_deg_s_ * kDegS2Vel));    /* 速度 */
            putI16(&data[6], kI16Nan);                          /* 最大扭矩不限 */
        }

        return snapTx(data);
    }

    /* 本帧应该用哪个 ID / 帧类型（读帧要扩展帧 + 0x8001） */
    uint32_t     getFrameID(void)   const { return frame_id_; }
    CanBus::Type getFrameType(void) const {
        return frame_ext_ ? CanBus::Type::EXTENDED : CanBus::Type::STANDARD;
    }

    /* ------------------------- 反馈 ------------------------- */
    bool matchRx(const FDCAN_RxHeaderTypeDef &header) const override {
        const uint32_t id = header.Identifier;
        rx_hdr_id_  = id;
        rx_hdr_ext_ = (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;

        if ((id >> 8) == (id_ & 0x7FU)) {
            return true;                 /* 正常回包：0x0100 + 目的地址 */
        }
        if (id == id_) {
            return true;
        }
        if (id == (0x8000U | (id_ & 0x7FU))) {
            return true;
        }
        return false;
    }

    void onRx(const uint8_t data[8], uint8_t len) override {
        ++rx_frames_;
        last_rx_id_  = rx_hdr_id_;
        last_rx_ext_ = rx_hdr_ext_;
        rx_len_      = (len > 8U) ? 8U : len;
        for (uint8_t i = 0; i < rx_len_; ++i) {
            raw_rx_[i] = data[i];
        }
        last_rx_tick_ = HAL_GetTick();

        /* 回帧 27 01 <pos:2> <vel:2> <tqe:2>（int16，位置量程 ±3.27 圈） */
        if ((rx_len_ >= 8U) && (data[0] == kRspRead) && (data[1] == kRegStatus)) {
            const int16_t p = getI16(&data[2]);
            const int16_t v = getI16(&data[4]);
            const int16_t t = getI16(&data[6]);

            fb_pos_deg_  = (p == kI16Nan) ? 0.0f
                          : (static_cast<float>(p) / kPosCountsPerRev * 360.0f);
            fb_vel_deg_s = (v == kI16Nan) ? 0.0f
                          : (static_cast<float>(v) / kVelCountsPerRps * 360.0f);
            fb_tqe_raw_  = t;
            fb_valid_    = true;
            ++fb_frames_;

            sum_deg_   = fb_pos_deg_;
            deg_speed_ = fb_vel_deg_s;
            torque_    = static_cast<float>(t);

            if (!synced_) {
                sync_deg_ = fb_pos_deg_;
                synced_   = true;
            }
            return;
        }

        /* 回帧 29 01 <pos:int32>（绝对位置，LSB 0.00001 圈 → ±21474 圈） */
        if ((rx_len_ >= 6U) && (data[0] == kRspRead32) && (data[1] == kRegStatus)) {
            const int32_t p32 =
                static_cast<int32_t>(static_cast<uint32_t>(data[2]) |
                                     (static_cast<uint32_t>(data[3]) << 8) |
                                     (static_cast<uint32_t>(data[4]) << 16) |
                                     (static_cast<uint32_t>(data[5]) << 24));
            pos32_cnt_   = p32;
            pos32_deg_   = static_cast<float>(p32) * kPos32Count2Deg;
            pos32_valid_ = true;
            ++fb32_frames_;
        }
    }

    /* ------------------------- 调试取值 ------------------------- */
    uint32_t getTxFrames(void) const { return tx_frames_; }
    uint32_t getRxFrames(void) const { return rx_frames_; }
    uint32_t getFbFrames(void) const { return fb_frames_; }
    uint32_t getTxFail(void)   const { return tx_fail_; }
    void     notifyTxFail(void)     { ++tx_fail_; }

    bool  isSynced(void)       const { return synced_; }
    bool  isFeedbackValid(void) const { return fb_valid_; }
    CtrlMode getMode(void)     const { return mode_; }
    float getCmdDegSpeed(void) const { return cmd_deg_s_; }
    float getTargetDegSpeed(void) const { return ref_deg_s_; }
    float getTargetDeg(void)   const { return ref_deg_; }
    float getFbPosDeg(void)    const { return fb_pos_deg_; }
    float getFbVelDegS(void)   const { return fb_vel_deg_s; }
    int16_t getFbTqeRaw(void)  const { return fb_tqe_raw_; }

    /* int32 绝对位置：超出 ±3.2767 圈时只有它还能看到真实位置 */
    bool   isPos32Valid(void)  const { return pos32_valid_; }
    float  getPos32Deg(void)   const { return pos32_deg_; }
    long   getPos32Count(void) const { return static_cast<long>(pos32_cnt_); }
    uint32_t getFb32Frames(void) const { return fb32_frames_; }

    /* 位置模式还差多少度（目标 - 实际），正数表示要往正方向走 */
    float getRemainDeg(void) const { return cmd_deg_ - pos32_deg_; }

    /* 位置模式真正下发的量，供串口打印核对（角度 → int16 count → 帧字节） */
    float getCmdDeg(void)      const { return cmd_deg_; }
    long  getCmdPosCount(void) const { return static_cast<long>(toI16(cmd_deg_ * kDeg2Pos)); }

    const uint8_t *getLastTxData(void) const { return last_tx_; }
    uint32_t getLastTxId(void)         const { return last_tx_id_; }
    uint8_t  isLastTxExt(void)         const { return last_tx_ext_; }
    uint8_t  getLastTxLen(void)        const { return 8U; }
    /* 只记录设定值帧（07 07 / 07 35），不会被每 20 拍的读状态帧冲掉 */
    const uint8_t *getLastCtrlData(void) const { return last_ctrl_; }
    uint32_t getLastCtrlId(void)         const { return last_ctrl_id_; }
    uint8_t  isLastCtrlExt(void)         const { return last_ctrl_ext_; }
    const uint8_t *getLastRxData(void) const { return raw_rx_; }
    uint32_t getLastRxId(void)         const { return last_rx_id_; }
    uint8_t  isLastRxExt(void)         const { return last_rx_ext_; }
    uint8_t  getLastRxLen(void)        const { return rx_len_; }

    const char *getStateName(void) const {
        if (svc_ != Svc::None) {
            return "ZEROING";
        }
        if (mode_ == CtrlMode::Disable) {
            return "DIS";
        }
        if (mode_ == CtrlMode::Speed) {
            return fb_valid_ ? "SPD" : "SPD/NOFB";
        }
        return "POS";
    }

private:
    /* -------- 回零 / 保存序列：每拍只发一帧，用 tick 计数推进 -------- */
    enum class Svc : uint8_t { None = 0, Hold, Stop, Zero, Wait, Save };

    bool buildService(uint8_t data[8]) {
        ++svc_tick_;

        switch (svc_) {
            case Svc::Hold:                      /* 先用「零速」把电机刹停（标准帧，已验证可用） */
                frame_id_  = id_;
                frame_ext_ = false;
                data[0] = kCmdNormal; data[1] = kCmdNormal;
                data[2] = 0x00; data[3] = 0x80;      /* 位置不限 */
                data[4] = 0x00; data[5] = 0x00;      /* 速度 0 */
                data[6] = 0x00; data[7] = 0x80;      /* 扭矩不限 */
                if (svc_tick_ >= kSvcHoldTicks) {
                    svc_ = Svc::Stop;
                    svc_tick_ = 0;
                }
                return snapTx(data, false);

            case Svc::Stop:                      /* 停机：回零必须在停机状态（官方要求） */
                frame_id_  = 0x8000U | (id_ & 0x7FU);
                frame_ext_ = true;
                data[0] = kCmdStop;
                data[1] = 0x00;
                data[2] = 0x00;
                if (svc_tick_ >= kSvcStopTicks) {
                    svc_ = Svc::Zero;
                    svc_tick_ = 0;
                }
                return snapTx(data, false);

            case Svc::Zero:                      /* 以当前位置为 0 点（只改 RAM） */
                frame_id_  = 0x8000U | (id_ & 0x7FU);
                frame_ext_ = true;
                data[0] = 0x40; data[1] = 0x01; data[2] = 0x04;
                data[3] = 0x64; data[4] = 0x20; data[5] = 0x63; data[6] = 0x0A;
                if (svc_tick_ >= kSvcZeroTicks) {
                    svc_ = Svc::Wait;
                    svc_tick_ = 0;
                }
                return snapTx(data, false);

            case Svc::Wait:                      /* 官方建议 1s 后再写 flash */
                if (svc_tick_ < kSvcWaitTicks) {
                    return false;                /* 这段时间不发帧 */
                }
                svc_ = svc_save_ ? Svc::Save : Svc::None;
                svc_tick_ = 0;
                return false;

            case Svc::Save:                      /* 05 B3 02 00 00 保存到 flash */
                frame_id_  = 0x8000U | (id_ & 0x7FU);
                frame_ext_ = true;
                data[0] = 0x05; data[1] = 0xB3; data[2] = 0x02;
                data[3] = 0x00; data[4] = 0x00;
                if (svc_tick_ >= kSvcSaveTicks) {
                    svc_ = Svc::None;
                }
                return snapTx(data, false);

            default:
                svc_ = Svc::None;
                return false;
        }
    }

    /* 控制帧/停帧的地址，见 kCtrlUseExtended 说明 */
    void setCtrlAddr(void) {
        if (kCtrlUseExtended) {
            frame_id_  = 0x8000U | (id_ & 0x7FU);
            frame_ext_ = true;
        } else {
            frame_id_  = id_;
            frame_ext_ = false;
        }
    }

    static int16_t toI16(float v) {
        if (v >= 32767.0f)  return 32767;
        if (v <= -32767.0f) return -32767;
        return static_cast<int16_t>(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
    }
    static void putI16(uint8_t *p, int16_t v) {
        p[0] = static_cast<uint8_t>(v & 0xFF);
        p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }
    static int16_t getI16(const uint8_t *p) {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                    (static_cast<uint16_t>(p[1]) << 8));
    }

    bool snapTx(const uint8_t *data, bool is_ctrl = true) {
        for (uint8_t i = 0; i < 8U; ++i) {
            last_tx_[i] = data[i];
        }
        /* 单独留一份「最后一条设定值帧」：每 20 拍会插一条读状态帧，
           否则串口打印的 hex 会周期性跳成读帧 */
        if (is_ctrl) {
            for (uint8_t i = 0; i < 8U; ++i) {
                last_ctrl_[i] = data[i];
            }
            last_ctrl_id_  = frame_id_;
            last_ctrl_ext_ = frame_ext_ ? 1U : 0U;
        }
        last_tx_id_  = frame_id_;
        last_tx_ext_ = frame_ext_ ? 1U : 0U;
        last_tx_tick_ = HAL_GetTick();
        ++tx_frames_;
        return true;
    }

    CtrlMode mode_{CtrlMode::Disable};

    float ref_deg_s_{0.0f};      /* 目标速度（度/秒） */
    float cmd_deg_s_{0.0f};      /* 斜坡后的下发速度（度/秒） */
    float ref_deg_{0.0f};        /* 位置模式：绝对目标角度（度） */
    float cmd_deg_{0.0f};        /* 位置模式：本拍下发的绝对角度（度） */
    float sync_deg_{0.0f};       /* 首次反馈锁存的位置（仅调试用） */
    float accel_limit_{1800.0f}; /* 度/秒²：0→240 度/秒约 0.13 秒 */
    float pos_speed_limit_{240.0f}; /* 位置模式行走速度上限（度/秒） */

    float   fb_pos_deg_{0.0f};
    float   fb_vel_deg_s{0.0f};
    int16_t fb_tqe_raw_{0};

    int32_t pos32_cnt_{0};
    float   pos32_deg_{0.0f};
    bool    pos32_valid_{false};
    uint32_t fb32_frames_{0U};
    uint8_t  read_alt_{0U};

    Svc      svc_{Svc::None};
    uint16_t svc_tick_{0U};
    bool     svc_save_{false};

    uint32_t frame_id_{0U};
    bool     frame_ext_{false};

    bool synced_{false};
    bool fb_valid_{false};

    uint32_t start_tick_{0U};
    uint32_t last_cmd_tick_{0U};
    uint32_t tick_{0U};
    uint32_t dis_tick_{kStopFrameTicks};

    uint8_t  raw_rx_[8]{};
    uint8_t  rx_len_{0U};
    uint32_t last_rx_id_{0U};
    uint8_t  last_rx_ext_{0U};
    uint32_t last_rx_tick_{0U};

    uint8_t  last_tx_[8]{};
    uint8_t  last_ctrl_[8]{};
    uint32_t last_ctrl_id_{0U};
    uint8_t  last_ctrl_ext_{0U};
    uint32_t last_tx_id_{0U};
    uint8_t  last_tx_ext_{0U};
    uint32_t last_tx_tick_{0U};

    uint32_t tx_frames_{0U};
    uint32_t rx_frames_{0U};
    uint32_t fb_frames_{0U};
    uint32_t tx_fail_{0U};

    /* matchRx 是 const，但要把帧头 ID 带给 onRx，只能 mutable */
    mutable uint32_t rx_hdr_id_{0U};
    mutable uint8_t  rx_hdr_ext_{0U};
};
