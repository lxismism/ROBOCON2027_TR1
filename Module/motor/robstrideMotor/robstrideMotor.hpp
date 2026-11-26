/**
 * @file robstrideMotor.hpp
 * @author Tim (2055144074@qq.com)
 * @brief 灵足时代EduLite05Motor 私有协议 运控模式驱动
 * @version 0.1
 * @date 2026-11-05
 * 
 *@note  这个灵足电机是单圈绝对值+多圈计数，掉电不记圈数
 *       上位机软件设置：can_master : 253   can_timeout : 2000(100ms)    zero_sta：1 
 *       zero_sta为0则上电角度0~360°取值，非常恶心，上电时角度位置比零小一点点，原来的零点就变成360度了
 *       zero_sta为1则上电读数落在[-180°,180°)，上电的时候离零点位置小于180度即可，现在用的是1
 *       上电把夹爪拉到最开！！！
 *
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "CanBus.hpp"
#include "MotorBase.hpp"
#include "math_utils.hpp"

class EL05Motor : public CanDevice , public  MotorBase{
public: 
    

    EL05Motor(CanBus *manager, uint32_t motor_id, uint32_t host_id = 0xFDU,
              float kp = 0.0f, float kd = 0.0f, float max_cmd = 6.0f,
              float limit_min_deg = -720.0f, float limit_max_deg = 720.0f //角度限位 单位：度
            )
            :CanDevice(manager,motor_id, true, motor_id, true){
                host_id_ = host_id;
                kp_ = kp;
                kd_ = kd;
                max_cmd_ = max_cmd;
                limit_min_deg_ = limit_min_deg;
                limit_max_deg_ = limit_max_deg;               
            }


    bool isEnabled() const { return is_enabled_; }
    uint8_t getModeState() const { return mode_state_; }
    uint8_t getFaultState() const { return fault_state_; }
    bool isChangingCanId() const { return id_chg_state_ != IdChangeState::IDLE; }

    /**
     * @brief 修改灵足电机 CAN ID（Stop -> Set_ID -> 可选 Save 到 Flash）
     * @param old_id      当前电机实际 CAN ID (1~127)
     * @param new_id      目标新 CAN ID (1~127)
     * @param save_flash  是否发送 0x16 保存到 Flash（默认 true，断电不丢失）
     * @return true 已启动修改流程；false 参数无效或正在修改中
     */
    bool requestChangeCanId(uint8_t old_id, uint8_t new_id, bool save_flash = true) {
        if (old_id == 0U || new_id == 0U || old_id > 127U || new_id > 127U ||
            id_chg_state_ != IdChangeState::IDLE) {
            return false;
        }
        id_chg_old_id_  = old_id;
        id_chg_new_id_  = new_id;
        id_chg_save_    = save_flash;
        id_chg_last_ms_ = 0U;
        id_chg_state_   = IdChangeState::SEND_STOP;
        is_enabled_     = false;
        return true;
    }

    //判断收到的是不是EL05的反馈帧
    bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header)const override {
        const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
        if(frame_is_ext != is_extid_) return false;

        //现在只检查type和motor id，去掉模式故障、host id
        static constexpr uint32_t kMask = 0x1F00FF00UL;
        const uint32_t expect = (kTypeFeedback << 24U) | ((id_ & 0xFFUL) << 8U);
        if((rx_header.Identifier & kMask) != expect) return false;

        rx_id_ = rx_header.Identifier;
        return true;
    }
    
    void onRx(const uint8_t data[8], uint8_t len) override {
        if (len<8U) return;

        //反馈id bit23~22：模式状态 ； bit21~16：故障信息
        mode_state_ = static_cast<uint8_t>((rx_id_ >> 22U) & 0x03U);
        fault_state_ = static_cast<uint8_t>((rx_id_ >> 16U) & 0x3FU);
        is_enabled_  = (mode_state_ == kModeRun);

        //Byte0~1: 当前角度 [0~65535]对应-4π~4π rad，高字节在前
        const uint16_t raw_pos = static_cast<uint16_t>((data[0] << 8U) | data[1]);
        const float raw_rad = math_utils::Map(static_cast<float>(raw_pos), 0.0f, 65535.0f,
                    kPosMinRad, kPosMaxRad);
        sum_deg_ = raw_rad * kRadToDeg;
        single_deg_ = math_utils::WrapAngle(sum_deg_, -180.0f, 180.0f);

        //Byte2~3: 当前角速度 [0~65535]对应-50~50 rad/s
        const uint16_t raw_speed = static_cast<uint16_t>((data[2] << 8U) | data[3]);
        const float raw_radspeed = math_utils::Map(static_cast<float>(raw_speed), 0.0f, 65535.0f,
                    kSpeedMinRad, kSpeedMaxRad);
        deg_speed_ = raw_radspeed * kRadToDeg;

        //Byte4~5: 当前扭矩 [0~65535]对应-6~6 N·m
        const uint16_t raw_torque = static_cast<uint16_t>((data[4] << 8U) | data[5]);
        torque_ = math_utils::Map(static_cast<float>(raw_torque), 0.0f, 65535.0f,
                kTorqueMin, kTorqueMax);

        //Byte6~7: 温度，原始值 = 摄氏度 × 10
        const uint16_t raw_temp = static_cast<uint16_t>((data[6] << 8U) | data[7]);
        temperature_ = static_cast<float>(raw_temp) / kTempScale;
    }

    bool buildTx(uint8_t data[8]) override {
        if (id_chg_state_ != IdChangeState::IDLE) {
            return buildIdChangeTx(data);
        }

        //发送分频：can3SendTask 每 1ms 调一次，每2ms次真正发一帧
        if(++send_cnt_ < kSendDiv){
            return false;
        }
        send_cnt_ = 0U;

        //掉线或未使能就重发使能帧，20ms一次
        if(isOffline() || !is_enabled_){
            const uint32_t now = HAL_GetTick();
            if((now - last_enable_ms_) < kEnableIntervalMs){
                return false;
            }
            last_enable_ms_ = now;
            packEnableCmd(data);
            return true;
        }

        packOperationCmd(data);
        return true;
    }

protected:
    bool isMotorOffline() const override{
        return isOffline();
    }

private:
    static constexpr float kPosMinRad   = -12.57f;
    static constexpr float kPosMaxRad   =  12.57f;
    static constexpr float kSpeedMinRad = -50.0f;
    static constexpr float kSpeedMaxRad =  50.0f;
    static constexpr float kTorqueMin   = -6.0f;  //torque n.力矩
    static constexpr float kTorqueMax   =  6.0f;
    static constexpr float kKpMin       =  0.0f;
    static constexpr float kKpMax       =  500.0f;
    static constexpr float kKdMin       =  0.0f;
    static constexpr float kKdMax       =  5.0f;
    static constexpr float kTempScale   =  10.0f;

    static constexpr uint32_t kTypeOperation = 0x01U;  //运控
    static constexpr uint32_t kTypeFeedback  = 0x02U;  //反馈
    static constexpr uint32_t kTypeEnable    = 0x03U;  //使能
    static constexpr uint32_t kTypeStop      = 0x04U;  //停止
    static constexpr uint32_t kTypeSetId     = 0x07U;  //修改 CAN ID
    static constexpr uint32_t kTypeSaveData  = 0x16U;  //保存参数到 Flash

    static constexpr uint8_t kModeRun = 2U;
    static constexpr uint32_t kEnableIntervalMs = 20U;
    static constexpr uint32_t kIdChgIntervalMs  = 30U;
    static constexpr uint8_t kSendDiv = 2U;

    enum class IdChangeState : uint8_t {
        IDLE = 0,
        SEND_STOP,
        SEND_SET_ID,
        SEND_SAVE
    };

    uint8_t host_id_{0xFDU};  //主机 CAN ID,EL05电机的host_id_都应该用此id
    float kp_{0.0f};
    float kd_{0.0f};
    float limit_min_deg_{-720.0f};   //软件角度下限，默认满量程,不限制
    float limit_max_deg_{ 720.0f};   //软件角度上限
    uint8_t mode_state_{0U};
    uint8_t fault_state_{0U};
    bool is_enabled_{false};
    uint32_t last_enable_ms_{0U};
    uint8_t send_cnt_{0U};
    mutable uint32_t rx_id_{0U};

    IdChangeState id_chg_state_{IdChangeState::IDLE};
    uint8_t id_chg_old_id_{1U};
    uint8_t id_chg_new_id_{1U};
    bool id_chg_save_{true};
    uint32_t id_chg_last_ms_{0U};

    static uint16_t floatToUint(float x, float x_min, float x_max){
        math_utils::Clamp(x, x_min, x_max);                         
        float u = math_utils::Map(x, x_min, x_max, 0.0f, 65535.0f);
        math_utils::Clamp(u, 0.0f, 65535.0f);                        
        return static_cast<uint16_t>(u);
    }


    //组装使能帧
    void packEnableCmd(uint8_t data[8]){
        for(uint8_t i = 0; i < 8U; ++i){
            data[i] = 0U;
        }
        tx_id_ = (kTypeEnable << 24U) | ((host_id_ & 0xFFUL) << 8U) | (id_ & 0xFFUL);
    }

    //组装运控帧
    void packOperationCmd(uint8_t data[8]){
        math_utils::Clamp(ref_deg_, limit_min_deg_, limit_max_deg_);  //软件角度限位
        const uint16_t raw_pos    = floatToUint(ref_deg_ * kDegToRad, kPosMinRad, kPosMaxRad);
        const uint16_t raw_speed  = floatToUint(ref_deg_speed_ * kDegToRad, kSpeedMinRad, kSpeedMaxRad);
        const uint16_t raw_kp     = floatToUint(kp_, kKpMin, kKpMax);
        const uint16_t raw_kd     = floatToUint(kd_, kKdMin, kKdMax);
        const uint16_t raw_torque = floatToUint(cmd_, kTorqueMin, kTorqueMax); //本驱动里cmd_为力矩前馈 单位：N·m

        data[0] = static_cast<uint8_t>(raw_pos >> 8U);
        data[1] = static_cast<uint8_t>(raw_pos & 0xFFU);
        data[2] = static_cast<uint8_t>(raw_speed >> 8U);
        data[3] = static_cast<uint8_t>(raw_speed & 0xFFU);
        data[4] = static_cast<uint8_t>(raw_kp >> 8U);
        data[5] = static_cast<uint8_t>(raw_kp & 0xFFU);
        data[6] = static_cast<uint8_t>(raw_kd >> 8U);
        data[7] = static_cast<uint8_t>(raw_kd & 0xFFU);

        tx_id_ = (kTypeOperation << 24U) | (static_cast<uint32_t>(raw_torque) << 8U) | (id_ & 0xFFUL);
    }

    bool buildIdChangeTx(uint8_t data[8]){
        const uint32_t now = HAL_GetTick();
        if(id_chg_last_ms_ != 0U && (now - id_chg_last_ms_) < kIdChgIntervalMs){
            return false;
        }
        id_chg_last_ms_ = now;

        for(uint8_t i = 0; i < 8U; ++i){
            data[i] = 0U;
        }

        switch(id_chg_state_){
            case IdChangeState::SEND_STOP:
                tx_id_ = (kTypeStop << 24U) | ((host_id_ & 0xFFUL) << 8U) | id_chg_old_id_;
                id_chg_state_ = IdChangeState::SEND_SET_ID;
                return true;

            case IdChangeState::SEND_SET_ID:
                tx_id_ = (kTypeSetId << 24U) |
                         (static_cast<uint32_t>(id_chg_new_id_) << 16U) |
                         ((host_id_ & 0xFFUL) << 8U) |
                         id_chg_old_id_;
                id_ = id_chg_new_id_;
                id_chg_state_ = id_chg_save_ ? IdChangeState::SEND_SAVE : IdChangeState::IDLE;
                return true;

            case IdChangeState::SEND_SAVE:
                for(uint8_t i = 0; i < 8U; ++i){
                    data[i] = static_cast<uint8_t>(i + 1U);
                }
                tx_id_ = (kTypeSaveData << 24U) | ((host_id_ & 0xFFUL) << 8U) | id_chg_new_id_;
                id_chg_state_ = IdChangeState::IDLE;
                return true;

            case IdChangeState::IDLE:
            default:
                return false;
        }
    }


};