/**
 * @file robstrideMotor.hpp
 * @author Tim (2055144074@qq.com)
 * @brief 灵足时代EduLite05Motor 私有协议 运控模式驱动
 * @version 0.1
 * @date 2026-11-05
 * 
 *@note  这个灵足电机是单圈绝对值+多圈计数，
 *       上位机软件设置：can_master : 253   can_timeout : 2000    zero_sta：1                      
 *       zero_sta为0则上电角度0~2π取值，非常恶心，上电时角度位置比零小一点点，原来的零点就变成360度了
 *       zero_sta为1则-π~π取值，上电的时候离零点位置小于180度即可，现在用的是1
 *       上电把夹爪拉到最开！！！
 *
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "CanBus.hpp"
#include "MotorBase.hpp"
#include "math_utils.hpp"

class EL05Motor : public CanDevice ,public  MotorBase{
public: 
    

    EL05Motor(CanBus *manager, uint32_t motor_id, uint32_t host_id = 0xFDU,
              float kp = 0.0f, float kd = 0.0f, float max_cmd = 6.0f
            )
            :CanDevice(manager,motor_id, true, motor_id, true){
                host_id_ =host_id;
                kp_ = kp;
                kd_ = kd;
                max_cmd_ = max_cmd;
                output_type_ = OutputType::NONE;
            }

    void setMIT(float pos_deg, float kp, float kd,
                float torque = 0.0f, float speed_deg_s = 0.0f)
    {
        setMotorDeg(pos_deg);           
        setMotorDegSpeed(speed_deg_s);  
        setMotorCmd(torque);            
        kp_ = kp;
        kd_ = kd;
    }

    bool isEnabled() const { return is_enabled_; }
    uint8_t getModeState() const { return mode_state_; }
    uint8_t getFaultState() const { return fault_state_; }

    //软件角度限位
    void setLimitDeg(float min_deg, float max_deg){
        limit_min_deg_ = min_deg;
        limit_max_deg_ = max_deg;
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
        sum_deg_ = raw_rad * RAD_2_DEG;
        single_deg_ = math_utils::WrapAngle(sum_deg_, -180.0f, 180.0f);

        //Byte2~3: 当前角速度 [0~65535]对应-50~50 rad/s
        const uint16_t raw_speed = static_cast<uint16_t>((data[2] << 8U) | data[3]);
        const float raw_radspeed = math_utils::Map(static_cast<float>(raw_speed), 0.0f, 65535.0f,
                    kSpeedMinRad, kSpeedMaxRad);
        deg_speed_ = raw_radspeed * RAD_2_DEG;

        //Byte4~5: 当前扭矩 [0~65535]对应-6~6 N·m
        const uint16_t raw_torque = static_cast<uint16_t>((data[4] << 8U) | data[5]);
        torque_ = math_utils::Map(static_cast<float>(raw_torque), 0.0f, 65535.0f,
                kTorqueMin, kTorqueMax);

        //Byte6~7: 温度，原始值 = 摄氏度 × 10
        const uint16_t raw_temp = static_cast<uint16_t>((data[6] << 8U) | data[7]);
        temperature_ = static_cast<float>(raw_temp) / kTempScale;
    }

    bool buildTx(uint8_t data[8]) override {
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

    static constexpr uint8_t kModeRun = 2U;
    static constexpr uint32_t kEnableIntervalMs = 20U;
    static constexpr uint8_t kSendDiv = 2U;

    uint8_t host_id_{0xFDU};  //主机 CAN ID,EL05电机的host_id_都应该用此id
    float kp_{0.0f};
    float kd_{0.0f};
    float limit_min_deg_{-720.0f};   //软件角度下线，默认满量程,不限制
    float limit_max_deg_{ 720.0f};   //软件角度上限
    uint8_t mode_state_{0U};
    uint8_t fault_state_{0U};
    bool is_enabled_{false};
    uint32_t last_enable_ms_{0U};
    uint8_t send_cnt_{0U};
    mutable uint32_t rx_id_{0U};

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
        const uint16_t raw_pos    = floatToUint(ref_deg_ * DEG_2_RAD, kPosMinRad, kPosMaxRad);
        const uint16_t raw_speed  = floatToUint(ref_deg_speed_ * DEG_2_RAD, kSpeedMinRad, kSpeedMaxRad);
        const uint16_t raw_kp     = floatToUint(kp_, kKpMin, kKpMax);
        const uint16_t raw_kd     = floatToUint(kd_, kKdMin, kKdMax);
        const uint16_t raw_torque = floatToUint(cmd_, kTorqueMin, kTorqueMax);

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


};