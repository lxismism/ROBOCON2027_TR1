/**
 * @file vescMotor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "MotorBase.hpp"
#include "CanBus.hpp"
#include "pid_controller.hpp"


class VESCMotor : public CanDevice , public MotorBase {
public:
    enum Command_ID {
        CAN_PACKET_SET_DUTY                     = 0U,
        CAN_PACKET_SET_CURRENT                  = 1U,
        CAN_PACKET_SET_CURRENT_BRAKE            = 2U,
        CAN_PACKET_SET_RPM                      = 3U,
        CAN_PACKET_SET_POS                      = 4U,
        CAN_PACKET_SET_CURRENT_REL              = 10U,
        CAN_PACKET_SET_CURRENT_BRAKE_REL        = 11U,
        CAN_PACKET_SET_CURRENT_HANDBRAKE        = 12U,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL    = 13U,

    };

    enum Status_ID {
        CAN_PACKET_STATUS   = 9U,
        CAN_PACKET_STATUS_2 = 14U,
        CAN_PACKET_STATUS_3 = 15U,
        CAN_PACKET_STATUS_4 = 16U,
        CAN_PACKET_STATUS_5 = 27U,
        CAN_PACKET_STATUS_6 = 58U
    };

    VESCMotor(CanBus *manager, uint32_t id, const uint8_t command_id = CAN_PACKET_SET_RPM, const uint8_t status_id = CAN_PACKET_STATUS,
        float num_of_pole_pairs = 21.0f, float reduction = 1.0f, float max_cmd = 20000.0f)
        : CanDevice(manager, id, true, id, true){

                num_of_pole_pairs_ = num_of_pole_pairs;
                reduction_ratio_ = reduction;
                max_cmd_ = max_cmd;
                command_id_ = command_id;
                status_id_ = status_id;
            }

    uint8_t getCommandID(){ return command_id_; }

    bool buildTx(uint8_t data[8]) override{

        if(isOffline()) return false;

        int32_t command = 0;

        switch (command_id_) {
            
            case CAN_PACKET_SET_CURRENT :{
                command = static_cast<int32_t>(cmdTrans());
            }break;

            case CAN_PACKET_SET_RPM :{
                command = static_cast<int32_t>(ref_deg_speed_ * static_cast<float>(num_of_pole_pairs_) * kDPSToRPM);
            }break;         

        }

        
        data[0] = static_cast<uint8_t>((command >> 24) & 0xFF);
        data[1] = static_cast<uint8_t>((command >> 16) & 0xFF);
        data[2] = static_cast<uint8_t>((command >>  8) & 0xFF);
        data[3] = static_cast<uint8_t>(command & 0xFF);

        return true;

    }

    void onRx(const uint8_t data[8], const uint8_t len) override {
        (void)len;
        /*data[0]~data[3]: ERPM 电转速*/
        int32_t ERPM = static_cast<int32_t>((data[0] << 24) |
                                            (data[1] << 16) |
                                            (data[2] <<  8) |
                                            data[3]);
        /*RPM = ERPM / num_of_pole_pairs(极对数)*/
        deg_speed_ = static_cast<float>(ERPM)  * kRPMToDPS / num_of_pole_pairs_;
        
        /*data[4]~data[5]: 电流，缩放因子为10*/
        int16_t raw_current = static_cast<int16_t>((data[4] << 8) | data[5]);
        current_ = raw_current / 10.0f;

        /*data[6]~data[7]: 占空比*/
        int16_t raw_duty = static_cast<int16_t>(data[6] << 8 | data[7]);
        duty_cycle_ = raw_duty / 1000.0f;
    }

    bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header) const override{
        const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
        if(frame_is_ext != is_extid_) return false;
        const uint8_t status_ID = (rx_header.Identifier >> 8) & 0xFFU;
        const uint8_t id = (rx_header.Identifier & 0xFFU);
        
        return ((status_ID == status_id_) && (id == id_));
    }

    float cmdTrans() { return cmd_ ; }
    

protected:
    bool isMotorOffline() const override{
        return isOffline();
    }

private:
    uint8_t command_id_;
    uint8_t status_id_;

    float num_of_pole_pairs_;
    float current_;
    float duty_cycle_;


};