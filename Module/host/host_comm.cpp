/**
 * @file host_comm.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "host_comm.hpp"

void HostComm::init(){
    resetState();
    stats_ = Stats{};

}

bool HostComm::registerMessage(uint8_t id, uint8_t len, Handler fn, void *user){

    if(len > kMaxPayload || fn == nullptr) return false;
    if(count_ >= kMaxMessages){ return false; }

    table_[count_].id = id;
    table_[count_].len = len;
    table_[count_].fn = fn;
    table_[count_].user = user;
    count_++;
    return true;
}

void HostComm::processByte(uint8_t byte){
    stats_.bytes_fed++;
    switch (rx_state_) {

        case rx_host_state_t::WAITING_FOR_HEAD1 :{
            if(byte == HOST_HEADER1_BYTE){
                rx_state_ = rx_host_state_t::WAITING_FOR_HEAD2;
            } else {
                stats_.drop_head++;
            }
        }break;

        case rx_host_state_t::WAITING_FOR_HEAD2 :{
            if(byte == HOST_HEADER2_BYTE){
                rx_state_ = rx_host_state_t::WAITING_FOR_ID;
            } else {
                stats_.drop_head++;
            }
        }break;

        case rx_host_state_t::WAITING_FOR_ID :{
            current_frame_.id = byte;
            rx_state_ = rx_host_state_t::WAITING_FOR_LEN;
        }break;

        case rx_host_state_t::WAITING_FOR_LEN :{
            current_frame_.len = byte;

            if(current_frame_.len > kMaxPayload){
                resetState();
                stats_.drop_len++;
                break;
            }

            rx_idx_ = 0U;
            std::memset(current_frame_.payload, 0U, kMaxPayload);

            rx_state_ = (current_frame_.len == 0U) ? rx_host_state_t::WAITING_FOR_CRCL :
                                                     rx_host_state_t::WAITING_FOR_DATA;

        }break;

        case rx_host_state_t::WAITING_FOR_DATA :{
            current_frame_.payload[rx_idx_++] = byte;
            if(rx_idx_ >= current_frame_.len){
                rx_state_ = rx_host_state_t::WAITING_FOR_CRCL;
            }
        }break;

        case rx_host_state_t::WAITING_FOR_CRCL :{
            current_frame_.crc = (byte & 0xFF);
            rx_state_ = rx_host_state_t::WAITING_FOR_CRCH;
        }break;

        case rx_host_state_t::WAITING_FOR_CRCH :{
            current_frame_.crc |= (byte << 8U);
            if(current_frame_.crc == calcFrameCrc(current_frame_)){
                rx_state_ = rx_host_state_t::WAITING_FOR_TAIL1;
            } else {
                resetState();
                stats_.drop_crc++;
            }
        }break;

        case rx_host_state_t::WAITING_FOR_TAIL1 :{
            if(byte == HOST_TAIL1_BYTE){
                rx_state_ = rx_host_state_t::WAITING_FOR_TAIL2;
            } else {
                resetState();
                stats_.drop_tail++;
            }
        }break;

        case rx_host_state_t::WAITING_FOR_TAIL2 :{
            if(byte == HOST_TAIL2_BYTE){
                dispatch();
            } else {
                stats_.drop_tail++;
            }
            resetState();
        }break;

        default:{
            resetState();
        }




    }


}

void HostComm::processBuffer(const uint8_t *data, uint16_t len){
    if(data == nullptr){
        return;
    }

    for(size_t i = 0; i < len; ++i){
        processByte(data[i]);
    }

}

size_t HostComm::buildFrame(uint8_t *out, size_t out_size, uint8_t id,
        const uint8_t *data, uint8_t len){
    
    if(out == nullptr) return 0U;
    if(len > kMaxPayload) return 0;        
    if(len + kLenExceptData > out_size) return 0U;

    out[0] = HOST_HEADER1_BYTE;
    out[1] = HOST_HEADER2_BYTE;
    out[2] = id;
    out[3] = len;


    for(uint8_t i = 0U; i < len; ++i){
        out[4 + i] = data[i];
    }

    uint16_t crc = 0xFFFFU;
    for(uint8_t i = 2U; i < 4U + len; ++i){
        crc = crc16(crc, out[i]);
    }       //此处的CRC校验实现得太丑陋了，但是暂时还没想到好的复用办法

    out[4 + len] = static_cast<uint8_t>(crc & 0xFF);
    out[4 + len + 1] = static_cast<uint8_t>((crc >> 8U) & 0xFF);

    out[4 + len + 2] = HOST_TAIL1_BYTE;
    out[4 + len + 3] = HOST_TAIL2_BYTE;

    return static_cast<size_t>(len + kLenExceptData);

}
