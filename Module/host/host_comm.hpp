/**
 * @file host_comm.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 这个上下位机通信的有局限，就是由于通信协议中
        len字段只有1字节长度，所以数据载荷最大绝对不可能
        超过256字节，只要这里不改就不可能，后续可以做出优化（但是由于我有很多个消息，一条消息也不用发这么多吧？）。
 * @version 0.1
 * @date 2026-11-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include <cstring>

class HostComm{

static constexpr uint8_t kMaxPayload = 64U;
static constexpr uint8_t kMaxMessages = 16U;

static constexpr uint16_t kLenExceptData = 8U;


struct Host_frame_t{
    uint8_t id;
    uint8_t len;
    uint8_t payload[kMaxPayload];
    uint16_t crc;
};


enum class rx_host_state_t : uint8_t {
    WAITING_FOR_HEAD1 = 0U,
    WAITING_FOR_HEAD2,
    WAITING_FOR_ID,
    WAITING_FOR_LEN,
    WAITING_FOR_DATA,
    WAITING_FOR_CRCL,
    WAITING_FOR_CRCH,
    WAITING_FOR_TAIL1,
    WAITING_FOR_TAIL2,
};

static constexpr uint8_t HOST_HEADER1_BYTE = 0xFF;
static constexpr uint8_t HOST_HEADER2_BYTE = 0xFE;
static constexpr uint8_t HOST_TAIL1_BYTE   = 0xFD;
static constexpr uint8_t HOST_TAIL2_BYTE   = 0xFB;


public:
    using Handler = void (*)(const uint8_t* data, uint8_t len, void *user);

    struct Stats{
        uint32_t bytes_fed{0U};    
        uint32_t frames_ok{0U};
        uint32_t drop_crc{0U};
        uint32_t drop_len{0U};
        uint32_t drop_head{0U};
        uint32_t drop_tail{0U};
        uint32_t drop_no_id{0U};
        uint32_t drop_no_fn{0U};
        uint32_t drop_len_mismatch{0U};    
    };
    
    HostComm() = default;

    void init(void);
    bool registerMessage(uint8_t id, uint8_t len, Handler fn, void *user);

    void processByte(uint8_t byte);
    void processBuffer(const uint8_t *data, uint16_t len);

    size_t buildFrame(uint8_t *out, size_t out_size, uint8_t id,
        const uint8_t *data, uint8_t len);

    const Stats &getStats() const { return stats_; }


private:
    Host_frame_t current_frame_{};
    rx_host_state_t rx_state_{rx_host_state_t::WAITING_FOR_HEAD1};
    uint8_t rx_idx_{};

    Stats stats_{};

    struct Entry{
        uint8_t id{0U};
        uint8_t len{0U};
        Handler fn{nullptr};
        void *user{nullptr};
    };

    Entry table_[kMaxMessages]{};
    uint8_t count_{};



    void resetState(){
        rx_state_ = rx_host_state_t::WAITING_FOR_HEAD1;
        rx_idx_ = 0U;
        current_frame_.id = 0U;
        current_frame_.len = 0U;
        std::memset(current_frame_.payload, 0U, kMaxPayload);
        current_frame_.crc = 0U;
    }

    static uint16_t crc16(uint16_t crc, uint8_t byte){
        crc ^= static_cast<uint16_t>(byte);
        for(uint8_t i = 0; i < 8U; ++i){
            if((crc & 1U) != 0U){
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001U);
            } else {
                crc = static_cast<uint16_t>(crc >> 1);
            }
        }
        return crc;
    }

    static uint16_t calcFrameCrc(const Host_frame_t &frame){
        uint16_t crc = 0xFFFFU;
        
        crc = crc16(crc, frame.id);
        crc = crc16(crc, frame.len);
        
        for(uint8_t i = 0U; i < frame.len; ++i){
            crc = crc16(crc, frame.payload[i]);
        }

        return crc;
    }

    void dispatch(void){
        for (uint8_t i = 0; i < count_; ++i) {
            if(table_[i].id != current_frame_.id) continue;
            if(table_[i].len != current_frame_.len) {
                stats_.drop_len_mismatch++;
                return;
            }
            if(table_[i].fn == nullptr){
                stats_.drop_no_fn++;
                return;
            }
            table_[i].fn(current_frame_.payload, current_frame_.len, table_[i].user);
            stats_.frames_ok++;
            return;
        }
        stats_.drop_no_id++;
    }

};