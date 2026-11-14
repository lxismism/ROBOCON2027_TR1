/**
 * @file WitMotionimu.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include "topic_pool.hpp"

class WitMotionimu{
public:

enum class rx_state_Witimu_t : uint8_t {
    WAIT_HEADER = 0,
    WAIT_TYPE,
    WAIT_DATA,
    WAIT_SUM
};

uint8_t processByte(uint8_t byte);
const pub_imu_data &getImuData() const { return newest_data_; }



private:

static constexpr uint8_t kRxBufLen = 11U;

rx_state_Witimu_t state_{rx_state_Witimu_t::WAIT_HEADER};
uint8_t rx_buf_[kRxBufLen]{};
uint8_t rx_idx_{};

pub_imu_data newest_data_{};
pub_imu_data offset_{};
bool is_inited_{false};

void resetState(){
    state_ = rx_state_Witimu_t::WAIT_HEADER;
    rx_idx_ = 0;
}

uint8_t calculateSum(uint8_t rx_buf[kRxBufLen]){
    uint16_t sum = 0;
    for(uint8_t i = 0; i < kRxBufLen - 1U; ++i){
        sum += rx_buf[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}



};