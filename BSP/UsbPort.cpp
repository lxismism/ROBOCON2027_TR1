/**
 * @file UsbPort.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "UsbPort.hpp"
#include "UsbPortC.h"
#include <cstring>
#include "usbd_cdc_if.h"



void UsbPort::onRxFromIsr(const uint8_t *data, uint32_t len){
    if(data == nullptr || len <= 0U) return;

    size_t offset = 0U;
    while(offset < len){

        size_t chunk = len - offset;
        if(chunk > kPacketPayload){
            chunk = kPacketPayload;
        }
        Packet packet{};
        packet.len = chunk;
        std::memcpy(packet.data, data + offset, packet.len);

        if(rx_queue_.TryPush(packet) != Algorithm::QueueError::OK){
            stats_.rx_dropped++;
            break;
        }

        stats_.rx_packet++;
        offset += chunk;
    }

    if(event_cb_ != nullptr){
        event_cb_(event_user_);
    }

}

bool UsbPort::WriteAsync(const uint8_t *data, uint16_t len){
    if(data == nullptr || len <= 0U) return false;

    size_t offset = 0U;
    bool pushed_any = false;
    while (offset < len) {
        size_t chunk = len - offset;
        if(chunk > kPacketPayload){
            chunk = kPacketPayload;
        }
        Packet packet{};
        packet.len = static_cast<uint16_t>(chunk);
        std::memcpy(packet.data, data + offset, packet.len);
        if(tx_queue_.TryPush(packet) != Algorithm::QueueError::OK){
            stats_.tx_dropped++;        //这里的tx_dropped实际上不是丢包的数量，而是丢包的次数，一次丢掉的包不一定只有一个
            break;
        }
        pushed_any = true;
        offset += chunk;
    }

    if(event_cb_ != nullptr && pushed_any){
        event_cb_(event_user_);
    }
    return pushed_any;
}

void UsbPort::PumpTx(){

    if(!tx_staging_valid_){
        if(tx_queue_.TryPop(tx_staging_) != Algorithm::QueueError::OK){
            return;
        }
        tx_staging_valid_ = true;
    }
    const uint8_t result = CDC_Transmit_HS(tx_staging_.data, tx_staging_.len);
    if(result == USBD_OK){
        tx_staging_valid_ = false;
        stats_.tx_packet++;
    } else {
        stats_.tx_busy++;
    }
}

void UsbPort::onTxCpltFromIsr(){
    if(event_cb_ != nullptr){
        event_cb_(event_user_);
    }
}


extern "C" void UsbPort_onRxFromIsr(const uint8_t *data, uint32_t len){
    UsbPort::Instance().onRxFromIsr(data, static_cast<uint16_t>(len));
}

extern "C" void UsbPort_onTxCpltFromIsr(void){
    UsbPort::Instance().onTxCpltFromIsr();
}