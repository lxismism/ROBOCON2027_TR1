/**
 * @file UsbPort.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include <cstddef>

#include "lockfree_queue.hpp"

class UsbPort{

    static constexpr size_t kQueueDepth = 16U;
    static constexpr size_t kPacketPayload = 64U;

public:

    struct Packet{
        uint16_t len{0U};
        uint8_t data[kPacketPayload];
    };

    struct Stats{
        uint32_t rx_packet{0U};
        uint32_t rx_dropped{0U};
        uint32_t tx_packet{0U};
        uint32_t tx_dropped{0U};
        uint32_t tx_busy{0U};
    };

    using EventCallback = void (*)(void *user);

    static UsbPort &Instance(){
        static UsbPort instance{};
        return instance;
    }

    void init(EventCallback cb, void *user){
        event_cb_ = cb;
        event_user_ = user;

        tx_staging_valid_ = false;

        stats_.rx_dropped = 0U;
        stats_.rx_packet = 0U;
        stats_.tx_packet = 0U;
        stats_.tx_busy = 0U;
        stats_.tx_dropped = 0U;
        
    }



    bool Read(Packet &packet){ return rx_queue_.TryPop(packet) == Algorithm::QueueError::OK;}
    void onRxFromIsr(const uint8_t *data, uint32_t len);

    bool WriteAsync(const uint8_t *data, uint16_t len);
    void PumpTx();
    void onTxCpltFromIsr();

    const Stats &getStats() const { return stats_; }
 

private:

    UsbPort() = default;
    UsbPort(const UsbPort &) = delete;
    UsbPort &operator=(const UsbPort &) = delete;

    EventCallback event_cb_{nullptr}; 
    void* event_user_{nullptr};
    using PacketQueue = Algorithm::MpscQueue<Packet, kQueueDepth>;
    PacketQueue rx_queue_{};
    PacketQueue tx_queue_{};
    Packet tx_staging_{};
    bool tx_staging_valid_{false};
    
    Stats stats_{};

};