/**
 * @file host_msgs.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief   每条消息的结构体大小最好是4的倍数
 * @version 0.1
 * @date 2026-11-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <cstddef>

struct VisionPose {
    static constexpr uint8_t id = 0x01;

    float    x;
    float    y;
    float    yaw;
    uint32_t timestamp_ms;
    uint32_t seq;

    static void unpack(const uint8_t *data, VisionPose &out) {
        std::memcpy(&out.x,            &data[0],  4U);
        std::memcpy(&out.y,            &data[4],  4U);
        std::memcpy(&out.yaw,          &data[8],  4U);
        std::memcpy(&out.timestamp_ms, &data[12], 4U);
        std::memcpy(&out.seq,          &data[16], 4U);
    }
};

static_assert(sizeof(VisionPose) == 20U, "VisionPose的结构体大小必须是20字节");
static_assert(offsetof(VisionPose, timestamp_ms) == 12U, "timestamp_ms偏移错");
static_assert(offsetof(VisionPose, seq) == 16U, "seq偏移错");
