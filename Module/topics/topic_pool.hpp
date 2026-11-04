/**
 * @file topic_pool.hpp
 * @author your name (you@domain.com)
 * @brief topic_pool里是发布订阅功能的消息包
 * @version 0.1
 * @date 2026-10-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "rm_pocket.hpp"
#include <stdint.h>
#include <stdbool.h>

#pragma pack(1)



struct pub_chassis_cmd{
    float linear_x_;
    float linear_y_;
    float deg_omega_;
};

struct pub_imu_data{
  float roll_deg;   // 滚转角
  float pitch_deg; // 俯仰角
  float yaw_deg;  // 偏航角，绕Z轴旋转，范围 -π ~ +π
};

struct pub_rc_data{
  uint16_t joyRHori;      //CH1
  uint16_t joyRVert;      //CH2
  uint16_t joyLVert;      //CH3
  uint16_t joyLHori;      //CH4

  RC_2_Pos_SW_State_t swA;      //CH5
  RC_3_Pos_SW_State_t swB;      //CH6
  RC_3_Pos_SW_State_t swC;      //CH7
  RC_2_Pos_SW_State_t swD;      //CH8
  RC_BTN_SW_State_t swE;      //CH9
  uint16_t pot;      //CH10 手柄背后的那个电位器

  uint16_t channel_11;      //CH11
  uint16_t channel_12;      //CH12
  uint16_t channel_13;      //CH13
  uint16_t channel_14;      //CH14
  uint16_t channel_15;      //CH15
  uint16_t channel_16;      //CH16
};  //size:28byte

struct pub_vision_pose{
  float x;
  float y;
  float yaw;
  uint32_t timestamp_ms;
};

#pragma pack()