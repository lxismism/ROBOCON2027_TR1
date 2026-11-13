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

enum class ControlMode : uint8_t {
  PURELY_MANUAL = 0,  //x方向速度， y方向速度， 目标yaw
  AUTOMATIC,  //x坐标， y坐标， 目标yaw
};


struct pub_chassis_cmd{
  /**
  * @brief 

    PURELY_MANUAL:
      x_tar:vx
      y_tar:vy
      yaw_deg: yaw_deg
    
    AUTOMATIC:
      x_tar:x_tar_pos
      y_tar:y_tar_pos
      yaw_deg: yaw_target_deg
  * 
  */

  ControlMode cmd_type;
  float x_tar;
  float y_tar;
  float yaw_deg;
  
};

struct pub_chassis_FK_pose{
  float x;
  float y;
  float yaw_deg;
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

  RC_BTN_SW_State_t last_swE;


};  //size:30byte

struct pub_vision_pose{
  float x;
  float y;
  float yaw;
  uint32_t timestamp_ms;
};

#pragma pack()