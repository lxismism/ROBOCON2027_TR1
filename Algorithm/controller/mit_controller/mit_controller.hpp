/**
 * @file mit_controller.h
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>

struct MIT_t{

    float Kp;
    float Kd;
    float target_pos;
    float measure_pos;
    float target_speed;
    float measure_speed;
    float torque_ff;
    
    float Err;

    float Pout, Dout;
    float Output;

    float Maxout;

    float DeadBand;

};

float MIT_Calculate(MIT_t *mit, float target_pos, float measure_pos, float target_speed, float measure_speed, float torque_ff);
void MIT_Reset(MIT_t *mit);

