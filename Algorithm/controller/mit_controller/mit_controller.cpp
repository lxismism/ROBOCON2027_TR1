/**
 * @file mit_controller.c
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "mit_controller.hpp"

#include "math_utils.hpp"

float MIT_Calculate(MIT_t *mit, float target_pos, float measure_pos, 
    float target_speed, float measure_speed, float torque_ff){

    mit->target_pos = target_pos;
    mit->measure_pos = measure_pos;
    mit->target_speed = target_speed;
    mit->measure_speed = measure_speed;
    mit->torque_ff = torque_ff;
    
    
    mit->Err = mit->target_pos - mit->measure_pos;

    if(std::fabs(mit->Err) < mit->DeadBand){
        mit->Err = 0.0f;
    }
    
    mit->Pout = mit->Kp * mit->Err;
    mit->Dout = mit->Kd * (mit->target_speed - mit->measure_speed); 

    mit->Output = mit->Pout + mit->Dout + mit->torque_ff; 

    math_utils::Clamp(mit->Output, -mit->Maxout, mit->Maxout);

    return mit->Output;
}

void MIT_Reset(MIT_t *mit){
    mit->Err = 0.0f;
    mit->Pout = 0.0f;
    mit->Dout = 0.0f;
    mit->Output = 0.0f;
}