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

#ifndef MIT_CONTROLLER_H
#define MIT_CONTROLLER_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>



typedef struct{

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

} MIT_t;

float MIT_Calculate(MIT_t *mit, float target_pos, float measure_pos, float target_speed, float measure_speed, float torque_ff);
void MIT_Reset(MIT_t *mit);

#ifdef __cplusplus
}
#endif


#endif