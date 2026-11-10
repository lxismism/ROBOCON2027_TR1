/**
 * @file math_utils.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "math_utils.hpp"
#include "arm_math.h"

namespace math_utils{

float sinDeg(float deg){
    return arm_sin_f32(deg * kDegToRad);
}

float cosDeg(float deg){
    return arm_cos_f32(deg * kDegToRad);
}

void sin_cos_deg(float deg, float &s, float c){
    arm_sin_cos_f32(deg, &s, &c);
}

}   //namespace math_utils