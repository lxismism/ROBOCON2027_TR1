/**
 * @file MotorBase.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "MotorBase.hpp"
#include "math_utils.hpp"

void MotorBase::deg_speed_init(float speed_kp, float speed_ki, float speed_kd, float speed_max_out, float speed_max_IL,
        float pid_speed_improve, float speed_deadband,float output_filter_rc){
    deg_speed_pid_.Kp = speed_kp;
    deg_speed_pid_.Ki = speed_ki;
    deg_speed_pid_.Kd = speed_kd;
    deg_speed_pid_.Maxout = speed_max_out;
    deg_speed_pid_.IntegralLimit = speed_max_IL;
    deg_speed_pid_.Output_LPF_RC = output_filter_rc;


    deg_speed_pid_.Improve = pid_speed_improve;
    deg_speed_pid_.DeadBand = speed_deadband;

    output_type_ = OutputType::SPEED;

    deg_speed_pid_.DWT_CNT       = DWT->CYCCNT;   //“从现在开始”，防止初始化后的那一帧直接越过20ms的掉线检测


}

void MotorBase::deg_init(float speed_kp, float speed_ki, float speed_kd, float speed_max_out, float speed_max_IL,
        float pid_speed_improve, float speed_deadband,
        float deg_kp, float deg_ki, float deg_kd, float max_deg, float deg_max_IL,
        float pid_deg_improve, float deg_deadband,float output_filter_rc){

    deg_speed_pid_.Kp = speed_kp;
    deg_speed_pid_.Ki = speed_ki;
    deg_speed_pid_.Kd = speed_kd;
    deg_speed_pid_.Maxout = speed_max_out;
    deg_speed_pid_.IntegralLimit = speed_max_IL;
    deg_speed_pid_.Output_LPF_RC = output_filter_rc;


    deg_speed_pid_.Improve = pid_speed_improve;
    deg_speed_pid_.DeadBand = speed_deadband;

    deg_speed_pid_.DWT_CNT       = DWT->CYCCNT;   //“从现在开始”，防止初始化后的那一帧直接越过20ms的掉线检测


    
    deg_pid_.Kp = deg_kp;
    deg_pid_.Ki = deg_ki;
    deg_pid_.Kd = deg_kd;
    deg_pid_.Maxout = max_deg;
    deg_pid_.IntegralLimit = deg_max_IL;

    deg_pid_.Improve = pid_deg_improve;
    deg_pid_.DeadBand = deg_deadband;

    output_type_ = OutputType::DEGREE;

    deg_pid_.DWT_CNT       = DWT->CYCCNT;   //“从现在开始”，防止初始化后的那一帧直接越过20ms的掉线检测


}

void MotorBase::deg_with_speed_init(float speed_kp, float speed_ki, float speed_kd, float speed_max_out, float speed_max_IL,
        float pid_speed_improve, float speed_deadband,
        float deg_kp, float deg_ki, float deg_kd, float max_deg, float deg_max_IL,
        float pid_deg_improve, float deg_deadband, float a_max, float output_filter_rc){

    deg_speed_pid_.Kp = speed_kp;
    deg_speed_pid_.Ki = speed_ki;
    deg_speed_pid_.Kd = speed_kd;
    deg_speed_pid_.Maxout = speed_max_out;
    deg_speed_pid_.IntegralLimit = speed_max_IL;
    deg_speed_pid_.Output_LPF_RC = output_filter_rc;

    deg_speed_pid_.Improve = pid_speed_improve;
    deg_speed_pid_.DeadBand = speed_deadband;

    deg_speed_pid_.DWT_CNT       = DWT->CYCCNT;   //“从现在开始”，防止初始化后的那一帧直接越过20ms的掉线检测



    
    deg_pid_.Kp = deg_kp;
    deg_pid_.Ki = deg_ki;
    deg_pid_.Kd = deg_kd;
    deg_pid_.Maxout = max_deg;
    deg_pid_.IntegralLimit = deg_max_IL;

    deg_pid_.Improve = pid_deg_improve;
    deg_pid_.DeadBand = deg_deadband;

    deg_pid_.DWT_CNT       = DWT->CYCCNT;   //“从现在开始”，防止初始化后的那一帧直接越过20ms的掉线检测


    output_type_ = OutputType::DEG_WITH_SPEED;

    deg_planner_.init(a_max);
    

}

void MotorBase::computeOutput(void){

    if(isMotorOffline()){
        resetPids();
        cmd_ = 0.0f; 
        return;
    }


    switch (output_type_) {
        
        case OutputType::SPEED :{
            pidDegSpeedUpdate();
        }break;
        case OutputType::DEGREE :{
            pidDegUpdate();
        }break;
        case OutputType::DEG_WITH_SPEED :{
            pidDegWithSpeedUpdate();
        }break;
        case OutputType::MIT :{
            mitUpdate();
        }break;

        default:{

        }

    }

}