/**
 * @file Motor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "Canbus.hpp"
#include "pid_controller.hpp"
#include "math_utils.hpp"
#include "ramp_planner.hpp"

class MotorBase{

enum class OutputType : uint8_t { NONE = 0, SPEED, DEGREE, DEG_WITH_SPEED, MIT};


public:
    MotorBase() = default;

    void setCmd(float cmd){
        if(cmd > max_cmd_){
            cmd = max_cmd_;
        }
        if(cmd < -max_cmd_){
            cmd = -max_cmd_;
        }
        cmd_ = cmd;
    }
    
    void setDegSpeed(float deg_speed) { 
        if(output_type_ == OutputType::NONE){
            setCmd(deg_speed);
        }
        ref_deg_speed_ = deg_speed; //此时的ref_deg_speed_只用作调试监看用
        
    }
    void setDeg(float deg) { 
        if(output_type_ == OutputType::NONE){
            setCmd(deg);
        }
        ref_deg_ = deg;
        
    }
    void setDegWithSpeed(float deg, float deg_speed){
        ref_deg_ = deg;
        ref_deg_speed_ = deg_speed;
    }


    void deg_init(float speed_kp, float speed_ki, float speed_kd, float speed_max_out, float speed_max_IL,
        float pid_speed_improve, float speed_deadband,
        float deg_kp, float deg_ki, float deg_kd, float max_deg, float deg_max_IL,
        float pid_deg_improve, float deg_deadband, float output_filter_rc = 0.0f);

    void deg_speed_init(float speed_kp, float speed_ki, float speed_kd, float speed_max_out, float speed_max_IL,
        float pid_speed_improve, float speed_deadband,float output_filter_rc = 0.0f);

    void deg_with_speed_init(float speed_kp, float speed_ki, float speed_kd, float speed_max_out, float speed_max_IL,
        float pid_speed_improve, float speed_deadband,
        float deg_kp, float deg_ki, float deg_kd, float max_deg, float deg_max_IL,
        float pid_deg_improve, float deg_deadband, float a_max, float output_filter_rc = 0.0f);

    

    void computeOutput(void);
    
    float getRefDeg(void) const { return ref_deg_; }
    float getRefDegSpeed(void) const { return ref_deg_speed_; }
    float getSingleDeg(void) const { return single_deg_; }
    float getSumDeg(void) const { return sum_deg_; }
    float getDegSpeed(void) const { return deg_speed_; }
    float getTorque(void) const {return torque_; }
    float getTemperature(void) const { return temperature_; }

protected:
    float cmd_{0.0f};
    float ref_deg_speed_{0.0f};
    float ref_deg_{0.0f};
    float ramp_deg_{0.0f};
    float ramp_deg_speed_{0.0f};
    
    float max_cmd_{99999.0f};
    float reduction_ratio_{1.0f};

    float single_deg_{0.0f};    //转子在单圈内的绝对位置
    float sum_deg_{0.0f};       //电机输出轴累计的角度
    float deg_speed_{0.0f};     //电机输出轴角速度
    float torque_{0.0f};        //输出轴扭矩
    float temperature_{0.0f};

    

    virtual bool isMotorOffline() const{
        return false;
    }

private:
    PID_t deg_speed_pid_{};
    PID_t deg_pid_{};
    S_RampPlanner deg_planner_{};
    OutputType output_type_{};
    void pidDegUpdate(void) {
        ref_deg_speed_ = PID_Calculate(&deg_pid_, sum_deg_, ref_deg_);
        cmd_ = PID_Calculate(&deg_speed_pid_, deg_speed_, ref_deg_speed_);
    }

    void pidDegSpeedUpdate(void){
        cmd_ = PID_Calculate(&deg_speed_pid_, deg_speed_, ref_deg_speed_);
    }

    void pidDegWithSpeedUpdate(void){
        ramp_deg_ = deg_planner_.computeOutput(ref_deg_, ref_deg_speed_);
        ramp_deg_speed_ = PID_Calculate(&deg_pid_, sum_deg_, ramp_deg_, deg_planner_.getAccel());
        cmd_ = PID_Calculate(&deg_speed_pid_, deg_speed_, ramp_deg_speed_);
    }

    void mitUpdate(void){

    }

    void resetPids(void){
        PID_Reset(&deg_pid_);
        PID_Reset(&deg_speed_pid_);
    }    


};


