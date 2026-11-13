/**
 * @file chassis_solution.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "chassis_solution.hpp"

#include "matrix_f32.hpp"
#include "math_utils.hpp"

#include "bsp_dwt.h"

using matrix_utils::Matrix;
using matrix_utils::Inv;
using matrix_utils::Trans;
using matrix_utils::Vector;


static constexpr float kMaxDt = 0.02f;
static constexpr float kHomeBecauseOfRest = 0.2f;


/**
* @brief Set the Velocity object,update state before use, unless current_celocity won't update
* 
* @param target_velocity 
*/
void SteerWheel::setVelocity(Velocity_t target_velocity){

    float deg_output = 0.0f;        //由于电机位置环的输入是累计的绝对角度，所以此处也必须用舵轮的累计角度计算，输入和输出必须匹配
    float degspeed_output = 0.0f;
    float diff = target_velocity.dir_deg - current_velocity_.dir_deg;


    if(diff > 90.0f){
        if(diff > 270.0f){
            target_velocity.dir_deg -= 360.0f;

        } else{
            target_velocity.dir_deg -= 180.0f;
            target_velocity.speed = -target_velocity.speed;
        }
    }
    else if(diff < -90.0f){
        if(diff < -270.0f){
            target_velocity.dir_deg += 360.0f;
            
        } else{
            target_velocity.dir_deg += 180.0f;
            target_velocity.speed = -target_velocity.speed;
        }
    }
    diff = target_velocity.dir_deg - current_velocity_.dir_deg;
    

    deg_output = (wheel_sum_deg_ + diff) * kWheelDirReduction;
    degspeed_output = (target_velocity.speed) * RAD_2_DEG * kWheelDriveReduction / kWheelRadmeter; 

    dirmotor_->setMotorDeg(deg_output);

    const float steer_err = deg_output - dirmotor_->getSumDeg();
    const float drive_scale = std::fmaxf(0.0f, std::cos(steer_err * DEG_2_RAD));
    drivemotor_->setMotorDegSpeed(degspeed_output * drive_scale);

}

/**
 * @brief Homing function about single steerwheel
 * 
 */
void SteerWheel::homing(void){
    if(hashoming_ || dirmotor_->isOffline()) return;

    ramp_deg_homing_ += (map_[homing_way_] * deg_homing_speed_);
    dirmotor_->setMotorDeg(ramp_deg_homing_);
    photogate_->update();
    
    switch (homing_state_) {
        case HomingState::Find_First_Zero :{
            if(photogate_->hasRisingEdge()){
                first_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_];
                homing_state_ = HomingState::Find_Second_Zero;
            } 
            else if(photogate_->hasFallingEdge()){
                first_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_^1];        //^1的作用：0变1,1变0，在此处是为了切换数组变量，这种方式优雅一点
                homing_state_ = HomingState::Find_Second_Zero;
            }
        }; break;

        case HomingState::Find_Second_Zero :{
            if(photogate_->hasRisingEdge()){
                second_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_];
                if(std::fabs(second_zero_ - first_zero_) > 170.0f){
                    homing_state_ = HomingState::Find_Third_Zero;
                }       //这个判断是为了防止光电门在边沿附近抖动导致误触发判断，需要超过170度才能保证，它经过的边沿跟第一次经过的不是同一个，不然没意义啊。
            }
            else if(photogate_->hasFallingEdge()){
                second_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_^1];
                if(std::fabs(second_zero_ - first_zero_) > 170.0f){
                    homing_state_ = HomingState::Find_Third_Zero;
                }       
            }
            

        }; break;

        case HomingState::Find_Third_Zero :{
            if(photogate_->hasRisingEdge()){
                third_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_];
                if(std::fabs(third_zero_ - second_zero_) > 170.0f){
                    homing_state_ = HomingState::Check;
                }       //这个判断是为了防止光电门在边沿附近抖动导致误触发判断，需要超过170度才能保证，它经过的边沿跟第一次经过的不是同一次，不然没意义啊。
            }
            else if(photogate_->hasFallingEdge()){
                third_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_^1];
                if(std::fabs(third_zero_ - second_zero_) > 170.0f){
                    homing_state_ = HomingState::Check;
                }       
            }
            
        }; break;

        case HomingState::Check :{
            const float first_offset_ = math_utils::WrapAngle(first_zero_, -180.0f, 180.0f);
            const float second_offset_ = math_utils::WrapAngle(second_zero_, -180.0f, 180.0f);
            const float third_offset_ = math_utils::WrapAngle(third_zero_, -180.0f, 180.0f);

            if(std::fabs(first_offset_ - second_offset_) < 5.0f &&
                std::fabs(second_offset_ - third_offset_) < 5.0f){
                deg_offset_ = (first_offset_ + second_offset_ + third_offset_) / 3.0f;
                homing_state_ = HomingState::Finish;
            } else {
                homing_state_ = HomingState::Find_First_Zero;
            }
        }; break;

        case HomingState::Finish :{
            hashoming_ = true;
        }; break;

    }
    
}

void SteerChassis::run(const ChassisTwist &twist){

    const float yaw = cur_world_pose_.yaw_deg * DEG_2_RAD;
    const float vx = twist.vx * std::cos(yaw) + twist.vy * std::sin(yaw);
    const float vy = -twist.vx * std::sin(yaw) + twist.vy * std::cos(yaw);
    const float omega = twist.deg_omega * DEG_2_RAD; 

    if(std::fabs(vx) < 0.0001f &&
       std::fabs(vy) < 0.0001f &&
       std::fabs(omega) < 0.0001f){

        velocitys_[0].speed = 0.0f;
        velocitys_[1].speed = 0.0f;
        velocitys_[2].speed = 0.0f;
        velocitys_[3].speed = 0.0f;

        float dt = DWT_GetDeltaT(&DWT_rest_cnt_);

        if(dt < kMaxDt){
            rest_time_ += dt;
        }
        
        if(rest_time_ > kHomeBecauseOfRest){
            for(uint8_t i = 0; i < kWheelNum; ++i){
                velocitys_[i].dir_deg = kWheelsRestDeg[i];
            }
        }
    } else {
        rest_time_ = 0.0f;

        velocitys_[0].speed = kDriveSign[0] * (std::sqrt((vx - omega * ky0) * (vx - omega * ky0) + 
                                     (vy + omega * kx0) * (vy + omega * kx0)));
        velocitys_[1].speed = kDriveSign[1] * (std::sqrt((vx - omega * ky1) * (vx - omega * ky1) + 
                                     (vy - omega * kx1) * (vy - omega * kx1)));
        velocitys_[2].speed = kDriveSign[2] * (std::sqrt((vx + omega * ky2) * (vx + omega * ky2) + 
                                     (vy - omega * kx2) * (vy - omega * kx2)));
        velocitys_[3].speed = kDriveSign[3] * (std::sqrt((vx + omega * ky3) * (vx + omega * ky3) + 
                                     (vy + omega * kx3) * (vy + omega * kx3)));

        desaturate();
        
        velocitys_[0].dir_deg = std::atan2(vy + omega * kx0 , vx - omega * ky0) * RAD_2_DEG;
        velocitys_[1].dir_deg = std::atan2(vy - omega * kx1 , vx - omega * ky1) * RAD_2_DEG;
        velocitys_[2].dir_deg = std::atan2(vy - omega * kx2 , vx + omega * ky2) * RAD_2_DEG;
        velocitys_[3].dir_deg = std::atan2(vy + omega * kx3 , vx + omega * ky3) * RAD_2_DEG;
        
    }


    for(uint8_t i = 0; i < kWheelNum; ++i){
        steerwheels_[i].setVelocity(velocitys_[i]);
    }

}

void SteerChassis::ForWardKine(const pub_imu_data &imu_data){

    float dt = DWT_GetDeltaT(&DWT_cnt_);        //单位:s
    if(dt > kMaxDt) return;
    if(dt <= 1e-6f) dt = 1e-6f;


    static constexpr Matrix<8, 3> JChassisToWheel{
        1, 0, -ky0,
        0, 1,  kx0,
        1, 0, -ky1,
        0,1,-kx1,
       1,0, ky2,
       0,1,-kx2,
       1,0, ky3,
       0,1, kx3,        
    };

    //伪逆矩阵
    static const Matrix<3, 8> JChassisToWheelPinv = 
        Inv(Trans(JChassisToWheel) * JChassisToWheel) * Trans(JChassisToWheel);     



    //     FK 里每个轮子取一次，存成局部变量
    const Velocity_t w0 = steerwheels_[0].getVelocity();
    const Velocity_t w1 = steerwheels_[1].getVelocity();
    const Velocity_t w2 = steerwheels_[2].getVelocity();
    const Velocity_t w3 = steerwheels_[3].getVelocity();

    // 轮速的矩阵
    Matrix<8, 1> JWheelsVelocity{
        kDriveSign[0] * w0.speed * std::cos(w0.dir_deg * DEG_2_RAD),
        kDriveSign[0] * w0.speed * std::sin(w0.dir_deg * DEG_2_RAD),
        kDriveSign[1] * w1.speed * std::cos(w1.dir_deg * DEG_2_RAD),
        kDriveSign[1] * w1.speed * std::sin(w1.dir_deg * DEG_2_RAD),
        kDriveSign[2] * w2.speed * std::cos(w2.dir_deg * DEG_2_RAD),
        kDriveSign[2] * w2.speed * std::sin(w2.dir_deg * DEG_2_RAD),
        kDriveSign[3] * w3.speed * std::cos(w3.dir_deg * DEG_2_RAD),
        kDriveSign[3] * w3.speed * std::sin(w3.dir_deg * DEG_2_RAD)
    };

    //底盘旋量
    Vector<3> twist{};

    cur_world_pose_.yaw_deg = imu_data.yaw_deg;
    const float yaw = imu_data.yaw_deg * DEG_2_RAD;
    //旋转矩阵
    const Matrix<3, 3> JVehicleToWorld{
        std::cos(yaw),-std::sin(yaw), 0.0f,
        std::sin(yaw), std::cos(yaw), 0.0f,
                   0.0f,             0.0f, 1.0f
    };

    twist = JVehicleToWorld * JChassisToWheelPinv * JWheelsVelocity;

    chassis_actual_twist_.vx = twist(0, 0);
    chassis_actual_twist_.vy = twist(1, 0);
    chassis_actual_twist_.deg_omega = twist(2, 0) * RAD_2_DEG;

    cur_world_pose_.x += chassis_actual_twist_.vx * dt;
    cur_world_pose_.y += chassis_actual_twist_.vy * dt;
    
}