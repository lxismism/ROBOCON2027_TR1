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


void SteerChassis::run(const pub_chassis_cmd &cmd){

    const float yaw = cur_world_pose_.face * DEG_2_RAD;
    const float vx = cmd.linear_x_ * std::cos(yaw) + cmd.linear_y_ * std::sin(yaw);
    const float vy = -cmd.linear_x_ * std::sin(yaw) + cmd.linear_y_ * std::cos(yaw);
    const float omega = cmd.deg_omega_ * DEG_2_RAD; 

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
            velocitys_[0].dir_deg =  std::atan2(kx0, -ky0) * RAD_2_DEG;
            velocitys_[1].dir_deg = std::atan2(-kx1, -ky1) * RAD_2_DEG;
            velocitys_[2].dir_deg =  std::atan2(-kx2, ky2) * RAD_2_DEG;
            velocitys_[3].dir_deg =   std::atan2(kx3, ky3) * RAD_2_DEG;
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

    cur_world_pose_.face = imu_data.yaw_deg;
    const float yaw = imu_data.yaw_deg * DEG_2_RAD;
    //旋转矩阵
    const Matrix<3, 3> JVehicleToWorld{
        std::cos(yaw),-std::sin(yaw), 0.0f,
        std::sin(yaw), std::cos(yaw), 0.0f,
                   0.0f,             0.0f, 1.0f
    };

    twist = JVehicleToWorld * JChassisToWheelPinv * JWheelsVelocity;

    chassis_actual_twist_.linear_x_ = twist(0, 0);
    chassis_actual_twist_.linear_y_ = twist(1, 0);
    chassis_actual_twist_.deg_omega_ = twist(2, 0) * RAD_2_DEG;

    cur_world_pose_.x += chassis_actual_twist_.linear_x_ * dt;
    cur_world_pose_.y += chassis_actual_twist_.linear_y_ * dt;
    
}