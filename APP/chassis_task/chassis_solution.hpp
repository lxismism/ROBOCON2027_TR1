/**
 * @file chassis_solution.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
    底盘解算的四个舵轮布局如下：
            1    0   

            2    3

 * @version 0.1
 * @date 2026-10-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include "gpio.h"

#include "MotorBase.hpp"
#include "CanBus.hpp"
#include "djiMotor.hpp"
#include "vescMotor.hpp"

#include "photogate.hpp"

#include "topic_pool.hpp"
#include "math_utils.hpp"
#include <array>

static constexpr uint8_t kWheelNum = 4U;        //轮数

static constexpr std::array<std::array<float, 2>, kWheelNum> edge_at_zeros{{
    {-60.0f, 120.0f},
    {120.0f, -60.0f},
    {105.0f, -75.0f},
    {105.0f, -75.0f}
}};

static constexpr uint8_t homing_ways[kWheelNum]{0, 1, 0, 1};

extern PhotoGate chassis_photogate1;
extern PhotoGate chassis_photogate2;
extern PhotoGate chassis_photogate3;
extern PhotoGate chassis_photogate4;
extern std::array<PhotoGate*, kWheelNum> chassis_photogates;

struct Velocity_t{
    float dir_deg;  //单位为°
    float speed;
};

struct ChassisPose{
    float x;
    float y;
    float yaw_deg;
};

struct ChassisTwist{
    float vx;
    float vy;
    float deg_omega;
};

class SteerWheel{
public:
    SteerWheel() = default;

    enum class HomingState : uint8_t{
        Find_First_Zero = 0,
        Find_Second_Zero,
        Find_Third_Zero,
        Check,
        Finish,
    };

    void init(C620Motor* dirmotor, VESCMotor* drivemotor, PhotoGate* photogate, 
              uint8_t homing_way, const std::array<float, 2>& edge_at_zero){
        dirmotor_ = dirmotor;
        drivemotor_ = drivemotor;
        photogate_ = photogate;
        homing_way_ = homing_way;
        edge_at_zero_ = edge_at_zero;
    }
    void setVelocity(Velocity_t target_velocity);
    void homing(void);
    bool IsReady(void) { return hashoming_; }
    Velocity_t getVelocity(){ return current_velocity_; }
    void updateWheelState(){
        updateDirDeg();
        updateSpeed();        
    }


private:
    static constexpr float kWheelDiameter = 0.104f;     //轮径
    static constexpr float kWheelRadmeter = kWheelDiameter / 2.0f;      //轮半径
    static constexpr float kWheelCirmeter = kPI * kWheelDiameter;        //轮周
    static constexpr float kWheelDriveReduction = 1.0f;     //电机输出轴->驱动轮的减速比
    static constexpr float kWheelDirReduction = 1.0f;       //电机输出轴->舵向轮的减速比

    C620Motor* dirmotor_{};
    VESCMotor* drivemotor_{};
    PhotoGate* photogate_{};

/*------------------------------homing----------------------------*/
    uint8_t homing_way_{0};  //默认逆时针转动来归零
    std::array<float, 2> map_{1.0f, -1.0f};


    std::array<float, 2> edge_at_zero_{-150.0f, 30.0f};     //这个数组存的，是两个cnc边沿，在极坐标中的真实位置，由且仅由安装位置决定。
                                                                    //编号为0的数组意为逆时针针转时，遇到的上升沿位置，这里根据0号轮填了个默认值。
    float deg_offset_{};
    float ramp_deg_homing_{};
    float deg_homing_speed_{0.1f};

    float first_zero_{};    
    float second_zero_{};
    float third_zero_{};

    HomingState homing_state_{};
    bool hashoming_{false};
/*------------------------------homing----------------------------*/

    Velocity_t current_velocity_{};
    Velocity_t last_velocity_{};
    float wheel_sum_deg_{0.0f};

    void updateDirDeg(){
        wheel_sum_deg_ = dirmotor_->getSumDeg() / kWheelDirReduction;
        current_velocity_.dir_deg = math_utils::WrapAngle(wheel_sum_deg_ - deg_offset_,
            -180.0f, 180.0f);
    }

    void updateSpeed(){
        current_velocity_.speed = drivemotor_->getDegSpeed() * DEG_2_RAD * kWheelRadmeter / kWheelDriveReduction;
    }

};

class SteerChassis{
    
    /*
        1    0   

        2    3
    */

public:
    SteerChassis(const std::array<C620Motor* , kWheelNum> &dirmotors,
                 const std::array<VESCMotor* , kWheelNum> &drivemotors){
        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].init(dirmotors[i], drivemotors[i], 
                chassis_photogates[i], homing_ways[i], edge_at_zeros[i]);
        }
    }

    void run(const ChassisTwist &cmd);

    void homing(void){
        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].homing();
        }
    }

    bool HasHoming(void){

        bool has_homing = (steerwheels_[0].IsReady() &&
                steerwheels_[1].IsReady() &&
                steerwheels_[2].IsReady() &&
                steerwheels_[3].IsReady());

        if(!has_homing){
            resetPose();
        }

        return has_homing;
    }

    void updateChassisState(const pub_imu_data &imu_data){
        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].updateWheelState();
        }
        ForWardKine(imu_data);
    }

    ChassisPose getChassisPose(){ return cur_world_pose_; }



private:


    //ky,kx的值即便在同一个机器人上，也应该是可以不同的，其中，它的原点取决于你想让机器人自转时，绕机器人的哪个点旋转
    static constexpr float ky0 = 0.18f;  //ky的含义是，点到x轴的距离
    static constexpr float kx0 = 0.1275f; //kx的含义是，点到y轴的距离
    static constexpr float ky1 = 0.18f;  //ky的含义是，点到x轴的距离
    static constexpr float kx1 = 0.1275f; //kx的含义是，点到y轴的距离
    static constexpr float ky2 = 0.215f;  //ky的含义是，点到x轴的距离
    static constexpr float kx2 = 0.2675f; //kx的含义是，点到y轴的距离
    static constexpr float ky3 = 0.215f;  //ky的含义是，点到x轴的距离
    static constexpr float kx3 = 0.2675f; //kx的含义是，点到y轴的距离

    static constexpr float kWheelsRestDeg[kWheelNum]{
        std::atan2(kx0, -ky0) * RAD_2_DEG,
        std::atan2(-kx1, -ky1) * RAD_2_DEG,
        std::atan2(-kx2, ky2) * RAD_2_DEG,
        std::atan2(kx3, ky3) * RAD_2_DEG
    };

    static constexpr std::array<float, 4> kDriveSign{1.0f, -1.0f, 1.0f, -1.0f}; //由于机械安装问题，每个轮子正转方向可能不一样，这个符号表以测试结果为准

    static constexpr float kWheelMaxSpeed = 2.5f;   //最大轮速，这个跟限制底盘最大移动速度不是一回事，它是为了避免解算时算出的单个轮子速度超过电机物理极限设计的

    SteerWheel steerwheels_[kWheelNum]{};
    Velocity_t velocitys_[kWheelNum]{};
    ChassisTwist chassis_actual_twist_{};

    ChassisPose cur_world_pose_{};
    ChassisPose chassis_target_pose_{};

    float rest_time_{};
    uint32_t DWT_rest_cnt_{}; //这个是为了计算静止时间设置的
    
    uint32_t DWT_cnt_{};    //这个是为了积分出位置设置的

    /**
     * @brief   稀释函数，即当有一个轮子速度超过设置的最大轮速
              就把所有轮子的速度都通过映射方式，使得原来目标速
              度最大的轮子的速度等于设置的物理极限（当然，这个
              设置的物理极限是留有很多余量的，u8电机哪有这么拉）
     * 
     */
    void desaturate(){
        float max_speed = 0.0f;
        
        for(uint8_t i = 0; i < kWheelNum; ++i){
            const float speed = std::fabs(velocitys_[i].speed);
            if(speed > max_speed){
                max_speed = speed;
            }
        }

        if(max_speed < kWheelMaxSpeed){
            return;
        }

        const float scale = kWheelMaxSpeed / max_speed;
        for(uint8_t i = 0; i < kWheelNum; ++i){
            velocitys_[i].speed *= scale;
        }

    }

    void ForWardKine(const pub_imu_data &imu_data);

    void resetPose(){
        cur_world_pose_.x = 0.0f;
        cur_world_pose_.y = 0.0f;
        cur_world_pose_.yaw_deg = 0.0f;
        DWT_cnt_ = 0U;
    }

};