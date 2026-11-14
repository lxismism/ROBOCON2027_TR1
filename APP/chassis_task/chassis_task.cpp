/**
 * @file chassis_task.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "chassis_task.h"
#include "chassis_solution.hpp"
#include "MotorBase.hpp"
#include "djiMotor.hpp"
#include "vescMotor.hpp"

#include "com_config.h"

#include "topics.hpp"
#include "topic_pool.hpp"

#include "pid_controller.h"

osThreadId_t Chassis_TaskHandle;

extern std::array<C620Motor*, kWheelNum> chassis_dirmotors;
extern std::array<VESCMotor*, kWheelNum> chassis_drivemotors;
SteerChassis SteerChassis_solver(chassis_dirmotors, chassis_drivemotors);

static TypedTopicPublisher<pub_chassis_FK_pose> chassis_FK_pose_pub("chassis_FK_pose");
static pub_chassis_FK_pose chassis_FK_pose_msg{};

static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8U);
static pub_chassis_cmd chassis_cmd_msg{};

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data_msg{};

PID_t pid_x_pos{.Kp = 6.5f, .Ki = 2.5f, .Kd = 0.02f, 
    .IntegralLimit = 1.5f, .Maxout = 1.5f, .DeadBand = 0.01f, .Improve = Derivative_On_Measurement};
PID_t pid_y_pos{.Kp = 6.5f, .Ki = 2.5f, .Kd = 0.02f, 
    .IntegralLimit = 1.5f, .Maxout = 1.5f, .DeadBand = 0.01f, .Improve = Derivative_On_Measurement};
PID_t pid_deg_pos{.Kp = 25.5f, .Ki = 10.1f, .Kd = 3.5f, 
    .IntegralLimit = 360.0f, .Maxout = 360.0f, .DeadBand = 0.01f, .Improve = Derivative_On_Measurement};

void ChassisForAuto();
void ChassisForManual();

void chassisTask(void *argument){
    
    TickType_t current = xTaskGetTickCount();
    vTaskDelayUntil(&current, 500);
    for(;;)
    {
        (void)imu_data_sub.TryGet(&imu_data_msg);

        if(!SteerChassis_solver.HasHoming()){
            SteerChassis_solver.homing();
        } else{
            SteerChassis_solver.updateChassisState(imu_data_msg);
            chassis_FK_pose_msg.x = SteerChassis_solver.getChassisPose().x;
            chassis_FK_pose_msg.y = SteerChassis_solver.getChassisPose().y;
            chassis_FK_pose_msg.yaw_deg = SteerChassis_solver.getChassisPose().yaw_deg;
            chassis_FK_pose_pub.Publish(chassis_FK_pose_msg);

            if(chassis_cmd_sub.TryGet(&chassis_cmd_msg)){       //ozone调试自动时注释此行
                switch (chassis_cmd_msg.cmd_type) {

                    case ControlMode::AUTOMATIC     :{
                        ChassisForAuto();
                    } break;
                    case ControlMode::PURELY_MANUAL :{
                        ChassisForManual();
                    } break;

                }
            }           //ozone调试自动时注释此行

        }

        vTaskDelayUntil(&current, 1);
    }
}

void ChassisForAuto(){
    if(chassis_cmd_msg.cmd_type != ControlMode::AUTOMATIC) return;

    ChassisTwist twist{};

    twist.vx = 
    PID_Calculate(&pid_x_pos, chassis_FK_pose_msg.x, chassis_cmd_msg.x_tar);
    twist.vy = 
    PID_Calculate(&pid_y_pos, chassis_FK_pose_msg.y, chassis_cmd_msg.y_tar);

    const float yaw_now = chassis_FK_pose_msg.yaw_deg;                 // 窗口值 [-180,180)
    const float yaw_ref = yaw_now + math_utils::WrapAngle(
                                    chassis_cmd_msg.yaw_deg - yaw_now, -180.0f, 180.0f);
    twist.deg_omega = PID_Calculate(&pid_deg_pos, yaw_now, yaw_ref);

    SteerChassis_solver.run(twist);
    
}

void ChassisForManual(){
    if(chassis_cmd_msg.cmd_type != ControlMode::PURELY_MANUAL) return;

    ChassisTwist twist{};
    
    twist.vx = chassis_cmd_msg.x_tar;
    twist.vy = chassis_cmd_msg.y_tar;

    const float yaw_now = chassis_FK_pose_msg.yaw_deg;                 // 窗口值 [-180,180)
    const float yaw_ref = yaw_now + math_utils::WrapAngle(
                                    chassis_cmd_msg.yaw_deg - yaw_now, -180.0f, 180.0f);
    twist.deg_omega = PID_Calculate(&pid_deg_pos, yaw_now, yaw_ref);

    SteerChassis_solver.run(twist);
}