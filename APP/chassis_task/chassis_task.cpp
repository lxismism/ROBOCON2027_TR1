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


osThreadId_t Chassis_TaskHandle;

extern std::array<C620Motor*, kWheelNum> chassis_dirmotors;
extern std::array<VESCMotor*, kWheelNum> chassis_drivemotors;
SteerChassis SteerChassis_solver(chassis_dirmotors, chassis_drivemotors);

static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8U);
static pub_chassis_cmd chassis_cmd_msg{};

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data_msg{};

void chassisTask(void *argument){
    
    TickType_t current = xTaskGetTickCount();
    vTaskDelayUntil(&current, 500);
    for(;;)
    {
        imu_data_sub.TryGet(&imu_data_msg);
        if(!SteerChassis_solver.HasHoming()){
            SteerChassis_solver.homing();
        } else{
            if(chassis_cmd_sub.TryGet(&chassis_cmd_msg)){
                SteerChassis_solver.run(chassis_cmd_msg, imu_data_msg);
            }
        }

        
        vTaskDelayUntil(&current, 1);
    }
}