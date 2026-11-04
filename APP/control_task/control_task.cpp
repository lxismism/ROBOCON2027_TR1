/**
 * @file control_task.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-29 2:03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "control_task.h"
#include "rm_pocket.hpp"

#include "math_utils.hpp"

#include "topics.hpp"
#include "topic_pool.hpp"

osThreadId_t Control_TaskHandle;

static TypedTopicSubscriber<pub_rc_data> rc_data_sub("rc_data", 8U);
static pub_rc_data rc_data_msg{};

static TypedTopicPublisher<pub_chassis_cmd> chassis_cmd_pub("chassis_cmd");
static pub_chassis_cmd chassis_cmd_msg{};


static constexpr float kChannelValueMin = 172.0f;
static constexpr float kChannelValueMid = 991.5f;
static constexpr float kChannelValueMax = 1811.0f;

static constexpr pub_chassis_cmd Max_Chassis_cmds = {1.5f, 1.5f, 180.0f};

void RC_Data_Process(){

    //TODO:如果在死区内就强行设置为0

    chassis_cmd_msg.linear_x_ = math_utils::Map(static_cast<float>(rc_data_msg.joyLHori),
         kChannelValueMin, kChannelValueMax,
         -Max_Chassis_cmds.linear_x_, Max_Chassis_cmds.linear_x_);
    chassis_cmd_msg.linear_y_ = math_utils::Map(static_cast<float>(rc_data_msg.joyLVert),
         kChannelValueMin, kChannelValueMax,
         -Max_Chassis_cmds.linear_y_, Max_Chassis_cmds.linear_y_);
    chassis_cmd_msg.deg_omega_ = math_utils::Map(static_cast<float>(rc_data_msg.joyRHori),
    kChannelValueMin, kChannelValueMax, 
    -Max_Chassis_cmds.deg_omega_, Max_Chassis_cmds.deg_omega_);

    math_utils::Clamp(chassis_cmd_msg.linear_x_, -Max_Chassis_cmds.linear_x_, Max_Chassis_cmds.linear_x_);
    math_utils::Clamp(chassis_cmd_msg.linear_y_, -Max_Chassis_cmds.linear_y_, Max_Chassis_cmds.linear_y_);
    math_utils::Clamp(chassis_cmd_msg.deg_omega_, -Max_Chassis_cmds.deg_omega_, Max_Chassis_cmds.deg_omega_);

    

}

void controlTask(void *argument){

    TickType_t current = xTaskGetTickCount();

    for(;;){

        if(rc_data_sub.TryGet(&rc_data_msg)){
            RC_Data_Process();
            chassis_cmd_pub.Publish(chassis_cmd_msg);
        }
        
        vTaskDelayUntil(&current, 5);
    }
}