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

#include "bsp_dwt.h"

#include "topics.hpp"
#include "topic_pool.hpp"

osThreadId_t Control_TaskHandle;

static TypedTopicSubscriber<pub_rc_data> rc_data_sub("rc_data", 8U);
static pub_rc_data rc_data_msg{};

static TypedTopicPublisher<pub_chassis_cmd> chassis_cmd_pub("chassis_cmd");
static pub_chassis_cmd chassis_cmd_msg{};
static TypedTopicSubscriber<pub_chassis_FK_pose> chassis_FK_pose_sub("chassis_FK_pose", 8U);
static pub_chassis_FK_pose chassis_FK_pose_msg{};

uint32_t dwt_cnt{};
bool state_machine_block{false};
float manual_yaw_deg_speed{};

static constexpr pub_chassis_cmd Max_Chassis_cmds = {ControlMode::PURELY_MANUAL ,1.5f, 1.5f, 180.0f};

static constexpr float kChannelValueMin = static_cast<float>(RmPocket::kChannelValueMin);
static constexpr float kChannelValueMid = static_cast<float>(RmPocket::kChannelValueMid);
static constexpr float kChannelValueMax = static_cast<float>(RmPocket::kChannelValueMax);
static constexpr float kJoyDeadBand = static_cast<float>(RmPocket::kJoyDeadBand);

static constexpr float kMaxDt = 0.02f;

enum class AutomaticState : uint8_t {
    STEP1 = 0,
    STEP2 = 1,
    STEP3 = 2,
    STEP4 = 3,
};
AutomaticState automaticState = AutomaticState::STEP1;

ControlMode control_mode = ControlMode::PURELY_MANUAL;
ControlMode last_control_mode = ControlMode::PURELY_MANUAL;


void RC_Data_Process(float dt);
void AutomaticProcess(float dt);
void PurelyManualProcess(float dt);
void nextStep();
void resetStep();

//为了避免冗长，这个专门用来让这个任务知晓各个机构的姿态
void TrygetPoses();

void controlTask(void *argument){

    TickType_t current = xTaskGetTickCount();

    for(;;){

        const float dt = DWT_GetDeltaT(&dwt_cnt);
        
        if(dt < kMaxDt){
            if(rc_data_sub.TryGet(&rc_data_msg)){
                RC_Data_Process(dt);
                chassis_cmd_pub.Publish(chassis_cmd_msg);
            }
        }
        
        vTaskDelayUntil(&current, 5);
    }
}

void RC_Data_Process(float dt){

    last_control_mode = control_mode;
    control_mode = (rc_data_msg.swD == RC_2_Pos_SW_State_t::DOWN) ?
        ControlMode::AUTOMATIC :
        ControlMode::PURELY_MANUAL;
    chassis_cmd_msg.cmd_type = control_mode;

    TrygetPoses();

    switch (control_mode) {
        case ControlMode::AUTOMATIC     : {
            AutomaticProcess(dt);
        } break;
        case ControlMode::PURELY_MANUAL : {
            PurelyManualProcess(dt);
        } break;
        
    }

}

void AutomaticProcess(float dt){

    

    if(last_control_mode != ControlMode::AUTOMATIC){
        resetStep();
    }
    if(rc_data_msg.swE == RC_BTN_SW_State_t::PRESSED &&
        rc_data_msg.swE != rc_data_msg.last_swE){
        nextStep();
    } 
    
    if(!state_machine_block){

        switch (automaticState) {

            case AutomaticState::STEP1 : {
                chassis_cmd_msg.x_tar = chassis_FK_pose_msg.x;
                chassis_cmd_msg.y_tar = chassis_FK_pose_msg.y;
                chassis_cmd_msg.yaw_deg = chassis_FK_pose_msg.yaw_deg;
            } break;
            case AutomaticState::STEP2 : {
                chassis_cmd_msg.x_tar -= 0.5f;
            } break;
            case AutomaticState::STEP3 : {
                chassis_cmd_msg.y_tar += 0.5f;
            } break; 
            case AutomaticState::STEP4 : {
                chassis_cmd_msg.yaw_deg += 90.0f;
            } break;

        }
        state_machine_block = true;

    }

}

void PurelyManualProcess(float dt){
    chassis_cmd_msg.x_tar = 
    std::abs(static_cast<float>(rc_data_msg.joyLHori) - kChannelValueMid) < kJoyDeadBand ? 
    0.0f :
    math_utils::Map(static_cast<float>(rc_data_msg.joyLHori),
        kChannelValueMin, kChannelValueMax,
        -Max_Chassis_cmds.x_tar, Max_Chassis_cmds.x_tar);
    chassis_cmd_msg.y_tar = 
    std::abs(static_cast<float>(rc_data_msg.joyLVert) - kChannelValueMid) < kJoyDeadBand ? 
    0.0f :
    math_utils::Map(static_cast<float>(rc_data_msg.joyLVert),
        kChannelValueMin, kChannelValueMax,
        -Max_Chassis_cmds.y_tar, Max_Chassis_cmds.y_tar);

    manual_yaw_deg_speed = 
    std::abs(static_cast<float>(rc_data_msg.joyRHori) - kChannelValueMid) < kJoyDeadBand ? 
    0.0f :
    math_utils::Map(static_cast<float>(rc_data_msg.joyRHori),
        kChannelValueMin, kChannelValueMax,
        -Max_Chassis_cmds.yaw_deg, Max_Chassis_cmds.yaw_deg);

    math_utils::Clamp(chassis_cmd_msg.x_tar, -Max_Chassis_cmds.x_tar, Max_Chassis_cmds.x_tar);
    math_utils::Clamp(chassis_cmd_msg.y_tar, -Max_Chassis_cmds.y_tar, Max_Chassis_cmds.y_tar);
    math_utils::Clamp(manual_yaw_deg_speed, -Max_Chassis_cmds.yaw_deg, Max_Chassis_cmds.yaw_deg);

    chassis_cmd_msg.yaw_deg += manual_yaw_deg_speed * dt;



} 

void TrygetPoses(){
    (void)chassis_FK_pose_sub.TryGet(&chassis_FK_pose_msg);
}

void nextStep(){
    switch (automaticState) {
        
        case AutomaticState::STEP1 : {
            automaticState = AutomaticState::STEP2;          
        } break;
        case AutomaticState::STEP2 : {
            automaticState = AutomaticState::STEP3;          
        } break;
        case AutomaticState::STEP3 : {
            automaticState = AutomaticState::STEP4;          

        } break; 
        case AutomaticState::STEP4 : {
            automaticState = AutomaticState::STEP1;          
        } break;
                 
    }

    state_machine_block = false;

}

void resetStep(){
    automaticState = AutomaticState::STEP1;
    state_machine_block = false;
}