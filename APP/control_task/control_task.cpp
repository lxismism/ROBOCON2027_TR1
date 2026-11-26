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

static TypedTopicPublisher<pub_claw_cmd> claw_cmd_pub("claw_cmd");
static pub_claw_cmd claw_cmd_msg{};
static TypedTopicSubscriber<pub_claw_state> claw_state_sub("claw_state", 8U);
static pub_claw_state claw_state_msg{};
static bool claw_activated{false};
static bool rise_synced{false};
static bool rotate_synced{false};
static bool grip_synced{false};
static bool claw2_activated{false};
static bool rise2_synced{false};
static bool rotate2_synced{false};
static bool grip2_synced{false};

static constexpr float kRiseStickSpeedDegS    = 360.0f;
static constexpr float kRotateStickSpeedDegS  = 90.0f;
static constexpr float kGripStickSpeedDegS    = 90.0f;
static constexpr float kRotateMinDeg          = -180.0f;
static constexpr float kRotateMaxDeg          = 180.0f;
static constexpr float kGripMinDeg            = -90.0f;
static constexpr float kGripMaxDeg            = 170.0f;

static constexpr float kRise2StickSpeedDegS   = 360.0f;
static constexpr float kRotate2StickSpeedDegS = 180.0f;
static constexpr float kGrip2StickSpeedDegS   = 90.0f;
static constexpr float kGrip2MinDeg           = -189.0f;
static constexpr float kGrip2MaxDeg           = 0.0f;

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
void Claw_RC_Process(float dt);
void nextStep();
void resetStep();

//为了避免冗长，这个专门用来让这个任务知晓各个机构的姿态
void TrygetPoses();

//这个为摇杆的映射，我为了不写很多次就写成这个函数
static float mapStickAxis(uint16_t raw, float max_speed) {
    const float val = static_cast<float>(raw);
    if (std::abs(val - kChannelValueMid) < kJoyDeadBand) {
        return 0.0f;
    }
    return math_utils::Map(val, kChannelValueMin, kChannelValueMax, -max_speed, max_speed);
}

static void syncClawInitialPoses() {
    if (!claw_activated) {
        if (!rise_synced && claw_state_msg.claw1.rise_ready) {
            claw_cmd_msg.claw1.rise_deg = claw_state_msg.claw1.rise_deg;
            rise_synced = true;
        }
        if (!rotate_synced && claw_state_msg.claw1.rotate_ready) {
            claw_cmd_msg.claw1.rotate_deg = claw_state_msg.claw1.rotate_deg;
            math_utils::Clamp(claw_cmd_msg.claw1.rotate_deg, kRotateMinDeg, kRotateMaxDeg);
            rotate_synced = true;
        }
        if (!grip_synced && claw_state_msg.claw1.grip_ready) {
            claw_cmd_msg.claw1.grip_deg = claw_state_msg.claw1.grip_deg;
            math_utils::Clamp(claw_cmd_msg.claw1.grip_deg, kGripMinDeg, kGripMaxDeg);
            grip_synced = true;
        }
    }

    if (!claw2_activated) {
        if (!rise2_synced && claw_state_msg.claw2.rise_ready) {
            claw_cmd_msg.claw2.rise_deg = claw_state_msg.claw2.rise_deg;
            rise2_synced = true;
        }
        if (!rotate2_synced && claw_state_msg.claw2.rotate_ready) {
            claw_cmd_msg.claw2.rotate_deg = claw_state_msg.claw2.rotate_deg;
            rotate2_synced = true;
        }
        if (!grip2_synced && claw_state_msg.claw2.grip_ready) {
            claw_cmd_msg.claw2.grip_deg = claw_state_msg.claw2.grip_deg;
            math_utils::Clamp(claw_cmd_msg.claw2.grip_deg, kGrip2MinDeg, kGrip2MaxDeg);
            grip2_synced = true;
        }
    }
}

void controlTask(void *argument){

    TickType_t current = xTaskGetTickCount();

    for(;;){

        const float dt = DWT_GetDeltaT(&dwt_cnt);
        
        while(claw_state_sub.TryGet(&claw_state_msg)){
        }
        syncClawInitialPoses();

        if(dt < kMaxDt){
            if(rc_data_sub.TryGet(&rc_data_msg)){
                RC_Data_Process(dt);
                chassis_cmd_pub.Publish(chassis_cmd_msg);
                claw_cmd_pub.Publish(claw_cmd_msg);
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
    Claw_RC_Process(dt);

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

void Claw_RC_Process(float dt){
    syncClawInitialPoses();

    if(rc_data_msg.swA != RC_2_Pos_SW_State_t::DOWN){
        return;
    }

    if(rc_data_msg.swB == RC_3_Pos_SW_State_t::UP){
        if(!claw_activated){
            claw_activated = true;
            claw_cmd_msg.claw1.active = true;
        }

        claw_cmd_msg.claw1.rise_deg   -= mapStickAxis(rc_data_msg.joyLVert, kRiseStickSpeedDegS) * dt;
        claw_cmd_msg.claw1.rotate_deg += mapStickAxis(rc_data_msg.joyRVert, kRotateStickSpeedDegS) * dt;
        claw_cmd_msg.claw1.grip_deg   += mapStickAxis(rc_data_msg.joyRHori, kGripStickSpeedDegS) * dt;

        math_utils::Clamp(claw_cmd_msg.claw1.rotate_deg, kRotateMinDeg, kRotateMaxDeg);
        math_utils::Clamp(claw_cmd_msg.claw1.grip_deg, kGripMinDeg, kGripMaxDeg);
    } else if(rc_data_msg.swB == RC_3_Pos_SW_State_t::DOWN){
        if(!claw2_activated){
            claw2_activated = true;
            claw_cmd_msg.claw2.active = true;
        }

        claw_cmd_msg.claw2.rise_deg   -= mapStickAxis(rc_data_msg.joyLVert, kRise2StickSpeedDegS) * dt;
        claw_cmd_msg.claw2.rotate_deg += mapStickAxis(rc_data_msg.joyRVert, kRotate2StickSpeedDegS) * dt;
        claw_cmd_msg.claw2.grip_deg   += mapStickAxis(rc_data_msg.joyRHori, kGrip2StickSpeedDegS) * dt;

        math_utils::Clamp(claw_cmd_msg.claw2.grip_deg, kGrip2MinDeg, kGrip2MaxDeg);
    }
    // swB == MIDDLE 时暂停更新，两套夹爪均保持当前目标角度
}

void TrygetPoses(){
    (void)chassis_FK_pose_sub.TryGet(&chassis_FK_pose_msg);
    while(claw_state_sub.TryGet(&claw_state_msg)){
    }
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