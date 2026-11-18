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
#include "ramp_planner.hpp"

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


//kp=6.5 位置环输出上限 1.5m/s，所以这里平移加速度留了余量
static constexpr float kMaxAccXY  = 1.5f;     // m/s^2    平移最大加速度
static constexpr float kJerkXY    = 20.0f;    // m/s^3    平移最大加加速度

static constexpr float kMaxAlphaW = 240.0f;   // deg/s^2  转向最大角加速度
static constexpr float kJerkW     = 2000.0f;  // deg/s^3  转向最大角加加速度

static S_RampPlanner ramp_vx{};
static S_RampPlanner ramp_vy{};
static S_RampPlanner ramp_w{};

//twist_target:各控制源算出来的"想要"的旋量      twist_now:经过规划后"真正下发"的旋量
static ChassisTwist twist_target{};
static ChassisTwist twist_now{};



void ChassisForAuto();
void ChassisForManual();

void chassisTask(void *argument){
    
    TickType_t current = xTaskGetTickCount();
    vTaskDelayUntil(&current, 500);

    //规划器的 jerk 默认值是按电流环尺度给的，必须在这里覆盖成底盘速度的单位
    ramp_vx.init(kJerkXY);
    ramp_vy.init(kJerkXY);
    ramp_w.init(kJerkW);

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

            //每 1ms 都推进一次规划，跟"这一拍有没有收到新指令"无关
            //否则规划器的 dt 会变成指令周期(5ms)，jerk 限制等于被放大 5 倍
            twist_now.vx        = ramp_vx.computeOutput(twist_target.vx,        kMaxAccXY);
            twist_now.vy        = ramp_vy.computeOutput(twist_target.vy,        kMaxAccXY);
            twist_now.deg_omega = ramp_w .computeOutput(twist_target.deg_omega, kMaxAlphaW);

            SteerChassis_solver.run(twist_now);
        }

        vTaskDelayUntil(&current, 1);
    }
}

void ChassisForAuto(){
    if(chassis_cmd_msg.cmd_type != ControlMode::AUTOMATIC) return;

    twist_target.vx = 
    PID_Calculate(&pid_x_pos, chassis_FK_pose_msg.x, chassis_cmd_msg.x_tar);
    twist_target.vy = 
    PID_Calculate(&pid_y_pos, chassis_FK_pose_msg.y, chassis_cmd_msg.y_tar);

    const float yaw_now = chassis_FK_pose_msg.yaw_deg;                 // 窗口值 [-180,180)
    const float yaw_ref = yaw_now + math_utils::WrapAngle(
                                    chassis_cmd_msg.yaw_deg - yaw_now, -180.0f, 180.0f);
    twist_target.deg_omega = PID_Calculate(&pid_deg_pos, yaw_now, yaw_ref);
}

void ChassisForManual(){
    if(chassis_cmd_msg.cmd_type != ControlMode::PURELY_MANUAL) return;

    twist_target.vx = chassis_cmd_msg.x_tar;
    twist_target.vy = chassis_cmd_msg.y_tar;

    const float yaw_now = chassis_FK_pose_msg.yaw_deg;                 // 窗口值 [-180,180)
    const float yaw_ref = yaw_now + math_utils::WrapAngle(
                                    chassis_cmd_msg.yaw_deg - yaw_now, -180.0f, 180.0f);
    twist_target.deg_omega = PID_Calculate(&pid_deg_pos, yaw_now, yaw_ref);
}
