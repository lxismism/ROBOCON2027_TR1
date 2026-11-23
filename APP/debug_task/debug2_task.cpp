/**
 * @file debug_task2.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "debug1_task.h"
#include "debug2_task.h"
#include "topics.hpp"
#include "usart.h"
#include "com_config.h"
#include "chassis_solution.hpp"
#include <stdio.h>
#include "dmMotor.hpp"
#include "math_utils.hpp"
#include "pid_controller.hpp"

osThreadId_t Debug2_TaskHandle;

extern VESCMotor chassis_drivemotor1;
extern VESCMotor chassis_drivemotor2;
extern VESCMotor chassis_drivemotor3;
extern VESCMotor chassis_drivemotor4;

extern C620Motor chassis_dirmotor1;
extern C620Motor chassis_dirmotor2;
extern C620Motor chassis_dirmotor3;
extern C620Motor chassis_dirmotor4;

extern PhotoGate chassis_photogate1;
extern PhotoGate chassis_photogate2;
extern PhotoGate chassis_photogate3;
extern PhotoGate chassis_photogate4;

extern DM4310Motor wrist_motor;
extern C610Motor extend_motor;
extern C620Motor lift_motor;

extern PID_t pid_x_pos;        //在 chassis_task.cpp 中定义
extern PID_t pid_y_pos;
extern PID_t pid_deg_pos;


static TypedTopicSubscriber<pub_chassis_FK_pose> chassis_FK_pose_sub("chassis_FK_pose", 8U);
static pub_chassis_FK_pose chassis_FK_pose_msg{};

static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8U);
static pub_chassis_cmd chassis_cmd_msg{};

uint32_t rising{};
uint32_t falling{};

void printChassisCurse();


// float mit_ref_deg{};
// float mit_ref_speed{};
// float mit_ref_torque{};

void debug2Task(void *argument){
    TickType_t currentTime = xTaskGetTickCount();

    for(;;){

        (void)chassis_FK_pose_sub.TryGet(&chassis_FK_pose_msg);
        (void)chassis_cmd_sub.TryGet(&chassis_cmd_msg);
        printChassisCurse();
        vTaskDelayUntil(&currentTime, 1);
    }

}

// void printChassisCurse(){
    
//     printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",chassis_FK_pose_msg.x, chassis_FK_pose_msg.y, chassis_FK_pose_msg.yaw_deg,chassis_cmd_msg.x_tar, chassis_cmd_msg.y_tar, chassis_cmd_msg.yaw_deg);
// }

void printChassisCurse(){

    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"      //1~8   : 四个舵向电机速度环，Ref、Measure 交替
           "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"      //9~16  : 四个舵向电机角度环，Ref、Measure 交替
           "%.3f,%.3f,"                                     //17~18 : 底盘角度环
           "%.3f,%.3f,"                                     //19~20 : x位置环
           "%.3f,%.3f\r\n"                                  //21~22 : y位置环
        ,chassis_dirmotor1.getSpeedPid().Ref      //速度环目标值
        ,chassis_dirmotor1.getSpeedPid().Measure
        ,chassis_dirmotor2.getSpeedPid().Ref
        ,chassis_dirmotor2.getSpeedPid().Measure
        ,chassis_dirmotor3.getSpeedPid().Ref
        ,chassis_dirmotor3.getSpeedPid().Measure
        ,chassis_dirmotor4.getSpeedPid().Ref
        ,chassis_dirmotor4.getSpeedPid().Measure

        ,chassis_dirmotor1.getDegPid().Ref
        ,chassis_dirmotor1.getDegPid().Measure
        ,chassis_dirmotor2.getDegPid().Ref
        ,chassis_dirmotor2.getDegPid().Measure
        ,chassis_dirmotor3.getDegPid().Ref
        ,chassis_dirmotor3.getDegPid().Measure
        ,chassis_dirmotor4.getDegPid().Ref
        ,chassis_dirmotor4.getDegPid().Measure

        ,pid_deg_pos.Ref
        ,pid_deg_pos.Measure

        ,pid_x_pos.Ref
        ,pid_x_pos.Measure

        ,pid_y_pos.Ref
        ,pid_y_pos.Measure
    );
}


void printDirDegSpeedCurse(){

    printf("%.3f,%.3f,%.3f,%.3f,"
           "%.3f,%.3f,%.3f,%.3f\r\n"
        ,chassis_dirmotor1.getRefDegSpeed()
        ,chassis_dirmotor2.getRefDegSpeed()
        ,chassis_dirmotor3.getRefDegSpeed()
        ,chassis_dirmotor4.getRefDegSpeed()

        ,chassis_dirmotor1.getDegSpeed()
        ,chassis_dirmotor2.getDegSpeed()
        ,chassis_dirmotor3.getDegSpeed()
        ,chassis_dirmotor4.getDegSpeed()
    );

}

void printDirDegCurse(){

    printf("%.3f,%.3f,%.3f,%.3f,"
           "%.3f,%.3f,%.3f,%.3f\r\n"
        ,chassis_dirmotor1.getRefDeg()
        ,chassis_dirmotor2.getRefDeg()
        ,chassis_dirmotor3.getRefDeg()
        ,chassis_dirmotor4.getRefDeg()

        ,chassis_dirmotor1.getSumDeg()
        ,chassis_dirmotor2.getSumDeg()
        ,chassis_dirmotor3.getSumDeg()
        ,chassis_dirmotor4.getSumDeg()
    );

}

void printDriveCurse(){

    printf("%.3f,%.3f,%.3f,%.3f,"
           "%.3f,%.3f,%.3f,%.3f\r\n"
        ,chassis_drivemotor1.getRefDegSpeed()
        ,chassis_drivemotor2.getRefDegSpeed()
        ,chassis_drivemotor3.getRefDegSpeed()
        ,chassis_drivemotor4.getRefDegSpeed()

        ,chassis_drivemotor1.getDegSpeed()
        ,chassis_drivemotor2.getDegSpeed()
        ,chassis_drivemotor3.getDegSpeed()
        ,chassis_drivemotor4.getDegSpeed()
    );

}