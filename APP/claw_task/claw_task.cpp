/**
 * @file claw_task.cpp
 * @brief 双夹爪执行任务：订阅 claw_cmd 并发布 claw_state
 *        - claw1：3508(0x202) + 高擎旧版(ID=1) + 灵足 EL05(ID=2)
 *        - claw2：3508(0x203) + 2006(0x204) + 达妙 DM4310(0x06)
 */

#include "claw_task.hpp"

#include "djiMotor.hpp"
#include "htdwLegacyMotor.hpp"
#include "robstrideMotor.hpp"
#include "dmMotor.hpp"

#include "topics.hpp"
#include "topic_pool.hpp"

extern C620Motor rise_motor;
extern HtdwLegacyMotor rotate_motor;
extern EL05Motor grip_motor;

extern C620Motor rise2_motor;
extern C610Motor rotate2_motor;
extern DM4310Motor grip2_motor;

osThreadId_t Claw_TaskHandle;

static TypedTopicPublisher<pub_claw_state> claw_state_pub("claw_state");
static TypedTopicSubscriber<pub_claw_cmd> claw_cmd_sub("claw_cmd", 8U);

pub_claw_state claw_state_msg{};
pub_claw_cmd claw_cmd_msg{};

static constexpr float kRotateMaxSpeedDegS = 360.0f;
static constexpr float kRotateMaxTorqueNm  = 10.0f;
static constexpr float kGripKp             = 5.0f;
static constexpr float kGripKd             = 0.5f;
static constexpr float kGrip2MaxSpeedDegS  = 180.0f;
static constexpr float kGrip2MinDeg        = -189.0f;
static constexpr float kGrip2MaxDeg        = 0.0f;

void clawTask(void *argument) {
    (void)argument;
    TickType_t current = xTaskGetTickCount();

    for (;;) {
        while (claw_cmd_sub.TryGet(&claw_cmd_msg)) {
        }

        // -------------------- 采集两套夹爪状态并统一发布 --------------------
        claw_state_msg.claw1.rise_ready   = (rise_motor.getLastRxTick() != 0U);
        claw_state_msg.claw1.rotate_ready = rotate_motor.hasFeedback();
        claw_state_msg.claw1.grip_ready   = (grip_motor.getLastRxTick() != 0U);
        claw_state_msg.claw1.rise_deg     = rise_motor.getSumDeg();
        claw_state_msg.claw1.rotate_deg   = rotate_motor.getSumDeg();
        claw_state_msg.claw1.grip_deg     = grip_motor.getSumDeg();

        claw_state_msg.claw2.rise_ready   = (rise2_motor.getLastRxTick() != 0U);
        claw_state_msg.claw2.rotate_ready = (rotate2_motor.getLastRxTick() != 0U);
        claw_state_msg.claw2.grip_ready   = (grip2_motor.getLastRxTick() != 0U);
        claw_state_msg.claw2.rise_deg     = rise2_motor.getSumDeg();
        claw_state_msg.claw2.rotate_deg   = rotate2_motor.getSumDeg();
        claw_state_msg.claw2.grip_deg     = grip2_motor.getSumDeg();

        claw_state_pub.Publish(claw_state_msg);

        // -------------------- 夹爪 1 执行 --------------------
        if (!claw_cmd_msg.claw1.active) {
            rise_motor.setDeg(claw_state_msg.claw1.rise_deg);
            rotate_motor.setQueryOnly();
            grip_motor.setMIT(claw_state_msg.claw1.grip_deg, 0.0f, 0.0f);
        } else {
            rise_motor.setDeg(claw_cmd_msg.claw1.rise_deg);

            if (rotate_motor.hasFeedback()) {
                rotate_motor.setPosVelTqe(claw_cmd_msg.claw1.rotate_deg,
                                          kRotateMaxSpeedDegS,
                                          kRotateMaxTorqueNm);
            } else {
                rotate_motor.setQueryOnly();
            }

            grip_motor.setMIT(claw_cmd_msg.claw1.grip_deg, kGripKp, kGripKd);
        }

        // -------------------- 夹爪 2 执行 --------------------
        if (!claw_cmd_msg.claw2.active) {
            rise2_motor.setDeg(claw_state_msg.claw2.rise_deg);
            rotate2_motor.setDeg(claw_state_msg.claw2.rotate_deg);
            grip2_motor.setDeg(claw_state_msg.claw2.grip_deg);
            grip2_motor.setDegSpeed(kGrip2MaxSpeedDegS);
        } else {
            float grip2_target_deg = claw_cmd_msg.claw2.grip_deg;
            math_utils::Clamp(grip2_target_deg, kGrip2MinDeg, kGrip2MaxDeg);

            rise2_motor.setDeg(claw_cmd_msg.claw2.rise_deg);
            rotate2_motor.setDeg(claw_cmd_msg.claw2.rotate_deg);
            grip2_motor.setDeg(grip2_target_deg);
            grip2_motor.setDegSpeed(kGrip2MaxSpeedDegS);
        }

        vTaskDelayUntil(&current, 5);
    }
}

