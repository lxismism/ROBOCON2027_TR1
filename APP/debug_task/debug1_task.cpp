/**
 * @file debug_task1.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief APP调试层，专门用来测试程序
 * @version 0.1
 * @date 2026-09-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "debug1_task.h"
#include "cmsis_os2.h"
#include "memory_map.h"
#include "double_buffer.hpp"
#include "lockfree_queue.hpp"
#include "UartPort.hpp"
#include "topics.hpp"

#include "Canbus.hpp"
#include "MotorBase.hpp"
#include "djiMotor.hpp"
#include "dmMotor.hpp"
#include "vescMotor.hpp"
#include "hightorqueMotor.hpp"
#include "htDebug.hpp"

#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.hpp"
#include "chassis_solution.hpp"
#include "chassis_task.h"
#include "math_utils.hpp"
#include "photogate.hpp"
#include "host_comm.hpp"

#include "UsbPort.hpp"
#include "UsbPortC.h"
#include <stdio.h>
#include <stdarg.h>
#include "usart.h"             /* huart4：调试口 115200 / huart3：遥控器口 420000 */
#include "fdcan.h"
osThreadId_t Debug1_TaskHandle;

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data{};

extern HightorqueMotor high_torque_motor;
extern CanTraceDevice fdcan3_trace;
extern volatile float   ht_target_deg_s;
extern volatile float   ht_target_deg;
extern volatile uint8_t ht_cmd_ack;
static TypedTopicSubscriber<pub_vision_pose> vision_pose_sub("vision_pose", 8U);
/* ==================== 高擎电机测试参数（只改这里） ====================
 * 8 字节经典帧，走 fdcan3，不影响达妙 / 大疆电机。
 *   m<度/秒>   速度模式：以该速度持续转
 *   p<度>      位置模式：转到绝对角度（电机内部位置环闭环）
 *   x          停转
 * 命令以回车结束。两种模式都不依赖电机回包；位置/速度标量不同，
 * 串口会打印换算后的 count 和 07 07 帧，方便逐字节对照定位问题。
 */
#define HT_BOOT_DEG_PER_SEC     0.0f     /* 上电默认目标速度 = 0（不转）
                                            —— 之前默认 240 度/秒，测几十秒就把
                                            输出轴卷出 ±3.27 圈量程，位置模式随即失效，
                                            所以改成上电静止，用 m 命令启动 */
#define HT_ACCEL_LIMIT_DEG_S2   1800.0f  /* 加速度限幅，0→240 度/秒约 0.13s */
#define HT_POS_SPEED_LIMIT      240.0f   /* 位置模式行走速度上限（07 35 的 val） */
#define HT_PRINT_PERIOD_MS      500U     /* 串口打印周期 */
#define HT_TRACE_PERIOD         (5U)     /* 每 5 次打印跟一次总线快照（2.5s） */
static pub_vision_pose vision_pose_msg{};
/* 调试输出：主口 UART4（PA0/PA1，115200，与 printf 同口），同时在 USART3
   回显一份（TR1 里该口是 420000，接了遥控器也能看）。用 HAL_UART_Transmit
   直接发，不依赖 printf 重定向。 */
static void htLog(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(buf) - 1) {
            n = (int)sizeof(buf) - 1;
        }
        HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)n, 50);
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 50);
    }
}

/* hex 打印：8 字节 → "07 07 00 80 6B 0A 00 80"（23 字符 + '\0'） */
static void hexStr(char *out, const uint8_t *d, uint8_t n)
{
    static const char kHex[] = "0123456789ABCDEF";
    uint8_t k = 0;
    for (uint8_t i = 0; i < n; ++i) {
        out[k++] = kHex[(d[i] >> 4) & 0x0FU];
        out[k++] = kHex[d[i] & 0x0FU];
        out[k++] = ' ';
    }
    if (k > 0U) {
        k--;
    }
    out[k] = '\0';
}

static const char *lecToStr(uint32_t lec)
{
    switch (lec) {
        case 0:  return "NONE";
        case 1:  return "STUFF";
        case 2:  return "FORM";
        case 3:  return "ACK";   /* 没人应答：只有自己在线 / 波特率不对 / 没接终端电阻 */
        case 4:  return "BIT1";
        case 5:  return "BIT0";
        case 6:  return "CRC";
        default: return "OTHER";
    }
}

static void printFdcan3Status(void)
{
    const uint32_t psr = hfdcan3.Instance->PSR;
    const uint32_t ecr = hfdcan3.Instance->ECR;
    const uint32_t lec = psr & 0x07U;
    const uint32_t tec = ecr & 0xFFU;
    const uint32_t rec = (ecr >> 8) & 0x7FU;

    htLog("[HT] C3 LEC=%s(%lu) TEC=%lu REC=%lu n=%lu\r\n",
          lecToStr(lec), (unsigned long)lec,
          (unsigned long)tec, (unsigned long)rec,
          (unsigned long)fdcan3_trace.count());
}

/* 最近 3 帧总线内容：确认达妙/大疆还在、确认电机回包 ID */
static void printFdcan3Trace(void)
{
    const uint32_t n = fdcan3_trace.count();
    if (n == 0U) {
        return;
    }

    const uint32_t depth = (n < 3U) ? n : 3U;
    for (uint32_t i = 0; i < depth; ++i) {
        const CanTraceDevice::Slot &s = fdcan3_trace.get(i);
        char hex[3U * 8U + 1U];
        hexStr(hex, s.data, s.len);
        htLog("[HT] BUS id=%04lX %c %uB | %s\r\n",
              (unsigned long)s.id, s.ext ? 'E' : 'S', (unsigned)s.len, hex);
    }
}

void debug1Task(void *argument)
{
    TickType_t currentTime = xTaskGetTickCount();
    (void)argument;

    /* 模式：1 = 速度，2 = 位置（由串口命令切换） */
    uint8_t want_mode = 1U;
    uint32_t print_cnt = 0;
    uint32_t last_tx = 0;
    uint32_t last_rx = 0;

    ht_target_deg_s = HT_BOOT_DEG_PER_SEC;
    high_torque_motor.setAccelLimit(HT_ACCEL_LIMIT_DEG_S2);
    high_torque_motor.setPositionSpeedLimit(HT_POS_SPEED_LIMIT);
    high_torque_motor.setTargetSpeed(ht_target_deg_s);

    htLog("\r\n[HT] boot: classic8 id=1 proto=07-07 boot=%ld deg/s accel=%ld\r\n",
          (long)HT_BOOT_DEG_PER_SEC, (long)HT_ACCEL_LIMIT_DEG_S2);
    htLog("[HT] cmd: m<deg/s> SPEED | p<deg> ABS-POS | x STOP | z REZERO | w SAVE\r\n");
    htLog("[HT] port: UART4 115200 (PA0/PA1, all cmds) | USART3 420000 (m/p only)\r\n");
    htLog("[HT] eg:  m120 -> 120 deg/s | p50 -> go to 50 deg | z -> set here as 0\r\n");

    for(;;)
    {
        imu_data_sub.TryGet(&imu_data);
        vision_pose_sub.TryGet(&vision_pose_msg);
        /* 串口命令回执：打印换算后的 count + 预期帧，方便逐字节对照定位问题 */
        const uint8_t ack = ht_cmd_ack;
        if (ack != 0U) {
            ht_cmd_ack = 0U;
            if (ack == 1U) {
                const long velcnt = (long)(ht_target_deg_s * HightorqueMotor::kDegS2Vel);
                want_mode = 1U;
                htLog("[CMD] m -> SPEED tgt=%ld deg/s velcnt=%ld %s | 07 07 00 80 <vel> 00 80\r\n",
                      (long)ht_target_deg_s, velcnt,
                      ((velcnt > 32767L) || (velcnt < -32767L)) ? "CLIP" : "ok");
            } else if (ack == 2U) {
                const long poscnt = (long)(ht_target_deg * HightorqueMotor::kDeg2Pos);
                want_mode = 2U;
                htLog("[CMD] p -> POS abs_tgt=%ld deg poscnt=%ld %s | 07 35 <vlim> 00 80 <pos>\r\n",
                      (long)ht_target_deg, poscnt,
                      ((poscnt > 32767L) || (poscnt < -32767L)) ? "CLIP" : "ok");
            } else if (ack == 3U) {
                want_mode = 1U;
                htLog("[CMD] x -> STOP\r\n");
            } else if (ack == 5U) {
                /* z：停机 → 以当前位置为 0 点。回零后把目标也归 0，避免立刻又跑 */
                want_mode = 2U;
                ht_target_deg = 0.0f;
                high_torque_motor.requestRezero(false);
                htLog("[CMD] z -> REZERO (here = 0 deg, RAM only)\r\n");
            } else if (ack == 6U) {
                want_mode = 2U;
                ht_target_deg = 0.0f;
                high_torque_motor.requestRezero(true);
                htLog("[CMD] w -> REZERO + SAVE to flash\r\n");
            } else {
                htLog("[CMD] bad, use m120 / p50 / x / z / w\r\n");
            }
        }

        /* 唯一的下发点：每 5ms 刷新一次目标，斜坡在 update() 里按真实时间做 */
        if (want_mode == 1U) {
            high_torque_motor.setTargetSpeed(ht_target_deg_s);
        } else {
            high_torque_motor.setTargetPosition(ht_target_deg);
        }

        if (++print_cnt >= (HT_PRINT_PERIOD_MS / 5U)) {
            print_cnt = 0U;

            /* 1) 最后一条「控制帧」07 07 ...（读状态帧 17 01 不会冲掉它） */
            {
                char hex[3U * 8U + 1U];
                hexStr(hex, high_torque_motor.getLastCtrlData(), 8U);
                htLog("[HT] CTRL id=%04lX %c | %s\r\n",
                      (unsigned long)high_torque_motor.getLastCtrlId(),
                      high_torque_motor.isLastCtrlExt() ? 'E' : 'S', hex);
            }

            /* 2) 收到的最后一帧 */
            if (high_torque_motor.getRxFrames() == 0U) {
                htLog("[HT] R <NO RX FRAME YET>\r\n");
            } else {
                char hex[3U * 8U + 1U];
                hexStr(hex, high_torque_motor.getLastRxData(),
                       high_torque_motor.getLastRxLen());
                htLog("[HT] R id=%04lX %c | %s\r\n",
                      (unsigned long)high_torque_motor.getLastRxId(),
                      high_torque_motor.isLastRxExt() ? 'E' : 'S', hex);
            }

            /* 3) 电机状态（全部整数，避免 %f 打空） */
            {
                const uint32_t tx = high_torque_motor.getTxFrames();
                const uint32_t rx = high_torque_motor.getRxFrames();
                htLog("[HT] %s sync=%d tx=%lu(+%lu) rx=%lu(+%lu) fb=%lu fail=%lu\r\n",
                      high_torque_motor.getStateName(),
                      (int)high_torque_motor.isSynced(),
                      (unsigned long)tx, (unsigned long)(tx - last_tx),
                      (unsigned long)rx, (unsigned long)(rx - last_rx),
                      (unsigned long)high_torque_motor.getFbFrames(),
                      (unsigned long)high_torque_motor.getTxFail());
                last_tx = tx;
                last_rx = rx;

                /* 按当前模式打印「真正下发的量」：位置看角度/poscnt，
                   速度看速度/velcnt；反馈量带在后面仅供对照（rx=0 时为 0）。 */
                if (high_torque_motor.getMode() == HightorqueMotor::CtrlMode::Position) {
                    /* 位置模式：pos32 是 int32 真实位置（能超出 ±3.27 圈），
                       rem 是「还差多少度」—— 正数往正方向走，接近 0 就是快到位了 */
                    if (high_torque_motor.isPos32Valid()) {
                        htLog("[HT] POS tgt=%ld poscnt=%ld | pos32=%ld mdeg rem=%ld mdeg\r\n",
                              (long)high_torque_motor.getTargetDeg(),
                              (long)high_torque_motor.getCmdPosCount(),
                              (long)(high_torque_motor.getPos32Deg() * 1000.0f),
                              (long)(high_torque_motor.getRemainDeg() * 1000.0f));
                    } else {
                        htLog("[HT] POS tgt=%ld poscnt=%ld | fb16=%ld mdeg pos32=NONE\r\n",
                              (long)high_torque_motor.getTargetDeg(),
                              (long)high_torque_motor.getCmdPosCount(),
                              (long)(high_torque_motor.getFbPosDeg() * 1000.0f));
                    }
                } else {
                    htLog("[HT] SPD tgt=%ld cmd=%ld velcnt=%ld | fb_mdeg=%ld fb_mdegs=%ld tqe=%d\r\n",
                          (long)high_torque_motor.getTargetDegSpeed(),
                          (long)high_torque_motor.getCmdDegSpeed(),
                          (long)(high_torque_motor.getCmdDegSpeed() * HightorqueMotor::kDegS2Vel),
                          (long)(high_torque_motor.getFbPosDeg() * 1000.0f),
                          (long)(high_torque_motor.getFbVelDegS() * 1000.0f),
                          (int)high_torque_motor.getFbTqeRaw());
                }

                /* int16 位置顶到 ±32767 说明输出轴已经卷出 ±3.27 圈量程，
                   绝对位置指令此刻不可达 —— 先 z 回零（或直接指定当前附近角度） */
                const long fb16 = (long)(high_torque_motor.getFbPosDeg() * 1000.0f);
                if ((fb16 > 1179000L) || (fb16 < -1179000L)) {
                    htLog("[HT] WARN pos16 saturated (%ld) -> send z to rezero\r\n", fb16);
                }
            }

            /* 4) 总线错误计数 */
            printFdcan3Status();

            /* 5) 每 2.5s 跟一次总线快照 */
            static uint8_t trace_div = 0;
            if (++trace_div >= HT_TRACE_PERIOD) {
                trace_div = 0;
                printFdcan3Trace();
            }
        }

        vTaskDelayUntil(&currentTime, 5);
    }

}