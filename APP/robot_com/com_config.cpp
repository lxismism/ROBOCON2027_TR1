/**
 * @file com_config.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "com_config.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "gpio.h"
#include "usart.h"
#include "fdcan.h"

#include "memory_map.h"
#include "UartPort.hpp"
#include "WitMotionimu.hpp"
#include "rm_pocket.hpp"

#include "UsbPort.hpp"
#include "host_comm.hpp"
#include "host_msgs.hpp"

#include "Canbus.hpp"
#include "MotorBase.hpp"
#include "djiMotor.hpp"
#include "vescMotor.hpp"
#include "dmMotor.hpp"
#include "robstrideMotor.hpp"

#include "topics.hpp"
#include "topic_pool.hpp"
#include <array>
#include "chassis_solution.hpp"

/*-------------------------------------gpio----------------------------------------*/
PhotoGate chassis_photogate1(PHOTOGATE1_GPIO_Port, PHOTOGATE1_Pin);
PhotoGate chassis_photogate2(PHOTOGATE2_GPIO_Port, PHOTOGATE2_Pin);
PhotoGate chassis_photogate3(PHOTOGATE3_GPIO_Port, PHOTOGATE3_Pin);
PhotoGate chassis_photogate4(PHOTOGATE4_GPIO_Port, PHOTOGATE4_Pin);
std::array<PhotoGate*, kWheelNum> chassis_photogates
    {&chassis_photogate1, &chassis_photogate2,
     &chassis_photogate3, &chassis_photogate4};
/*-------------------------------------gpio----------------------------------------*/


/*-------------------------------------fdcan1----------------------------------------*/

osThreadId_t can1Send_TaskHandle;
CanBus fdcan1_bus(hfdcan1);

VESCMotor chassis_drivemotor1(&fdcan1_bus, 101, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_RPM, VESCMotor::CAN_PACKET_STATUS,
    21.0f, 1.0f, 20000.0f, 0.0f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);

VESCMotor chassis_drivemotor2(&fdcan1_bus, 102, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_RPM,VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.0f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);

VESCMotor chassis_drivemotor3(&fdcan1_bus, 103, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_RPM, VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.00f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);

VESCMotor chassis_drivemotor4(&fdcan1_bus, 104, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_RPM, VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.0f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);



std::array<VESCMotor*, 4> chassis_drivemotors{&chassis_drivemotor1, &chassis_drivemotor2,
                                              &chassis_drivemotor3, &chassis_drivemotor4};



/*-------------------------------------fdcan1----------------------------------------*/



/*-------------------------------------fdcan2----------------------------------------*/
osThreadId_t can2Send_TaskHandle;
CanBus fdcan2_bus(hfdcan2);

C620Motor chassis_dirmotor1(&fdcan2_bus, 0x201, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    15.0f, 0.0f, 0.5f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 0.1f);

C620Motor chassis_dirmotor2(&fdcan2_bus, 0x202, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    15.0f, 0.0f, 0.5f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 0.1f);

C620Motor chassis_dirmotor3(&fdcan2_bus, 0x203, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    15.0f, 0.0f, 0.5f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 0.1f);

C620Motor chassis_dirmotor4(&fdcan2_bus, 0x204, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    15.0f, 0.0f, 0.5f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 0.1f);

std::array<C620Motor*, 4> chassis_dirmotors{&chassis_dirmotor1, &chassis_dirmotor2,
                                            &chassis_dirmotor3, &chassis_dirmotor4};
                                            

/*-------------------------------------fdcan2----------------------------------------*/



/*-------------------------------------fdcan3----------------------------------------*/



osThreadId_t can3Send_TaskHandle;

CanBus fdcan3_bus(hfdcan3);

DM4310Motor wrist_motor(&fdcan3_bus, 0x06, 0x06, DM4310Motor::PosWithSpeed);

C610Motor extend_motor(&fdcan3_bus, 0x202, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 36.0f,10000.0f, 0.005f,
    16.0f, 5.0f, 0.0f, 10000.0f, 10000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f,
    5.0f);


C620Motor lift_motor(&fdcan3_bus, 0x201, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 3591.0f / 187.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    20.0f, 0.0f, 0.0f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 1.0f);

EL05Motor grip_motor1(&fdcan3_bus, 0x7F,0xFD,0.0f,0.0f,6.0f);
EL05Motor grip_motor2(&fdcan3_bus, 0x7E, 0xFD, 0.0f, 0.01f, 6.0f);
/*================ 灵足夹爪限位参数（每次改零点要重新测！）================*/
static constexpr float kGrip1LimitMinDeg = -1.0f;  
static constexpr float kGrip1LimitMaxDeg =  370.0f; 
static constexpr float kGrip2LimitMinDeg = -3.0f;   
static constexpr float kGrip2LimitMaxDeg =  365.0f;   

/*-------------------------------------fdcan3----------------------------------------*/


/*-------------------------------------usart2----------------------------------------*/
osThreadId_t uart2Process_TaskHandle;

void onUart2RxCb(const uint8_t *data, size_t len, void *user);

DMA_BUFFER_ATTR static uint8_t uart2_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart2_tx_dma[64];


UartPort uart2_port(&huart2, uart2_rx_dma, sizeof(uart2_rx_dma),
                    uart2_tx_dma, sizeof(uart2_tx_dma), onUart2RxCb, nullptr);

osSemaphoreId_t uart2_rx_semaphore = NULL;

WitMotionimu wit_imu{};

static TypedTopicPublisher<pub_imu_data> imu_data_pub("imu_data");
static pub_imu_data imu_data{};
/*-------------------------------------usart2----------------------------------------*/


/*-------------------------------------usart3----------------------------------------*/
osThreadId_t uart3Process_TaskHandle;

void onUart3RxCb(const uint8_t *data, size_t len, void *user);

DMA_BUFFER_ATTR static uint8_t uart3_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart3_tx_dma[64];


UartPort uart3_port(&huart3, uart3_rx_dma, sizeof(uart3_rx_dma),
                    uart3_tx_dma, sizeof(uart3_tx_dma), onUart3RxCb, nullptr);

osSemaphoreId_t uart3_rx_semaphore = NULL;

RmPocket rm_pocket{};

static TypedTopicPublisher<pub_rc_data> rc_data_pub("rc_data");
static pub_rc_data rc_data_msg{};

/*-------------------------------------usart3----------------------------------------*/


/*--------------------------------------usb------------------------------------------*/

osThreadId_t usbProcess_TaskHandle;

osSemaphoreId_t usb_semaphore = NULL;

void onUsbCb(void *user);


HostComm host_comm{};
VisionPose host_visionPose_data{};
volatile uint32_t host_frame_ok{};//用来看有没有丢包的，如果跟seq相等，就是没丢。
void onVisionPoseMsg(const uint8_t *data, uint8_t len, void *user);

static TypedTopicPublisher<pub_vision_pose> vision_pose_pub("vision_pose");
static pub_vision_pose vision_pose_msg{};


/*--------------------------------------usb------------------------------------------*/


uint8_t comServiceInit(){

    canFilterInit(&hfdcan1, FDCAN_EXTENDED_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan1);

    canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan2);

    canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    canFilterInit(&hfdcan3, FDCAN_EXTENDED_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    bspCanInit(&hfdcan3);

    fdcan1_bus.init();
    fdcan2_bus.init();
    fdcan3_bus.init();

    fdcan1_bus.registerDevice(&chassis_drivemotor1);
    fdcan1_bus.registerDevice(&chassis_drivemotor2);
    fdcan1_bus.registerDevice(&chassis_drivemotor3);
    fdcan1_bus.registerDevice(&chassis_drivemotor4);

    fdcan2_bus.registerDevice(&chassis_dirmotor1);
    fdcan2_bus.registerDevice(&chassis_dirmotor2);
    fdcan2_bus.registerDevice(&chassis_dirmotor3);
    fdcan2_bus.registerDevice(&chassis_dirmotor4); 
    
    fdcan3_bus.registerDevice(&extend_motor);
    fdcan3_bus.registerDevice(&wrist_motor);
    fdcan3_bus.registerDevice(&lift_motor);
    fdcan3_bus.registerDevice(&grip_motor1);
    fdcan3_bus.registerDevice(&grip_motor2);

    //灵足夹爪软件限位
    grip_motor1.setLimitDeg(kGrip1LimitMinDeg, kGrip1LimitMaxDeg);
    grip_motor2.setLimitDeg(kGrip2LimitMinDeg, kGrip2LimitMaxDeg);

    uart2_rx_semaphore = osSemaphoreNew(1, 0, NULL);
    uart2_port.startRxDmaIdle();

    uart3_rx_semaphore = osSemaphoreNew(1, 0, NULL);
    uart3_port.startRxDmaIdle();

    usb_semaphore = osSemaphoreNew(1, 0, NULL);
    UsbPort::Instance().init(onUsbCb, nullptr);

    host_comm.init();
    if(!host_comm.registerMessage(VisionPose::id, sizeof(VisionPose), onVisionPoseMsg, nullptr)){
        Error_Handler();
    };

    return 0;
}



/*-------------------------------------usart2----------------------------------------*/
void onUart2RxCb(const uint8_t *data, size_t len, void *user){
    (void)user;
    if(data != nullptr && len > 0 && uart2_rx_semaphore != nullptr){
        osSemaphoreRelease(uart2_rx_semaphore);
    }
}

void uart2RxProcessTask(void *argument){

    (void)argument;
    for(;;){
        osSemaphoreAcquire(uart2_rx_semaphore, osWaitForever);
        
        UartPort::Packet packet{};
        while(uart2_port.Read(packet)){
            for(uint8_t i = 0; i < packet.len; ++i){
                uint8_t frame_type = wit_imu.processByte(packet.data[i]);
                if(frame_type == 0x53){
                    imu_data = wit_imu.getImuData();
                    imu_data_pub.Publish(imu_data);
                }
            }
        }
    }
}

/*-------------------------------------usart2----------------------------------------*/

/*-------------------------------------usart3----------------------------------------*/
void onUart3RxCb(const uint8_t *data, size_t len, void *user){
    (void)user;
    if(data != nullptr && len > 0 && uart3_rx_semaphore != nullptr){
        osSemaphoreRelease(uart3_rx_semaphore);
    }
}

void uart3RxProcessTask(void *argument){

    (void)argument;
    for(;;){
        osSemaphoreAcquire(uart3_rx_semaphore, 50);
        
        UartPort::Packet packet{};
        while(uart3_port.Read(packet)){
            for(uint8_t i = 0; i < packet.len; ++i){
                if(rm_pocket.processByte(packet.data[i])){
                    const auto &rc_data = rm_pocket.getRCState();
                    rc_data_msg.joyRHori = rc_data.joyRHori;
                    rc_data_msg.joyRVert = rc_data.joyRVert;
                    rc_data_msg.joyLVert = rc_data.joyLVert;
                    rc_data_msg.joyLHori = rc_data.joyLHori;

                    rc_data_msg.swA = rc_data.swA;
                    rc_data_msg.swB = rc_data.swB;
                    rc_data_msg.swC = rc_data.swC;
                    rc_data_msg.swD = rc_data.swD;
                    rc_data_msg.swE = rc_data.swE;
                    rc_data_msg.pot = rc_data.pot;

                    rc_data_pub.Publish(rc_data_msg);

                }
            }
        }
        if(rm_pocket.isOffline()){
            rc_data_msg.joyRHori = 992U;
            rc_data_msg.joyRVert = 992U;
            rc_data_msg.joyLVert = 992U;
            rc_data_msg.joyLHori = 992U;

            rc_data_msg.swA = RC_2_Pos_SW_State_t::UP;
            rc_data_msg.swB = RC_3_Pos_SW_State_t::UP;
            rc_data_msg.swC = RC_3_Pos_SW_State_t::UP;
            rc_data_msg.swD = RC_2_Pos_SW_State_t::UP;
            rc_data_msg.swE = RC_BTN_SW_State_t::RELEASED;
            rc_data_msg.pot = 992U;

            rc_data_pub.Publish(rc_data_msg);
        }
    }
}

/*-------------------------------------usart3----------------------------------------*/


/*--------------------------------------usb------------------------------------------*/

void onUsbCb(void *user){
    (void)user;
    if(usb_semaphore != nullptr){
        osSemaphoreRelease(usb_semaphore);
    }
}

void usbProcessTask(void *argument){

    (void)argument;

    for(;;){
        osSemaphoreAcquire(usb_semaphore, osWaitForever);
        UsbPort::Packet packet{};
        while(UsbPort::Instance().Read(packet)){
            host_comm.processBuffer(packet.data, packet.len);
        }
        UsbPort::Instance().PumpTx();
    }
}

void onVisionPoseMsg(const uint8_t *data, uint8_t len, void *user){
    if(data == nullptr || len != sizeof(VisionPose)) return;
    VisionPose::unpack(data, host_visionPose_data);
    vision_pose_msg.x = host_visionPose_data.x;
    vision_pose_msg.y = host_visionPose_data.y;
    vision_pose_msg.yaw = host_visionPose_data.yaw;
    vision_pose_msg.timestamp_ms = host_visionPose_data.timestamp_ms;
    vision_pose_pub.Publish(vision_pose_msg);
    host_frame_ok++;
}

/*--------------------------------------usb------------------------------------------*/


/*-------------------------------------fdcan1----------------------------------------*/

void can1SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){        

        {
            chassis_drivemotor1.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor1.getCommandID() << 8) |
                (chassis_drivemotor1.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor1.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor2.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor2.getCommandID() << 8) |
                (chassis_drivemotor2.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor2.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor3.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor3.getCommandID() << 8) |
                (chassis_drivemotor3.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor3.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor4.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor4.getCommandID() << 8) |
                (chassis_drivemotor4.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor4.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }

        vTaskDelayUntil(&current, 1);
    }
}
/*-------------------------------------fdcan1----------------------------------------*/


/*-------------------------------------fdcan2----------------------------------------*/

void can2SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){     
        
        chassis_dirmotor1.update();
        chassis_dirmotor2.update();
        chassis_dirmotor3.update();
        chassis_dirmotor4.update();

        uint8_t data[8] = {0};
        CanBus::ClassicPack pack = {};

        uint32_t motor_ids[4] = {0x201, 0x202, 0x203 ,0x204};
        int16_t commands[4] = {
            static_cast<int16_t>(chassis_dirmotor1.cmdTrans()),
            static_cast<int16_t>(chassis_dirmotor2.cmdTrans()),
            static_cast<int16_t>(chassis_dirmotor3.cmdTrans()),
            static_cast<int16_t>(chassis_dirmotor4.cmdTrans())
        };

        packDJIMotorCanMsg(0x200, motor_ids, commands, 4U, data);

        pack.id = 0x200;
        pack.type = CanBus::Type::STANDARD;
        pack.dlc = FDCAN_DLC_BYTES_8;
        for(uint8_t i = 0; i < 8; ++i) pack.data[i] = data[i];
        fdcan2_bus.addCanMsg(pack);
        

        vTaskDelayUntil(&current, 1);
    }
}
/*-------------------------------------fdcan2----------------------------------------*/


/*-------------------------------------fdcan3----------------------------------------*/

//灵足 EL05 发帧：buildTx 返回 true 才把这一帧塞进发送队列
static void sendEl05Frame(CanBus &bus, EL05Motor &motor){
    CanBus::ClassicPack pack{};
    pack.dlc  = FDCAN_DLC_BYTES_8;
    pack.type = CanBus::Type::EXTENDED;
    if(motor.buildTx(pack.data)){
        pack.id = motor.getTxID();
        bus.addCanMsg(pack);
    }
}

void can3SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){

        {   
            extend_motor.update();
            lift_motor.update();

            uint8_t data[8]{};
            uint32_t motor_ids[4] = {0x201, 0x202, 0, 0};
            int16_t commands[4] = {static_cast<int16_t>(lift_motor.cmdTrans()),
                                   static_cast<int16_t>(extend_motor.cmdTrans()), 0, 0};
            CanBus::ClassicPack pack{};
            pack.type = CanBus::Type::STANDARD;
            pack.dlc = FDCAN_DLC_BYTES_8;
            pack.id = 0x200;
            packDJIMotorCanMsg(pack.id, motor_ids, commands, 4, data);
            for(uint8_t i = 0; i < 8U; ++i) pack.data[i] = data[i]; 

            fdcan3_bus.addCanMsg(pack);
        }
            
        {
            CanBus::ClassicPack pack{};
            pack.dlc = FDCAN_DLC_BYTES_8;
            pack.id = wrist_motor.getTxID();
            pack.type = CanBus::Type::STANDARD;
            if(wrist_motor.buildTx(pack.data)){
                fdcan3_bus.addCanMsg(pack);
            }
        }

         
        sendEl05Frame(fdcan3_bus, grip_motor1);
        sendEl05Frame(fdcan3_bus, grip_motor2);

        vTaskDelayUntil(&current, 1);
        
    }

}

/*-------------------------------------fdcan3----------------------------------------*/


