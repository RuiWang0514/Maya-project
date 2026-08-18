#ifndef USER_CAN_HANDLE_H_
#define USER_CAN_HANDLE_H_

#include <stdint.h>

void CANA_Initialization();

void CANA_Ready();

void CANB_Initialization();

void CANB_Ready();

void CANBMsgSend();

void CANAMsgSend();

void CANAControlCmdGet(void);

void CANAPidMsgGet(void);

void RE38_CanThrottle_UpdateFromRx(void);

void TempSend();

void DataSend();

void PT100send();

void DTCSend();

void MotorCtrl310Send(void);

void MotorCtrl311Send(void);

extern unsigned char motor310txdata[8];

extern unsigned char motor311txdata[8];

extern unsigned char re38_can_rx_data[8];

extern volatile uint16_t re38_can_rx_msg_received;

extern volatile float re38_can_target_rpm;

extern int DC_bus_voltage;                /* Byte0-1 直流母线电压（0x310） */

extern int16_t  motor310_torque;          /* Byte2-3 电机力矩 */

extern uint16_t speed_resolver;           /* Byte4-5 电机转速（0x310） */

extern unsigned char motor310_status;          /* Byte6 状态故障 */

extern int motor310_iq_ref_avg;             /* Iq_ref 10次均值 */

extern int motor310_current_rms;            /* 由 iq_ref_avg*sqrt(2)/2 得到，外发 Byte7 *0.5 */

extern int CAN_BOOTLOADER_STOPMOTOR_FLAG;

/* 0x311 外发数据 */

extern unsigned char motor311_reserved_b2;     /* Byte2 预留 */

extern uint16_t motor311_torque_limit;    /* Byte3-4 转矩限值 */

extern unsigned char motor311_ctrl_fault;        /* Byte5 控制器硬件故障 */

extern unsigned char motor311_motor_fault;       /* Byte6 电机故障 */

extern unsigned char motor311_life_signal;       /* Byte7 生命信号 */

/* CAN 帧偏置（组帧时使用） */

#define MOTOR310_RPM_OFFSET    (10000)

#define MOTOR311_TEMP_OFFSET   (50)

/* 0x310 Byte6 状态位（1=有效/故障） */

#define MOTOR310_STA_STANDBY       (0x01u)

#define MOTOR310_STA_CHARGE_DONE   (0x02u)

#define MOTOR310_STA_RUNNING       (0x04u)

#define MOTOR310_STA_FAULT         (0x08u)

#define MOTOR310_STA_POSITION_DONE (0x10u)

/* 0x311 Byte5 控制器故障位 */

#define MOTOR311_FLT_B5_GENERAL_PIPE   (0x01u)

#define MOTOR311_FLT_B5_IGBT_OVERTEMP  (0x08u)

#define MOTOR311_FLT_B5_IGBT_TEMP_ALM  (0x10u)

#define MOTOR311_FLT_B5_DC_OVERVOLT    (0x20u)

#define MOTOR311_FLT_B5_DC_UNDERVOLT   (0x40u)

#define MOTOR311_FLT_B5_OUTPUT_OC      (0x80u)

/* 0x311 Byte6 电机故障位 */

#define MOTOR311_FLT_B6_OVERSPEED      (0x01u)

#define MOTOR311_FLT_B6_MOT_TEMP_ALM   (0x02u)

#define MOTOR311_FLT_B6_MOT_OVERTEMP   (0x04u)

void Message_clear(void);

void CANABootMsgGet();

#endif /* USER_CAN_HANDLE_H_ */

