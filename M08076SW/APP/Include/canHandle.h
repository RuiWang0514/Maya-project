#ifndef USER_CAN_HANDLE_H_
#define USER_CAN_HANDLE_H_

#include <stdint.h>

/* ---------- 初始化 ---------- */
void CANA_Initialization();
void CANA_Ready();
void CANB_Initialization();
void CANB_Ready();

/* ---------- 发送 ---------- */
void CANBMsgSend();
void CANAMsgSend();
void TempSend();
void DataSend();
void PT100send();
void DTCSend();
void PhaseCurrentRMSSend();
/* ---------- 接收解析（主循环调用） ---------- */
void CANAControlCmdGet(void);
void CANAPidMsgGet(void);
void RE38_CanThrottle_UpdateFromRx(void);
void CANABootMsgGet();
void Message_clear(void);

/* ---------- 共享变量 ---------- */
extern unsigned char re38_can_rx_data[8];
extern volatile uint16_t re38_can_rx_msg_received;
extern volatile float re38_can_target_rpm;
extern volatile uint16_t CAN_Feedback_Spd_RPM;

extern int DC_bus_voltage;                /* 0x200110 Byte2：直流母线电压 / 4 */
extern uint16_t speed_resolver;           /* 0x200110 Byte0-1：电机转速 rpm */

extern int CAN_BOOTLOADER_STOPMOTOR_FLAG;

#endif /* USER_CAN_HANDLE_H_ */
