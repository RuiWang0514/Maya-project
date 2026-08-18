#ifndef FAULT_DETECT_H_
#define FAULT_DETECT_H_

#include <stdint.h>

/* ---------- 阈值---------- */
#define FAULT_OVERVOLTAGE_TRIP_VOLTAGE       500
#define FAULT_OVERVOLTAGE_TRIP_COUNT         3

#define FAULT_OVERCURRENT_TRIP_CURRENT_AMP   700
#define FAULT_OVERCURRENT_TRIP_COUNT         5

#define FAULT_OVERSPEED_TRIP_RPM             2700.0f

#define FAULT_UNDERVOLTAGE_TRIP_VOLTAGE      290      /* 欠压警告阈值 V */
#define FAULT_UNDERVOLTAGE_TRIP_COUNT        1000    /* 50ms @ 20kHz ISR */

#define FAULT_BOARD_OVERTEMP_C               (105.0f) /* 控制板过温警告 °C */
#define FAULT_BOARD_OVERTEMP_COUNT           (10u)    /* 10 × 主循环≈10ms → ≈100ms */

/* RE38 CAN 油门帧超时（主循环约 10ms/轮来计算1 tick）*/
#define FAULT_RE38_CAN_MAIN_LOOP_MS              (10u)
#define FAULT_RE38_CAN_TIMEOUT_WARN_MS           (500u)   /* >500ms → timeout_warn */
#define FAULT_RE38_CAN_TIMEOUT_LOST_MS           (1000u)  /* >1000ms → timeout_lost */
#define FAULT_RE38_CAN_TIMEOUT_WARN_TICKS        (FAULT_RE38_CAN_TIMEOUT_WARN_MS / FAULT_RE38_CAN_MAIN_LOOP_MS)
#define FAULT_RE38_CAN_TIMEOUT_LOST_TICKS        (FAULT_RE38_CAN_TIMEOUT_LOST_MS / FAULT_RE38_CAN_MAIN_LOOP_MS)

/* ---------- DTC_code1---------- */
#define FAULT_DTC_OVERVOLTAGE                0x01u
#define FAULT_DTC_OVERCURRENT                0x02u
#define FAULT_DTC_UNDERVOLTAGE               0x04u
#define FAULT_DTC_SIC_OVERTEMP               0x08u
#define FAULT_DTC_OVERSPEED                  0x10u
#define FAULT_DTC_MOTOR_OVERTEMP             0x20u
#define FAULT_DTC_BOARD_OVERTEMP             0x40u
#define FAULT_DTC_RESOLVER                   0x80u

/* ---------- API：三域解耦 + 聚合查询 ---------- */
void FaultOvervoltage_Update(float dc_bus_voltage);
void FaultOvervoltage_Clear(void);

void FaultOvercurrent_Update(float ia, float ib, float ic);
void FaultOvercurrent_Clear(void);

void FaultOverspeed_Update(float speed_rpm);
void FaultOverspeed_Clear(void);

void FaultUndervoltage_Update(float dc_bus_voltage);
void FaultUndervoltage_Clear(void);

int  FaultProtect_IsTripped(void);

void FaultProtect_IsrUpdate(float dc_bus_voltage,
                            float ia, float ib, float ic,
                            float speed_rpm);

/* 硬件域：ISR 采样 TZ/GPIO，仅 DTC latch，不参与软件 trip */
void FaultHardware_Update(void);
void FaultHardware_Clear(void);

/* ---------- 全局状态（定义在 fault_detect.c） ---------- */
extern int fault_overcurrent;
extern int fault_overvoltage;
extern int fault_overspeed;
extern int fault_undervoltage;
extern int fault_SiC_overtemp;
extern int fault_motor_overtemp;
extern int fault_board_overtemp;
extern int fault_resolver;        /* 旋变故障 */
extern int fault_re38_can_timeout_warn;   /* 油门帧超时警告（>500ms 且 ≤1000ms） */
extern int fault_re38_can_timeout_lost;   /* 油门帧超时丢失（>1000ms），不停机 */
extern int fault_re38_can_bus_quality_warn; /* CAN 总线质量预警（跟随 CAN_STATUS_EWARN），不停机 */

extern int TripZone_flag;

extern uint16_t DTC_code1;
extern uint16_t DTC_code2;
extern uint16_t DTC_code3;

#endif /* FAULT_DETECT_H_ */
