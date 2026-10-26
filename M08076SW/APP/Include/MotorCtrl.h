/******************************************************************************
 * @file    MotorCtrl.h
 * @brief   APP - Motor protection and stop function declarations
 * @layer   APP
 * @note    Motor control strategy.
 ******************************************************************************/
#ifndef MOTORCTRL_H_
#define MOTORCTRL_H_

#include "F28x_Project.h"
#include <stdint.h>
#ifndef MATH_TYPE
#define MATH_TYPE 1
#endif
#include "IQmathLib.h"
#include "rmp_cntl_027.h"

/* ---------- Motor protection ---------- */
extern void M1_ForceAllPwmLowByTZ(void);
extern void M1_StopMotor(void);
extern void M1_ClearToStartMotor(void);

/* ---------- Speed ramp soft landing ----------
 * @param rc               速度 Ramp 对象 M1_speed_rc1
 * @param pu_fullscale_rpm pu 满量程 rpm，传 M1_speed1.BaseRpm（标定值，非当前转速）
 */
extern void M1_RampSoftLandingUpdate(RMPCNTL_027 *rc, float pu_fullscale_rpm);

/* ---------- Throttle ISR (20kHz, ramfuncs) ---------- */
extern void MotorISR_Throttle_Apply(void);
extern void HostCanThrottle_ApplyInIsr(void);
extern void RE38_Throttle_ApplyInIsr(void);
extern float RE38_ClampTargetRpm(float rpm);
#ifdef SMO
extern void RE38_Fault_Start_Update(uint16_t resolver_fault_active,
                                    int32_t smo_estimated_speed_rpm);
extern void RE38_Fault_Start_Reset(void);
#endif

/* 故障重启封波倒计时（定义在 MotorCtrl.c，主循环递减；ISR 内 RestartHold_ApplyInIsr 读取） */
extern volatile uint32_t M1_RestartTicker;

#endif /* MOTORCTRL_H_ */
