/******************************************************************************
 * @file    SystemInit.h
 * @brief   APP - Motor protection, stop, and sound function declarations
 * @layer   APP
 * @note    Parameter_Init() and M1_AlgoInitStart() remain declared in
 *          HPMC_main_80.c (they use TI algorithm library header types that
 *          cannot be safely included in multiple compilation units).
 ******************************************************************************/
#ifndef SYSTEMINIT_H_
#define SYSTEMINIT_H_

#include "F28x_Project.h"
#include <stdint.h>

/* ---------- Motor protection ---------- */
void M1_StopMotor(void);
void M1_ClearToStartMotor(void);
void Manual_OCM1A(void);
void Manual_OCM1B(void);
void Manual_OCM1C(void);

/* ---------- Motor sound ---------- */
void M1_MotorSound(void);
uint16_t MotorSoundPeriod(void);
uint16_t MotorSoundTimes(uint16_t Times);

#endif /* SYSTEMINIT_H_ */
