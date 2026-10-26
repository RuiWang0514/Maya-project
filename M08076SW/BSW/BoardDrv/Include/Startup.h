/******************************************************************************
 * @file    Startup.h
 * @brief   BSW BoardDrv - Hardware startup function declarations
 * @layer   BSW/BoardDrv
 * @note    Function bodies extracted from HPMC_main_80.c.
 *          These functions are called from main() in their original order.
 *          Do NOT change the call order in main().
 ******************************************************************************/
#ifndef STARTUP_H_
#define STARTUP_H_

#include "F28x_Project.h"
#include <stdint.h>

/* ---------- ADC configuration and startup ---------- */
void ConfigureADC(void);
void ADC_Startup(void);

/* ---------- CLA memory and task configuration ---------- */
void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);

/* ---------- System clock / PIE and ePWM time-base clock ---------- */
void InitClockAndPie(void);
void PwmTbclk_Stop(void);
void PwmTbclk_Start(void);

#endif /* STARTUP_H_ */
