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
void configureDAC(uint16_t dac_num);

/* ---------- CLA memory and task configuration ---------- */
void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);

/* ---------- CLA end-of-task ISR callbacks ---------- */
__interrupt void cla1Isr1(void);
__interrupt void cla1Isr2(void);
__interrupt void cla1Isr3(void);
__interrupt void cla1Isr4(void);
__interrupt void cla1Isr5(void);
__interrupt void cla1Isr6(void);
__interrupt void cla1Isr7(void);
__interrupt void cla1Isr8(void);

/* ---------- SCI-B (Telemetry serial port) ---------- */
void Init_SCIB_GPIO(void);
void Init_SCIB(void);
void Scib_xmit(int a);

#endif /* STARTUP_H_ */
