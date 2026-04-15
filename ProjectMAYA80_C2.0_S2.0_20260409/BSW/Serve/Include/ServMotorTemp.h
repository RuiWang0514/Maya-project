/*
 * spi_resolver.h
 *
 *  Created on: 2020?5?4?
 *      Author: quop_
 */

#include "F28x_Project.h"

#include "HPMC_Settings.h"


#define TEMP1_CS_H (GpioDataRegs.GPBSET.bit.GPIO57=1)
#define TEMP1_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO57=1)
#define TEMP2_CS_H (GpioDataRegs.GPBSET.bit.GPIO62=1)
#define TEMP2_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO62=1)

uint16_t MAX6755B_READ(uint16_t addr);

void MAX6755B_temp_cal(void);

uint16_t MAX6755C_READ(uint16_t addr);
uint16_t MAX6755A_READ1(uint16_t addr);
uint16_t MAX6755C_READ2(uint16_t addr);
void MAX6755A_temp_cal(void);

/* PT100 五路电机测温（由 ServMotorTemp.c 实现） */
void PT100_temp(void);
float PT100_measurement(uint16_t adcresult);
float NTC_avg_PTB1(int NTC_SIC_temp_int);
float NTC_avg_PTB2(int NTC_SIC_temp_int);
float NTC_avg_PTA1(int NTC_SIC_temp_int);
float NTC_avg_PTA2(int NTC_SIC_temp_int);
float NTC_avg_PTC(int NTC_SIC_temp_int);
float NTC_filter_PTB1(int NTC_SIC_temp_int);
float NTC_filter_PTB2(int NTC_SIC_temp_int);

extern float Mea_PTA1, Mea_PTA2, Mea_PTB1, Mea_PTB2, Mea_PTC;
extern int Mea_PTA1_avg_1, Mea_PTA2_avg_1, Mea_PTB1_avg_1, Mea_PTB2_avg_1, Mea_PTC_avg_1;
extern int motor_overtemp_flag;












