/*
 * spi_resolver.h

 *      Author: quop_
 */

#include "F28x_Project.h"

#include "HPMC_Settings.h"


void NTC_temp(void);
float NTC_SIC_temp(uint16_t adcresult);
float NTC_SIC_temp1(uint16_t adcresult);
float NTC_SIC_temp2(uint16_t adcresult);
float NTC_SIC_temp3(uint16_t adcresult);
float NTC_SIC_temp4(uint16_t adcresult);
float NTC_SIC_temp5(uint16_t adcresult);
float NTC_duty_measurement(uint16_t adcresult);
float NTC_duty_measurement80(float duty);
float NTC_filter_A1(int NTC_SIC_temp_int);
float NTC_avg_A1(int NTC_SIC_temp_int);
float NTC_filter_B1(int NTC_SIC_temp_int);
float NTC_filter_C1(int NTC_SIC_temp_int);
float NTC_filter_A2(int NTC_SIC_temp_int);
float NTC_filter_B2(int NTC_SIC_temp_int);
float NTC_filter_C2(int NTC_SIC_temp_int);
float NTC_avg_A2(int NTC_SIC_temp_int);
float NTC_avg_B2(int NTC_SIC_temp_int);
float NTC_avg_C2(int NTC_SIC_temp_int);
float NTC_avg(int NTC_SIC_temp_int);
