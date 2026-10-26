#ifndef SERVNTCTEMP_H_
#define SERVNTCTEMP_H_

#include "F28x_Project.h"
#include "HPMC_Settings.h"

/* SiC 栅极 NTC 三路测温 —— deg C 浮点。 3 相 ADC 源瞬时温度（deg C） */
extern float NTC_SIC_M1A, NTC_SIC_M1B, NTC_SIC_M1C;
/* 3 相 ECAP 源瞬时温度（deg C） */
extern float NTC_SIC_M1A_ECAP, NTC_SIC_M1B_ECAP, NTC_SIC_M1C_ECAP;
/* 三路SiC ADC 平均温度（deg C），CAN 上报 */
extern float NTC_SIC_M1_AvgC;
/* M1 六路中最热温度（deg C），供电流削减，NTC_temp() 刷新 */
extern float NTC_SIC_M1_HottestC;

/* 后台刷新：ADC/ECAP 采样 → deg C → 三相均值 → 最热点 → 过温判定 → 电流削减更新 */
void  NTC_temp(void);

/* 过温故障清除（重启序列调用） */
void  Overtemp_SicClearFaultsOnRestart(void);

/* SiC NTC 过温电流削减：后台 NTC_temp() 刷新后更新，ISR 只读 AllowedPercent */
void  SicCurrentWeaken_Update(void);
float SicCurrentWeaken_GetAllowedPercent(void);

#endif /* SERVNTCTEMP_H_ */
