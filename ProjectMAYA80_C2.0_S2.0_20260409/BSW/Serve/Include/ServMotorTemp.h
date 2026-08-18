#ifndef SERVMOTORTEMP_H_
#define SERVMOTORTEMP_H_

#include "F28x_Project.h"
#include "HPMC_Settings.h"

/* PT100 电机定子测温 —— 4 路（PTA1/PTA2/PTB1/PTB2），PTC 硬件已移除。
 * 内部 deg C 浮点；CAN 发送方在赋值处自行 +40 编码为无符号字节。 */

/* 4 路瞬时温度（deg C），每次 PT100_temp() 刷新 */
extern float PT100_PTA1_InstantC;
extern float PT100_PTA2_InstantC;
extern float PT100_PTB1_InstantC;
extern float PT100_PTB2_InstantC;

/* 4 路块平均温度（deg C），块末更新、块间保持 */
extern float PT100_PTA1_AvgC;
extern float PT100_PTA2_AvgC;
extern float PT100_PTB1_AvgC;
extern float PT100_PTB2_AvgC;

/* 4 路中最热平均温度（deg C），供过温判定/电流削减，PT100_temp() 刷新 */
extern float PT100_HottestAvgC;

/* 后台刷新：ADC 采样 → deg C → 块平均 → 最热点 → 过温判定 → 电流削减更新 */
void  PT100_temp(void);

/* 过温故障清除（重启序列调用） */
void  Overtemp_MotorClearFaultsOnRestart(void);

/* 电机 PT100 过温电流削减：后台 PT100_temp() 刷新后更新，ISR 只读 AllowedPercent */
void  MotorCurrentWeaken_Update(void);
float MotorCurrentWeaken_GetAllowedPercent(void);

#endif /* SERVMOTORTEMP_H_ */
