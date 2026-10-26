/******************************************************************************
 * @file    pid_gain_schedule.h
 * @brief   转速分段线性插值 PID 增益调度
 *
 * 按实际转速在 Low/High 两套端点 Kp/Ki 之间线性插值，写入转速环与 Id/Iq 电流环。
 * - rpm <= PID_INTERP_RPM_LOW     ：使用 Low 端点
 * - PID_INTERP_RPM_LOW ~ HIGH     ：Low 与 High 线性过渡
 * - rpm >= PID_INTERP_RPM_HIGH    ：使用 High 端点
 *
 * CAN 在线调参（CAN_PID_adjust=1）时，CAN 值同时写入 Low/High 端点，插值路径不变，
 * 但因两端相同，运行增益恒等于 CAN 下发值。
 ******************************************************************************/
#ifndef PID_GAIN_SCHEDULE_H_
#define PID_GAIN_SCHEDULE_H_

#include <stdint.h>

/* 单套端点：转速环 + Id/Iq 电流环，各 Kp/Ki 共 6 个参数 */
typedef struct
{
    float spd_kp;
    float spd_ki;
    float id_kp;
    float id_ki;
    float iq_kp;
    float iq_ki;
} PidEndpointParams_t;

void PidGainSchedule_Init(void);
void PidGainSchedule_Update(int32_t speed_rpm);
void PidGainSchedule_ApplyCanToBothEndpoints(void);
void PidGainSchedule_RestoreDefaultEndpoints(void);

#endif /* PID_GAIN_SCHEDULE_H_ */
