/******************************************************************************
 * @file    ServMotorTemp.c
 * @brief   BSW Serve - PT100 电机定子测温（4 路：PTA1/PTA2/PTB1/PTB2）
 * @layer   BSW/Serve
 * @note    内部一律 deg C 浮点；
 *          块平均：累加 block_sum，满 PT100_AVG_BLOCK_LEN 算均值并复位，
 *          块间保持上一块均值（与原 NTC_avg_P* 行为一致）。
 ******************************************************************************/
#include "HPMC_Settings.h"
#include "ServMotorTemp.h"

#define PT100_AVG_BLOCK_LEN  (100)   /* 保持原 NTC_motor_avg_buf 行为 */

/* 块平均内部状态（static，无需 buffer 数组：块平均只需 sum + count） */
typedef struct {
    float block_sum;
    int   block_count;
} PT100_BlockAvg_t;

PT100_BlockAvg_t pt100_pta1_avg, pt100_pta2_avg, pt100_ptb1_avg, pt100_ptb2_avg;

/* 公开温度（deg C） */
float PT100_PTA1_InstantC, PT100_PTA2_InstantC, PT100_PTB1_InstantC, PT100_PTB2_InstantC;
float PT100_PTA1_AvgC,     PT100_PTA2_AvgC,     PT100_PTB1_AvgC,     PT100_PTB2_AvgC;
float PT100_HottestAvgC;

volatile uint16_t motor_overtemp_ticker = 0;

/* 后台更新，供 ISR 读取；单位：允许最大 Iq 占 pu 满量程的百分比 [0,100] */
static float motor_current_weaken_allowed_percent = 100.0f;

/* 静态函数前置声明 */
static float PT100_AdcToTempC(uint16_t adc_result);
static void  PT100_BlockAvgPush(PT100_BlockAvg_t *block_state,
                                float sample_c,
                                float *avg_out);
float MotorCurrentWeaken_LinearPercent(float temp_c,
                                       float start_temp_c,
                                       float end_temp_c,
                                       float start_percent,
                                       float end_percent);
#if MOTOR_OVERTEMP_ENABLE
static int   Overtemp_MotorReadyForRestart(void);
static void  MotorOvertemp_Update(void);
#endif

/******************************************************************************
 * ADC 原始值 → PT100 电阻 → deg C
 ******************************************************************************/
static float PT100_AdcToTempC(uint16_t adc_result)
{
    float pt100_res = 33000.0f / (16.5f - 5.0f * 0.09456047f
                                  - adc_result * 3.3f * ADC_PU_SCALE_FACTOR) - 2000.0f;
    return (pt100_res - 100.0f) * 2.568845047266748869f;
}

/******************************************************************************
 * 块平均：累加 block_sum，满 PT100_AVG_BLOCK_LEN 算均值写 *avg_out 并复位；
 * 块间 *avg_out 保持上一块均值。
 ******************************************************************************/
static void PT100_BlockAvgPush(PT100_BlockAvg_t *block_state,
                               float sample_c,
                               float *avg_out)
{
    block_state->block_sum   += sample_c;
    block_state->block_count += 1;
    if (block_state->block_count >= PT100_AVG_BLOCK_LEN)
    {
        *avg_out               = block_state->block_sum / (float)PT100_AVG_BLOCK_LEN;
        block_state->block_sum   = 0.0f;
        block_state->block_count = 0;
    }
}

/******************************************************************************
 * 线性插值百分比（过温电流削减用）
 ******************************************************************************/
float MotorCurrentWeaken_LinearPercent(float temp_c,
                                       float start_temp_c,
                                       float end_temp_c,
                                       float start_percent,
                                       float end_percent)
{
    if (end_temp_c <= start_temp_c)
    {
        if (temp_c >= end_temp_c)
        {
            return end_percent;
        }
        return start_percent;
    }

    if (temp_c <= start_temp_c)
    {
        return start_percent;
    }
    if (temp_c >= end_temp_c)
    {
        return end_percent;
    }

    return start_percent + ((temp_c - start_temp_c) / (end_temp_c - start_temp_c))
           * (end_percent - start_percent);
}

/******************************************************************************
 * 电机 PT100 过温电流削减：根据最热平均温度更新允许电流百分比
 ******************************************************************************/
void MotorCurrentWeaken_Update(void)
{
#if TEMPERATURE_CURRENT_WEAKEN_ENABLE
    float motor_temp_c = PT100_HottestAvgC;

    motor_current_weaken_allowed_percent = MotorCurrentWeaken_LinearPercent(
        motor_temp_c,
        MOTOR_CURRENT_WEAKEN_START_TEMPERATURE_C,
        MOTOR_CURRENT_WEAKEN_END_TEMPERATURE_C,
        MOTOR_CURRENT_WEAKEN_START_PERCENT,
        MOTOR_CURRENT_WEAKEN_END_PERCENT);
#else
    motor_current_weaken_allowed_percent = 100.0f;
#endif
}

float MotorCurrentWeaken_GetAllowedPercent(void)
{
    return motor_current_weaken_allowed_percent;
}

/******************************************************************************
 * 过温故障判定与清除
 ******************************************************************************/
#if MOTOR_OVERTEMP_ENABLE
static int Overtemp_MotorReadyForRestart(void)
{
    return (PT100_HottestAvgC < MOTOR_OVERTEMP_CLEAR_C);
}

static void MotorOvertemp_Update(void)
{
    if (fault_motor_overtemp == 0)
    {
        if (PT100_HottestAvgC > MOTOR_OVERTEMP_TRIP_C)
        {
            motor_overtemp_ticker++;

            if (motor_overtemp_ticker >= OVERTEMP_MOTOR_FAULT_CONFIRM)
            {
                fault_motor_overtemp = 1;
                DTC_code1 |= FAULT_DTC_MOTOR_OVERTEMP;
            }
        }
        else
        {
            motor_overtemp_ticker = 0;
        }
    }
    else
    {
        motor_overtemp_ticker = 0;
    }
}

void Overtemp_MotorClearFaultsOnRestart(void)
{
    if (fault_motor_overtemp == 0 && (DTC_code1 & FAULT_DTC_MOTOR_OVERTEMP) == 0u)
    {
        return;
    }

    if (!Overtemp_MotorReadyForRestart())
    {
        return;
    }

    fault_motor_overtemp = 0;
    motor_overtemp_ticker = 0;
    DTC_code1 &= (uint16_t)~FAULT_DTC_MOTOR_OVERTEMP;
}
#else
void Overtemp_MotorClearFaultsOnRestart(void)
{
}
#endif

/******************************************************************************
 * PT100 后台刷新：ADC 采样 → deg C → 块平均 → 最热点 → 过温判定 → 电流削减
 ******************************************************************************/
void PT100_temp(void)
{
    /* 1. 瞬时温度（deg C） */
    PT100_PTA1_InstantC = PT100_AdcToTempC(Mea_PTA1_Raw);
    PT100_PTA2_InstantC = PT100_AdcToTempC(Mea_PTA2_Raw);
    PT100_PTB1_InstantC = PT100_AdcToTempC(Mea_PTB1_Raw);
    PT100_PTB2_InstantC = 0;

    /* 2. 块平均（deg C，块末更新、块间保持） */
    PT100_BlockAvgPush(&pt100_pta1_avg, PT100_PTA1_InstantC, &PT100_PTA1_AvgC);
    PT100_BlockAvgPush(&pt100_pta2_avg, PT100_PTA2_InstantC, &PT100_PTA2_AvgC);
    PT100_BlockAvgPush(&pt100_ptb1_avg, PT100_PTB1_InstantC, &PT100_PTB1_AvgC);
    PT100_PTB2_AvgC = 0;

    /* 3. 最热平均温度（4 路取最大） */
    PT100_HottestAvgC = PT100_PTA1_AvgC;
    if (PT100_PTA2_AvgC > PT100_HottestAvgC) PT100_HottestAvgC = PT100_PTA2_AvgC;
    if (PT100_PTB1_AvgC > PT100_HottestAvgC) PT100_HottestAvgC = PT100_PTB1_AvgC;
    if (PT100_PTB2_AvgC > PT100_HottestAvgC) PT100_HottestAvgC = PT100_PTB2_AvgC;

    /* 4. 过温判定 + 电流削减更新 */
#if MOTOR_OVERTEMP_ENABLE
    MotorOvertemp_Update();
#else
    motor_overtemp_ticker = 0;
    if (fault_motor_overtemp != 0)
    {
        fault_motor_overtemp = 0;
        DTC_code1 &= (uint16_t)~FAULT_DTC_MOTOR_OVERTEMP;
    }
#endif
    MotorCurrentWeaken_Update();
}
