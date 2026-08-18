/******************************************************************************
 * @file    ServNTCTemp.c
 * @brief   BSW Serve - SiC 栅极 NTC 测温
 * @layer   BSW/Serve
 * @note    内部一律 deg C 浮点；
 *          过温判定用瞬时任意路SiC温度超限（快结温保护，不走平均——与 PT100 的 avg-based
 *          不同，属物理差异）。电流削减用 6 路最热温度。
 ******************************************************************************/
#include "HPMC_Settings.h"
#include "ServNTCTemp.h"

/* 6 路结温（deg C），相温名保持不变 */
float NTC_SIC_M1A, NTC_SIC_M1B, NTC_SIC_M1C;
float NTC_SIC_M1A_ECAP, NTC_SIC_M1B_ECAP, NTC_SIC_M1C_ECAP;

/* 聚合量（deg C），M1 风格命名 */
float NTC_SIC_M1_AvgC;       /* 三相 ADC 算术均值，CAN 上报 */
float NTC_SIC_M1_HottestC;   /* 6 路最热，供电流削减 */

volatile uint16_t SiC_over_temp_ticker = 0;

/* 后台更新，供 ISR 读取；单位：允许最大 Iq 占 pu 满量程的百分比 [0,100] */
static float sic_current_weaken_allowed_percent = 100.0f;

/* 静态函数前置声明 */
static float NTC_SIC_AdcToTempC(uint16_t adc_result);
static float NTC_SIC_EcapDutyToTempC(float duty);
float SicCurrentWeaken_LinearPercent(float temp_c,
                                     float start_temp_c,
                                     float end_temp_c,
                                     float start_percent,
                                     float end_percent);
static int   NTC_SIC_AnyPhaseOverTrip(void);
static int   Overtemp_SicReadyForRestart(void);
static void  NTC_SIC_OvertempUpdate(void);

/******************************************************************************
 * ADC 原始值 → NTC 电阻 → deg C（原 NTC_SIC_temp5）
 ******************************************************************************/
static float NTC_SIC_AdcToTempC(uint16_t adc_result)
{
    float ntc_res = ((330.0f * 0.8846f) / (0.8846f - adc_result * 3.3f * ADC_PU_SCALE_FACTOR)
                     - 330.0f) * 0.001f;
    float term2 = (1.0f / 3436.0f) * logf(ntc_res / 5.0f);
    return 1.0f / (0.003354f + term2) - 273.15f;
}

/******************************************************************************
 * ECAP 占空比 → NTC 电阻 → deg C（原 NTC_duty_measurement80）
 ******************************************************************************/
static float NTC_SIC_EcapDutyToTempC(float duty)
{
    float temp_vol = 0.8f * duty + 0.1f;
    float ntc_res = ((330.0f * 0.8846f) / (0.8846f - temp_vol) - 330.0f) * 0.001f;
    float term2 = (1.0f / 3436.0f) * logf(ntc_res / 5.0f);
    return 1.0f / (0.003354f + term2) - 273.15f;
}

/******************************************************************************
 * 线性插值百分比（过温电流削减用）
 ******************************************************************************/
float SicCurrentWeaken_LinearPercent(float temp_c,
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
 * SiC NTC 过温电流削减：根据 6 路最热温度更新允许电流百分比
 ******************************************************************************/
void SicCurrentWeaken_Update(void)
{
#if TEMPERATURE_CURRENT_WEAKEN_ENABLE
    float temp_c = NTC_SIC_M1_HottestC;

    sic_current_weaken_allowed_percent = SicCurrentWeaken_LinearPercent(
        temp_c,
        SIC_CURRENT_WEAKEN_START_TEMPERATURE_C,
        SIC_CURRENT_WEAKEN_END_TEMPERATURE_C,
        SIC_CURRENT_WEAKEN_START_PERCENT,
        SIC_CURRENT_WEAKEN_END_PERCENT);
#else
    sic_current_weaken_allowed_percent = 100.0f;
#endif
}

float SicCurrentWeaken_GetAllowedPercent(void)
{
    return sic_current_weaken_allowed_percent;
}

/******************************************************************************
 * 过温故障判定与清除
 ******************************************************************************/
static int NTC_SIC_AnyPhaseOverTrip(void)
{
    return (NTC_SIC_M1A > SIC_OVERTEMP_TRIP_C ||
            NTC_SIC_M1B > SIC_OVERTEMP_TRIP_C ||
            NTC_SIC_M1C > SIC_OVERTEMP_TRIP_C) ? 1 : 0;
}

static int Overtemp_SicReadyForRestart(void)
{
    return (NTC_SIC_M1A < SIC_OVERTEMP_CLEAR_C && NTC_SIC_M1B < SIC_OVERTEMP_CLEAR_C &&
            NTC_SIC_M1C < SIC_OVERTEMP_CLEAR_C && NTC_SIC_M1A_ECAP < SIC_OVERTEMP_CLEAR_C &&
            NTC_SIC_M1B_ECAP < SIC_OVERTEMP_CLEAR_C && NTC_SIC_M1C_ECAP < SIC_OVERTEMP_CLEAR_C);
}

static void NTC_SIC_OvertempUpdate(void)
{
    if (fault_SiC_overtemp == 0)
    {
        if (NTC_SIC_AnyPhaseOverTrip())
        {
            SiC_over_temp_ticker++;
            if (SiC_over_temp_ticker >= OVERTEMP_SIC_FAULT_CONFIRM)
            {
                fault_SiC_overtemp = 1;
                DTC_code1 |= FAULT_DTC_SIC_OVERTEMP;
            }
        }
        else
        {
            SiC_over_temp_ticker = 0;
        }
    }
    else
    {
        SiC_over_temp_ticker = 0;
    }
}

void Overtemp_SicClearFaultsOnRestart(void)
{
    if (fault_SiC_overtemp == 0 && (DTC_code1 & FAULT_DTC_SIC_OVERTEMP) == 0u)
    {
        return;
    }

    if (!Overtemp_SicReadyForRestart())
    {
        return;
    }

    fault_SiC_overtemp = 0;
    SiC_over_temp_ticker = 0;
    DTC_code1 &= (uint16_t)~FAULT_DTC_SIC_OVERTEMP;
}

/******************************************************************************
 * SiC 后台刷新：ADC/ECAP 采样 → deg C → 三相均值 → 最热点 → 过温判定 → 电流削减
 ******************************************************************************/
void NTC_temp(void)
{
    /* 1. 6 路相温（deg C） */
    NTC_SIC_M1A      = NTC_SIC_AdcToTempC(M1_Temp_PhaseA_Raw);
    NTC_SIC_M1B      = NTC_SIC_AdcToTempC(M1_Temp_PhaseB_Raw);
    NTC_SIC_M1C      = NTC_SIC_AdcToTempC(M1_Temp_PhaseC_Raw);
    NTC_SIC_M1A_ECAP = NTC_SIC_EcapDutyToTempC(ECAP1_duty);
    NTC_SIC_M1B_ECAP = NTC_SIC_EcapDutyToTempC(ECAP2_duty);
    NTC_SIC_M1C_ECAP = NTC_SIC_EcapDutyToTempC(ECAP3_duty);

    /* 2. 三相 ADC 算术均值（CAN 上报） */
    NTC_SIC_M1_AvgC = (NTC_SIC_M1A + NTC_SIC_M1B + NTC_SIC_M1C) / 3.0f;

    /* 3. 6 路最热温度（供电流削减） */
    NTC_SIC_M1_HottestC = NTC_SIC_M1A;
    if (NTC_SIC_M1B      > NTC_SIC_M1_HottestC) NTC_SIC_M1_HottestC = NTC_SIC_M1B;
    if (NTC_SIC_M1C      > NTC_SIC_M1_HottestC) NTC_SIC_M1_HottestC = NTC_SIC_M1C;
    if (NTC_SIC_M1A_ECAP > NTC_SIC_M1_HottestC) NTC_SIC_M1_HottestC = NTC_SIC_M1A_ECAP;
    if (NTC_SIC_M1B_ECAP > NTC_SIC_M1_HottestC) NTC_SIC_M1_HottestC = NTC_SIC_M1B_ECAP;
    if (NTC_SIC_M1C_ECAP > NTC_SIC_M1_HottestC) NTC_SIC_M1_HottestC = NTC_SIC_M1C_ECAP;

    /* 4. 过温判定 + 电流削减更新 */
    NTC_SIC_OvertempUpdate();
    SicCurrentWeaken_Update();
}
