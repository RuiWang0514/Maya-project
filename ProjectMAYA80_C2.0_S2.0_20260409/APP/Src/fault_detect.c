/******************************************************************************
 * @file    fault_detect.c
 * @brief   APP - Overvoltage / overcurrent / overspeed fault detect and DTC latch
 * @layer   APP
 ******************************************************************************/
#include "fault_detect.h"
#include "HPMC_Settings.h"

#ifdef _FLASH
#pragma CODE_SECTION(FaultOvervoltage_Update,"ramfuncs");
#pragma CODE_SECTION(FaultOvercurrent_Update,"ramfuncs");
#pragma CODE_SECTION(FaultOverspeed_Update,"ramfuncs");
#pragma CODE_SECTION(FaultUndervoltage_Update,"ramfuncs");
#pragma CODE_SECTION(FaultProtect_IsTripped,"ramfuncs");
#pragma CODE_SECTION(FaultProtect_IsrUpdate,"ramfuncs");
#pragma CODE_SECTION(FaultHardware_Update,"ramfuncs");
#endif

int fault_overcurrent = 0;
int fault_overvoltage = 0;
int fault_overspeed = 0;
int fault_undervoltage = 0;
int fault_SiC_overtemp = 0;
int fault_motor_overtemp = 0;
int fault_board_overtemp = 0;
int fault_resolver = 0;
int fault_re38_can_timeout_warn = 0;
int fault_re38_can_timeout_lost = 0;
int fault_re38_can_bus_quality_warn = 0;

uint16_t DTC_code1 = 0x0000;
uint16_t DTC_code2 = 0x0000;
uint16_t DTC_code3 = 0x0000;

extern uint16_t Cla_AD2SB_Fault;

int TripZone_flag = 0;

volatile static uint16_t dc_fault_ticker = 0;
volatile static uint16_t overvoltage_ticker = 0;
volatile static uint16_t undervoltage_ticker = 0;

static _iq overcurrent_trip_pu =
    _IQ((float)FAULT_OVERCURRENT_TRIP_CURRENT_AMP / (float)BASE_CURRENT);
static int overcurrent_phase_a_latched = 0;
static int overcurrent_phase_b_latched = 0;
static int overcurrent_phase_c_latched = 0;
static uint16_t overcurrent_phase_a_count = 0;
static uint16_t overcurrent_phase_b_count = 0;
static uint16_t overcurrent_phase_c_count = 0;

void FaultOvervoltage_Update(float dc_bus_voltage)
{
    if (dc_bus_voltage > (float)FAULT_OVERVOLTAGE_TRIP_VOLTAGE)
    {
        overvoltage_ticker++;
    }

    if (overvoltage_ticker >= FAULT_OVERVOLTAGE_TRIP_COUNT)
    {
        fault_overvoltage = 1;
        DTC_code1 |= FAULT_DTC_OVERVOLTAGE;
    }
}

void FaultOvervoltage_Clear(void)
{
    fault_overvoltage = 0;
    overvoltage_ticker = 0;
    DTC_code1 &= (uint16_t)~FAULT_DTC_OVERVOLTAGE;
}

void FaultOvercurrent_Update(float ia, float ib, float ic)
{
    if ((ia > overcurrent_trip_pu) || (ia < -overcurrent_trip_pu))
    {
        overcurrent_phase_a_count++;
    }
    else
    {
        overcurrent_phase_a_count = 0;
    }

    if ((ib > overcurrent_trip_pu) || (ib < -overcurrent_trip_pu))
    {
        overcurrent_phase_b_count++;
    }
    else
    {
        overcurrent_phase_b_count = 0;
    }

    if ((ic > overcurrent_trip_pu) || (ic < -overcurrent_trip_pu))
    {
        overcurrent_phase_c_count++;
    }
    else
    {
        overcurrent_phase_c_count = 0;
    }

    if (overcurrent_phase_a_count >= FAULT_OVERCURRENT_TRIP_COUNT)
    {
        overcurrent_phase_a_latched = 1;
        overcurrent_phase_a_count = 0;
    }

    if (overcurrent_phase_b_count >= FAULT_OVERCURRENT_TRIP_COUNT)
    {
        overcurrent_phase_b_latched = 1;
        overcurrent_phase_b_count = 0;
    }

    if (overcurrent_phase_c_count >= FAULT_OVERCURRENT_TRIP_COUNT)
    {
        overcurrent_phase_c_latched = 1;
        overcurrent_phase_c_count = 0;
    }

    if (overcurrent_phase_a_latched == 1)
    {
        fault_overcurrent = 1;
    }
    if (overcurrent_phase_b_latched == 1)
    {
        fault_overcurrent = 1;
    }
    if (overcurrent_phase_c_latched == 1)
    {
        fault_overcurrent = 1;
    }

    if (fault_overcurrent == 1)
    {
        DTC_code1 |= FAULT_DTC_OVERCURRENT;
    }
}

void FaultOvercurrent_Clear(void)
{
    fault_overcurrent = 0;
    overcurrent_phase_a_latched = 0;
    overcurrent_phase_b_latched = 0;
    overcurrent_phase_c_latched = 0;
    overcurrent_phase_a_count = 0;
    overcurrent_phase_b_count = 0;
    overcurrent_phase_c_count = 0;
    DTC_code1 &= (uint16_t)~FAULT_DTC_OVERCURRENT;
}

void FaultOverspeed_Update(float speed_rpm)
{
    if (speed_rpm > FAULT_OVERSPEED_TRIP_RPM)
    {
        fault_overspeed = 1;
        DTC_code1 |= FAULT_DTC_OVERSPEED;
    }
}

void FaultOverspeed_Clear(void)
{
    fault_overspeed = 0;
    DTC_code1 &= (uint16_t)~FAULT_DTC_OVERSPEED;
}

/* 欠压警告：持续低于阈值置位；电压回升自清；不参与 FaultProtect_IsTripped */
void FaultUndervoltage_Update(float dc_bus_voltage)
{
    if (dc_bus_voltage < (float)FAULT_UNDERVOLTAGE_TRIP_VOLTAGE)
    {
        undervoltage_ticker++;
        if (undervoltage_ticker >= FAULT_UNDERVOLTAGE_TRIP_COUNT)
        {
            fault_undervoltage = 1;
            DTC_code1 |= FAULT_DTC_UNDERVOLTAGE;
            undervoltage_ticker = FAULT_UNDERVOLTAGE_TRIP_COUNT;
        }
    }
    else
    {
        undervoltage_ticker = 0;
        fault_undervoltage = 0;
        DTC_code1 &= (uint16_t)~FAULT_DTC_UNDERVOLTAGE;
    }
}

void FaultUndervoltage_Clear(void)
{
    fault_undervoltage = 0;
    undervoltage_ticker = 0;
    DTC_code1 &= (uint16_t)~FAULT_DTC_UNDERVOLTAGE;
}

int FaultProtect_IsTripped(void)
{
    return (fault_overvoltage || fault_overcurrent ||
            fault_overspeed || fault_SiC_overtemp ||
            fault_motor_overtemp);
}

void FaultProtect_IsrUpdate(float dc_bus_voltage, float ia, float ib, float ic, float speed_rpm)
{
    FaultOvervoltage_Update(dc_bus_voltage);
    FaultUndervoltage_Update(dc_bus_voltage);
    FaultOvercurrent_Update(ia, ib, ic);
    FaultOverspeed_Update(speed_rpm);
}

void FaultHardware_Update(void)
{
    if (((EPwm1Regs.TZFLG.bit.OST)) == 1)
    {
        TripZone_flag = 1;
    }
    else
    {
        TripZone_flag = 0;
    }

    if (GpioDataRegs.GPADAT.bit.GPIO10 == 0) /* U_Dr_Fault_C, low active */
    {
        DTC_code2 |= 0x01u;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO94 == 0) /* V_Dr_Fault_C */
    {
        DTC_code2 |= 0x02u;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO93 == 0) /* W_Dr_Fault_C */
    {
        DTC_code2 |= 0x04u;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO92 == 0) /* Fault_S1 trip zone */
    {
        DTC_code2 |= 0x08u;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO90 == 0) /* U_Dr_Ready_C */
    {
        DTC_code2 |= 0x10u;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO85 == 0) /* V_Dr_Ready_C */
    {
        DTC_code2 |= 0x20u;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO86 == 0) /* W_Dr_Ready_C */
    {
        DTC_code2 |= 0x40u;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO88 == 0) /* Ready_S1 */
    {
        DTC_code2 |= 0x80u;
    }

    if (GpioDataRegs.GPBDAT.bit.GPIO48 == 0) /* Fault_DC */
    {
        dc_fault_ticker++;
    }
    else
    {
        dc_fault_ticker = 0;
    }

    if (dc_fault_ticker >= 10)
    {
        DTC_code3 |= 0x08u;
        dc_fault_ticker = 0;
    }

    if (GpioDataRegs.GPBDAT.bit.GPIO53 == 0) /* Fault_5V_A1 */
    {
        DTC_code3 |= 0x10u;
    }

    /* 旋变：Cla_AD2SB_Fault!=0 锁存上报；不参与 FaultProtect_IsTripped */
    if (Cla_AD2SB_Fault != 0u)
    {
        fault_resolver = 1;
        DTC_code1 |= FAULT_DTC_RESOLVER;
    }
}

void FaultHardware_Clear(void)
{
    DTC_code2 = 0;
    DTC_code3 = 0;
    dc_fault_ticker = 0;
}
