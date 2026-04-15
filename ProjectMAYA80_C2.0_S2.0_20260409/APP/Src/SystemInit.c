/******************************************************************************
 * @file    SystemInit.c
 * @brief   APP - Motor stop, protection, and sound functions
 * @layer   APP
 * @note    Function bodies extracted from HPMC_main_80.c.
 *          Parameter_Init() and M1_AlgoInitStart() remain in HPMC_main_80.c
 *          because they use TI algorithm library types whose headers define
 *          global variables (cannot be included in multiple compilation units).
 ******************************************************************************/
#include "HPMC_Settings.h"

#ifdef _FLASH
#pragma CODE_SECTION(M1_StopMotor,"ramfuncs");
#endif

/* --- Extern declarations for globals defined in HPMC_main_80.c --- */
extern uint32_t SoundTime;
extern uint16_t SoundTimeCount;
extern uint32_t SoundPeriod;
extern uint32_t Stop_count;
extern int    M1_clearmotor;

/******************************************************************************
 * @brief   Force Trip Zone to stop all M1 PWM outputs (emergency stop)
 ******************************************************************************/
void M1_StopMotor(void)
{
    Stop_count++;
    EALLOW;
    EPWM_M1_PhaseC_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseB_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseA_Regs.TZFRC.bit.OST = 0x1;

    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
}

void Manual_OCM1A(void)
{
    EALLOW;
    EPWM_M1_PhaseA_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
}

void Manual_OCM1B(void)
{
    EALLOW;
    EPWM_M1_PhaseB_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
}

void Manual_OCM1C(void)
{
    EALLOW;
    EPWM_M1_PhaseC_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
}

/******************************************************************************
 * @brief   Clear Trip Zone flags to re-enable M1 PWM outputs
 ******************************************************************************/
void M1_ClearToStartMotor(void)
{
    EALLOW;
    EPWM_M1_PhaseC_Regs.TZFRC.bit.OST = 0x0;
    EPWM_M1_PhaseB_Regs.TZFRC.bit.OST = 0x0;
    EPWM_M1_PhaseA_Regs.TZFRC.bit.OST = 0x0;
    EPWM_M1_PhaseC_Regs.TZCLR.bit.OST = 0x1;
    EPWM_M1_PhaseB_Regs.TZCLR.bit.OST = 0x1;
    EPWM_M1_PhaseA_Regs.TZCLR.bit.OST = 0x1;
    EDIS;
    M1_clearmotor = 0;
}

/******************************************************************************
 * @brief   Generate motor sound (placeholder - code body commented in original)
 ******************************************************************************/
void M1_MotorSound(void)
{
    /* Motor sound generation code was commented out in original */
}

uint16_t MotorSoundPeriod(void)
{
    if(SoundTime < SoundPeriod)
    {
        M1_MotorSound();
    }
    else M1_StopMotor();
    SoundTime++;
    if(SoundTime >= 2 * SoundPeriod)
    {
        SoundTime = 1;
        SoundTimeCount++;
    }
    return SoundTimeCount;
}

uint16_t MotorSoundTimes(uint16_t Times)
{
    uint16_t j;
    j = MotorSoundPeriod();
    if(j <= Times) return 0;
    else return 1;
}
