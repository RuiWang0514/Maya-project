#include "trip_zone.h"

extern uint16_t M1_pwmon;
extern uint16_t M2_pwmon;
extern uint16_t M1_Algo_control_start;

__interrupt void epwm1_tzint_isr(void)
{
    EPwm1TZIntCount++;

    EALLOW;

    EPwm1Regs.TZCLR.bit.INT = 1;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

__interrupt void epwm2_tzint_isr(void)
{
    EPwm2TZIntCount++;

    EALLOW;

    EPwm2Regs.TZCLR.bit.INT = 1;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

__interrupt void epwm3_tzint_isr(void)
{
    EPwm3TZIntCount++;

    EALLOW;

    EPwm3Regs.TZCLR.bit.INT = 1;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

__interrupt void epwm7_tzint_isr(void)
{
    EPwm7TZIntCount++;

    EALLOW;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

__interrupt void epwm8_tzint_isr(void)
{
    EPwm8TZIntCount++;

    EALLOW;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

__interrupt void epwm9_tzint_isr(void)
{
    EPwm9TZIntCount++;

    EALLOW;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}
