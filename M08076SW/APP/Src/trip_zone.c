#include "trip_zone.h"

__interrupt void epwm1_tzint_isr(void)
{
    EALLOW;

    EPwm1Regs.TZCLR.bit.INT = 1;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

__interrupt void epwm2_tzint_isr(void)
{
    EALLOW;

    EPwm2Regs.TZCLR.bit.INT = 1;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

__interrupt void epwm3_tzint_isr(void)
{
    EALLOW;

    EPwm3Regs.TZCLR.bit.INT = 1;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}
