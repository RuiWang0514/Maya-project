/******************************************************************************
 * @file    Startup.c
 * @brief   BSW BoardDrv - Hardware startup functions (ADC, DAC, CLA, SCI)
 * @layer   BSW/BoardDrv
 * @note    Function bodies extracted from HPMC_main_80.c.
 *          Called from main() - do NOT change the call order there.
 ******************************************************************************/
#include "Startup.h"
#include "HPMC_Settings.h"
#include "CLAShared.h"

/* DAC reference source selection (moved from HPMC_main_80.c) */
#define REFERENCE_VDAC      0
#define REFERENCE_VREF      1
#define REFERENCE           REFERENCE_VREF

/* DAC_PTR is defined in HPMC_main_80.c, referenced here via extern */
extern volatile struct DAC_REGS *DAC_PTR[];

/******************************************************************************
 * @brief   Configure ADC channels and SOC triggers for motor current/voltage sensing
 * @note    Must be called after InitSysCtrl() and before TBCLKSYNC=1.
 *          Called from main() in HPMC_main_80.c.
 ******************************************************************************/
void ConfigureADC(void)
{
    ADC_Startup();
    EALLOW;

    // Setting up link from EPWM to ADC.
    // SOCA/SOCB use CMPB on the up-count side so all ADC sampling starts
    // after PWM counter zero plus the configured low-side blanking delay.
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPB;
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm1Regs.ETCLR.bit.SOCA = 1;

    EPwm1Regs.ETSEL.bit.SOCBSEL = ET_CTRU_CMPB;
    EPwm1Regs.ETPS.bit.SOCBPRD  = ET_1ST;
    EPwm1Regs.ETSEL.bit.SOCBEN  = 1;
    EPwm1Regs.ETCLR.bit.SOCB = 1;

    // Prioritize SOC0 so current feedback conversions run before slower
    // voltage, throttle, and temperature conversions triggered at the same point.
    AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 1;
    AdcbRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 1;
    AdccRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 1;
    AdcdRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 1;

    // M1 current A-A4 B-C4 C-B4
    AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 2;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 19;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 0x05;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 2;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 19;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 0x05;

    AdccRegs.ADCSOC0CTL.bit.CHSEL     = 2;
    AdccRegs.ADCSOC0CTL.bit.ACQPS     = 19;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = 0x05;

    AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 2;
    AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 19;
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 0x05; // Analog ch2  AdcdResultRegs.ADCRESULT0

    AdcaRegs.ADCSOC1CTL.bit.CHSEL     = 3;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 19;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = 0x6;

    AdcbRegs.ADCSOC1CTL.bit.CHSEL     = 3;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS     = 19;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL   = 0x6;

    AdccRegs.ADCSOC1CTL.bit.CHSEL     = 3;
    AdccRegs.ADCSOC1CTL.bit.ACQPS     = 19;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL   = 0x6;

    AdcdRegs.ADCSOC1CTL.bit.CHSEL     = 3;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS     = 19;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL   = 0x6;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL     = 4;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 19;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = 0x06;

    AdccRegs.ADCSOC2CTL.bit.CHSEL     = 4;
    AdccRegs.ADCSOC2CTL.bit.ACQPS     = 19;
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL   = 0x06; // Analog ch3  AdccResultRegs.ADCRESULT2

    AdcdRegs.ADCSOC2CTL.bit.CHSEL     = 4;
    AdcdRegs.ADCSOC2CTL.bit.ACQPS     = 19;
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL   = 0x06; // 15V power supply  AdcdResultRegs.ADCRESULT2

    AdcaRegs.ADCSOC3CTL.bit.CHSEL     = 1;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS     = 19;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL   = 0x06;

    AdcbRegs.ADCSOC3CTL.bit.CHSEL     = 1;
    AdcbRegs.ADCSOC3CTL.bit.ACQPS     = 19;
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL   = 0x06;

    AdcdRegs.ADCSOC3CTL.bit.CHSEL     = 1;
    AdcdRegs.ADCSOC3CTL.bit.ACQPS     = 19;
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL   = 0x06; // Analog ch1  AdcdResultRegs.ADCRESULT3

    AdcaRegs.ADCSOC4CTL.bit.CHSEL     = 0;
    AdcaRegs.ADCSOC4CTL.bit.ACQPS     = 19;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL   = 0x6;

    AdcbRegs.ADCSOC4CTL.bit.CHSEL     = 0;
    AdcbRegs.ADCSOC4CTL.bit.ACQPS     = 19;
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL   = 0x6;

    AdcdRegs.ADCSOC4CTL.bit.CHSEL     = 0;
    AdcdRegs.ADCSOC4CTL.bit.ACQPS     = 19;
    AdcdRegs.ADCSOC4CTL.bit.TRIGSEL   = 0x6;

    AdcaRegs.ADCSOC5CTL.bit.CHSEL     = 5;
    AdcaRegs.ADCSOC5CTL.bit.ACQPS     = 19;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL   = 0x6;

    AdcbRegs.ADCSOC5CTL.bit.CHSEL     = 14;
    AdcbRegs.ADCSOC5CTL.bit.ACQPS     = 19;
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL   = 0x6;

    AdccRegs.ADCSOC5CTL.bit.CHSEL     = 15;
    AdccRegs.ADCSOC5CTL.bit.ACQPS     = 19;
    AdccRegs.ADCSOC5CTL.bit.TRIGSEL   = 0x6;

    EDIS;
}

/******************************************************************************
 * @brief   Configure a single DAC channel
 * @param   dac_num  DAC channel index (into DAC_PTR array)
 ******************************************************************************/
void configureDAC(uint16_t dac_num)
{
    EALLOW;
    DAC_PTR[dac_num]->DACCTL.bit.DACREFSEL = REFERENCE;
    DAC_PTR[dac_num]->DACOUTEN.bit.DACOUTEN = 1;
    DAC_PTR[dac_num]->DACVALS.all = 0;
    DELAY_US(10);
    EDIS;
}

/******************************************************************************
 * @brief   Power up all four ADC modules (A/B/C/D) with 12-bit single-ended config
 * @note    Called internally by ConfigureADC()
 ******************************************************************************/
void ADC_Startup(void)
{
    uint16_t i;

    EALLOW;

    // ADC-A
    AdcaRegs.ADCCTL2.bit.PRESCALE   = 6;
    AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // ADC-B
    AdcbRegs.ADCCTL2.bit.PRESCALE   = 6;
    AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // ADC-C
    AdccRegs.ADCCTL2.bit.PRESCALE   = 6;
    AdccRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // ADC-D
    AdcdRegs.ADCCTL2.bit.PRESCALE   = 6;
    AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
    AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // Delay >1ms for ADC power-up
    for(i = 0; i < 1000; i++){
        __asm("   RPT#255 || NOP");
    }

    EDIS;
}

/******************************************************************************
 * @brief   Configure CLA memory mapping (program/data RAM assignment)
 * @note    Copies CLA code from Flash to RAM when _FLASH is defined
 ******************************************************************************/
void CLA_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;

    EALLOW;
#ifdef _FLASH
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
            (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;

    EDIS;
}

/******************************************************************************
 * @brief   Initialize CLA1 task vectors, triggers, and enable IACK
 * @note    Task2 is triggered by EPWM1INT for resolver reading
 ******************************************************************************/
void CLA_initCpu1Cla1(void)
{
    EALLOW;
    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);

    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.all = M_INT2;

    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = CLA_TRIG_EPWM1INT;

    EDIS;
}
