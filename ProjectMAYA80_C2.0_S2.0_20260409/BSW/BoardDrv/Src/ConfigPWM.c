#include "ConfigPWM.h"

// Used to indirectly access all EPWM modules
volatile struct EPWM_REGS *ePWM[] = {
        &EPwm1Regs,         //intentional: (ePWM[0] not used)
        &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
        &EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs};


void PWM_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db)
{

    EALLOW;

    (*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load same 027
    (*ePWM[n]).TBPRD = period / 2; // up-down mode , Tpwm=2*TBPRD*Ttbclk
    (*ePWM[n]).TBPHS.bit.TBPHS = 0;
    (*ePWM[n]).TBCTR = 0;
    (*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;   // same 027
    (*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;       //same027
    (*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;       //same 027

    (*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;     //027-EN
    (*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"  027-0

    // Counter Compare Submodule Registers  same with 027, lack B
    (*ePWM[n]).CMPA.bit.CMPA = 0; // set duty 0% initially

    // CMPB is reserved for delayed ADC SOCA/SOCB trigger; CMPA remains
    // the SVPWM duty compare value.
    (*ePWM[n]).CMPB.bit.CMPB = (uint16_t)ADC_SAMPLE_DELAY_TICKS;

    (*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[n]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    (*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    (*ePWM[n]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    // Action Qualifier SubModule Registers  same with 027
    (*ePWM[n]).AQCTLA.bit.CAU = AQ_SET;
    (*ePWM[n]).AQCTLA.bit.CAD = AQ_CLEAR;

    // Active high complementary PWMs - Set up the deadband  same with 027
    (*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
    (*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    (*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    (*ePWM[n]).DBRED = db;
    (*ePWM[n]).DBFED = db;

    EDIS;
}


void Fault_GPIO_init(void)
{
    EALLOW; // below registers are "protected", allow access.

    //GPIO68 --- U1-PS
    GPIO_SetupPinOptions(68, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(68,0,0);
    GpioDataRegs.GPCSET.bit.GPIO68=1;
    //GPIO69 --- V1-PS
    GPIO_SetupPinOptions(69, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(69,0,0);
    GpioDataRegs.GPCSET.bit.GPIO69=1;
    //GPIO67 --- W1-PS
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(67,0,0);
    GpioDataRegs.GPCSET.bit.GPIO67=1;

    //GPIO13 --- U1-RI
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(13,0,0);
    GpioDataRegs.GPASET.bit.GPIO13=1;
    //GPIO87 --- V1-RI
    GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(87,0,0);
    GpioDataRegs.GPCSET.bit.GPIO87=1;
    //GPIO83 --- W1-RI
    GPIO_SetupPinOptions(83, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(83,0,0);
    GpioDataRegs.GPCSET.bit.GPIO83=1;

    DELAY_US(10);

    //GPIO13 --- U1-RI
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(13,0,0);
    GpioDataRegs.GPACLEAR.bit.GPIO13=1;
    //GPIO87 --- V1-RI
    GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(87,0,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO87=1;
    //GPIO83 --- W1-RI
    GPIO_SetupPinOptions(83, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(83,0,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO83=1;

    GPIO_SetupPinOptions(77, GPIO_OUTPUT, GPIO_ASYNC);//U 12V Fault
    GPIO_SetupPinOptions(73, GPIO_OUTPUT, GPIO_ASYNC);//V 12V Fault
    GPIO_SetupPinOptions(76, GPIO_OUTPUT, GPIO_ASYNC);//W 12V Fault
    GpioDataRegs.GPCSET.bit.GPIO73=1;
    GpioDataRegs.GPCSET.bit.GPIO76=1;
    GpioDataRegs.GPCSET.bit.GPIO77=1;

    EDIS;
}

void GateDriver_EnableOutputs(void)
{

    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;

}

void Cla_GPIO_Init(void)
{
    EALLOW;

    GpioCtrlRegs.GPCCSEL1.bit.GPIO65 = 0x1;
    GpioCtrlRegs.GPBCSEL4.bit.GPIO59 = 0x1;
    GpioCtrlRegs.GPCCSEL1.bit.GPIO64 = 0x1;

    EDIS;
}

void InitTZGpio(void)
{
    EALLOW;

    GPIO_SetupPinOptions(92, GPIO_INPUT, GPIO_ASYNC);//TZ Fault

    EDIS;

    EALLOW;

    GpioCtrlRegs.GPCPUD.bit.GPIO92 = 0;
    GpioCtrlRegs.GPCQSEL2.bit.GPIO92 = 3;
    InputXbarRegs.INPUT1SELECT = 92;       // Set TZ1 source to GPIO-pin

    EDIS;
}

void InitFaultGpio(void)
{
    EALLOW;

    GPIO_SetupPinOptions(10, GPIO_INPUT, GPIO_ASYNC);//U_Dr_Faulut_C
    GPIO_SetupPinOptions(94, GPIO_INPUT, GPIO_ASYNC);//V_Dr_Faulut_C
    GPIO_SetupPinOptions(93, GPIO_INPUT, GPIO_ASYNC);//W_Dr_Faulut_C

    GPIO_SetupPinOptions(90, GPIO_INPUT, GPIO_ASYNC);//U_Dr_Ready_C
    GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_ASYNC);//V_Dr_Ready_C
    GPIO_SetupPinOptions(86, GPIO_INPUT, GPIO_ASYNC);//W_Dr_Ready_C

    GPIO_SetupPinOptions(88, GPIO_INPUT, GPIO_ASYNC);//Ready_S1

    GPIO_SetupPinOptions(48, GPIO_INPUT, GPIO_ASYNC);//Fault DC meas_C

    GPIO_SetupPinOptions(53, GPIO_INPUT, GPIO_ASYNC);//5V fault A1
    GPIO_SetupPinOptions(52, GPIO_INPUT, GPIO_ASYNC);//5V fault A2
    GPIO_SetupPinOptions(51, GPIO_INPUT, GPIO_ASYNC);//5V fault A3

    EDIS;
}


void InitDigitalGPIO(void)
{
    EALLOW;

    GPIO_SetupPinOptions(RE38_THROTTLE_SW_GPIO, GPIO_INPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RE38_THROTTLE_SW_GPIO, 0, 0);

    EDIS;
}

void Configure_PWM(void)
{
    EALLOW;

    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(8,0,0);
    GpioDataRegs.GPASET.bit.GPIO8 = 1;

    PWM_1ch_UpDwnCnt_CNF(1,INV_PWM_TICKS,DEAD_TIME);
    PWM_1ch_UpDwnCnt_CNF(2,INV_PWM_TICKS,DEAD_TIME);
    PWM_1ch_UpDwnCnt_CNF(3,INV_PWM_TICKS,DEAD_TIME);

    EDIS;

    // Default PWM state is hardware-safe: all M1 phase outputs are forced low
    // by Trip Zone before the PWM pins are connected to the gate drivers.
    EALLOW;

    EPwm1Regs.TZSEL.bit.OSHT1 = 1;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;

    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;

    EPWM_M1_PhaseA_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseB_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseC_Regs.TZFRC.bit.OST = 0x1;

    EDIS;
    /*End TZ Force low*/

    // configure 2 and 3 as slaves
    EALLOW;

    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  //0x0 EPWMxSYNC
    EPwm1Regs.TBCTL.bit.PHSEN    = TB_DISABLE;
    EPwm1Regs.TBPHS.bit.TBPHS    = 0;
    EPwm1Regs.TBCTL.bit.PHSDIR   = TB_DOWN;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x11;

    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm2Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
    EPwm2Regs.TBPHS.bit.TBPHS    = 0;
    EPwm2Regs.TBCTL.bit.PHSDIR   = TB_DOWN;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x11;

    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm3Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
    EPwm3Regs.TBPHS.bit.TBPHS    = 0;
    EPwm3Regs.TBCTL.bit.PHSDIR   = TB_DOWN;
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x11;

    EDIS;

    // ***********************************
    // Set up GPIOs for PWM functions
    // **************************************
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

}


void EPWM1_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db)
{
    EALLOW;
        // Time Base SubModule Registers
    EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load same 027
    EPwm1Regs.TBPRD = period / 2; // PWM frequency = 1 / period
    EPwm1Regs.TBPHS.bit.TBPHS = 0;
    EPwm1Regs.TBCTR = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;   // same 027
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //same027
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV1;       //same 027

    EPwm1Regs.TBCTL.bit.PHSEN    = TB_DISABLE;     //027-EN
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"  027-0

    // Counter Compare Submodule Registers  same with 027, lack B
    EPwm1Regs.CMPA.bit.CMPA = (int)period/4; // set duty 0% initially
    EPwm1Regs.CMPB.bit.CMPB = (int)period/4; // set duty 0% initially
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    // Action Qualifier SubModule Registers  same with 027
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Set up the deadband  same with 027
    EPwm1Regs.DBCTL.bit.IN_MODE  = DBA_ALL;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm1Regs.DBRED = db;
    EPwm1Regs.DBFED = db;

    EDIS;
}


void EPWM2_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db)
{
    EALLOW;
        // Time Base SubModule Registers
    EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load same 027
    EPwm2Regs.TBPRD = period / 2; // PWM frequency = 1 / period
    EPwm2Regs.TBPHS.bit.TBPHS = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;   // same 027
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //same027
    EPwm2Regs.TBCTL.bit.CLKDIV    = TB_DIV1;       //same 027

    EPwm2Regs.TBCTL.bit.PHSEN    = TB_DISABLE;     //027-EN
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"  027-0

    // Counter Compare Submodule Registers  same with 027, lack B
    EPwm2Regs.CMPA.bit.CMPA = (int)period/4; // set duty 0% initially
    EPwm2Regs.CMPB.bit.CMPB = (int)period/4; // set duty 0% initially
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    // Action Qualifier SubModule Registers  same with 027
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Set up the deadband  same with 027
    EPwm2Regs.DBCTL.bit.IN_MODE  = DBA_ALL;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm2Regs.DBRED = db;
    EPwm2Regs.DBFED = db;

    EDIS;
}

void EPWM3_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db)
{
    EALLOW;
        // Time Base SubModule Registers
    EPwm3Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load same 027
    EPwm3Regs.TBPRD = period / 2; // PWM frequency = 1 / period
    EPwm3Regs.TBPHS.bit.TBPHS = 0;
    EPwm3Regs.TBCTR = 0;
    EPwm3Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;   // same 027
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //same027
    EPwm3Regs.TBCTL.bit.CLKDIV    = TB_DIV1;       //same 027

    EPwm3Regs.TBCTL.bit.PHSEN    = TB_DISABLE;     //027-EN
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"  027-0

    // Counter Compare Submodule Registers  same with 027, lack B
    EPwm3Regs.CMPA.bit.CMPA = (int)period/4; // set duty 0% initially
    EPwm3Regs.CMPB.bit.CMPB = (int)period/4; // set duty 0% initially
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    // Action Qualifier SubModule Registers  same with 027
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Set up the deadband  same with 027
    EPwm3Regs.DBCTL.bit.IN_MODE  = DBA_ALL;
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm3Regs.DBRED = db;
    EPwm3Regs.DBFED = db;

    EDIS;
}

void EPWM4_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db)
{
    EALLOW;
        // Time Base SubModule Registers
    EPwm4Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load same 027
    EPwm4Regs.TBPRD = period / 2; // PWM frequency = 1 / period
    EPwm4Regs.TBPHS.bit.TBPHS = 0;
    EPwm4Regs.TBCTR = 0;
    EPwm4Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;   // same 027
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //same027
    EPwm4Regs.TBCTL.bit.CLKDIV    = TB_DIV1;       //same 027

    EPwm4Regs.TBCTL.bit.PHSEN    = TB_DISABLE;     //027-EN
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"  027-0

    // Counter Compare Submodule Registers  same with 027, lack B
    EPwm4Regs.CMPA.bit.CMPA = (int)period/4; // set duty 0% initially
    EPwm4Regs.CMPB.bit.CMPB = (int)period/4; // set duty 0% initially
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    // Action Qualifier SubModule Registers  same with 027
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Set up the deadband  same with 027
    EPwm4Regs.DBCTL.bit.IN_MODE  = DBA_ALL;
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm4Regs.DBRED = db;
    EPwm4Regs.DBFED = db;

    EDIS;
}

void EPWM5_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db)
{
    EALLOW;
        // Time Base SubModule Registers
    EPwm5Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load same 027
    EPwm5Regs.TBPRD = period / 2; // PWM frequency = 1 / period
    EPwm5Regs.TBPHS.bit.TBPHS = 0;
    EPwm5Regs.TBCTR = 0;
    EPwm5Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;   // same 027
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //same027
    EPwm5Regs.TBCTL.bit.CLKDIV    = TB_DIV1;       //same 027

    EPwm5Regs.TBCTL.bit.PHSEN    = TB_DISABLE;     //027-EN
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"  027-0

    // Counter Compare Submodule Registers  same with 027, lack B
    EPwm5Regs.CMPA.bit.CMPA = (int)period/4; // set duty 0% initially
    EPwm5Regs.CMPB.bit.CMPB = (int)period/4; // set duty 0% initially
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    // Action Qualifier SubModule Registers  same with 027
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Set up the deadband  same with 027
    EPwm5Regs.DBCTL.bit.IN_MODE  = DBA_ALL;
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm5Regs.DBRED = db;
    EPwm5Regs.DBFED = db;

    EDIS;
}

void EPWM6_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db)
{
    EALLOW;
        // Time Base SubModule Registers
    EPwm6Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load same 027
    EPwm6Regs.TBPRD = period / 2; // PWM frequency = 1 / period
    EPwm6Regs.TBPHS.bit.TBPHS = 0;
    EPwm6Regs.TBCTR = 0;
    EPwm6Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;   // same 027
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       //same027
    EPwm6Regs.TBCTL.bit.CLKDIV    = TB_DIV1;       //same 027

    EPwm6Regs.TBCTL.bit.PHSEN    = TB_DISABLE;     //027-EN
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"  027-0

    // Counter Compare Submodule Registers  same with 027, lack B
    EPwm6Regs.CMPA.bit.CMPA = (int)period/4; // set duty 0% initially
    EPwm6Regs.CMPB.bit.CMPB = (int)period/4; // set duty 0% initially
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    // Action Qualifier SubModule Registers  same with 027
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Set up the deadband  same with 027
    EPwm6Regs.DBCTL.bit.IN_MODE  = DBA_ALL;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm6Regs.DBRED = db;
    EPwm6Regs.DBFED = db;

    EDIS;
}


