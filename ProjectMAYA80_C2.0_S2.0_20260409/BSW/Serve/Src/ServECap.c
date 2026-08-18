#include "ServECap.h"
#include "HPMC_Settings.h"
#include "ServSysTick.h"

extern uint32_t IsrTicker;

#define PROTOCOL_UNKNOWN    0
#define PROTOCOL_PWM        1
#define PROTOCOL_DSHOT150   2
#define PROTOCOL_DSHOT300   3
#define PROTOCOL_DSHOT600   4
#define PROTOCOL_ERROR      5
void Dshot_In_bit(uint16_t inbit);



void InitDshot1(void)
{
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.ECAP1_INT = &ecap1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    ECap1Regs.ECEINT.all = 0x0000;        // Disable all capture interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;         // Clear all CAP interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;     // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;   // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;    // One-shot
//    ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;      // Stop at 4 events
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 1;
    ECap1Regs.ECCTL1.bit.CAP1POL = 1;        // Falling edge
    ECap1Regs.ECCTL1.bit.CAP2POL = 0;        // Rising edge
//    ECap1Regs.ECCTL1.bit.CAP3POL = 1;        // Falling edge
//    ECap1Regs.ECCTL1.bit.CAP4POL = 0;        // Rising edge
    ECap1Regs.ECCTL1.bit.CTRRST1 = 1;        // Difference operation
    ECap1Regs.ECCTL1.bit.CTRRST2 = 1;        // Difference operation
//    ECap1Regs.ECCTL1.bit.CTRRST3 = 1;        // Difference operation
//    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;        // Difference operation
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;       // Enable sync in
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;      // Pass through
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;        // Enable capture units

    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;      // Start Counter
    ECap1Regs.ECCTL2.bit.REARM = 1;          // arm one-shot
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;        // Enable CAP1-CAP4 register loads
//    ECap1Regs.ECEINT.bit.CEVT4 = 1;          // 4 events = interrupt
    ECap1Regs.ECEINT.bit.CEVT2 = 1;
}

void InitDshot2(void)
{
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.ECAP2_INT = &ecap2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    ECap2Regs.ECEINT.all = 0x0000;        // Disable all capture interrupts
    ECap2Regs.ECCLR.all = 0xFFFF;         // Clear all CAP interrupt flags
    ECap2Regs.ECCTL1.bit.CAPLDEN = 0;     // Disable CAP1-CAP4 register loads
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;   // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap2Regs.ECCTL2.bit.CONT_ONESHT = 1;    // One-shot
//    ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;      // Stop at 4 events
    ECap2Regs.ECCTL2.bit.STOP_WRAP = 1;
    ECap2Regs.ECCTL1.bit.CAP1POL = 1;        // Falling edge
    ECap2Regs.ECCTL1.bit.CAP2POL = 0;        // Rising edge
//    ECap1Regs.ECCTL1.bit.CAP3POL = 1;        // Falling edge
//    ECap1Regs.ECCTL1.bit.CAP4POL = 0;        // Rising edge
    ECap2Regs.ECCTL1.bit.CTRRST1 = 1;        // Difference operation
    ECap2Regs.ECCTL1.bit.CTRRST2 = 1;        // Difference operation
//    ECap1Regs.ECCTL1.bit.CTRRST3 = 1;        // Difference operation
//    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;        // Difference operation
    ECap2Regs.ECCTL2.bit.SYNCI_EN = 0;       // Enable sync in
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;      // Pass through
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;        // Enable capture units

    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;      // Start Counter
    ECap2Regs.ECCTL2.bit.REARM = 1;          // arm one-shot
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;        // Enable CAP1-CAP4 register loads
//    ECap1Regs.ECEINT.bit.CEVT4 = 1;          // 4 events = interrupt
    ECap2Regs.ECEINT.bit.CEVT2 = 1;
}

void InitDshot3(void)
{
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.ECAP3_INT = &ecap3_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    ECap3Regs.ECEINT.all = 0x0000;        // Disable all capture interrupts
    ECap3Regs.ECCLR.all = 0xFFFF;         // Clear all CAP interrupt flags
    ECap3Regs.ECCTL1.bit.CAPLDEN = 0;     // Disable CAP1-CAP4 register loads
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;   // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap3Regs.ECCTL2.bit.CONT_ONESHT = 1;    // One-shot
//    ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;      // Stop at 4 events
    ECap3Regs.ECCTL2.bit.STOP_WRAP = 1;
    ECap3Regs.ECCTL1.bit.CAP1POL = 1;        // Falling edge
    ECap3Regs.ECCTL1.bit.CAP2POL = 0;        // Rising edge
//    ECap1Regs.ECCTL1.bit.CAP3POL = 1;        // Falling edge
//    ECap1Regs.ECCTL1.bit.CAP4POL = 0;        // Rising edge
    ECap3Regs.ECCTL1.bit.CTRRST1 = 1;        // Difference operation
    ECap3Regs.ECCTL1.bit.CTRRST2 = 1;        // Difference operation
//    ECap1Regs.ECCTL1.bit.CTRRST3 = 1;        // Difference operation
//    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;        // Difference operation
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 0;       // Enable sync in
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;      // Pass through
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;        // Enable capture units

    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;      // Start Counter
    ECap3Regs.ECCTL2.bit.REARM = 1;          // arm one-shot
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;        // Enable CAP1-CAP4 register loads
//    ECap1Regs.ECEINT.bit.CEVT4 = 1;          // 4 events = interrupt
    ECap3Regs.ECEINT.bit.CEVT2 = 1;
}


void InitECap1Gpio(uint16_t pin)
{
    EALLOW;
    InputXbarRegs.INPUT7SELECT = pin;       // Set eCAP1 source to GPIO-pin
    EDIS;
}
void InitECap2Gpio(uint16_t pin)
{
    EALLOW;
    InputXbarRegs.INPUT8SELECT = pin;       // Set eCAP2 source to GPIO-pin
    EDIS;
}

void InitECap3Gpio(uint16_t pin)
{
    EALLOW;
    InputXbarRegs.INPUT9SELECT = pin;       // Set eCAP3 source to GPIO-pin
    EDIS;
}

int ECap1_cap1=0;
int ECap1_cap2=0;
bool ECAP1_FLAG=0;
float ECAP1_duty=0.9;
__interrupt void ecap1_isr(void)
{
   ECap1_cap1=ECap1Regs.CAP1;
   ECap1_cap2=ECap1Regs.CAP2;
   if(ECAP1_FLAG==0)
   {
       ECAP1_FLAG=1;
       ECAP1_duty=0.90;
   }
   else
   {
       ECAP1_duty=(float)ECap1_cap1/(ECap1_cap2+ECap1_cap1);
   }

   ECap1Regs.ECCLR.bit.CEVT2 = 1;
   ECap1Regs.ECCLR.bit.INT = 1;
   ECap1Regs.ECCTL2.bit.REARM = 1;

   //
   // Acknowledge this __interrupt to receive more __interrupts from group 4
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

}
int ECap2_cap1=0;
int ECap2_cap2=0;
bool ECAP2_FLAG=0;
float ECAP2_duty=0.9;
__interrupt void ecap2_isr(void)
{
   ECap2_cap1=ECap2Regs.CAP1;
   ECap2_cap2=ECap2Regs.CAP2;
   if(ECAP2_FLAG==0)
   {
       ECAP2_FLAG=1;
       ECAP2_duty=0.90;
   }
   else
   {
       ECAP2_duty=(float)ECap2_cap1/(ECap2_cap2+ECap2_cap1);
   }

   ECap2Regs.ECCLR.bit.CEVT2 = 1;
   ECap2Regs.ECCLR.bit.INT = 1;
   ECap2Regs.ECCTL2.bit.REARM = 1;

   //
   // Acknowledge this __interrupt to receive more __interrupts from group 4
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
int ECap3_cap1=0;
int ECap3_cap2=0;
bool ECAP3_FLAG=0;
float ECAP3_duty=0.9;
__interrupt void ecap3_isr(void)
{
   ECap3_cap1=ECap3Regs.CAP1;
   ECap3_cap2=ECap3Regs.CAP2;
   if(ECAP3_FLAG==0)
   {
       ECAP3_FLAG=1;
       ECAP3_duty=0.90;
   }
   else
   {
       ECAP3_duty=(float)ECap3_cap1/(ECap3_cap2+ECap3_cap1);
   }

   ECap3Regs.ECCLR.bit.CEVT2 = 1;
   ECap3Regs.ECCLR.bit.INT = 1;
   ECap3Regs.ECCTL2.bit.REARM = 1;

   //
   // Acknowledge this __interrupt to receive more __interrupts from group 4
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
void ECAP1_measurement(void)
{
    ECap1_cap1=ECap1Regs.CAP1;
    ECap1_cap2=ECap1Regs.CAP2;

    ECAP1_duty=(float)(4000-ECap1_cap2)/(4000);

    ECap1Regs.ECCLR.bit.CEVT2 = 1;
    ECap1Regs.ECCTL2.bit.REARM = 1;

}

void ECAP2_measurement(void)
{
    ECap2_cap1=ECap2Regs.CAP1;
    ECap2_cap2=ECap2Regs.CAP2;

    ECAP2_duty=(float)(4000-ECap2_cap2)/(4000);

    ECap2Regs.ECCLR.bit.CEVT2 = 1;
    ECap2Regs.ECCTL2.bit.REARM = 1;
}


void ECAP3_measurement(void)
{
    ECap3_cap1=ECap3Regs.CAP1;
    ECap3_cap2=ECap3Regs.CAP2;

    ECAP3_duty=(float)(4000-ECap3_cap2)/(4000);

    ECap3Regs.ECCLR.bit.CEVT2 = 1;
    ECap3Regs.ECCTL2.bit.REARM = 1;
}
