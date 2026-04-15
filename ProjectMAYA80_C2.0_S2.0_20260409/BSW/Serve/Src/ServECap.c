#include "ServECap.h"
#include "HPMC_Settings.h"
#include "Linear_Conversion.h"
#include "ServSysTick.h"
#define CapErr 0.8
float CapDetect = 0;
uint16_t ThrottleErr = 0;

uint32_t eCAPt1,eCAPt2,eCAPT1,eCAPT2,eCAPT,eCAPD1,eCAPD2,crc_cor_cnt,crc_error_cnt=0;
//float TimeCap;
int eCAPt3,eCAPt4=0;
int eCAPflag = 0;
int eCAPdT = 0;
uint32_t DshotSystick=0;
//static uint32_t lastSystick = 0;
extern uint32_t IsrTicker;

#define PROTOCOL_UNKNOW     0
#define PROTOCOL_PWM        1
#define PROTOCOL_DSHOT150   2
#define PROTOCOL_DSHOT300   3
#define PROTOCOL_DSHOT600   4
#define PROTOCOL_ERROR      5
void Dshot_In_bit(uint16_t inbit);

uint16_t PROTOCOL = PROTOCOL_UNKNOW;
uint32_t iThrottle = 0;
uint16_t enTelemetry = 0;
uint16_t pwmerrorticker=0;
uint32_t Dshotticker=0;
uint32_t ecapticker=0;
uint16_t ecapflag=0;
uint16_t ecapstartflag=0;

uint32_t  dshot_rcv_buffer    =   0x0000;
uint32_t  dshot_rcv_buffer12    =   0x0000;
uint32_t  dshot_rcv_buffer8    =   0x0000;
uint32_t  dshot_rcv_buffer4    =   0x0000;
uint32_t  dshot_rcv_buffer128    =   0x0000;
uint16_t   dshot_rcv_count     =   0;



//__interrupt void ecap1_isr(void)
//{
////    GpioDataRegs.GPCSET.bit.GPIO85 = 1;
//    ecapticker++;
//   Uint32 pulse_width,cycle_time=0;
////   ECap1Regs.ECCLR.bit.CEVT4 = 1;
//   ECap1Regs.ECCLR.bit.CEVT2 = 1;
//   ECap1Regs.ECCLR.bit.INT = 1;
//   ECap1Regs.ECCTL2.bit.REARM = 1;
//
//   // Acknowledge this interrupt to receive more interrupts from group 4
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
//
//   eCAPt1 = ECap1Regs.CAP1;   // Fetch Time-Stamp captured at T1  DutyOnTime1
//   eCAPt2 = ECap1Regs.CAP2;   // Fetch Time-Stamp captured at T2  DutyOffTime1
////   eCAPt3 = ECap1Regs.CAP3;   // Fetch Time-Stamp captured at T3  DutyOnTime2
////   eCAPt4 = ECap1Regs.CAP4;   // Fetch Time-Stamp captured at T4  DutyOffTime2
//   eCAPT1 = eCAPt1 + eCAPt2;          //Period1
////   eCAPT2 = eCAPt3 + eCAPt4;          //Period2
//   pulse_width = eCAPt1;
////   pulse_width = eCAPt1;
//   eCAPdT = eCAPT1-eCAPT;
//   if ((eCAPdT<1000) && (eCAPdT>-1000) && (PROTOCOL == PROTOCOL_UNKNOW))
//   {
//       eCAPflag = 1;
//   }
//   else
//       {
//       eCAPflag = 0;
////       pulse_width = 0;
////       cycle_time = 0;
////       return;
//       eCAPT = eCAPT1;
//       }
//
//
//
//   if ((PROTOCOL == PROTOCOL_UNKNOW) &&(eCAPflag == 1))
//   {
//        cycle_time = eCAPT1;
//       if(cycle_time > 1500) //if pulse width > 7500ns (1500*5) should be PWM mode
//           PROTOCOL = PROTOCOL_PWM;
//       else if (cycle_time > 1200 && cycle_time < 1400) //dshot150 cycle time 6666.67ns
//           PROTOCOL = PROTOCOL_DSHOT150;
//       else if (cycle_time > 600 && cycle_time < 700)   //dshot300 cycle time 3333.33ns
//           PROTOCOL = PROTOCOL_DSHOT300;
//       else if (cycle_time > 250 && cycle_time < 400)   //dshot600 cycle time 1666.67ns
//           PROTOCOL = PROTOCOL_DSHOT600;
//   }
//
//   switch(PROTOCOL)
//   {
//   case PROTOCOL_UNKNOW:
//       break;
//   case PROTOCOL_PWM:
//
//           if ((pulse_width > 180000) && (pulse_width < 420000))  //check range : 1000us to 2000us
//           {
//           iThrottle=pulse_width;
////           TimeCap = (iThrottle*Sysclock)/1000;
//           }
//           else pwmerrorticker++;
//
////       if(TimeCap>1500) TimeCap=1800;
////       if(TimeCap>MaxThrot) TimeCap=MaxThrot;
////       if(TimeCap>=1000 && TimeCap<1200) TimeCap=TimeCap-0;
////       else if(TimeCap>=1200 && TimeCap<1911) TimeCap=TimeCap-1;
////       else if(TimeCap>=1911) TimeCap=TimeCap-2;
//       break;
//   case PROTOCOL_DSHOT150:
////       if (pulse_width > 900 && eCAPflag == 1) // > 4500ns high
//       if (pulse_width > 900)
////           Dshot_In_bit( 1 , IsrTicker*5 );
//           Dshot_In_bit(1);
//       else
////           Dshot_In_bit( 0 , IsrTicker*5 );
//           Dshot_In_bit(0);
//       break;
//   case PROTOCOL_DSHOT300:
////       if (pulse_width > 400 && eCAPflag == 1) // > 2000ns high
//       if (pulse_width > 400)
////           Dshot_In_bit( 1 , IsrTicker*5 );
//           Dshot_In_bit(1);
//       else
////           Dshot_In_bit( 0 , IsrTicker*5 );
//           Dshot_In_bit(0);
//       break;
//   case PROTOCOL_DSHOT600:
////       if (pulse_width > 200 && eCAPflag == 1) // > 1000ns high
//       if (pulse_width > 200)
////           Dshot_In_bit( 1 , IsrTicker*5 );
//           Dshot_In_bit(1);
//       else
////           Dshot_In_bit( 0 , IsrTicker*5 );
//           Dshot_In_bit(0);
//       break;
//   default:
//       break;
//   }
//
////   GpioDataRegs.GPCCLEAR.bit.GPIO85 = 1;
//}



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


//void Dshot_In_bit(Uint8 inbit)
//{
////    static Uint32 lastSystick = 0;
//
////    eCAPt4=eCAPt1-eCAPt3;
////    eCAPt3=eCAPt2;
////    if(ecapflag==0)
////    {
////           if((eCAPt4>5000)||(eCAPt4<-5000))
////           {
////             ecapstartflag=1;
////             ecapflag=1;
////           }
////    }
//
//
////    if ( ( inSystick - lastSystick ) > 1000)   // clean all value if timeout
////    {
////        dshot_rcv_buffer = 0;
////        dshot_rcv_count = 0;
////        lastSystick = inSystick;
////        DshotSystick++;
////        return;
////    }
////    lastSystick = inSystick;
//  if(ecapstartflag==1)
//   {
//    dshot_rcv_buffer = (dshot_rcv_buffer << 1 ) | inbit;
//    dshot_rcv_count++;
//    if ( dshot_rcv_count == 16 )
//    {
////        eCAPt1 = 0;
////        eCAPt2 = 0;
////        ECap1Regs.CAP1 = 0;
////        ECap1Regs.CAP2 = 0;
////        dshot_rcv_count = 0;
////        if(dshot_rcv_buffer==0)
////        {
////            ecapstartflag=0;
////            ecapflag=0;
////            return;
////        }
////        dshot_rcv_buffer12 = (dshot_rcv_buffer & 0xf000) >> 12;
////        dshot_rcv_buffer8 = (dshot_rcv_buffer & 0xf00) >> 8;
////        dshot_rcv_buffer4 = (dshot_rcv_buffer & 0xf0) >> 4;
////        dshot_rcv_buffer128 = dshot_rcv_buffer12 ^ dshot_rcv_buffer8;
////        if( (dshot_rcv_buffer128 ^ dshot_rcv_buffer4) == (dshot_rcv_buffer & 0xf))
////        if ( ((dshot_rcv_buffer & 0xf000) >> 12 ) ^ ((dshot_rcv_buffer & 0xf00) >> 8 ) ^ ((dshot_rcv_buffer & 0xf0) >> 4 ) == (dshot_rcv_buffer & 0xf)  )
//        if(((((dshot_rcv_buffer & 0xf000) >> 12) ^ ((dshot_rcv_buffer & 0xf00) >> 8)) ^ ((dshot_rcv_buffer & 0xf0) >> 4)) == (dshot_rcv_buffer & 0xf))
//        {
//            //if CRC success
//            iThrottle = dshot_rcv_buffer >> 5 ;
////            enTelemetry = (dshot_rcv_buffer & 0x0010) >> 4;
////            TimeCap = (iThrottle >>1) + 977;
////            if (TimeCap<1000) TimeCap=1000;
////            if (TimeCap>1999) TimeCap=2000;
//////            if (TimeCap<1140) Dshotticker++;
////            TimeCap = (int)(TimeCap);
//////            if(TimeCap>MaxThrot) TimeCap=MaxThrot;
////            crc_cor_cnt++;
//
//
//        }
//        else crc_error_cnt++;
//        dshot_rcv_buffer = 0;
//        ecapstartflag=0;
////        ecapflag=0;
//    }
//   }
//  if(eCAPt2>8000)
//  {
////      if(dshot_rcv_count<16)
////          {
////          ecapisr1++;
////          GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1;
////          }
////      else ecapisr2++;
////      GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1;
//      ecapstartflag=1;
//      dshot_rcv_count=0;
//      dshot_rcv_buffer = 0;
////      ecapisr++;
//  }
//}
//
//
////void Dshot_In_bit(Uint8 inbit,Uint32 inSystick)
////{
//////    static Uint32 lastSystick = 0;
////    if ( ( inSystick - lastSystick ) > 1000)   // clean all value if timeout
////    {
////        dshot_rcv_buffer = 0;
////        dshot_rcv_count = 0;
////        lastSystick = inSystick;
////        DshotSystick++;
////        return;
////    }
//////    lastSystick = inSystick;
////    dshot_rcv_buffer = (dshot_rcv_buffer << 1 ) | inbit;
////    dshot_rcv_count++;
////    if ( dshot_rcv_count >= 16 )
////    {
////        eCAPt1 = 0;
////        eCAPt2 = 0;
////        ECap1Regs.CAP1 = 0;
////        ECap1Regs.CAP2 = 0;
////        dshot_rcv_count = 0;
////        dshot_rcv_buffer12 = (dshot_rcv_buffer & 0xf000) >> 12;
////        dshot_rcv_buffer8 = (dshot_rcv_buffer & 0xf00) >> 8;
////        dshot_rcv_buffer4 = (dshot_rcv_buffer & 0xf0) >> 4;
////        dshot_rcv_buffer128 = dshot_rcv_buffer12 ^ dshot_rcv_buffer8;
////        if( (dshot_rcv_buffer128 ^ dshot_rcv_buffer4) == (dshot_rcv_buffer & 0xf))
//////        if ( ((dshot_rcv_buffer & 0xf000) >> 12 ) ^ ((dshot_rcv_buffer & 0xf00) >> 8 ) ^ ((dshot_rcv_buffer & 0xf0) >> 4 ) == (dshot_rcv_buffer & 0xf)  )
////        {
////            //if CRC success
////            iThrottle = dshot_rcv_buffer >> 5 ;
////            enTelemetry = (dshot_rcv_buffer & 0x0010) >> 4;
////            TimeCap = iThrottle * 0.5 + 977;
////            if (TimeCap<1000) TimeCap=1000;
////            if (TimeCap>1999) TimeCap=2000;
//////            if (TimeCap<1140) Dshotticker++;
////            TimeCap = (int)(TimeCap);
//////            if(TimeCap>MaxThrot) TimeCap=MaxThrot;
////            crc_cor_cnt++;
////
////        }
////        else crc_error_cnt++;
////        dshot_rcv_buffer = 0;
////    }
////}
//

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

   //EPwm1Regs.TBCTR;
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
//    if(ECAP1_FLAG==0)
//    {
//    ECAP1_FLAG=1;
//    ECAP1_duty=0.90;
//    }
//    else if(ECap1_cap1+ECap1_cap2>4000)
//    {
//        ECAP1_duty=0.90;
//    }
//    else
//    {
//    //ECAP1_duty=(float)ECap1_cap1/(ECap1_cap2+ECap1_cap1);
//       ECAP1_duty=(float)(4000-ECap1_cap2)/(4000);
//    }

//    if(ECap1_cap1+ECap1_cap2<4100||ECap1_cap1+ECap1_cap2>3800)
//    {
        ECAP1_duty=(float)(4000-ECap1_cap2)/(4000);
//    }

    ECap1Regs.ECCLR.bit.CEVT2 = 1;
    ECap1Regs.ECCTL2.bit.REARM = 1;

}

void ECAP2_measurement(void)
{
    ECap2_cap1=ECap2Regs.CAP1;
    ECap2_cap2=ECap2Regs.CAP2;
//    if(ECAP2_FLAG==0)
//    {
//        ECAP2_FLAG=1;
//        ECAP2_duty=0.90;
//    }
//    else if(ECap2_cap1+ECap2_cap2>4000)
//    {
//        ECAP2_duty=0.90;
//    }
//    else
//    {
//        //ECAP2_duty=(float)ECap2_cap1/(ECap2_cap2+ECap2_cap1);
//        ECAP2_duty=(float)(4000-ECap2_cap2)/(4000);
//    }

//    if(ECap2_cap1+ECap2_cap2<4100||ECap2_cap1+ECap2_cap2>3800)
//    {
        ECAP2_duty=(float)(4000-ECap2_cap2)/(4000);
//    }

    ECap2Regs.ECCLR.bit.CEVT2 = 1;
    ECap2Regs.ECCTL2.bit.REARM = 1;
}


void ECAP3_measurement(void)
{
    ECap3_cap1=ECap3Regs.CAP1;
    ECap3_cap2=ECap3Regs.CAP2;
//    if(ECAP3_FLAG==0)
//    {
//        ECAP3_FLAG=1;
//        ECAP3_duty=0.90;
//    }
//    else if(ECap3_cap1+ECap3_cap2>4000||ECap3_cap1+ECap3_cap2<3800)
//    {
//        ECAP3_duty=0.90;
//    }
//    else
//    {
//        //ECAP3_duty=(float)ECap3_cap1/(ECap3_cap2+ECap3_cap1);
//        ECAP3_duty=(float)(4000-ECap3_cap2)/(4000);
//
//    }

//    if(ECap3_cap1+ECap3_cap2<4100||ECap3_cap1+ECap3_cap2>3800)
//    {
        ECAP3_duty=(float)(4000-ECap3_cap2)/(4000);
//    }

    ECap3Regs.ECCLR.bit.CEVT2 = 1;
    ECap3Regs.ECCTL2.bit.REARM = 1;
}
