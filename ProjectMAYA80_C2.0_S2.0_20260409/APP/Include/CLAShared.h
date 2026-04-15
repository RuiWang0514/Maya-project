#ifndef CLASHARED_H
#define CLASHARED_H

#ifdef __cplusplus
extern "C" {
#endif


  #include "IQmathLib.h"
  #include "F28x_Project.h"

// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them

extern float Cla_angle;
extern float Cla_angleB;
extern float Cla_mech_angleB;
//extern float Cla_angleB111;
//extern float Cla_mech_angleB111;
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
//__interrupt void Cla1Task3();
//__interrupt void Cla1Task4();
//__interrupt void Cla1Task5();
//__interrupt void Cla1Task6();
//__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#define Cla_AD2SB_SAMPLE_H (GpioDataRegs.GPCSET.bit.GPIO64=1)
#define Cla_AD2SB_SAMPLE_L (GpioDataRegs.GPCCLEAR.bit.GPIO64=1)
#define Cla_AD2SB_WR_H (GpioDataRegs.GPCSET.bit.GPIO65=1)
#define Cla_AD2SB_WR_L (GpioDataRegs.GPCCLEAR.bit.GPIO65=1)
#define Cla_AD2SB_CS_H (GpioDataRegs.GPBSET.bit.GPIO59=1)
#define Cla_AD2SB_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO59=1)

void Cla_AD2SB_Write(uint16_t addr,uint16_t data);
uint16_t Cla_AD2SB_Read(uint16_t addr);
void Cla_AD2SB_Reset(void);
uint16_t Cla_AD2SB_Postion_Read(void);
void Cla_delay(uint16_t);
extern uint32_t cal_ecap1;
extern uint32_t cal_ecap2;
extern float Cla_ALIGN_ANGLE;
extern uint32_t cal_ecap3;
extern uint32_t cal_ecap4;
extern uint16_t cla_rcv_countflg;
extern float TimeCap;

#define CLA_PROG_ENABLE      0x0001
#define CLARAM0_ENABLE       0x0010
#define CLARAM1_ENABLE       0x0020
#define CLARAM2_ENABLE       0x0040
#define CLA_RAM0CPUE         0x0100
#define CLA_RAM1CPUE         0x0200
#define CLA_RAM2CPUE         0x0400

#define cla_Sysclock 5

#define cla_PROTOCOL_UNKNOW     0
#define cla_PROTOCOL_PWM        1
#define cla_PROTOCOL_DSHOT150   2
#define cla_PROTOCOL_DSHOT300   3
#define cla_PROTOCOL_DSHOT600   4

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of CLA_SHARED definition
