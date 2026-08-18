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
extern float Cla_ResolverElecTheta_pu;
extern float Cla_ResolverMechTheta_pu;
extern uint16_t Cla_AD2SB_Fault;
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task8();

#define Cla_AD2SB_SAMPLE_H (GpioDataRegs.GPCSET.bit.GPIO64=1)
#define Cla_AD2SB_SAMPLE_L (GpioDataRegs.GPCCLEAR.bit.GPIO64=1)
#define Cla_AD2SB_WR_H (GpioDataRegs.GPCSET.bit.GPIO65=1)
#define Cla_AD2SB_WR_L (GpioDataRegs.GPCCLEAR.bit.GPIO65=1)
#define Cla_AD2SB_CS_H (GpioDataRegs.GPBSET.bit.GPIO59=1)
#define Cla_AD2SB_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO59=1)

uint16_t Cla_AD2SB_Read(uint16_t addr);
uint16_t Cla_AD2SB_Position_Read(void);
void Cla_delay(uint16_t);

#define CLA_PROG_ENABLE      0x0001
#define CLARAM0_ENABLE       0x0010
#define CLARAM1_ENABLE       0x0020
#define CLARAM2_ENABLE       0x0040
#define CLA_RAM0CPUE         0x0100
#define CLA_RAM1CPUE         0x0200
#define CLA_RAM2CPUE         0x0400

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of CLA_SHARED definition
