#ifndef USER_CONFIGPWM_H_
#define USER_CONFIGPWM_H_

//U1V1W1
#define SET_U1_RI  (GpioDataRegs.GPASET.bit.GPIO13 = 1)
#define CLR_U1_RI  (GpioDataRegs.GPACLEAR.bit.GPIO13 = 1)
#define SET_U1_LE  (GpioDataRegs.GPCSET.bit.GPIO79 = 1)
#define CLR_U1_LE  (GpioDataRegs.GPCCLEAR.bit.GPIO79 = 1)
#define SET_U1_PS  (GpioDataRegs.GPCSET.bit.GPIO68 = 1)
#define CLR_U1_PS  (GpioDataRegs.GPCCLEAR.bit.GPIO68 = 1)

#define SET_V1_RI  (GpioDataRegs.GPCSET.bit.GPIO87 = 1)
#define CLR_V1_RI  (GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1)
#define SET_V1_LE  (GpioDataRegs.GPCSET.bit.GPIO82 = 1)
#define CLR_V1_LE  (GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1)
#define SET_V1_PS  (GpioDataRegs.GPCSET.bit.GPIO69 = 1)
#define CLR_V1_PS  (GpioDataRegs.GPCCLEAR.bit.GPIO69 = 1)

#define SET_W1_RI  (GpioDataRegs.GPCSET.bit.GPIO83 = 1)
#define CLR_W1_RI  (GpioDataRegs.GPCCLEAR.bit.GPIO83 = 1)
#define SET_W1_LE  (GpioDataRegs.GPCSET.bit.GPIO81 = 1)
#define CLR_W1_LE  (GpioDataRegs.GPCCLEAR.bit.GPIO81 = 1)
#define SET_W1_PS  (GpioDataRegs.GPCSET.bit.GPIO67 = 1)
#define CLR_W1_PS  (GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1)


#include "HPMC_Settings.h"

void EPWM1_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db) ;
void EPWM2_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db) ;
void EPWM3_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db) ;
void EPWM4_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db) ;
void EPWM5_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db) ;
void EPWM6_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db) ;
void Configure_PWM(void);
void InitTZGpio(void);
void InitFaultGpio(void);
void InitDigitalGPIO(void);
void Fault_GPIO_init(void);
void GateDriver_EnableOutputs(void);
void Cla_GPIO_Init(void);



#endif /* USER_CONFIGPWM_H_ */
