#include "HPMC_Settings.h"
#ifndef SERVECAP_H_
#define SERVECAP_H_

extern uint32_t eCAPt1,eCAPt2,eCAPT1,eCAPT2,eCAPD1,eCAPD2;
extern int eCAPt3,eCAPt4;
extern float TimeCap;
extern float ECAP1_duty;
extern float ECAP2_duty;
extern float ECAP3_duty;
#define Sysclock 5 //200MHz, Sysclock is 5ns

#include <stdint.h>

__interrupt void ecap1_isr(void);
__interrupt void ecap2_isr(void);
__interrupt void ecap3_isr(void);
void InitDshot1(void);
void InitDshot2(void);
void InitDshot3(void);
void InitECap1Gpio(uint16_t pin);
void InitECap2Gpio(uint16_t pin);
void InitECap3Gpio(uint16_t pin);
void ECAP1_measurement(void);
void ECAP2_measurement(void);
void ECAP3_measurement(void);
#endif /* SERVECAP_H_ */

