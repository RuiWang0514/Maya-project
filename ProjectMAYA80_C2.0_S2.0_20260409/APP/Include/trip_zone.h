#include "HPMC_Settings.h"

interrupt void epwm1_tzint_isr(void);
interrupt void epwm2_tzint_isr(void);
interrupt void epwm3_tzint_isr(void);
interrupt void epwm7_tzint_isr(void);
interrupt void epwm8_tzint_isr(void);
interrupt void epwm9_tzint_isr(void);

uint32_t  EPwm1TZIntCount;
uint32_t  EPwm2TZIntCount;
uint32_t  EPwm3TZIntCount;
uint32_t  EPwm7TZIntCount;
uint32_t  EPwm8TZIntCount;
uint32_t  EPwm9TZIntCount;
