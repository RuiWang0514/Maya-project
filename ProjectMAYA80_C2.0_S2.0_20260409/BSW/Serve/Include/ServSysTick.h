#ifndef USER_SYSTICK_H_
#define USER_SYSTICK_H_

#include "HPMC_Settings.h"
uint64_t micros(void);
uint32_t mils(void);
void InitSystick(void);

#endif /* USER_SYSTICK_H_ */
