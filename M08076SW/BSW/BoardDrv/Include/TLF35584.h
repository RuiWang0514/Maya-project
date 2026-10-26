#include <stdint.h>
#include "F28x_Project.h"

#include "HPMC_Settings.h"

void TLF35584_spi_init(void);

#define TLF35584_CS_H (GpioDataRegs.GPBSET.bit.GPIO57=1)
//#define TLF35584_CS_H (GpioDataRegs.GPBCLEAR.bit.GPIO57=1)
#define TLF35584_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO57=1)
#define TLF35584_WDI_H (GpioDataRegs.GPCSET.bit.GPIO78=1)
#define TLF35584_WDI_L (GpioDataRegs.GPCCLEAR.bit.GPIO78=1)


//address
#define  PROTCFG   0x03      /*protect register*/
#define  SYSPCFG1 0x05      /*err pin*/
#define  WDCFG0   0x06      /*watchdog enable or disable*/
#define  WWDCFG1 0x0A       /*write open window time*/
#define  WWDCFG0 0x09       /*write clse window time*/

#define PROTSTAT 0x28        /*protect register's status*/
#define RWDCFG0  0x0D        /*watchdog status*/
#define RSYSPCFG1 0x0C       /*err pin status*/
#define RWWDCFG1  0x11       /*read open window time*/
#define RWWDCFG0  0x10       /*read clse window time*/
#define DEVSTAT  0x27       //device status


uint16_t calcParity(uint16_t data);
uint16_t TLF35584_SPI_Transaction(uint16_t cmd);
uint16_t TLF35584_ReadReg(uint16_t addr);
void TLF35584_WriteReg(uint16_t addr, uint16_t data);
void TLF35584_Trigger_WWD(void);
void TLF35584_GoToNormal(void);
void init_ERR_Timer(void);
void TLF35584_DisableERRandWWD(void);
void TLF35584_EnableWWDandERR(void);
void TLF35584_flag_error_clear(void);
void TLF35584_ConfigWWD(uint16_t closed_time, uint16_t open_time);
void InitGpio43And50(void);
void TLF35584_StartWatchdog(void);
