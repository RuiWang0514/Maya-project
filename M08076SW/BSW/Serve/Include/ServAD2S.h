#include "F28x_Project.h"

#include "HPMC_Settings.h"

void SPI_init(void);
void SPI_fifo_init(void);
void SPIB_GPIO_Init(void);

void AD2SB_Write(uint16_t addr,uint16_t data);
uint16_t AD2SB_Read(uint16_t addr);
void AD2SB_Regs_Init(void);
void AD2SB_Reset(void);

/* AD2S-A 引脚（当前硬件只接 AD2S-B，保留作引脚对照） */
#define AD2S_SAMPLE_H (GpioDataRegs.GPBSET.bit.GPIO60=1)
#define AD2S_SAMPLE_L (GpioDataRegs.GPBCLEAR.bit.GPIO60=1)
#define AD2S_RESET_H (GpioDataRegs.GPASET.bit.GPIO22=1)
#define AD2S_RESET_L (GpioDataRegs.GPACLEAR.bit.GPIO22=1)
#define AD2SA_CS_H (GpioDataRegs.GPBSET.bit.GPIO54=1)
#define AD2SA_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO54=1)

/* AD2S-B 引脚：CPU 侧 SPI 访问用，CLA 侧另有 Cla_AD2SB_* 宏 */
#define AD2SB_SAMPLE_H (GpioDataRegs.GPCSET.bit.GPIO64=1)
#define AD2SB_SAMPLE_L (GpioDataRegs.GPCCLEAR.bit.GPIO64=1)
#define AD2SB_WR_H (GpioDataRegs.GPCSET.bit.GPIO65=1)
#define AD2SB_WR_L (GpioDataRegs.GPCCLEAR.bit.GPIO65=1)
#define AD2SB_CS_H (GpioDataRegs.GPBSET.bit.GPIO59=1)
#define AD2SB_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO59=1)
