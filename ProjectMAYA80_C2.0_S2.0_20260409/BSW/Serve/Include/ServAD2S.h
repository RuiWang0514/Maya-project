#include "F28x_Project.h"

#include "HPMC_Settings.h"

void SPI_init(void);
void SPI_fifo_init(void);
void SPIA_GPIO_Init(void);
void SPIB_GPIO_Init(void);
void SPIC_GPIO_Init(void);

void AD2S_Write(uint16_t addr,uint16_t data);
void AD2SB_Write(uint16_t addr,uint16_t data);
//void AD2SC_Write(uint16_t addr,uint16_t data);

uint16_t AD2S_Read(uint16_t addr);
uint16_t AD2SB_Read(uint16_t addr);
//uint16_t AD2SC_Read(uint16_t addr);

void AD2S_Regs_Init(void);
void AD2SB_Regs_Init(void);
//void AD2SC_Regs_Init(void);

void AD2S_Reset(void);
void AD2SB_Reset(void);
//void AD2SC_Reset(void);

uint16_t AD2S_Speed_Read(void);
uint16_t AD2SB_Speed_Read(void);
//uint16_t AD2SC_Speed_Read(void);

uint16_t AD2S_Position_Read(void);
uint16_t AD2SB_Position_Read(void);
//uint16_t AD2SC_Position_Read(void);

void AD2S_Write_DA7F(uint16_t addr);

void AD2S_Angle_cal(void);
void AD2SB_Angle_cal(void);
//void AD2SC_Angle_cal(void);

//uint16_t AD2S1210_READ(uint16_t addr);
//void Delay(uint16_t nCount) ;
////GPIO3 --- WR/FSYNC
//#define NCS_L (GpioDataRegs.GPACLEAR.bit.GPIO3=1)
//#define NCS_H (GpioDataRegs.GPASET.bit.GPIO3=1)
////GPIO4 --- SCLK
//#define SCLK_L (GpioDataRegs.GPACLEAR.bit.GPIO4=1)
//#define SCLK_H (GpioDataRegs.GPASET.bit.GPIO4=1)
////GPIO5 --- SDI
//#define SDI_L (GpioDataRegs.GPACLEAR.bit.GPIO5=1)
//#define SDI_H (GpioDataRegs.GPASET.bit.GPIO5=1)
////GPIO6 --- SDO
//#define SDO (GpioDataRegs.GPADAT.bit.GPIO6)

#define AD2S_SAMPLE_H (GpioDataRegs.GPBSET.bit.GPIO60=1)
#define AD2S_SAMPLE_L (GpioDataRegs.GPBCLEAR.bit.GPIO60=1)

#define AD2S_RESET_H (GpioDataRegs.GPASET.bit.GPIO22=1)
#define AD2S_RESET_L (GpioDataRegs.GPACLEAR.bit.GPIO22=1)



#define AD2SA_CS_H (GpioDataRegs.GPBSET.bit.GPIO54=1)
#define AD2SA_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO54=1)

#define AD2SB_SAMPLE_H (GpioDataRegs.GPCSET.bit.GPIO64=1)
#define AD2SB_SAMPLE_L (GpioDataRegs.GPCCLEAR.bit.GPIO64=1)
#define AD2SB_WR_H (GpioDataRegs.GPCSET.bit.GPIO65=1)
#define AD2SB_WR_L (GpioDataRegs.GPCCLEAR.bit.GPIO65=1)
#define AD2SB_CS_H (GpioDataRegs.GPBSET.bit.GPIO59=1)
#define AD2SB_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO59=1)

//#define AD2SC_SAMPLE_H (GpioDataRegs.GPBSET.bit.GPIO39=1)
//#define AD2SC_SAMPLE_L (GpioDataRegs.GPBCLEAR.bit.GPIO39=1)
//#define AD2SC_WR_H (GpioDataRegs.GPBSET.bit.GPIO38=1)
//#define AD2SC_WR_L (GpioDataRegs.GPBCLEAR.bit.GPIO38=1)
//#define AD2SC_CS_H (GpioDataRegs.GPBSET.bit.GPIO37=1)
//#define AD2SC_CS_L (GpioDataRegs.GPBCLEAR.bit.GPIO37=1)
//#define AD2S_WR_H (GpioDataRegs.GPASET.bit.GPIO4=1)
//#define AD2S_WR_L (GpioDataRegs.GPACLEAR.bit.GPIO4=1)
