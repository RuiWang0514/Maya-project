#ifndef _PGA411_H_
#define _PGA411_H_

//#include "hal.h" /* hardware interface */
//#include "F28x_Project.h"

/******************************************************************************
 * MACROS
 *****************************************************************************/

/* PGA411-Q1 registers and their offsets */
#define     DEV_OVUV1       0x00
#define     DEV_OVUV2       0x01
#define     DEV_OVUV3       0x02
#define     DEV_OVUV4       0x03
#define     DEV_OVUV5       0x04
#define     DEV_OVUV6       0x05
#define     DEV_TLOOP_CFG   0x06
#define     DEV_AFE_CFG     0x07
#define     DEV_PHASE_CFG   0x08
#define     DEV_CONFIG1     0x09
#define     DEV_CONTROL1    0x0A /* device unlock needed */
#define     DEV_CONTROL2    0x0B /* device unlock needed */
#define     DEV_CONTROL3    0x0C
#define     DEV_STAT1       0x0D
#define     DEV_STAT2       0x0E
#define     DEV_STAT3       0x0F
#define     DEV_STAT4       0x10
#define     DEV_STAT5       0x11
#define     DEV_STAT6       0x12
#define     DEV_STAT7       0x13
#define     DEV_CLCRC       0x14
#define     DEV_CRC         0x15
#define     CRCCALC         0x16
#define     DEV_EE_CTRL1    0x17 /*EEPROM unlock needed */
#define     DEV_CRC_CTRL1   0x18
#define     DEV_EE_CTRL4    0x19
#define     DEV_UNLK_CTRL1  0x1A
#define     DUMMY_REG       0x1B /* this simplifies continus read */

#define     PGA411_REG_COUNT    27

#define     DUMMY    0x00 /* dummy for SPI read */

// This macro clears the AO2 nrst
#define CLR_nrst  (GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1)

// This macro sets the  AO2 nrst
#define SET_nrst  (GpioDataRegs.GPBSET.bit.GPIO56 = 1)

// This macro clears the  rst
#define CLR_FAULT_rst  (GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1)

// This macro sets the   rst
#define SET_FAULT_rst  (GpioDataRegs.GPCSET.bit.GPIO95 = 1)

/******************************************************************************
 * ENUMS
 *****************************************************************************/

enum {READ, WRITE}; /* Enums selects SPI operation */

enum {DIAG, NORMAL}; /* Operational states of PGA */

/******************************************************************************
 * TYPEDEFS
 *****************************************************************************/

typedef union
{
    /* outgoing data frame (from master to slave) */
    struct
    {
        /* reverse order in bitfields, starting from bit 0 */
        unsigned long    mcrc: 6;      /* master's CRC for data, bits 0..5 */
        unsigned long    reserved: 2;  /* reserved, always 0, bits 6..7  */
        unsigned long    dataout: 16;  /* data, MSB first, bits 8..23 */
        unsigned long    addr: 8;      /* register address, bits 24..31 */
    };
    /* incomming data frame (to master from slave */
    struct
    {
        unsigned long    scrc: 6;      /* slave's CRC for data, bits 0..5 */
        unsigned long    stat: 2;      /* status of SPI communication, bits 6..7 */
        unsigned long    datain: 16;   /* data, MSB first, bits 8..23 */
        unsigned long    ecbk: 8;      /* register address, bits 24..31 */
    };
    /* and finaly the whole frame */
    struct
    {
        unsigned long    frame;
    };
} pga411_spi_frame_t;

typedef struct
{
    /* registers definition */
    const unsigned int read_add;   /* SPI read address */
    const unsigned int write_add;  /* SPI write address */
    const unsigned int def_val;    /* default PGA value */
    unsigned int real_val;         /* realtime register content */
} pga411_reg_t;

/******************************************************************************
 * VARIABLES, CONSTANTS
 *****************************************************************************/

/******************************************************************************
 * SYSTEM INTERFACE FUNCTIONS PROTOTYPES
 *****************************************************************************/

/* Initialization PGA411-Q1 related hardware */
void pga411_Init(void);

/* Perform PGA411-Q1 reset using GPIO */
void pga411_Reset(void);

/* Perform PGA411-Q1 FAULT reset using GPIO */
void pga411_FaultReset(void);

/* Read data from defined register */
unsigned int pga411_ReadReg(unsigned int reg);

/* Write data to defined register */
void pga411_WriteReg(unsigned int reg, unsigned int data);

/* Read all registers from PGA */
void pga411_ReadAll(void);

/* Change state diagnostic/normal */
void pga411_State(unsigned int state);

/* Device unlock (must be in diagnostic state) */
void pga411_DeviceUnlock(void);

/* EEPROM unlock (must be in diagnostic state) */
void pga411_EEPROMUnlock(void);

/* Write default configuration to PGA */
void pga411_DefaultCfg(void);


void SPI_read(void);

//void Resolver_read(void);

void PGA411_Init(void);
//void Resolver_Requestdata(void);
void Resolver_CLRRXFIFO(void);


pga411_spi_frame_t pga411_XmitSPI(unsigned int dir, unsigned int reg, unsigned int wdata);
#endif
