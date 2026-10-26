/******************************************************************************
 * @file    Servi2c.h
 * @brief   BSW Service - I2C low-level driver interface (I2Ca/I2Cb + GPIO bitbang)
 * @layer   BSW/Serve
 * @note    Hardware-level I2C primitives for F2837xD I2Cb peripheral and
 *          GPIO-based software I2C (bitbang).
 ******************************************************************************/
#ifndef SERVI2C_H_
#define SERVI2C_H_

#include "F28x_Project.h"
#include <stdint.h>

/* ---------- GPIO bitbang macros for software I2C ---------- */
#define SCL_H   (GpioDataRegs.GPBSET.bit.GPIO41 = 1)
#define SCL_L   (GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1)
#define SDA_H   (GpioDataRegs.GPBSET.bit.GPIO40 = 1)
#define SDA_L   (GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1)
#define SDA_IN  (GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0)
#define SDA_OUT (GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1)
#define SDA_data GpioDataRegs.GPBDAT.bit.GPIO40

#define PAGE_SIZE 64

/* ---------- I2Cb hardware peripheral functions ---------- */
void   InitI2C(void);
uint16_t I2CB_WriteData(struct I2CMSG *msg);
uint16_t I2CB_ReadData(struct I2CMSG *msg);

/* ---------- I2Cb interrupt handler ---------- */
__interrupt void i2c_int1b_isr(void);

/* ---------- ISR-related globals (defined in Servi2c.c) ---------- */
extern struct I2CMSG  I2cMsgOut1;
extern struct I2CMSG  I2cMsgIn1;
extern struct I2CMSG *CurrentMsgPtr;

#endif /* SERVI2C_H_ */
