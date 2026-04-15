/******************************************************************************
 * @file    i2cHandle.h
 * @brief   APP - Application-level I2C / EEPROM operations
 * @layer   APP
 * @note    High-level EEPROM read/write functions built on top of BSW Servi2c
 *          primitives. Uses GPIO bitbang I2C for single/two-byte EEPROM access,
 *          and I2Cb FIFO for two-byte register-style access.
 ******************************************************************************/
#ifndef I2CHANDLE_H_
#define I2CHANDLE_H_

#include "Servi2c.h"

/* ---------- EEPROM single/two-byte operations (bitbang) ---------- */
void   I2C_WriteOneByte(uint16_t addr, uint16_t data);
void   I2C_WriteTwoByte(uint16_t addr, uint16_t data);
uint16_t  I2C_ReadOneByte(uint16_t addr, int ack);
uint16_t I2C_ReadTwoByte(uint16_t addr, int readlens);

/* ---------- I2Cb FIFO two-byte register access ---------- */
uint16_t I2CWriteTwoByteData(uint16_t addr, uint16_t data);
uint16_t I2CReadTwoByteData(uint16_t addr);

/* ---------- Globals (defined in i2cHandle.c) ---------- */
extern uint16_t I2Cwrite_count;
extern uint16_t I2Cread_count;
extern int DTC_read;

#endif /* I2CHANDLE_H_ */
