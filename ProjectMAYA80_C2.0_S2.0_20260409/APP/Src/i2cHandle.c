/******************************************************************************
 * @file    i2cHandle.c
 * @brief   APP - Application-level I2C / EEPROM operations
 * @layer   APP
 * @note    High-level EEPROM read/write using GPIO bitbang I2C primitives
 *          (from BSW/Serve/Servi2c) and I2Cb FIFO register-style access.
 ******************************************************************************/
#include "i2cHandle.h"

uint16_t I2Cwrite_count, I2Cread_count = 0;
int DTC_read = 0;

/******************************************************************************
 * @brief   Write one byte to EEPROM via bitbang I2C (slave addr 0xA0)
 ******************************************************************************/
void I2C_WriteOneByte(uint16_t addr, uint16_t data)
{
    uint16_t addrH, addrL;
    addrH = addr >> 8;
    addrL = addr & 0xff;
    I2C_start();
    I2C_WriteByte(0xA0);
    I2C_Wait_ack();
    I2C_WriteByte(addrH);
    I2C_Wait_ack();
    I2C_WriteByte(addrL);
    I2C_Wait_ack();
    I2C_WriteByte(data);
    I2C_Wait_ack();
    I2C_stop();
}

/******************************************************************************
 * @brief   Write two bytes to EEPROM with page boundary handling
 ******************************************************************************/
void I2C_WriteTwoByte(uint16_t addr, uint16_t data)
{
    uint16_t tmp, lentop, lenlow, lens;
    int lentmp = 2;

    tmp = addr % PAGE_SIZE;
    tmp = PAGE_SIZE - tmp;
    if(tmp >= lentmp) lenlow = lentmp;
    else lenlow = tmp;

    lentop = lentmp - lenlow;
    for(lens = 0; lens < lenlow; lens++)
    {
        I2C_WriteOneByte(addr + lens, (data >> (8 * (lentmp - 1 + lens))) & 0xff);
    }

    addr = addr + lenlow;
    for(lens = 0; lens < lentop; lens++)
    {
        I2C_WriteOneByte(addr + lens, (data >> (8 * (lentop - 1 - lens))) & 0xff);
    }
}

/******************************************************************************
 * @brief   Read one byte from EEPROM via bitbang I2C (slave addr 0xA0/0xA1)
 ******************************************************************************/
uint16_t I2C_ReadOneByte(uint16_t addr, int ack)
{
    uint16_t temp, addrH, addrL;
    addrH = addr >> 8;
    addrL = addr & 0xff;
    I2C_start();
    I2C_WriteByte(0xA0);
    I2C_Wait_ack();
    I2C_WriteByte(addrH);
    I2C_Wait_ack();
    I2C_WriteByte(addrL);
    I2C_Wait_ack();

    I2C_start();
    I2C_WriteByte(0xA1);
    I2C_Wait_ack();

    temp = I2C_ReadByte(ack);
    I2C_stop();
    return temp;
}

/******************************************************************************
 * @brief   Read two bytes from EEPROM (sequential single-byte reads)
 ******************************************************************************/
uint16_t I2C_ReadTwoByte(uint16_t addr, int readlens)
{
    uint16_t temp = 0;
    int t;
    for(t = 0; t < readlens; t++)
    {
        temp <<= 8;
        if(t == (readlens - 1))
        {
            temp += I2C_ReadOneByte(addr + t, 0);
        }
        else
        {
            temp += I2C_ReadOneByte(addr + t, 1);
        }
    }
    return temp;
}

/******************************************************************************
 * @brief   Write 16-bit data to I2Cb EEPROM via FIFO (slave 0x0050)
 ******************************************************************************/
uint16_t I2CWriteTwoByteData(uint16_t addr, uint16_t data)
{
    uint16_t i;
    if(I2cbRegs.I2CMDR.bit.STP == 1)
    {
       return I2C_STP_NOT_READY_ERROR;
    }

    if(I2cbRegs.I2CSTR.bit.BB == 1)
    {
       return I2C_BUS_BUSY_ERROR;
    }

    I2cbRegs.I2CSAR.all = 0x0050;
    I2cbRegs.I2CCNT = 4;

    for(i = 0; i < 2; i++)
    {
        if(i == 0)
        {
            I2cbRegs.I2CDXR.all = (addr >> 8) & 0xFF;
        }
        else
        {
            I2cbRegs.I2CDXR.all = addr & 0xFF;
        }
    }
    I2cbRegs.I2CDXR.all = (data >> 8) & 0xFF;
    I2cbRegs.I2CDXR.all = data & 0xFF;
    I2Cwrite_count++;
    I2cbRegs.I2CMDR.all = 0x6E20;

    while(I2cbRegs.I2CSTR.bit.BB);

    return 1;
}

/******************************************************************************
 * @brief   Read 16-bit data from I2Cb EEPROM via FIFO (slave 0x0050)
 ******************************************************************************/
uint16_t I2CReadTwoByteData(uint16_t addr)
{
    int i;
    uint16_t I2CreceiveBuf0, I2CreceiveBuf1;
    uint16_t I2Cread;

    I2cbRegs.I2CSAR.all = 0x0050;
    I2cbRegs.I2CCNT = 2;

    for(i = 0; i < 2; i++)
    {
        if(i == 0)
        {
            I2cbRegs.I2CDXR.all = (addr >> 8) & 0xFF;
        }
        else
        {
            I2cbRegs.I2CDXR.all = addr & 0xFF;
        }
    }
    I2cbRegs.I2CMDR.all = 0x6620;
    DELAY_US(1);
    I2cbRegs.I2CCNT = 2;
    I2cbRegs.I2CMDR.all = 0x6C20;

    I2CreceiveBuf0 = I2cbRegs.I2CDRR.all;
    I2CreceiveBuf1 = I2cbRegs.I2CDRR.all;
    I2cbRegs.I2CMDR.all = 0x6C20;
    I2Cread = I2CreceiveBuf0 << 8 + I2CreceiveBuf1;

    I2Cread_count++;

    return I2Cread;
}
