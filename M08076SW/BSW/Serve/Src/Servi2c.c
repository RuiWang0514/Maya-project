/******************************************************************************
 * @file    Servi2c.c
 * @brief   BSW Service - I2C low-level driver (I2Ca/I2Cb + GPIO bitbang)
 * @layer   BSW/Serve
 * @note    Contains hardware-level I2C peripheral init, read/write, ISR,
 *          and GPIO bitbang primitives.
 ******************************************************************************/
#include "Servi2c.h"

#define I2C_SLAVE_ADDR        0x50
#define I2C_NUMBYTES          4
#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x01

/* ---------- Global message buffers used by ISR ---------- */
struct I2CMSG I2cMsgOut1 = { I2C_MSGSTAT_INACTIVE,
                             I2C_SLAVE_ADDR,
                             I2C_NUMBYTES,
                             I2C_EEPROM_HIGH_ADDR,
                             I2C_EEPROM_LOW_ADDR,
                             0x12,
                             0x34 };

struct I2CMSG I2cMsgIn1 = { I2C_MSGSTAT_INACTIVE,
                             I2C_SLAVE_ADDR,
                             I2C_NUMBYTES,
                             I2C_EEPROM_HIGH_ADDR,
                             I2C_EEPROM_LOW_ADDR };

struct I2CMSG *CurrentMsgPtr;

/******************************************************************************
 * @brief   Initialize I2Cb peripheral (GPIO mux, clock, FIFO)
 * @note    Called from main() during startup sequence
 ******************************************************************************/
void InitI2C(void)
{
    EALLOW;
    GPIO_SetupPinMux(40, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 6);

    I2cbRegs.I2CSAR.all = 0x0050;
    I2cbRegs.I2CPSC.all = 19;          // Prescaler - need 7-12 Mhz on module clk
    I2cbRegs.I2CCLKL = 16;            // NOTE: must be non zero
    I2cbRegs.I2CCLKH = 9;             // NOTE: must be non zero
    I2cbRegs.I2CIER.all = 0x24;
    I2cbRegs.I2CMDR.all = 0x0020;

    I2cbRegs.I2CFFTX.all = 0x6000;    // Enable FIFO mode and TXFIFO
    I2cbRegs.I2CFFRX.all = 0x2040;    // Enable RXFIFO, clear RXFFINT

    EDIS;
}

/******************************************************************************
 * @brief   Write data to I2Cb slave
 * @param   msg  Pointer to I2C message structure
 * @return  I2C_SUCCESS or error code
 ******************************************************************************/
uint16_t I2CB_WriteData(struct I2CMSG *msg)
{
   uint16_t i;

   if (I2cbRegs.I2CMDR.bit.STP == 1)
   {
      return I2C_STP_NOT_READY_ERROR;
   }

   I2cbRegs.I2CSAR.all = msg->SlaveAddress;

   if (I2cbRegs.I2CSTR.bit.BB == 1)
   {
      return I2C_BUS_BUSY_ERROR;
   }

   I2cbRegs.I2CCNT = msg->NumOfBytes + 2;

   I2cbRegs.I2CDXR.all = msg->MemoryHighAddr;
   I2cbRegs.I2CDXR.all = msg->MemoryLowAddr;

   for (i = 0; i < msg->NumOfBytes; i++)
   {
      I2cbRegs.I2CDXR.all = *(msg->MsgBuffer + i);
   }

   I2cbRegs.I2CMDR.all = 0x6E20;

   return I2C_SUCCESS;
}

/******************************************************************************
 * @brief   Read data from I2Cb slave (two-phase: address then read)
 * @param   msg  Pointer to I2C message structure
 * @return  I2C_SUCCESS or error code
 ******************************************************************************/
uint16_t I2CB_ReadData(struct I2CMSG *msg)
{
   if (I2cbRegs.I2CMDR.bit.STP == 1)
   {
      return I2C_STP_NOT_READY_ERROR;
   }

   I2cbRegs.I2CSAR.all = msg->SlaveAddress;

   if (msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
   {
      if (I2cbRegs.I2CSTR.bit.BB == 1)
      {
         return I2C_BUS_BUSY_ERROR;
      }
      I2cbRegs.I2CCNT = 2;
      I2cbRegs.I2CDXR.all = msg->MemoryHighAddr;
      I2cbRegs.I2CDXR.all = msg->MemoryLowAddr;
      I2cbRegs.I2CMDR.all = 0x2620;
   }
   else if (msg->MsgStatus == I2C_MSGSTAT_RESTART)
   {
      I2cbRegs.I2CCNT = msg->NumOfBytes;
      I2cbRegs.I2CMDR.all = 0x2C20;
   }

   return I2C_SUCCESS;
}

/******************************************************************************
 * @brief   I2Cb interrupt handler - processes SCD and ARDY events
 * @note    Accesses CurrentMsgPtr, I2cMsgIn1, I2cMsgOut1 global message buffers
 ******************************************************************************/
__interrupt void i2c_int1b_isr(void)
{
   uint16_t IntSource, i;

   IntSource = I2cbRegs.I2CISRC.all;

   if (IntSource == I2C_SCD_ISRC)
   {
      if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
      {
         CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
      }
      else
      {
         if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
         {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
         }
         else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)
         {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;

            for (i = 0; i < I2C_NUMBYTES; i++)
            {
              CurrentMsgPtr->MsgBuffer[i] = I2cbRegs.I2CDRR.all;
            }
         }
       }
   }
   else if (IntSource == I2C_ARDY_ISRC)
   {
      if (I2cbRegs.I2CSTR.bit.NACK == 1)
      {
         I2cbRegs.I2CMDR.bit.STP = 1;
         I2cbRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
      }
      else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
      {
         CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;
      }
   }
   else
   {
      __asm("   ESTOP0");
   }

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}
