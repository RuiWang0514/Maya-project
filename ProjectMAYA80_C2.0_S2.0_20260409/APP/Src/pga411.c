#include "pga411.h"
#include "HPMC_Settings.h"
#include "IQmathLib.h"

uint16_t pga411_err;

extern void M1_StopMotor(void);

/******************************************************************************
 * MACROS
 *****************************************************************************/
#define    ENABLE_CRC_LUT
#define    ENABLE_FAST_READ
#define    CRC_BITCOUNT     24 /* top 24 bits of uint32_t for CRC6 */
#define    CRC_BYTECOUNT    3 /* top 3 B of uint32_t for CRC6 */
#define    CRC_INITSEED     0x3F /* initial seed for CRC6 */
#define    CRC_POLYNOM      0x5B /* CRC = X6+X4+X3+X+1 */

#define    SPI_DUMMY        0xF0 /* dummy SPI data for read operation */

#define     COMMAND_READANGLE   0x4100f001

#define    EMPTY_LOOP

/* PUSH data to SPI buffer - left adjusted */
#define SPIB_PUSH(TX) SpibRegs.SPITXBUF = (TX << 8)
/* POP data from SPI buffer - right adjusted */
#define SPIB_POP SpibRegs.SPIRXBUF
/******************************************************************************
 * ENUMS
 *****************************************************************************/

/******************************************************************************
 * TYPEDEFS
 *****************************************************************************/
uint32_t hal_Xmit4BSPI(uint32_t frame);
/******************************************************************************
 * VARIABLES, CONSTANTS
 *****************************************************************************/

volatile pga411_spi_frame_t pga411_spi_frame = {.addr = 0x00,
                                                .dataout = 0x0000,
                                                .reserved = 0x0,
                                                .mcrc = 0x00
                                               };

pga411_reg_t pga411_regs[PGA411_REG_COUNT + 1] =
{
    /* memory constants init */
    [DEV_OVUV1]     = {.read_add = 0x53, .write_add = 0x87, .def_val = 0x8B40},//0
//    [DEV_OVUV2]     = {.read_add = 0x6B, .write_add = 0x26, .def_val = 0x00ED},//1 1st
//    [DEV_OVUV2]     = {.read_add = 0x6B, .write_add = 0x26, .def_val = 0x00FD},//1 2nd
    [DEV_OVUV2]     = {.read_add = 0x6B, .write_add = 0x26, .def_val = 0x00FF},//1
    [DEV_OVUV3]     = {.read_add = 0x65, .write_add = 0x17, .def_val = 0xFCFF},//2
    [DEV_OVUV4]     = {.read_add = 0xEC, .write_add = 0x39, .def_val = 0x07F2},//3//0x07F2//OUR PREVIOUs 0x07E2
    [DEV_OVUV5]     = {.read_add = 0x52, .write_add = 0x75, .def_val = 0x1C00},//4
    [DEV_OVUV6]     = {.read_add = 0xE9, .write_add = 0x83, .def_val = 0x038F},//5
    [DEV_TLOOP_CFG] = {.read_add = 0xA6, .write_add = 0x42, .def_val = 0x0514},//6
//    [DEV_AFE_CFG]   = {.read_add = 0xC2, .write_add = 0x91, .def_val = 0x0005},//7
    [DEV_AFE_CFG]   = {.read_add = 0xC2, .write_add = 0x91, .def_val = 0x0000},//7
    [DEV_PHASE_CFG] = {.read_add = 0x57, .write_add = 0x85, .def_val = 0x1800},//8
    //Exciter Mode Select: 11-10 EXTMODE 10: 7-VRMS Mode
    //Exciter Offset Voltage Adjust (FootRoom):9-6 EXTOUT 0101: 1.5 V
    [DEV_CONFIG1]   = {.read_add = 0xBE, .write_add = 0xEB, .def_val = 0x0002},//9//0x0002//OUR PREVIOUs 0x0005
    //Exciter Power Supply Voltage Output Select:2-0 MODEVEXT 101: 15 V
    //Exciter Frequency Select:5-3 SELFEXT 000: 10 kHz
    [DEV_CONTROL1]  = {.read_add = 0x90, .write_add = 0x0D, .def_val = 0x0000},//10
    [DEV_CONTROL2]  = {.read_add = 0x63, .write_add = 0x38, .def_val = 0x0000},//11
    [DEV_CONTROL3]  = {.read_add = 0xDD, .write_add = 0xAE, .def_val = 0x0003},//12
    [DEV_STAT1]     = {.read_add = 0x81, .write_add = 0x00, .def_val = 0x0000},//13
    [DEV_STAT2]     = {.read_add = 0x4D, .write_add = 0x00, .def_val = 0x0000},//14
    [DEV_STAT3]     = {.read_add = 0x84, .write_add = 0x00, .def_val = 0x0000},//15
    [DEV_STAT4]     = {.read_add = 0x1F, .write_add = 0x00, .def_val = 0x0000},//16
    [DEV_STAT5]     = {.read_add = 0x41, .write_add = 0x00, .def_val = 0x0000},//17
    [DEV_STAT6]     = {.read_add = 0x6F, .write_add = 0x00, .def_val = 0x0000},//18
    [DEV_STAT7]     = {.read_add = 0xE1, .write_add = 0x00, .def_val = 0x0000},//19
    [DEV_CLCRC]     = {.read_add = 0x4F, .write_add = 0xFC, .def_val = 0x0000},//20
    [DEV_CRC]       = {.read_add = 0x0F, .write_add = 0xE7, .def_val = 0x0000},//21
    [CRCCALC]       = {.read_add = 0xD9, .write_add = 0xFF, .def_val = 0x0000},//22
    [DEV_EE_CTRL1]  = {.read_add = 0xE3, .write_add = 0x6E, .def_val = 0x0000},//23
    [DEV_CRC_CTRL1] = {.read_add = 0x7A, .write_add = 0xB6, .def_val = 0x0000},//24
    [DEV_EE_CTRL4]  = {.read_add = 0xBA, .write_add = 0x56, .def_val = 0x0000},//25
    [DEV_UNLK_CTRL1] = {.read_add = 0x64, .write_add = 0x95, .def_val = 0x0000},//26
    /* for continuous read we need to read 28 times. This significantly helps */
    [DUMMY_REG]     = {.read_add = 0x53, .write_add = 0x87, .def_val = 0x0000},//27
};

/******************************************************************************
 * INTERNAL FUNCTIONS PROTOTYPES
 *****************************************************************************/

/* calculate CRC6 for output data, function takes first 3B left aligned data */
uint16_t pga411_crc2(uint32_t datin);

/* Xmit data to PGA over SPI, for reg use defined macros, return incoming frame */
pga411_spi_frame_t pga411_XmitSPI(uint16_t dir, uint16_t reg, uint16_t wdata);

uint16_t Resolver_Getdata();

/******************************************************************************
 * INTERNAL FUNCTIONS
 *****************************************************************************/

/* CRC6 calculation - optimized for PGA411 */

#ifdef ENABLE_CRC_LUT
unsigned char CRC_Table[256];

void gen_CRCtable()
{
    uint16_t i=0;
    unsigned char crc;
    for(i=0;i<256;i++)
    {
        crc=i;
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        CRC_Table[i]=crc;
    }

}

#endif

uint16_t pga411_crc2(uint32_t datin)
{
    uint16_t byte_idx, crc = (CRC_INITSEED << 2);

    /* byte by byte starting from most significant (3-2-1) */
    for (byte_idx = CRC_BYTECOUNT; byte_idx >= 1; byte_idx--)
    {
        /* XOR-in new byte from left to right */
        crc ^= ((datin >> (byte_idx << 3)) & 0x000000FF);

#ifdef ENABLE_CRC_LUT
        crc=CRC_Table[crc];
#else
        // successive 8 times
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
        crc = crc << 1 ^ (crc & 0x80 ? (CRC_POLYNOM << 2) : 0);
#endif
    }

    return (crc >> 2 & CRC_INITSEED); /*restore two bit offset */
}

/******************************************************************************
 * SYSTEM INTERFACE FUNCTIONS
 *****************************************************************************/

/* Xmit data to PGA over SPI, for reg use defined macros */
pga411_spi_frame_t pga411_XmitSPI(uint16_t dir, uint16_t reg, uint16_t wdata)
{
    pga411_spi_frame_t out, in;

    /* do we read data ? */
    if (dir == READ) { out.addr = pga411_regs[reg].read_add; } /* read address */
    /* or write data ? */
    else { out.addr = pga411_regs[reg].write_add; } /* write address */

    /* compose the rest of the frame */
    out.dataout = wdata; /* real data (W) or dummy data (R) */
    out.reserved = 0x00; /* always zero */
//    CLR_nrst;
    out.mcrc = pga411_crc2(out.frame); /* calculate TX CRC */
//    SET_nrst;
    in.frame = hal_Xmit4BSPI(out.frame);


    /* check RX CRC */
//    CLR_nrst;
    if ((pga411_crc2(in.frame) != in.scrc)||(in.stat!=0))
    {
        //hal_assert(); /* if error -> terminate */
        pga411_err++;// =1;
    }
//    SET_nrst;
    return (in);
}

/* Read data from defined register */
uint16_t pga411_ReadReg(uint16_t reg)
{
    /* first read returns whatever , first time just send read command and target addr*/
    pga411_XmitSPI(READ, reg, SPI_DUMMY);
    /* second read returns requested data , second time read the device would return the data we want*/
    return (pga411_XmitSPI(READ, reg, SPI_DUMMY).datain);
}

/* Read data from defined register */
uint16_t pga411_ReadAngle()
{
#ifdef ENABLE_FAST_READ
    pga411_spi_frame_t in;
    static pga411_spi_frame_t correct_frame;

    in.frame = hal_Xmit4BSPI(COMMAND_READANGLE);
    in.frame = hal_Xmit4BSPI(COMMAND_READANGLE);

    /* check RX CRC */
    CLR_nrst;
    if ((pga411_crc2(in.frame) != in.scrc)||(in.stat!=0))
    {
        //hal_assert(); /* if error -> terminate */
        pga411_err++;// =1;
        SET_nrst;
        return (correct_frame.datain);
    }
    SET_nrst;
    correct_frame=in;
    return (correct_frame.datain);

#else

    static pga411_spi_frame_t correct_frame;
    pga411_spi_frame_t frame_temp;
    /* first read returns whatever , first time just send read command and target addr*/

    pga411_XmitSPI(READ, DEV_STAT5, SPI_DUMMY);

    /* second read returns requested data , second time read the device would return the data we want*/
    frame_temp=pga411_XmitSPI(READ, DEV_STAT5, SPI_DUMMY);

    if (!((pga411_crc2(frame_temp.frame) != frame_temp.scrc)||(frame_temp.stat!=0)))
    {
        correct_frame=frame_temp;
    }

    return (correct_frame.datain);
#endif
}

/* Write data to defined register */
void pga411_WriteReg(uint16_t reg, uint16_t data)
{
    /* here we just making it nice, can be macro too */
    pga411_XmitSPI(WRITE, reg, data);
}

/* Read all registers from PGA */

pga411_reg_t pga411_regs_read[PGA411_REG_COUNT + 1] ;
void pga411_ReadAll(void)
{
    int i;
    uint16_t rx_data;

    /* register by register (27+1) */
    /* PGA411 always returns previous read register value */
    /* thats why 28 reads are needed (first read returns whatever data) */
    for (i = 0; i < PGA411_REG_COUNT + 1; i++)
    {
        rx_data = pga411_XmitSPI(READ, i, SPI_DUMMY).datain;

        /*boolean trick - save data if not first cycle */
        if (i)
        {
            pga411_regs_read[i - 1].real_val = rx_data;
        }
    }
}

/* Change state diagnostic/normal */
void pga411_State(uint16_t state)
{
    uint16_t temp;

    /* Enter Diagnostic state */
    if (state == DIAG)
    {
        /* read content of  DEV_CONTROL3 register */
        temp = pga411_ReadReg(DEV_CONTROL3);
        temp |= 0x0004; /* set bit SPIDIAG */
        pga411_WriteReg(DEV_CONTROL3, temp); /* finish R-M-W */
    }
    /* Go back to normal state */
    else
    {
        /* read content of  DEV_CONTROL1 register */
        temp = pga411_ReadReg(DEV_CONTROL1);
        temp |= 0x0001; /* set bit DIAGEXIT */
        pga411_WriteReg(DEV_CONTROL1, temp); /* finish R-M-W */
    }
}

/* Device unlock (must be in diagnostic state) */
void pga411_DeviceUnlock(void)
{
    pga411_WriteReg(DEV_UNLK_CTRL1, 0x000F);
    pga411_WriteReg(DEV_UNLK_CTRL1, 0x0055);
    pga411_WriteReg(DEV_UNLK_CTRL1, 0x00AA);
    pga411_WriteReg(DEV_UNLK_CTRL1, 0x00F0);
}

/* EEPROM unlock (must be in diagnostic state) */
void pga411_EEPROMUnlock(void)
{
    pga411_WriteReg(DEV_EE_CTRL4, 0x000F);
    pga411_WriteReg(DEV_EE_CTRL4, 0x0055);
    pga411_WriteReg(DEV_EE_CTRL4, 0x00AA);
    pga411_WriteReg(DEV_EE_CTRL4, 0x00F0);
}

/* Write default configuration to PGA */
void pga411_DefaultCfg(void)
{
    int i;
    /* go to diagnostic state */
    pga411_State(DIAG);
    /* unlock registers */
    pga411_DeviceUnlock();

    /* first 12 registers to default */
    /* note: for some u need to unlocked them first ! */
    for (i = 0; i < 13; i++)
    {
        pga411_XmitSPI(WRITE, i, pga411_regs[i].def_val);
    }

    /* go to normal state */
    pga411_State(NORMAL);

//    pga411_ReadAll();
}
void pga411_DefaultCfgRead(void)
{
//    int i;
    /* go to diagnostic state */
//    pga411_State(DIAG);
    /* unlock registers */
//    pga411_DeviceUnlock();

    /* first 12 registers to default */
    /* note: for some u need to unlocked them first ! */
//    pga411_State(NORMAL);

    pga411_ReadAll();
}

/* Exchanges 4 Bytes via SPI, blocks until finished, 32b architecture = no pointer */
uint32_t hal_Xmit4BSPI(uint32_t frame)
{
    uint32_t rxval = 0x00000000; /* incoming data */
    uint32_t wait_counter=0;
    /* empty RX and TX 4B FIFOs */
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; /* reset FIFOs */
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;
    asm(" NOP"); /* probably not needed */
    asm(" NOP"); /* probably not needed */
    SpibRegs.SPIFFTX.bit.TXFIFO = 1;
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; /* release from reset */
    /* quick & dirty ! */
    SPIB_PUSH(frame >> 24); /* first byte (from most significant) */
    SPIB_PUSH(frame >> 16);
    SPIB_PUSH(frame >> 8);
    SPIB_PUSH(frame); /* last byte */

    /* wait until the transmission is finished */
//    while (SpibRegs.SPIFFRX.bit.RXFFST < 4) {EMPTY_LOOP};
    for (wait_counter=0; (wait_counter<100)&&(SpibRegs.SPIFFRX.bit.RXFFST < 4); wait_counter++) {}

    /* get data from RX FIFO */
    rxval = SPIB_POP; /* first byte (from the most significant) */

    rxval = ((rxval << 8) | SPIB_POP);

    rxval = ((rxval << 8) | SPIB_POP);

    rxval = ((rxval << 8) | SPIB_POP); /* last byte */

    /* NCS must remain high at least 200ns */
 //   DELAY_US(1);

    return (rxval);
}


void PGA411_Init()
{
    CLR_nrst;
    DELAY_US(10000);
    SET_nrst;
    DELAY_US(200000); // 10 ms deglitch period when the exciter enables
//    pga411_DefaultCfg(); // default config defined in "pga411_regs"
    DELAY_US(200000); // 10 ms deglitch period when the exciter enables
    //pga411_FaultReset(); // clear faults if any
    CLR_FAULT_rst;
    DELAY_US(600000);
    SET_FAULT_rst;
    DELAY_US(10000);

    GpioDataRegs.GPESET.bit.GPIO131=1;
}

uint16_t NeedReInitPGA=1;
uint16_t NeedReInitPGARead=1;
uint16_t PGARead=0;
uint16_t PGAticker=0;


unsigned char trans_sta=0;

void Resolver_Requestdata()
{
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; /* reset FIFOs */
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;
    asm(" NOP"); /* probably not needed */
    asm(" NOP"); /* probably not needed */
    SpibRegs.SPIFFTX.bit.TXFIFO = 1;
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; /* release from reset */
    /* quick & dirty ! */
//    SpibRegs.SPITXBUF=(COMMAND_READANGLE >> 16); /* first byte (from most significant) */
    SPIB_PUSH(COMMAND_READANGLE >> 24); /* first byte (from most significant) */
    SPIB_PUSH(COMMAND_READANGLE >> 16);
    SPIB_PUSH(COMMAND_READANGLE >> 8);
    SPIB_PUSH(COMMAND_READANGLE); /* last byte */
//    DELAY_US(1);
}

void Resolver_CLRRXFIFO()
{
    uint32_t rxval;

    rxval = SPIB_POP; /* first byte (from the most significant) */
    rxval = ((rxval << 8) | SPIB_POP);
    rxval = ((rxval << 8) | SPIB_POP);
    rxval = ((rxval << 8) | SPIB_POP); /* last byte */
}

pga411_spi_frame_t error_frame;
uint32_t miss_cnt=0;
unsigned char rx_cnt=0;

uint16_t Resolver_Getdata()
{
    uint32_t rxval;
    pga411_spi_frame_t in;
    static pga411_spi_frame_t correct_frame;


    if(SpibRegs.SPIFFRX.bit.RXFFST >= 4)
    {

        rxval = SPIB_POP; /* first byte (from the most significant) */
        rxval = ((rxval << 8) | SPIB_POP);
        rxval = ((rxval << 8) | SPIB_POP);
        rxval = ((rxval << 8) | SPIB_POP); /* last byte */
        in.frame=rxval;
        if ((pga411_crc2(in.frame) != in.scrc))//||(in.stat!=0))
        {
            pga411_err++;// =1;
            error_frame=in;
            return (correct_frame.datain);
        }
        correct_frame=in;
    }
    else
    {
        miss_cnt++;
        Resolver_CLRRXFIFO();
    }
    return (correct_frame.datain);
}
