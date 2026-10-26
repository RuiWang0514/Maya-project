#include "HPMC_Settings.h"
#include "spi_resolver.h"

float ALIGN_offset = 0.0;
float Eangle_resolver_spi = 0.0, Eangle_resolver_tmp = 0.0;
extern int Error_Pos, Error_Spd;

uint16_t NoB = 0, SPI_data;
uint16_t  mech_angle_spi;

void SPI_Resolver_Init()
{
    spi_init();
    SAMPLE_Set;
    SPISTE_Set;
    RDVEL_Set;
    spi_fifo_init();
}

void spi_init()
{
    EALLOW; // below registers are "protected", allow access.
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;  //SPI software reset. You should clear this bit before the changes
    SpiaRegs.SPICCR.bit.SPICHAR = 0xf;   //Character Length Control Bits 3-0~16bit
    SpiaRegs.SPICCR.bit.SPILBK = 0;      //SPI loop back mode disabled
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1; //Shift Clock Polarity.
                                         // 1-Data is output on falling edge and input on rising edge.
                                         // 0-Data is output on rising edge and input on falling edge.

    SpiaRegs.SPICTL.all = 0x0006;        // master, enables transimission  disables interrupt
    SpiaRegs.SPIBRR.all = 0x0000;        // rate=LSPCLK/(SPIBBR+1)=7.5M     LSPCLK=15M   LOSPCP
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;  //SPI software reset. You should set this bit before resuming operation
    EDIS;
}

void spi_fifo_init(void)
{
    SpiaRegs.SPIFFTX.all = 0xE040;       //
    SpiaRegs.SPIFFRX.all = 0x2040;       //
    SpiaRegs.SPIFFCT.all = 0x0000;       //
}

void spi_read(void)
{
    int wait_counter;
    SAMPLE_Clear;
    //Position Reading
    SpiaRegs.SPITXBUF = dummy;
    for (wait_counter = 0; (wait_counter < 100) && (SpiaRegs.SPIFFRX.bit.RXFFST == 0); wait_counter++) {}
    SPI_data = SpiaRegs.SPIRXBUF;
    NoB = oneBitsInUShort(SPI_data);

    if (((SPI_data & BIT1) == 0) || ((SPI_data & BIT2) == 0))
        Error_Pos = 1;

    if (((SPI_data & BIT3) == BIT3) && ((NoB & BIT0) == BIT0)) //Check for pos bit and data parity
    {
        mech_angle_spi = SPI_data >> 4;
    }

    SAMPLE_Set;
    //Start of position calculation
    Eangle_resolver_tmp = ((mech_angle_spi / 4096.0) * (POLES * 2.0)) - ALIGN_offset;
    Eangle_resolver_spi = ((Eangle_resolver_tmp) - (float)((long)(Eangle_resolver_tmp)));    // get fraction
    //End of position calculation
}

unsigned char oneBitsInUShort (unsigned short x) {
    return oneBitsInUChar [x >>    8]
         + oneBitsInUChar [x &  0xff];
}
