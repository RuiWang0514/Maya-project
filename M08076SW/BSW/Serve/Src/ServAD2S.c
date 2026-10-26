#include "ServAD2S.h"

void SPI_init(void)
{
    SPIB_GPIO_Init();

    EALLOW; // below registers are "protected", allow access.

    // GPIO65 --- /WR1
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(65,0,0);
    GpioDataRegs.GPCSET.bit.GPIO65 = 1;

    // /SAMPLE1
    GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(64,0,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;

    // /CS1
    GPIO_SetupPinOptions(59, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(59,0,0);
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;

    GPIO_SetupPinOptions(14, GPIO_INPUT, GPIO_ASYNC); //  LOT
    GPIO_SetupPinMux(14, 0, 0);
    GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_ASYNC); //  DOS
    GPIO_SetupPinMux(15, 0, 0);

    // spib
    SpibRegs.SPICCR.bit.SPISWRESET = 0;  // SPI software reset. You should clear this bit before the changes
    SpibRegs.SPICCR.bit.SPICHAR = 0x7;   // Character Length Control Bits 3-0~16bit    Set as 8bit mode
    SpibRegs.SPICCR.bit.SPILBK = 0;      // SPI loop back mode disabled
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; // Shift Clock Polarity.
                                       // 1-Data is output on falling edge and input on rising edge.
                                       // 0-Data is output on rising edge and input on falling edge.
    SpibRegs.SPICTL.all = 0x0006;        // master, enables transimission  disables interrupt
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;
    SpibRegs.SPIBRR.all = 5;            // rate = LSPCLK/(SPIBBR+1)     LSPCLK = 25M   LOSPCP
    SpibRegs.SPIPRI.bit.FREE = 1;
    SpibRegs.SPICCR.bit.SPISWRESET = 1;

    EDIS;
}

void SPIB_GPIO_Init(void)
{
    EALLOW;

    /* Enable internal pull-up for the selected pins */
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.

    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;   // Enable pull-up on GPIO58
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;   // Enable pull-up on GPIO60
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;   // Enable pull-up on GPIO61

    /* Set qualification for selected pins to asynch only */
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.

    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;  // Asynch input GPIO58
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO60
    GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3; // Asynch input GPIO61

    /* Configure SPI-B pins using GPIO regs*/
    // This specifies which of the possible GPIO pins will be SPI functional pins.
    // Comment out other unwanted lines.

    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 1; // GPIO58 peripheral mux: GMUX = 1
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 2; // GPIO58 peripheral mux: MUX = 2

    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 1; // GPIO60 peripheral mux: GMUX = 1
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 2; // GPIO60 peripheral mux: MUX = 2

    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 1; // GPIO61 peripheral mux: GMUX = 1
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 2; // GPIO61 peripheral mux: MUX = 2

    EDIS;
}

void SPI_fifo_init(void)
{
    EALLOW;

    SpibRegs.SPIFFTX.bit.SPIRST = 0x1;    // SPI FIFO can resume transmit or receive. No effect to the SPI registers bits.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1;  // SPI FIFO enhancements are enabled.
    SpibRegs.SPIFFTX.bit.TXFIFO = 0x1;    // Release transmit FIFO from reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 0x1;    // Write 1 to clear SPIFFTX[TXFFINT] flag.
    SpibRegs.SPIFFTX.bit.TXFFIENA = 0x0;  // TX FIFO interrupt based on TXFFIL match (less than or equal to) will be disabled.
//
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 0x1;    // Write 0 does not affect RXFFOVF flag bit, Bit reads back a zero.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0x1;   // 1h (R/W) = Re-enable receive FIFO operation
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 0x1;    // Write 1 to clear SPIFFRX[RXFFINT] flag
    SpibRegs.SPIFFRX.bit.RXFFIENA = 0x0;      // RX FIFO interrupt based on RXFFIL match (greater than or equal to) will be disabled.
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x0;

    SpibRegs.SPIFFCT.all = 0x0000;

    EDIS;
}

uint16_t old_dataB_test = 0;
uint16_t new_dataB_test = 0;

void AD2SB_Write(uint16_t addr,uint16_t data){

    AD2SB_WR_L;

    while (new_dataB_test != data){
    SpibRegs.SPITXBUF = (addr<<8);
    while (SpibRegs.SPIFFRX.bit.RXFFST < 1){};
    old_dataB_test = SpibRegs.SPIRXBUF;

    AD2SB_WR_H;

    DELAY_US(1);

    AD2SB_WR_L;

    SpibRegs.SPITXBUF = (data<<8);
    while (SpibRegs.SPIFFRX.bit.RXFFST < 1){};
    new_dataB_test = SpibRegs.SPIRXBUF;

    AD2SB_WR_H;

    DELAY_US(1);
    }
}

uint16_t AD2SB_Read(uint16_t addr)
{
    uint16_t rxval;

    AD2SB_WR_L;
    SpibRegs.SPITXBUF = (addr<<8);
    while (SpibRegs.SPIFFRX.bit.RXFFST < 1){};
    rxval = (SpibRegs.SPIRXBUF);
    AD2SB_WR_H;
    DELAY_US(1);
    AD2SB_WR_L;
    SpibRegs.SPITXBUF = (addr<<8);
    while (SpibRegs.SPIFFRX.bit.RXFFST < 1){};
    rxval = (SpibRegs.SPIRXBUF);
    AD2SB_WR_H;
    DELAY_US(1);

    return rxval;
}

uint16_t AD2SB_Fault;

void AD2SB_Reset()
{

    AD2SB_CS_L;
    AD2SB_SAMPLE_L;

    DELAY_US(1);
    AD2SB_SAMPLE_H;
    DELAY_US(20);

    AD2SB_SAMPLE_L;
    DELAY_US(5);
    AD2SB_SAMPLE_H;

    AD2SB_SAMPLE_L;
    DELAY_US(1);
    AD2SB_SAMPLE_H;
    AD2SB_Fault = AD2SB_Read(0xff);
    AD2SB_SAMPLE_L;
    DELAY_US(1);
    AD2SB_SAMPLE_H;
    DELAY_US(1);
    AD2SB_CS_H;
}

uint16_t AD2SB_Regs[10];

void AD2SB_Regs_Init()
{

    AD2SB_Reset();

    AD2SB_CS_L;

    DELAY_US(1);

    AD2SB_Write(0x92,0x75); //14bit resolver resolution
    AD2SB_Write(0x91,0x28);
    AD2SB_Write(0x8b,0x69);
    AD2SB_Write(0x8c,0x3c);
    AD2SB_Write(0x88,0x3a);
    AD2SB_Write(0x89,0x6c);
    AD2SB_Write(0x8a,0x0a);

    AD2SB_Regs[0] = AD2SB_Read(0x88); // LOS threshold
    AD2SB_Regs[1] = AD2SB_Read(0x89); // DOS overrange threshold
    AD2SB_Regs[2] = AD2SB_Read(0x8A); // DOS mismatch threshold
    AD2SB_Regs[3] = AD2SB_Read(0x8B); // DOS reset maximum
    AD2SB_Regs[4] = AD2SB_Read(0x8C); // DOS reset minimum
    AD2SB_Regs[5] = AD2SB_Read(0x8D); // LOT High
    AD2SB_Regs[7] = AD2SB_Read(0x91); // Excitation Frequency
    AD2SB_Regs[8] = AD2SB_Read(0x92); // Control
    AD2SB_Regs[9] = AD2SB_Read(0xff); // Fault
    while (AD2SB_Regs[0]!=0x01){
        AD2SB_Write(0x88,0x01); // 38mv
        AD2SB_Regs[0] = AD2SB_Read(0x88);
    }

    while (AD2SB_Regs[2]!=0x7F){
        AD2SB_Write(0x8A,0x7F); // 4.826V
        AD2SB_Regs[2] = AD2SB_Read(0x8A);
    }

    DELAY_US(20);

    AD2SB_SAMPLE_H;
    AD2SB_SAMPLE_L;

    AD2SB_Fault = AD2SB_Read(0xff);

    AD2SB_CS_H;

}

float ALIGN_ANGLE = 0.76; // Maya120 motor 023-1p,0.185-2p
// Maya120 motor UNNC 0.79-2P

/* M1_ResolverElecTheta_pu：FOC 电角度，由 MotorControlISR 写入并读取
 * Eangle_resolverC：SPIC 通道电角度，HPMC_main_80.c 在 angle_channel == SPIC 时读取 */
float M1_ResolverElecTheta_pu;
float Eangle_resolverC;

