#include "ServAD2S.h"


void SPI_init(void)
{
    SPIB_GPIO_Init();

    EALLOW; // below registers are "protected", allow access.

    //GPIO65 --- /WR1
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(65,0,0);
    GpioDataRegs.GPCSET.bit.GPIO65=1;

    // /SAMPLE1
    GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(64,0,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO64=1;

    // /CS1
    GPIO_SetupPinOptions(59, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(59,0,0);
    GpioDataRegs.GPBSET.bit.GPIO59=1;

    GPIO_SetupPinOptions(14, GPIO_INPUT, GPIO_ASYNC);//LOT
    GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_ASYNC);//ROS

    //spib
    SpibRegs.SPICCR.bit.SPISWRESET=0;  //SPI software reset. You should clear this bit before the changes
    SpibRegs.SPICCR.bit.SPICHAR=0x7;   //Character Length Control Bits 3-0~16bit    Set as 8bit mode
    SpibRegs.SPICCR.bit.SPILBK=0;      //SPI loop back mode disabled
    SpibRegs.SPICCR.bit.CLKPOLARITY=0; //Shift Clock Polarity.
                                       // 1-Data is output on falling edge and input on rising edge.
                                       // 0-Data is output on rising edge and input on falling edge.
    SpibRegs.SPICTL.all=0x0006;        // master, enables transimission  disables interrupt
    SpibRegs.SPICTL.bit.CLK_PHASE=0;
//    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0x10; //LSPCLK = 50M
    SpibRegs.SPIBRR.all=5;            // rate=LSPCLK/(SPIBBR+1)     LSPCLK=25M   LOSPCP
    SpibRegs.SPIPRI.bit.FREE = 1;
    SpibRegs.SPICCR.bit.SPISWRESET=1;

    EDIS;

#ifdef ENABLE_CRC_LUT
    gen_CRCtable();
#endif

}


void SPIA_GPIO_Init(void)
{

}


void SPIB_GPIO_Init(void)
{
    EALLOW;

 /* Enable internal pull-up for the selected pins */
 // Pull-ups can be enabled or disabled by the user.
 // This will enable the pullups for the specified pins.
 // Comment out other unwanted lines.

     GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
     GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;   // Enable pull-up on GPIO19 (SPISTEA)
     GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;   // Enable pull-up on GPIO18 (SPICLKA)

 /* Set qualification for selected pins to asynch only */
 // This will select asynch (no qualification) for the selected pins.
 // Comment out other unwanted lines.

     GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;  // Asynch input GPIO3 (SPISOMIA)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO18 (SPICLKA)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3; // Asynch input GPIO19 (SPISTEA)

 /* Configure SPI-A pins using GPIO regs*/
 // This specifies which of the possible GPIO pins will be SPI functional pins.
 // Comment out other unwanted lines.

     GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 1; // Configure GPIO16 as SPISIMOA
     GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 2; // Configure GPIO16 as SPISIMOA

     GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 1; // Configure GPIO16 as SPISIMOA
     GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 2; // Configure GPIO17 as SPISOMIA

     GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 1; // Configure GPIO16 as SPISIMOA
     GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 2; // Configure GPIO18 as SPICLKA

     EDIS;
}

void SPIC_GPIO_Init(void)
{
    EALLOW;

 /* Enable internal pull-up for the selected pins */
 // Pull-ups can be enabled or disabled by the user.
 // This will enable the pullups for the specified pins.
 // Comment out other unwanted lines.

    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;   // Enable pull-up on GPIO19 (SPISTEA)
    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;   // Enable pull-up on GPIO18 (SPICLKA)

 /* Set qualification for selected pins to asynch only */
 // This will select asynch (no qualification) for the selected pins.
 // Comment out other unwanted lines.

     GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 3;  // Asynch input GPIO3 (SPISOMIA)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 3; // Asynch input GPIO18 (SPICLKA)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 3; // Asynch input GPIO19 (SPISTEA)

 /* Configure SPI-A pins using GPIO regs*/
 // This specifies which of the possible GPIO pins will be SPI functional pins.
 // Comment out other unwanted lines.

     GpioCtrlRegs.GPBGMUX2.bit.GPIO50 = 1; // Configure GPIO16 as SPISIMOA
     GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 2; // Configure GPIO16 as SPISIMOA

     GpioCtrlRegs.GPBGMUX2.bit.GPIO51 = 1; // Configure GPIO16 as SPISIMOA
     GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 2; // Configure GPIO17 as SPISOMIA

     GpioCtrlRegs.GPBGMUX2.bit.GPIO52 = 1; // Configure GPIO16 as SPISIMOA
     GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 2; // Configure GPIO18 as SPICLKA

     EDIS;
}

void SPI_fifo_init(void)
{
    EALLOW;

    SpibRegs.SPIFFTX.bit.SPIRST=0x1;    // SPI FIFO can resume transmit or receive. No effect to the SPI registers bits.
    SpibRegs.SPIFFTX.bit.SPIFFENA=0x1;  // SPI FIFO enhancements are enabled.
    SpibRegs.SPIFFTX.bit.TXFIFO=0x1;    //Release transmit FIFO from reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR=0x1;    // Write 1 to clear SPIFFTX[TXFFINT] flag.
    SpibRegs.SPIFFTX.bit.TXFFIENA=0x0;  // TX FIFO interrupt based on TXFFIL match (less than or equal to) will be disabled.
//
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=0x1;    //Write 0 does not affect RXFFOVF flag bit, Bit reads back a zero.
    SpibRegs.SPIFFRX.bit.RXFIFORESET=0x1;   //1h (R/W) = Re-enable receive FIFO operation
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=0x1;    //Write 1 to clear SPIFFRX[RXFFINT] flag
    SpibRegs.SPIFFRX.bit.RXFFIENA=0x0;      //RX FIFO interrupt based on RXFFIL match (greater than or equal to) will be disabled.
    SpibRegs.SPIFFRX.bit.RXFFIL=0x0;

    SpibRegs.SPIFFCT.all = 0x0000;

    EDIS;
}


uint16_t old_data_test=0;
uint16_t old_dataB_test=0;

uint16_t new_data_test=0;
uint16_t new_dataB_test=0;

uint16_t AD2S_Test_Ticker=0;
uint16_t AD2SB_Test_Ticker=0;

void AD2S_Write(uint16_t addr,uint16_t data)
{

    while(new_data_test!=data)
    {
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    old_data_test=SpibRegs.SPIRXBUF;

    DELAY_US(1);

    SpibRegs.SPITXBUF=(data<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    new_data_test=SpibRegs.SPIRXBUF;

    DELAY_US(1);
    }
}

void AD2SB_Write(uint16_t addr,uint16_t data){

    AD2SB_WR_L;

    while(new_dataB_test!=data){
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    old_dataB_test=SpibRegs.SPIRXBUF;

    AD2SB_WR_H;

    DELAY_US(1);

    AD2SB_WR_L;

    SpibRegs.SPITXBUF=(data<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    new_dataB_test=SpibRegs.SPIRXBUF;

    AD2SB_WR_H;

    DELAY_US(1);
    }
}

void AD2S_Write_DA7F(uint16_t addr){
    while(new_data_test!=0x7F){
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    old_data_test=SpibRegs.SPIRXBUF;
    DELAY_US(1);
    SpibRegs.SPITXBUF=(0x7F<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    new_data_test=SpibRegs.SPIRXBUF;
    DELAY_US(1);
    }
}
void AD2S_Write_DA01(uint16_t addr){
    while(new_data_test!=0x01){
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    old_data_test=SpibRegs.SPIRXBUF;
    DELAY_US(1);
    SpibRegs.SPITXBUF=(0x01<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    new_data_test=SpibRegs.SPIRXBUF;
    DELAY_US(1);
    }
}

uint16_t AD2S_Read(uint16_t addr)
{
    uint16_t rxval;

//    AD2S_WR_L;
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    rxval = (SpibRegs.SPIRXBUF);
//    AD2S_WR_H;
    DELAY_US(1);
//    AD2S_WR_L;
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    rxval = (SpibRegs.SPIRXBUF);
//    AD2S_WR_H;
    DELAY_US(1);

    return rxval;
}

uint16_t AD2SB_Read(uint16_t addr)
{
    uint16_t rxval;

    AD2SB_WR_L;
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    rxval = (SpibRegs.SPIRXBUF);
    AD2SB_WR_H;
//    GpioDataRegs.GPCCLEAR.bit.GPIO81 = 1;
    DELAY_US(1);
    AD2SB_WR_L;
    SpibRegs.SPITXBUF=(addr<<8);
    while(SpibRegs.SPIFFRX.bit.RXFFST<1){};
    rxval = (SpibRegs.SPIRXBUF);
    AD2SB_WR_H;
    DELAY_US(1);

    return rxval;
}


uint16_t AD2S_Speed_Read(){
    uint16_t rxspd;


    rxspd = AD2S_Read(0x82);
    rxspd = (rxspd<<8) | AD2S_Read(0x83) ;

    return rxspd;
}

uint16_t AD2SB_Speed_Read(){
    uint16_t rxspd;


    rxspd = AD2SB_Read(0x82);
    rxspd = (rxspd<<8) | AD2SB_Read(0x83) ;

    return rxspd;
}

uint16_t AD2S_Fault;
uint16_t AD2SB_Fault;

uint16_t AD2S_Position_Read(){
    uint16_t rxpos;

    rxpos = AD2S_Read(0x80);
    rxpos = (rxpos<<8) | AD2S_Read(0x81) ;


    return rxpos;
}

uint16_t AD2SB_Position_Read(){
    uint16_t rxpos;

    rxpos = AD2SB_Read(0x80);

    rxpos = (rxpos<<8) | AD2SB_Read(0x81) ;


    return rxpos;
}


void AD2S_Reset()
{
    AD2S_RESET_L;
    DELAY_US(1);
    AD2S_SAMPLE_H;
    DELAY_US(20);
    AD2S_RESET_H;
    DELAY_US(50000);
    AD2S_SAMPLE_L;
    DELAY_US(5);
    AD2S_SAMPLE_H;

    AD2S_SAMPLE_L;
    DELAY_US(1);
    AD2S_SAMPLE_H;
    AD2S_Fault = AD2S_Read(0xff);
    AD2S_SAMPLE_L;
    DELAY_US(1);
    AD2S_SAMPLE_H;
    DELAY_US(1);

}

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

uint16_t AD2S_Regs[10];
uint16_t AD2SB_Regs[10];

void AD2S_Regs_Init()
{
    AD2S_Reset();

    AD2S_Write(0x92,0x7f);
    AD2S_Write(0x91,0x21);//set resolver frequency to 10k
    AD2S_Write(0x8b,0x7f);
    AD2S_Write(0x8c,0x01);
    AD2S_Write(0x88,0x01);
    AD2S_Write(0x89,0x7f);
    AD2S_Write(0x8a,0x7f);

    AD2S_Regs[0]=AD2S_Read(0x88); //LOS threshold
    AD2S_Regs[1]=AD2S_Read(0x89); //DOS overrange threshold
    AD2S_Regs[2]=AD2S_Read(0x8A); //DOS mismatch threshold
    AD2S_Regs[3]=AD2S_Read(0x8B); //DOS reset maximum
    AD2S_Regs[4]=AD2S_Read(0x8C); //DOS reset minimum
    AD2S_Regs[5]=AD2S_Read(0x8D); //LOT High
    AD2S_Regs[7]=AD2S_Read(0x91); //Excitation Frequency
    AD2S_Regs[8]=AD2S_Read(0x92); //Control
    while(AD2S_Regs[0]!=0x01){
        AD2S_Write(0x88,0x01);
        AD2S_Regs[0]=AD2S_Read(0x88);
    }

    while(AD2S_Regs[2]!=0x7F){
        AD2S_Write(0x8A,0x7F);
        AD2S_Regs[2]=AD2S_Read(0x8A);
    }
}

void AD2SB_Regs_Init()
{

    AD2SB_Reset();

    AD2SB_CS_L;

    DELAY_US(1);

    AD2SB_Write(0x92,0x7f);
    AD2SB_Write(0x91,0x28);//set resolver frequency to 10k
    AD2SB_Write(0x8b,0x69);//DOS??ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¯Ã‚Â¿Ã‚Â½???? ?????38mv   3.99V
    AD2SB_Write(0x8c,0x3c);//DOS??ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¯Ã‚Â¿Ã‚Â½??ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¯Ã‚Â¿Ã‚Â½???              2.28V
    AD2SB_Write(0x88,0x3a);//LOS???                     2.204V
    AD2SB_Write(0x89,0x6c);//DOS????????                4.104V
    AD2SB_Write(0x8a,0x0a);//DOS????                  0.38V

    AD2SB_Regs[0]=AD2SB_Read(0x88); //LOS threshold
    AD2SB_Regs[1]=AD2SB_Read(0x89); //DOS overrange threshold
    AD2SB_Regs[2]=AD2SB_Read(0x8A); //DOS mismatch threshold
    AD2SB_Regs[3]=AD2SB_Read(0x8B); //DOS reset maximum
    AD2SB_Regs[4]=AD2SB_Read(0x8C); //DOS reset minimum
    AD2SB_Regs[5]=AD2SB_Read(0x8D); //LOT High
    AD2SB_Regs[7]=AD2SB_Read(0x91); //Excitation Frequency
    AD2SB_Regs[8]=AD2SB_Read(0x92); //Control
    AD2SB_Regs[9]=AD2SB_Read(0xff); //Fault
    while(AD2SB_Regs[0]!=0x01){
        AD2SB_Write(0x88,0x01);//38mv
        AD2SB_Regs[0]=AD2SB_Read(0x88);
    }

    while(AD2SB_Regs[2]!=0x7F){
        AD2SB_Write(0x8A,0x7F);//4.826V
        AD2SB_Regs[2]=AD2SB_Read(0x8A);
    }

    DELAY_US(20);

    AD2SB_SAMPLE_H;
    AD2SB_SAMPLE_L;

    AD2SB_Fault = AD2SB_Read(0xff);

    AD2SB_CS_H;

}

_iq15  mech_angle;
_iq15  mech_angleB;

float ALIGN_ANGLE= 0.76;//Maya120 motor 023-1p,0.185-2p
//Maya120 motor UNNC 0.79-2P
int resolver_dir=0;//M60 motor
int resolver_dirB=1;//Maya70V2 motor

float angle16;
float angle16B;


_iq15 angle_tmp;
_iq15 angle_tmpB;

float M1_ResolverElecTheta_pu;
float Eangle_resolverC;

uint32_t angle_raw=0;
uint32_t angle_rawB=0;


void AD2S_Angle_cal(){

    static float correct_angle=_IQ15(0.0);
    AD2S_SAMPLE_L;
    DELAY_US(1);
    AD2S_Test_Ticker++;
    if(AD2S_Test_Ticker>=5000){
        AD2S_Fault = AD2S_Read(0xff);
        AD2S_Test_Ticker=0;;
    }
    angle_raw=AD2S_Position_Read();
    mech_angle=_IQ15(((float)angle_raw)/65536.0);
    if(resolver_dir!=1)
    {
#ifdef F10
        angle_tmp=_IQ15frac((-1.0*mech_angle)*(POLES/2.0/1.0));
#endif
#ifdef MPMOTOR
        //angle_tmp=_IQ15frac((-1.0*mech_angle)*(POLES/2.0/3.0));
        angle_tmp=_IQ15frac((-1.0*mech_angle)*(POLES/2.0/2.0)); // mpb motor
#endif
    }
    else
    {
#ifdef F10
        angle_tmp=_IQ15frac((mech_angle)*(POLES/2.0/1.0));
#endif
#ifdef MPMOTOR
        //angle_tmp=_IQ15frac((mech_angle)*(POLES/2.0/3.0));
        angle_tmp=_IQ15frac((-1.0*mech_angle)*(POLES/2.0/2.0)); // mpb motor
#endif
    }

    angle16=_IQ15frac(1.0+angle_tmp+ALIGN_ANGLE);//_IQ15frac(angle_tmp);

    if((angle16>=_IQ15(0.0))&&(angle16<=_IQ15(1.0)))
    {
        correct_angle=angle16;
        M1_ResolverElecTheta_pu=correct_angle;
    }

    AD2S_SAMPLE_H;
    DELAY_US(1);
    AD2S_SAMPLE_L;
    DELAY_US(1);
    AD2S_SAMPLE_H;
    DELAY_US(1);
}
volatile int AD2SB_fault_ticker = 0;

void AD2SB_Angle_cal()
{

    AD2SB_CS_L;

    static float correct_angleB=_IQ15(0.0);
    AD2SB_SAMPLE_L;
    DELAY_US(1);
    AD2SB_Test_Ticker++;
    if(AD2SB_Test_Ticker>=5000)
    {

        AD2SB_Fault = AD2SB_Read(0xff);

        AD2SB_Test_Ticker=0;
        if(AD2SB_Fault!=0)
        {
            AD2SB_fault_ticker++;
        }
    }

    angle_rawB=AD2SB_Position_Read();

    mech_angleB=_IQ15(((float)angle_rawB)/65536.0);
    if(resolver_dirB!=1)
    {
#ifdef F10
        angle_tmp=_IQ15frac((-1.0*mech_angle)*(POLES/2.0/1.0));
#endif
#ifdef MPMOTOR
        angle_tmpB=_IQ15frac((-1.0*mech_angleB)*(POLES/2.0/1.0));
#endif
    }
    else
    {
#ifdef F10
        angle_tmp=_IQ15frac((mech_angle)*(POLES/2.0/1.0));
#endif
#ifdef MPMOTOR
        angle_tmpB=_IQ15frac((mech_angleB)*(POLES/2.0/1.0));
#endif
    }

    angle16B=_IQ15frac(1.0+angle_tmpB+ALIGN_ANGLE);//_IQ15frac(angle_tmp);

    if((angle16B>=_IQ15(0.0))&&(angle16B<=_IQ15(1.0)))
    {
        correct_angleB=angle16B;
        M1_ResolverElecTheta_pu=correct_angleB;
    }

    AD2SB_SAMPLE_H;
    DELAY_US(1);
    AD2SB_SAMPLE_L;
    DELAY_US(1);
    AD2SB_SAMPLE_H;
    DELAY_US(1);

    AD2SB_CS_H;
}
