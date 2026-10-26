#include "TLF35584.h"

void TLF35584_spi_init(void)
{
    EALLOW;

    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;   // Enable pull-up on GPIO54
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;   // Enable pull-up on GPIO55
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;   // Enable pull-up on GPIO56

    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3;  // Asynch input GPIO54
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3; // Asynch input GPIO55
    GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 3; // Asynch input GPIO56

    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 0; // GPIO54 peripheral mux: GMUX = 0
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1; // GPIO54 peripheral mux: MUX = 1
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 0; // GPIO55 peripheral mux: GMUX = 0
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1; // GPIO55 peripheral mux: MUX = 1
    GpioCtrlRegs.GPBGMUX2.bit.GPIO56 = 0; // GPIO56 peripheral mux: GMUX = 0
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1; // GPIO56 peripheral mux: MUX = 1

    // GPIO57 --- /CS
    GPIO_SetupPinOptions(57, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(57,0,0);
    GpioDataRegs.GPBSET.bit.GPIO57 = 1;
    // WDI
    GPIO_SetupPinOptions(78, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(78,0,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;
    // ERR
    GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(42,0,0);
    GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1;

    SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // SPI software reset. You should clear this bit before the changes
    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;   // Character length = SPICHAR + 1 = 16 bits (TLF35584 16-bit frame)
    SpiaRegs.SPICCR.bit.SPILBK = 0;      // SPI loop back mode disabled
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; // Shift Clock Polarity.
    // 1-Data is output on falling edge and input on rising edge.
    // 0-Data is output on rising edge and input on falling edge.
    SpiaRegs.SPICTL.all = 0x0006;        // master, enables transimission  disables interrupt
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
    SpiaRegs.SPIBRR.all = 0x09;            // rate = LSPCLK/(SPIBBR+1)     LSPCLK = 25M   LOSPCP
    SpiaRegs.SPIPRI.bit.FREE = 1;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    // fifo
    SpiaRegs.SPIFFTX.bit.SPIRST = 0x1;    // SPI FIFO can resume transmit or receive. No effect to the SPI registers bits.
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 0x1;  // SPI FIFO enhancements are enabled.
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0x1;    // Release transmit FIFO from reset
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 0x1;    // Write 1 to clear SPIFFTX[TXFFINT] flag.
    SpiaRegs.SPIFFTX.bit.TXFFIENA = 0x0;  // TX FIFO interrupt based on TXFFIL match (less than or equal to) will be disabled.
    //
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 0x1;    // Write 0 does not affect RXFFOVF flag bit, Bit reads back a zero.
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0x1;   // 1h (R/W) = Re-enable receive FIFO operation
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 0x1;    // Write 1 to clear SPIFFRX[RXFFINT] flag
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0x0;      // RX FIFO interrupt based on RXFFIL match (greater than or equal to) will be disabled.
    SpiaRegs.SPIFFRX.bit.RXFFIL = 0x0;

    SpiaRegs.SPIFFCT.all = 0x0000;

    EDIS;
}

uint16_t calcParity(uint16_t data)
{
    uint16_t ones = 0;
    uint16_t i;
    // 只检查高15位 (bit15 - bit1)
    for (i = 1; i < 16; i++) {
        if (data & (1 << i)) {
            ones++;
        }
    }
    // 如果1的个数是奇数，则奇偶位为1；偶数为0
    return (ones % 2) ? 1 : 0;
}
volatile uint16_t spi_trans_ticker = 0;
uint16_t TLF35584_SPI_Transaction(uint16_t cmd)
{
    uint16_t parity = calcParity(cmd);   // 计算奇偶校验
    uint16_t txData = cmd | parity;      // 将奇偶位放到 LSB

    uint16_t rxData;

    // 拉低片选
    TLF35584_CS_L;

    DELAY_US(1); // 短延时，满足时序要求

    // 发送数据 (SPI 收发同步进行)
    SpiaRegs.SPITXBUF = txData;

    // 等待接收完成 (SPI 中断或轮询)
    while (SpiaRegs.SPIFFRX.bit.RXFFST != 1) {} // 等待 FIFO 有数据
    rxData = SpiaRegs.SPIRXBUF;

    // 拉高片选
    DELAY_US(1);
    spi_trans_ticker++;
    TLF35584_CS_H;

    return rxData;
}

uint16_t readreg_cmd1 = 0;

uint16_t TLF35584_ReadReg(uint16_t addr)
{
    // 读命令: CMD = 0, ADDR = addr, DATA = 0x00
    uint16_t cmd = (0 << 15) | ((addr & 0x3F) << 9) | (0x00 << 1);
    readreg_cmd1 = cmd;
    uint16_t response = TLF35584_SPI_Transaction(cmd);

    return response;
}

void TLF35584_WriteReg(uint16_t addr, uint16_t data)
{
    // 写命令: CMD = 1, ADDR = addr, DATA = data
    uint16_t cmd = (1u << 15) | ((0x3F & addr) << 9) | ((0xFF & data) << 1);

    TLF35584_SPI_Transaction(cmd);
}

void TLF35584_Trigger_WWD(void)
{
    uint16_t status = TLF35584_ReadReg(0x17); // 读取 WWDSCMD 状态寄存器
    uint16_t trigStatus = (status >> 8) & 0x01; // 假设 TRIG_STATUS 在 bit1

    uint16_t data;
    if (trigStatus) {
        data = 0x00; // 如果状态为1，写入0触发
    } else {
        data = 0x01; // 如果状态为0，写入1触发
    }

    // 构建写命令: 地址0x17, 数据=data
    TLF35584_WriteReg(0x17, data);
}

int TLF_normal_flag = 0;

uint16_t TLF_devctrl =0;
uint16_t TLF_devctrln =0;

void TLF35584_GoToNormal(void)
{
    // 步骤1: 等待60us (内部处理时间) [citation:8]
    DELAY_US(60);

    // 步骤2: 发送状态切换命令
    // 假设切换到 NORMAL 的配置数据为 0xEA (示例值, 需参考数据手册)
    uint16_t devctrl_data = 0xEA;
    uint16_t devctrln_data = ~devctrl_data; // 必须按位取反 [citation:5][citation:8]
    devctrln_data = devctrln_data&0xFF;
    // 写入 DEVCTRL (地址0x15)
    TLF35584_WriteReg(0x15, devctrl_data);
    // 立即连续写入 DEVCTRLN (地址0x16)
    TLF35584_WriteReg(0x16, devctrln_data);
    // 此时片选上升沿将生效，切换状态
    DELAY_US(60);
    // 步骤3: 验证状态 (读取 0x27 状态寄存器)
    uint16_t status = TLF35584_ReadReg(0x27);
    if ((status & 0x07) == 0x02) { // 0x02表示NORMAL
        // 切换成功
        TLF_normal_flag = 1;
    }
    TLF_devctrln = TLF35584_ReadReg(0x16);
    TLF_devctrl = TLF35584_ReadReg(0x15);

}

uint16_t timer0 = 0;

volatile uint16_t err_toggle = 0;

/**
 * @brief CPU 定时器 0 中断服务程序
 */
__interrupt void cpu_timer0_isr(void)
{
    if (err_toggle) {
        GpioDataRegs.GPBSET.bit.GPIO42 = 1;
    } else {
        GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1;
    }
    err_toggle = !err_toggle;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    timer0 ++;
}

void init_ERR_Timer(void)
{
    // 假设 CPU 频率 200MHz，每 20μs 中断一次 (25kHz 半周期 = 20μs)
    uint32_t period = 5000;   // 200M / 2 / 25k = 4000

    // 注册中断向量
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;

    // 停止定时器
    CpuTimer0Regs.TCR.bit.TSS = 1;

    // 设置周期
    CpuTimer0Regs.PRD.all = period - 1;
    CpuTimer0Regs.TPR.all = 0;
    CpuTimer0Regs.TPRH.all = 0;

    // 控制设置
    CpuTimer0Regs.TCR.bit.TRB = 1;           // 重载
    CpuTimer0Regs.TCR.bit.SOFT = 1;
    CpuTimer0Regs.TCR.bit.FREE = 1;           // 自由运行
    CpuTimer0Regs.TCR.bit.TIE = 1;            // 使能中断

    // 使能 PIE 中断
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER |= M_INT1;

    // 启动定时器
    CpuTimer0Regs.TCR.bit.TSS = 0;
    EDIS;
}
uint16_t USYSPCFG1_STS05 = 0;
uint16_t LSYSPCFG1_STS05 = 0;
uint16_t ULSYSPCFG1_STS05 = 0;
uint16_t RWDCFG0_STS0D = 0;
uint16_t RSYSPCFG1_STS0C = 0;
void TLF35584_DisableERRandWWD(void)
{

    TLF35584_Trigger_WWD();

    ULSYSPCFG1_STS05 = TLF35584_ReadReg(0x28);

    // 1. 解锁受保护寄存器序列 [citation:4]
    // 序列：0xAB, 0xEF, 0x56, 0x12 (连续写入 PROTCFG 地址 0x03)
    TLF35584_WriteReg(0x03, 0xAB);
    TLF35584_WriteReg(0x03, 0xEF);
    TLF35584_WriteReg(0x03, 0x56);
    TLF35584_WriteReg(0x03, 0x12);

    // 可选：读取 PROTSTAT (地址 0x28) 验证解锁是否成功，此处为简化省略
    USYSPCFG1_STS05 = TLF35584_ReadReg(0x28);
    // 2. 禁用 ERR 监控 (SYSPCFG1 地址 0x05) [citation:2][citation:4]
    TLF35584_WriteReg(0x05, 0x00); // 清除 bit0 (ERREN)

    // 3. 禁用窗口看门狗 (WDCFG0 地址 0x06) [citation:2][citation:4][citation:5]
    TLF35584_WriteReg(0x06, 0x93); // 清除 bit0 (WWDEN)

    // 4. 加锁受保护寄存器序列 [citation:4]
    // 序列：0xDF, 0x34, 0xBE, 0xCA (连续写入 PROTCFG 地址 0x03)
    TLF35584_WriteReg(0x03, 0xDF);
    TLF35584_WriteReg(0x03, 0x34);
    TLF35584_WriteReg(0x03, 0xBE);
    TLF35584_WriteReg(0x03, 0xCA);

    // 5. 等待至少60us，让新配置生效 [citation:1][citation:2][citation:4]
    DELAY_US(60);
    LSYSPCFG1_STS05 = TLF35584_ReadReg(0x28);
    RWDCFG0_STS0D = TLF35584_ReadReg(0x0D);
    RSYSPCFG1_STS0C = TLF35584_ReadReg(0x0C);

}

void TLF35584_EnableWWDandERR(void)
{

    // 1. 解锁受保护寄存器序列 [citation:4]
    // 序列：0xAB, 0xEF, 0x56, 0x12 (连续写入 PROTCFG 地址 0x03)
    TLF35584_WriteReg(0x03, 0xAB);
    TLF35584_WriteReg(0x03, 0xEF);
    TLF35584_WriteReg(0x03, 0x56);
    TLF35584_WriteReg(0x03, 0x12);

    TLF35584_WriteReg(0x05, 0x08);
    DELAY_US(60);
    TLF35584_WriteReg(0x06, 0x9B);

    // 4. 加锁受保护寄存器
    TLF35584_WriteReg(0x03, 0xDF);
    TLF35584_WriteReg(0x03, 0x34);
    TLF35584_WriteReg(0x03, 0xBE);
    TLF35584_WriteReg(0x03, 0xCA);

    DELAY_US(60);
    LSYSPCFG1_STS05 = TLF35584_ReadReg(0x28);
    RWDCFG0_STS0D = TLF35584_ReadReg(0x0D);
    RSYSPCFG1_STS0C = TLF35584_ReadReg(0x0C);
}

void TLF35584_ConfigWWD(uint16_t closed_time, uint16_t open_time)
{
    // 1. 解锁受保护寄存器
    TLF35584_WriteReg(0x03, 0xAB);
    TLF35584_WriteReg(0x03, 0xEF);
    TLF35584_WriteReg(0x03, 0x56);
    TLF35584_WriteReg(0x03, 0x12);

    // 2. 配置关闭窗口时间[citation:3]
    TLF35584_WriteReg(0x09, closed_time & 0xFF);

    // 3. 配置开启窗口时间[citation:3]
    TLF35584_WriteReg(0x0A, open_time & 0xFF);

    // 4. 加锁受保护寄存器
    TLF35584_WriteReg(0x03, 0xDF);
    TLF35584_WriteReg(0x03, 0x34);
    TLF35584_WriteReg(0x03, 0xBE);
    TLF35584_WriteReg(0x03, 0xCA);
    DELAY_US(60);
}

void TLF35584_flag_error_clear(void)
{
    // 1. 解锁受保护寄存器
    TLF35584_WriteReg(0x1B, 0xFF);
    TLF35584_WriteReg(0x1C, 0xFF);
    TLF35584_WriteReg(0x1D, 0xFF);

    DELAY_US(60);
}

/******************************************************************************
 * @brief   GPIO43 low, delay 100 ms, GPIO50 clear then set
 * @note    GPIO50 is cleared again in TLF35584_StartWatchdog()
 ******************************************************************************/
void InitGpio43And50(void)
{
    EALLOW;
    GPIO_SetupPinOptions(43, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(43,0,0);
    GpioDataRegs.GPBCLEAR.bit.GPIO43 = 1;

    DELAY_US(100000);

    GPIO_SetupPinOptions(50, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(50,0,0);
    GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
    GpioDataRegs.GPBSET.bit.GPIO50 = 1;
    EDIS;
}

/******************************************************************************
 * @brief   Configure and enable the TLF35584 window watchdog and ERR monitor
 * @note    Last step before the main loop; the main loop must then trigger the WWD
 ******************************************************************************/
void TLF35584_StartWatchdog(void)
{
    TLF35584_ConfigWWD(0x00, 0x03);
    TLF35584_EnableWWDandERR();
    TLF35584_flag_error_clear();
    GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
    TLF35584_Trigger_WWD();
}
