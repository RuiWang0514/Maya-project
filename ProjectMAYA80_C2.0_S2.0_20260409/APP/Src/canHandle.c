#include "HPMC_Settings.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "driverlib/can.h"
#include "TLF35584.h"
#define CAN_ABO_DELAY_10MS_TICKS   (2000000UL) /* 10 ms * 200 MHz Clock Source= 0.01 * 200000000 = 2000000 ticks */

volatile unsigned long g_bErrFlagA = 0;
volatile unsigned long g_bErrFlagB = 0;


tCANMsgObject sTXCANMessageA;
tCANMsgObject TxCANTempMessageA;
tCANMsgObject TxCANDataMessageA;
tCANMsgObject TxPT100DataMessageA;
tCANMsgObject TxDTCDataMessageA;
tCANMsgObject TxMotor310MessageA;
tCANMsgObject TxMotor311MessageA;
tCANMsgObject RxControlCmdMessageA;
tCANMsgObject RxPidCmdMessageA;
tCANMsgObject RxRE38CanAngleMessageA;
tCANMsgObject RXCANA_boot;
tCANMsgObject TXCANA_boot;


unsigned char ucTXMsgDataA[8] ={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char tempdata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char operatedata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char PT100data[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char DTCdata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char motor310txdata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char motor311txdata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char RXMsgBOOT[8];
unsigned char TXCANA[8];
/* 0x310 协议变量（实际物理量，外发前自行写入或映射外部变量） */
int16_t  motor310_torque = 0;           /* Byte2-3 电机力矩 */
unsigned char motor310_status = 0;           /* Byte6 状态故障 */
int motor310_iq_ref_avg = 0;            /* Iq_ref 10次均值 */
int motor310_current_rms = 0;           /* 由 iq_ref_avg 换算，外发时 *0.25，DBC *4 还原 */

/* 0x311 协议变量 */
unsigned char motor311_reserved_b2 = 0;      /* Byte2 预留 */
uint16_t motor311_torque_limit = 0;     /* Byte3-4 转矩限值（当前未使用） */
unsigned char motor311_ctrl_fault = 0;       /* Byte5 控制器硬件故障 */
unsigned char motor311_motor_fault = 0;      /* Byte6 电机故障 */
unsigned char motor311_life_signal = 0;      /* Byte7 逻辑程序生命信号 */

unsigned char RxControlCmdDataA[8];
unsigned char RxPidCmdDataA[8];
unsigned char re38_can_rx_data[8];
volatile uint16_t CANA_pid_msg_received = 0;
volatile uint16_t CANA_ctrl_msg_received = 0;
volatile uint16_t re38_can_rx_msg_received = 0;
volatile float re38_can_target_rpm = RE38_THROTTLE_IDLE_RPM;
static uint16_t re38_can_rx_timeout_cnt = 0;
static uint16_t re38_can_rx_ever_ok = 0;

tCANMsgObject sTXCANMessageB;
tCANMsgObject sRXCANMessageB;
unsigned char ucTXMsgDataB[8];
unsigned char ucRXMsgDataB[8];
extern float RE38_ClampTargetRpm(float rpm);

extern ThrottleType_e throttle_type;
extern PIDREG3_027 M1_pid_CurrLp_iq;
extern PIDREG3_027 M1_pid_CurrLp_id;
extern PIDREG3_027 M1_pid_SpdLp;


void CANBMsgRead(tCANMsgObject *pMsgObject)
{
    unsigned char len=0;
    for(len = 0;len < pMsgObject->ui32MsgLen;len++)
    {
        ucRXMsgDataB[len] = *(pMsgObject->pucMsgData + len);

    }
}


//
// CANIntHandler - This function is the interrupt handler for the CAN
//                 peripheral. It checks for the cause of the interrupt, and
//                 maintains a count of all messages that have been
//                 transmitted.
//
__interrupt void CANBIntHandler(void)
{
    unsigned long ulStatus;

    ulStatus = CANIntStatus(CANB_BASE, CAN_INT_STS_CAUSE);

    if(ulStatus == CAN_INT_INT0ID_STATUS)
    {
        ulStatus = CANStatusGet(CANB_BASE, CAN_STS_CONTROL);

        if(((ulStatus & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7) &&
           ((ulStatus & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
        {
            g_bErrFlagB = 1;
        }
    }
    else if(ulStatus == 1)
    {
        CANIntClear(CANB_BASE, 1);
        g_bErrFlagB = 0;
    }
    else if(ulStatus == 2)
    {
        CANMessageGet(CANB_BASE, 2, &sRXCANMessageB, true);
        CANBMsgRead(&sRXCANMessageB);
        CANIntClear(CANB_BASE, 2);
        g_bErrFlagB = 0;
    }
    else if(ulStatus == 3)
    {
        CANMessageGet(CANB_BASE, 3, &sRXCANMessageB, true);
        CANBMsgRead(&sRXCANMessageB);
        CANIntClear(CANB_BASE, 3);
        g_bErrFlagB = 0;
    }
    else if(ulStatus == 4)
    {
        CANMessageGet(CANB_BASE, 4, &sRXCANMessageB, true);
        CANBMsgRead(&sRXCANMessageB);
        CANIntClear(CANB_BASE, 4);
        g_bErrFlagB = 0;
    }
    else
    {
    }

    CANGlobalIntClear(CANB_BASE, CAN_GLB_INT_CANINT0);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


__interrupt void CANAIntHandler(void)
{
    unsigned long ulStatus;

    ulStatus = CANIntStatus(CANA_BASE, CAN_INT_STS_CAUSE);

    if(ulStatus == CAN_INT_INT0ID_STATUS)
    {
        ulStatus = CANStatusGet(CANA_BASE, CAN_STS_CONTROL);

        if(((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7) &&
           ((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
        {
            g_bErrFlagA = 1;
        }
    }
    else if(ulStatus == 1)
    {
        CANIntClear(CANA_BASE, 1);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 2)
    {
        CANIntClear(CANA_BASE, 2);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 3)
    {
        CANIntClear(CANA_BASE, 3);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 4)
    {
        CANMessageGet(CANA_BASE, 4, &RxControlCmdMessageA, true);
        CANIntClear(CANA_BASE, 4);
        CANA_ctrl_msg_received = 1u;
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 8)
    {
        CANMessageGet(CANA_BASE, 8, &RxPidCmdMessageA, true);
        CANIntClear(CANA_BASE, 8);
        CANA_pid_msg_received = 1u;
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 9)
    {
        CANMessageGet(CANA_BASE, 9, &RxRE38CanAngleMessageA, true);
        CANIntClear(CANA_BASE, 9);
        re38_can_rx_msg_received = 1u;
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 6)
    {
        CANIntClear(CANA_BASE, 6);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 7)
    {
        CANIntClear(CANA_BASE, 7);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 5)
    {
        CANIntClear(CANA_BASE, 5);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 10)
    {
        CANIntClear(CANA_BASE, 10);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 11)
    {
        CANIntClear(CANA_BASE, 11);
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 12)
    {
        CANMessageGet(CANA_BASE, 12, &RXCANA_boot, true);
        CANIntClear(CANA_BASE, 12);
        g_bErrFlagA = 0;
    }
    else
    {
    }

    CANGlobalIntClear(CANA_BASE, CAN_GLB_INT_CANINT0);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


void CANA_Initialization()
{
    //CANA
    GPIO_SetupPinMux(70, GPIO_MUX_CPU1, 5); // GPIO18 - CANRXA
    GPIO_SetupPinMux(71, GPIO_MUX_CPU1, 5); // GPIO19 - CANTXA
    GPIO_SetupPinOptions(70, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(71, GPIO_OUTPUT, GPIO_PUSHPULL);
    //
    // Initialize the CAN controller
    //
    CANInit(CANA_BASE);

    //
    // Setup CAN to be clocked off the PLL output clock (500kHz CAN-Clock)
    //
    CANClkSourceSelect(CANA_BASE, 0);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() is used to determine the clock rate that
    // is used for clocking the CAN peripheral.  This can be replaced with a
    // fixed value if you know the value of the system clock, saving the extra
    // function call.  For some parts, the CAN peripheral is clocked by a fixed
    // 8 MHz regardless of the system clock in which case the call to
    // SysCtlClockGet() should be replaced with 8000000.  Consult the data
    // sheet for more information about CAN peripheral clocking.
    //
    CANBitRateSet(CANA_BASE, 200000000, 500000);


    /* Enable Auto-Bus-On */
    HWREGH(CANA_BASE + CAN_O_CTL) |= CAN_CTL_ABO;
    /* Delay 10ms */
    HWREG(CANA_BASE + CAN_O_ABOTR) = CAN_ABO_DELAY_10MS_TICKS;


    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    CANIntEnable(CANA_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
}

void CANB_Initialization()
{

    //CANB
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 6); // GPIO7 - CANRXB
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 6); // GPIO6 - CANTXB
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);

    //
    // Initialize the CAN controller
    //
    CANInit(CANB_BASE);

    //
    // Setup CAN to be clocked off the PLL output clock (500kHz CAN-Clock)
    //

    CANClkSourceSelect(CANB_BASE, 0);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() is used to determine the clock rate that
    // is used for clocking the CAN peripheral.  This can be replaced with a
    // fixed value if you know the value of the system clock, saving the extra
    // function call.  For some parts, the CAN peripheral is clocked by a fixed
    // 8 MHz regardless of the system clock in which case the call to
    // SysCtlClockGet() should be replaced with 8000000.  Consult the data
    // sheet for more information about CAN peripheral clocking.
    //

    CANBitRateSet(CANB_BASE, 200000000, 500000);

    /* Enable Auto-Bus-On */
    HWREGH(CANB_BASE + CAN_O_CTL) |= CAN_CTL_ABO;
    /* Delay 10ms */
    HWREG(CANB_BASE + CAN_O_ABOTR) = CAN_ABO_DELAY_10MS_TICKS;


    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    CANIntEnable(CANB_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
}

void CANB_Ready()
{
    CANEnable(CANB_BASE);

    //
    // Enable CAN Global Interrupt line0
    //
    CANGlobalIntEnable(CANB_BASE, CAN_GLB_INT_CANINT0);

    //
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 4 bytes that will contain an incrementing
    // value.  Initially it will be set to 0x12345678.
    //
    ucTXMsgDataB[0] = 0x00;
    ucTXMsgDataB[1] = 0x00;
    ucTXMsgDataB[2] = 0x00;
    ucTXMsgDataB[3] = 0x00;
    ucTXMsgDataB[4] = 0x00; //16-bit for 1000rpm
    ucTXMsgDataB[5] = 0x00;
    ucTXMsgDataB[6] = 0x00;//start on
    ucTXMsgDataB[7] = 0x00;

    sTXCANMessageB.ui32MsgID = 0x19F;              // V2.0 oil pump TX ID
    sTXCANMessageB.ui32MsgIDMask = 0;                   // no mask needed for TX
    sTXCANMessageB.ui32Flags = MSG_OBJ_TX_INT_ENABLE;   // enable interrupt on TX
    sTXCANMessageB.ui32MsgLen = sizeof(ucTXMsgDataB);   // size of message is 8
    sTXCANMessageB.pucMsgData = ucTXMsgDataB;           // ptr to message content

    //
    // Initialize the message object that will be used for receiving CAN
    // messages.
    //


//    *(unsigned long long*)ucRXMsgDataB = 0;
//    sRXCANMessageB.ui32MsgID = 0x101001A0;                      // CAN message ID - use 1
//    sRXCANMessageB.ui32MsgIDMask = 0;                  // no mask needed for TX
//    sRXCANMessageB.ui32Flags = MSG_OBJ_RX_INT_ENABLE;  // enable interrupt on RX
//    sRXCANMessageB.ui32MsgLen = sizeof(ucRXMsgDataB);   // size of message is 8
//    sRXCANMessageB.pucMsgData = ucRXMsgDataB;           // ptr to message content

    //
    // Setup the message object being used to receive messages
    //

//    CANMessageSet(CANB_BASE, 4, &sRXCANMessageB, MSG_OBJ_TYPE_RX);

}

void CANA_Ready()
{
    CANEnable(CANA_BASE);

    //
    // Enable CAN Global Interrupt line0
    //
    CANGlobalIntEnable(CANA_BASE, CAN_GLB_INT_CANINT0);

    //
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 4 bytes that will contain an incrementing
    // value.  Initially it will be set to 0x12345678.
    //

    sTXCANMessageA.ui32MsgID = 0x200110;                      // COMAC MCU Status 10
    sTXCANMessageA.ui32MsgIDMask = 0;                  // no mask needed for TX
    sTXCANMessageA.ui32Flags = MSG_OBJ_TX_INT_ENABLE;  // enable interrupt on TX
    sTXCANMessageA.ui32MsgLen = sizeof(ucTXMsgDataA);   // size of message is 8
    sTXCANMessageA.pucMsgData = ucTXMsgDataA;           // ptr to message content

    TxCANTempMessageA.ui32MsgID = 0x1000F0;
    TxCANTempMessageA.ui32MsgIDMask=1;
    TxCANTempMessageA.ui32Flags=MSG_OBJ_TX_INT_ENABLE;
    TxCANTempMessageA.ui32MsgLen=sizeof(tempdata);
    TxCANTempMessageA.pucMsgData=tempdata;

    TxCANDataMessageA.ui32MsgID = 0x1000F1;
    TxCANDataMessageA.ui32MsgIDMask=0;
    TxCANDataMessageA.ui32Flags=MSG_OBJ_TX_INT_ENABLE;
    TxCANDataMessageA.ui32MsgLen=sizeof(operatedata);
    TxCANDataMessageA.pucMsgData=operatedata;

    TxPT100DataMessageA.ui32MsgID = 0x1000F2;
    TxPT100DataMessageA.ui32MsgIDMask=0;
    TxPT100DataMessageA.ui32Flags=MSG_OBJ_TX_INT_ENABLE;
    TxPT100DataMessageA.ui32MsgLen=sizeof(PT100data);
    TxPT100DataMessageA.pucMsgData=PT100data;

    TxDTCDataMessageA.ui32MsgID = 0x1000F4;
    TxDTCDataMessageA.ui32MsgIDMask=0;
    TxDTCDataMessageA.ui32Flags=MSG_OBJ_TX_INT_ENABLE;
    TxDTCDataMessageA.ui32MsgLen=sizeof(DTCdata);
    TxDTCDataMessageA.pucMsgData=DTCdata;

    TxMotor310MessageA.ui32MsgID = 0x310;
    TxMotor310MessageA.ui32MsgIDMask = 0;
    TxMotor310MessageA.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    TxMotor310MessageA.ui32MsgLen = sizeof(motor310txdata);
    TxMotor310MessageA.pucMsgData = motor310txdata;

    TxMotor311MessageA.ui32MsgID = 0x311;
    TxMotor311MessageA.ui32MsgIDMask = 0;
    TxMotor311MessageA.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    TxMotor311MessageA.ui32MsgLen = sizeof(motor311txdata);
    TxMotor311MessageA.pucMsgData = motor311txdata;

    //
    // Initialize the message object that will be used for receiving CAN
    // messages.
    //

    *(unsigned long *)RxControlCmdDataA = 0;
    RxControlCmdMessageA.ui32MsgID = 0x10A001;
    RxControlCmdMessageA.ui32MsgIDMask = 0;
    RxControlCmdMessageA.ui32Flags = MSG_OBJ_RX_INT_ENABLE;
    RxControlCmdMessageA.ui32MsgLen = sizeof(RxControlCmdDataA);
    RxControlCmdMessageA.pucMsgData = RxControlCmdDataA;

    *(unsigned long *)RxPidCmdDataA = 0;
    RxPidCmdMessageA.ui32MsgID = 0x10A002;
    RxPidCmdMessageA.ui32MsgIDMask = 0;
    RxPidCmdMessageA.ui32Flags = MSG_OBJ_RX_INT_ENABLE;
    RxPidCmdMessageA.ui32MsgLen = sizeof(RxPidCmdDataA);
    RxPidCmdMessageA.pucMsgData = RxPidCmdDataA;

    *(unsigned long *)re38_can_rx_data = 0;
    RxRE38CanAngleMessageA.ui32MsgID = RE38_CAN_MSG_ID;
    RxRE38CanAngleMessageA.ui32MsgIDMask = 0;
    RxRE38CanAngleMessageA.ui32Flags = MSG_OBJ_RX_INT_ENABLE;
    RxRE38CanAngleMessageA.ui32MsgLen = sizeof(re38_can_rx_data);
    RxRE38CanAngleMessageA.pucMsgData = re38_can_rx_data;


    *(unsigned long *)RXMsgBOOT=0;
    RXCANA_boot.ui32MsgID = 0x655;                      // COMAC Aircraft CMD A
    RXCANA_boot.ui32MsgIDMask = 0;                  // no mask needed for TX
    RXCANA_boot.ui32Flags = MSG_OBJ_RX_INT_ENABLE;  // enable interrupt on RX
    RXCANA_boot.ui32MsgLen = sizeof(RXMsgBOOT);   // size of message is 8
    RXCANA_boot.pucMsgData = RXMsgBOOT;           // ptr to message content

    TXCANA_boot.ui32MsgID = 0x5D5;
    TXCANA_boot.ui32MsgIDMask = 0;
    TXCANA_boot.ui32Flags = 0;
    TXCANA_boot.ui32MsgLen = sizeof(TXCANA);
    TXCANA_boot.pucMsgData = TXCANA;
    TXCANA[0]= 0x4F;
    TXCANA[1]= 0x51;
    TXCANA[2]= 0x1F;
    TXCANA[3]= 0x01;
    TXCANA[4]= 0x01;
    TXCANA[5]= 0x00;
    TXCANA[6]= 0x00;
    TXCANA[7]= 0x00;

    CANMessageSet(CANA_BASE, 4, &RxControlCmdMessageA, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, 8, &RxPidCmdMessageA, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, 9, &RxRE38CanAngleMessageA, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, 12, &RXCANA_boot, MSG_OBJ_TYPE_RX);
}


extern float NTC_SIC_M1A,NTC_SIC_M1B,NTC_SIC_M1C;
extern float M1_real_voltage;
extern float M1_speed_resolver;
extern float Temp_board;
int DC_bus_voltage =0;
uint16_t speed_resolver =0;
float est_torque = 0.0f;

void CANBMsgSend()
{
    uint16_t pump_speed = 1600;
        uint16_t CanTX_pump_speed = pump_speed + 8191;

        ucTXMsgDataB[0] = 0x01;
        ucTXMsgDataB[1] = (CanTX_pump_speed >> 8) & 0xff;
        ucTXMsgDataB[2] = CanTX_pump_speed & 0xff;
        ucTXMsgDataB[3] = 0x32;
        ucTXMsgDataB[4] = 0x00;
        ucTXMsgDataB[5] = 0x00;
        ucTXMsgDataB[6] = 0x00;
        ucTXMsgDataB[7] = 0x00;

        CANMessageSet(CANB_BASE, 1, &sTXCANMessageB, MSG_OBJ_TYPE_TX);
}

void CANAMsgSend()
{
    int output_power;

    DC_bus_voltage = (int)(M1_real_voltage / 4);

    if(M1_pid_CurrLp_iq.Ref <= 0.0f)
    {
        est_torque = 0;
    }
    else
    {
        float torque_constant = 1.40f;
        float Iq_normal = M1_pid_CurrLp_iq.Ref * (float)BASE_CURRENT;
        float RMS_phaseCurrent = Iq_normal * 0.7071f;
        est_torque = torque_constant * RMS_phaseCurrent;
    }
    output_power = (int)(est_torque * M1_speed_resolver / 9550.0f);
    speed_resolver = (uint16_t)M1_speed_resolver;

    ucTXMsgDataA[0] = (speed_resolver & 0xff);
    ucTXMsgDataA[1] = (speed_resolver >> 8) & 0xff;
    ucTXMsgDataA[2] = DC_bus_voltage;
    ucTXMsgDataA[3] = (unsigned char)(NTC_SIC_M1_AvgC + 40.0f);
    ucTXMsgDataA[4] = output_power;
    ucTXMsgDataA[5] = (unsigned char)((M1_pid_SpdLp.Out + 1.0f) * 100.0f);
    ucTXMsgDataA[6] = DTC_code1;
    ucTXMsgDataA[7] = DTC_code2;

    CANMessageSet(CANA_BASE, 1, &sTXCANMessageA, MSG_OBJ_TYPE_TX);
}

void TempSend()
{
    tempdata[0] = (unsigned char)(NTC_SIC_M1A + 40.0f);
    tempdata[1] = (unsigned char)(NTC_SIC_M1B + 40.0f);
    tempdata[2] = (unsigned char)(NTC_SIC_M1C + 40.0f);
    tempdata[3] = (unsigned char)(PT100_PTA1_InstantC + 40.0f);
    tempdata[4] = (unsigned char)(PT100_PTA2_InstantC + 40.0f);
    tempdata[5] = (unsigned char)(PT100_PTB1_InstantC + 40.0f);
    tempdata[6] = (unsigned char)(PT100_PTB2_InstantC + 40.0f);
    tempdata[7] = (int)Temp_board + 40;

    CANMessageSet(CANA_BASE, 2, &TxCANTempMessageA, MSG_OBJ_TYPE_TX);
}

void DataSend()
{
    int iq_ref = (int)((M1_pid_CurrLp_iq.Ref + 1.0f) * 7000.0f);
    int iq_fdb = (int)((M1_pid_CurrLp_iq.Fdb + 1.0f) * 7000.0f);
    int id_ref = (int)((M1_pid_CurrLp_id.Ref + 1.0f) * 7000.0f);
    int id_fdb = (int)((M1_pid_CurrLp_id.Fdb + 1.0f) * 7000.0f);

    operatedata[0] = iq_ref & 0xff;
    operatedata[1] = (iq_ref & 0xff00) >> 8;
    operatedata[2] = id_ref & 0xff;
    operatedata[3] = (id_ref & 0xff00) >> 8;
    operatedata[4] = iq_fdb & 0xff;
    operatedata[5] = (iq_fdb & 0xff00) >> 8;
    operatedata[6] = id_fdb & 0xff;
    operatedata[7] = (id_fdb & 0xff00) >> 8;

    CANMessageSet(CANA_BASE, 3, &TxCANDataMessageA, MSG_OBJ_TYPE_TX);
}


void PT100send()
{
    PT100data[0] = (unsigned char)(PT100_PTA1_AvgC + 40.0f);
    PT100data[1] = (unsigned char)(PT100_PTA2_AvgC + 40.0f);
    PT100data[2] = (unsigned char)(PT100_PTB1_AvgC + 40.0f);
    PT100data[3] = (unsigned char)(PT100_PTB2_AvgC + 40.0f);
    PT100data[4] = 0;   /* PTC 硬件已移除，保留字节为 0 维持 CAN 协议 */
    PT100data[5] = 0;
    PT100data[6] = 0;
    PT100data[7] = 0;

    CANMessageSet(CANA_BASE, 5, &TxPT100DataMessageA, MSG_OBJ_TYPE_TX);
}

extern float analog1;
extern float analog2;
extern float analog3;

void DTCSend()
{
    uint16_t analog = 0;
    analog=analog1 * 10000;

    DTCdata[0] = DTC_code1;
    DTCdata[1] = DTC_code2;
    DTCdata[2] = DTC_code3;
    DTCdata[3] = analog & 0xff;
    DTCdata[4] = (analog & 0xff00) >> 8;
    DTCdata[5] = 0;
    DTCdata[6] = 0;
    DTCdata[7] = 0;

    CANMessageSet(CANA_BASE, 10, &TxDTCDataMessageA, MSG_OBJ_TYPE_TX);
}

extern uint16_t M1_Algo_control_start;

void MotorCtrl310Send(void)
{
    Motor310_UpdateCurrentRms();

    motor310txdata[0] = (unsigned char)((uint16_t)DC_bus_voltage & 0xFF);
    motor310txdata[1] = (unsigned char)(((uint16_t)DC_bus_voltage >> 8) & 0xFF);

    uint16_t CanTX_est_Torque = (uint16_t)est_torque;
    motor310txdata[2] = (unsigned char)(CanTX_est_Torque & 0xFF);
    motor310txdata[3] = (unsigned char)((CanTX_est_Torque >> 8) & 0xFF);

    uint16_t speed_tx = (uint16_t)(speed_resolver + MOTOR310_RPM_OFFSET);

    motor310txdata[4] = (unsigned char)(speed_tx & 0xFF);
    motor310txdata[5] = (unsigned char)((speed_tx >> 8) & 0xFF);

    motor310_status &= (unsigned char)(~(MOTOR310_STA_STANDBY
                                  | MOTOR310_STA_RUNNING
                                  | MOTOR310_STA_FAULT));

    if(M1_Algo_control_start == 0u)//电机待机（待更新待机的具体状态）
    {
        motor310_status |= MOTOR310_STA_STANDBY;
    }

    if(M1_Algo_control_start == 1u)//电机运行（待更新运行的具体状态）
    {
        motor310_status |= MOTOR310_STA_RUNNING;
    }

    if((DTC_code1 != 0u) || (DTC_code2 != 0u) || (DTC_code3 != 0u))//电机故障以DTCcode
    {
        motor310_status |= MOTOR310_STA_FAULT;
    }

    motor310txdata[6] = motor310_status;
    motor310txdata[7] = 0;//(unsigned char)((float)motor310_current_rms * 0.5f);

    CANMessageSet(CANA_BASE, 6, &TxMotor310MessageA, MSG_OBJ_TYPE_TX);
}

extern float PT100_HottestAvgC;
extern float NTC_SIC_M1_AvgC;

void MotorCtrl311Send(void)
{
    motor311txdata[0] = (unsigned char)((int)NTC_SIC_M1_AvgC + MOTOR311_TEMP_OFFSET);
    motor311txdata[1] = (unsigned char)((int)PT100_HottestAvgC + MOTOR311_TEMP_OFFSET);
//    motor311txdata[2] = motor311_reserved_b2;
//    motor311txdata[3] = (unsigned char)(motor311_torque_limit & 0xFFu);
//    motor311txdata[4] = (unsigned char)((motor311_torque_limit >> 8) & 0xFFu);
    motor311txdata[2] = 0;
    motor311txdata[3] = 0;
    motor311txdata[4] = 0;

    motor311_ctrl_fault &= (unsigned char)(~(MOTOR311_FLT_B5_IGBT_OVERTEMP
                                      | MOTOR311_FLT_B5_IGBT_TEMP_ALM
                                      | MOTOR311_FLT_B5_DC_OVERVOLT
                                      | MOTOR311_FLT_B5_DC_UNDERVOLT
                                      | MOTOR311_FLT_B5_OUTPUT_OC));

    if(fault_SiC_overtemp != 0) //NTC过温
    {
        motor311_ctrl_fault |= MOTOR311_FLT_B5_IGBT_OVERTEMP;
    }

    if((NTC_SIC_M1_AvgC > (SIC_OVERTEMP_TRIP_C - 5)) && (NTC_SIC_M1_AvgC < SIC_OVERTEMP_TRIP_C))//NTC温度临界值报警90-95
    {
        motor311_ctrl_fault |= MOTOR311_FLT_B5_IGBT_TEMP_ALM;
    }

    if(fault_overvoltage != 0) //直流过压
    {
        motor311_ctrl_fault |= MOTOR311_FLT_B5_DC_OVERVOLT;
    }

    if(fault_undervoltage != 0) //直流欠压警告
    {
        motor311_ctrl_fault |= MOTOR311_FLT_B5_DC_UNDERVOLT;
    }

    if((fault_overcurrent == 1) || (DTC_code2 & 0x08u == 0x08u))//输出过流故障（包含软件过流+硬件总tripzone）
    {
        motor311_ctrl_fault |= MOTOR311_FLT_B5_OUTPUT_OC;
    }


    motor311txdata[5] = motor311_ctrl_fault;


    motor311_motor_fault &= (unsigned char)(~(MOTOR311_FLT_B6_OVERSPEED
                                       | MOTOR311_FLT_B6_MOT_TEMP_ALM
                                       | MOTOR311_FLT_B6_MOT_OVERTEMP));

    if(fault_overspeed == 1)//电机超速
    {
        motor311_motor_fault |= MOTOR311_FLT_B6_OVERSPEED;
    }

    /* 真实 °C：临界区 CLEAR~TRIP（140~150），对齐 SiC 的 (TRIP-5)~TRIP 写法 */
    if ((PT100_HottestAvgC > MOTOR_OVERTEMP_CLEAR_C) && (PT100_HottestAvgC < MOTOR_OVERTEMP_TRIP_C))
    {
        motor311_motor_fault |= MOTOR311_FLT_B6_MOT_TEMP_ALM;
    }

    if(fault_motor_overtemp != 0)//电机过温
    {
        motor311_motor_fault |= MOTOR311_FLT_B6_MOT_OVERTEMP;
    }

    motor311txdata[6] = motor311_motor_fault;

    motor311txdata[7] = 0;
//    motor311txdata[7] = motor311_life_signal;

    CANMessageSet(CANA_BASE, 7, &TxMotor311MessageA, MSG_OBJ_TYPE_TX);
}


uint16_t CAN_speed_set = 0;
uint16_t CAN_control_command = 0;
uint16_t CAN_Enable = 0;
uint16_t CAN_PWM_Enable = 0;
uint16_t CAN_torque_pwm_reset = 0;
float CAN_pid_SpdLp_kp = 0.8f;
float CAN_pid_SpdLp_ki = 0.0016f;
float CAN_pid_CurrLp_id_kp = 0.62f;
float CAN_pid_CurrLp_id_ki = 0.004f;
float CAN_pid_CurrLp_iq_kp = 0.62f;
float CAN_pid_CurrLp_iq_ki = 0.004f;
int CAN_PID_adjust = 0;
int CAN_iqref = 0;
int CAN_idref = 0;//dqset
uint16_t CAN_RampStep = 0;

void CANAControlCmdGet(void)
{
    if (throttle_type != HOST_CAN_THROTTLE)
    {
        return;
    }

    if(CANA_ctrl_msg_received == 0u)
    {
        return;
    }

    CANA_ctrl_msg_received = 0u;

    CAN_speed_set = (uint16_t)(RxControlCmdDataA[0] | (RxControlCmdDataA[1] << 8));

    CAN_iqref = RxControlCmdDataA[2];

    CAN_control_command = RxControlCmdDataA[3];

    CAN_PWM_Enable = (RxControlCmdDataA[4] >> 0) & 0x01;

    CAN_RampStep = RxControlCmdDataA[6];

    CAN_Enable = (RxControlCmdDataA[7] >> 0) & 0x01;

    CAN_torque_pwm_reset = (RxControlCmdDataA[7] >> 1) & 0x01;

    CAN_PID_adjust = (RxControlCmdDataA[7] >> 2) & 0x01;

    if(CAN_iqref > 100)
    {
        CAN_iqref = 100;
    }

    if(CAN_idref > 100)
    {
        CAN_idref = 100;
    }
    else if (CAN_idref < -100)
    {
        CAN_idref = -100;
    }
}

void RE38_CanThrottle_UpdateFromRx(void)
{
    uint16_t rpm_raw;
    float rpm;
    uint32_t cana_status;

    if (throttle_type != RE38_CAN_THROTTLE)
    {
        return;
    }

    /* CAN 总线质量预警：跟随硬件 EWARN（约 10ms/次），与帧超时独立 */
    cana_status = CANStatusGet(CANA_BASE, CAN_STS_CONTROL);
    if (cana_status & CAN_STATUS_EWARN)
    {
        fault_re38_can_bus_quality_warn = 1;
    }
    else
    {
        fault_re38_can_bus_quality_warn = 0;
    }

    if (re38_can_rx_msg_received != 0u)
    {
        re38_can_rx_msg_received = 0u;

        /* Byte7 高字节、Byte8 低字节，1:1 转速后再 × RE38_CAN_RPM_SCALE */
        rpm_raw = (uint16_t)((re38_can_rx_data[6] << 8) | re38_can_rx_data[7]);
        rpm = (float)rpm_raw * RE38_CAN_RPM_SCALE;
        re38_can_target_rpm = RE38_ClampTargetRpm(rpm);

        re38_can_rx_timeout_cnt = 0u;
        re38_can_rx_ever_ok = 1u;
        fault_re38_can_timeout_warn = 0;
        fault_re38_can_timeout_lost = 0;
        return;
    }

    /* 无新帧：首帧成功前不计时；之后保持上次转速，双级超时置 flag，不停机 */
    if (re38_can_rx_ever_ok == 0u)
    {
        return;
    }

    if (re38_can_rx_timeout_cnt < 0xFFFF)
    {
        re38_can_rx_timeout_cnt++;
    }

    if (re38_can_rx_timeout_cnt > FAULT_RE38_CAN_TIMEOUT_LOST_TICKS)
    {
        fault_re38_can_timeout_lost = 1;
        fault_re38_can_timeout_warn = 0;
    }
    else if (re38_can_rx_timeout_cnt > FAULT_RE38_CAN_TIMEOUT_WARN_TICKS)
    {
        fault_re38_can_timeout_warn = 1;
        fault_re38_can_timeout_lost = 0;
    }
    else
    {
        fault_re38_can_timeout_warn = 0;
        fault_re38_can_timeout_lost = 0;
    }
}

void CANAPidMsgGet(void)
{
    if(CANA_pid_msg_received == 0u)
    {
        return;
    }

    CANA_pid_msg_received = 0u;//接收中断函数处置1

    CAN_pid_SpdLp_kp = RxPidCmdDataA[0] * 0.01f;
    CAN_pid_SpdLp_ki = RxPidCmdDataA[1] * 0.0001f;
    CAN_pid_CurrLp_id_kp = RxPidCmdDataA[2] * 0.01f;
    CAN_pid_CurrLp_id_ki = (RxPidCmdDataA[3] | (RxPidCmdDataA[4] << 8)) * 0.00001f;
    CAN_pid_CurrLp_iq_kp = RxPidCmdDataA[5] * 0.01f;
    CAN_pid_CurrLp_iq_ki = (RxPidCmdDataA[6] | (RxPidCmdDataA[7] << 8)) * 0.00001f;
}

int CAN_BOOTLOADER_STOPMOTOR_FLAG = 0;
void CANABootMsgGet()
{
    if(RXMsgBOOT[0] == 0x40 && RXMsgBOOT[1] == 0x51 && RXMsgBOOT[2] == 0x1F
       && RXMsgBOOT[3] == 0x01)
    {
        Message_clear();
        TXCANA[0] = 0x4F;
        TXCANA[1] = 0x51;
        TXCANA[2] = 0x1F;
        TXCANA[3] = 0x01;
        TXCANA[4] = 0x01;
        TXCANA[5] = 0x00;
        TXCANA[6] = 0x00;
        TXCANA[7] = 0x00;
        CANMessageSet(CANA_BASE, 11, &TXCANA_boot,MSG_OBJ_TYPE_TX);
    }

    if(RXMsgBOOT[0] == 0x2F && RXMsgBOOT[1] == 0x51 && RXMsgBOOT[2] == 0x1F
       && RXMsgBOOT[3] == 0x01 && RXMsgBOOT[4] == 0x00)
    {
        Message_clear();
        TXCANA[0] = 0x60;
        TXCANA[1] = 0x51;
        TXCANA[2] = 0x1F;
        TXCANA[3] = 0x01;
        TXCANA[4] = 0x00;
        TXCANA[5] = 0x00;
        TXCANA[6] = 0x00;
        TXCANA[7] = 0x00;
        CANMessageSet(CANA_BASE, 11, &TXCANA_boot,MSG_OBJ_TYPE_TX);
        TLF35584_DisableERRandWWD();
        DELAY_US(1000);
        CAN_BOOTLOADER_STOPMOTOR_FLAG = 1;

        /* 跳转 Boot(0x080000) 前关闭 APP 中断与外设，与 Boot JumpToApp() 对称 */

        /* CPU 级：禁止可屏蔽中断 */
        DINT;           /* 全局关中断(INTM=1) */
        IER = 0;        /* 关闭所有 CPU 中断通道使能 */
        IFR = 0;        /* 清除已挂起的中断标志 */


        CpuTimer0Regs.TCR.bit.TSS = 1;  /* 停止计数 */
        CpuTimer0Regs.TCR.bit.TIE = 0;  /* 关闭定时器中断使能 */
        CpuTimer0Regs.TCR.bit.TIF = 1;  /* 清除溢出标志 */

        CpuTimer1Regs.TCR.bit.TSS = 1;  /* 停止计数 */
        CpuTimer1Regs.TCR.bit.TIE = 0;  /* 关闭定时器中断使能 */
        CpuTimer1Regs.TCR.bit.TIF = 1;  /* 清除溢出标志 */

        /* PIE / 外设级：关闭 APP 已使能的中断源 */
        CANGlobalIntDisable(CANA_BASE, CAN_GLB_INT_CANINT0);  /* 关闭 CANA 全局中断 */
        PieCtrlRegs.PIEIER9.bit.INTx5 = 0;  /* CANA0_INT */
        PieCtrlRegs.PIEIER9.bit.INTx7 = 0;  /* CANB0_INT */
        PieCtrlRegs.PIEIER3.bit.INTx1 = 0;  /* EPWM1_INT，电机控制 ISR */
        PieCtrlRegs.PIEIER2.bit.INTx1 = 0;  /* EPWM1_TZ_INT */
        PieCtrlRegs.PIEIER2.bit.INTx2 = 0;  /* EPWM2_TZ_INT */
        PieCtrlRegs.PIEIER2.bit.INTx3 = 0;  /* EPWM3_TZ_INT */
        PieCtrlRegs.PIEIER1.bit.INTx7 = 0;  /* TIMER0_INT，和看门狗相关*/
        __asm("    LB 0x080000");  /* 长跳转至 Boot codestart，勿用函数指针调用 */
        //CAN_BOOT_JumpToApplication(0x080000);
    }

}
void Message_clear(void)
{
    uint8_t Message_num=0;
    for(Message_num=0; Message_num<8; Message_num++)
    {
        RXMsgBOOT[Message_num]=0x00;
    }
}

