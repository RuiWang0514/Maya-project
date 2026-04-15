#include "HPMC_Settings.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "driverlib/can.h"


//volatile unsigned long g_ulTxMsgCount = 0; // A counter that keeps track of the
//                                           // number of times the TX interrupt
//                                           // has occurred, which should match
//                                           // the number of TX messages that
//                                           // were sent.
//volatile unsigned long g_ulRxMsgCount = 0;
//unsigned long u32CanAErrorStatus;
//volatile unsigned long g_bErrFlag = 0;     // A flag to indicate that some
//                                           // transmission error occurred.

volatile unsigned long g_ulTxMsgCountA = 0; // A counter that keeps track of the
                                           // number of times the TX interrupt
                                           // has occurred, which should match
                                           // the number of TX messages that
                                           // were sent.
volatile unsigned long g_ulRxMsgCountA = 0;
unsigned long u32CanAErrorStatusA;
volatile unsigned long g_bErrFlagA = 0;     // A flag to indicate that some
                                           // transmission error occurred.

volatile unsigned long g_ulTxMsgCountB = 0; // A counter that keeps track of the
                                           // number of times the TX interrupt
                                           // has occurred, which should match
                                           // the number of TX messages that
                                           // were sent.
volatile unsigned long g_ulRxMsgCountB = 0;
unsigned long u32CanAErrorStatusB;
volatile unsigned long g_bErrFlagB = 0;     // A flag to indicate that some
                                           // transmission error occurred.

//tCANMsgObject sTXCANMessage;
//tCANMsgObject sRXCANMessage;
//unsigned char ucTXMsgData[4] ={ 0x00, 0x00, 0x00, 0x00 };
//unsigned char ucRXMsgData[4];
//unsigned long u32CntTXMsgData = 0x12345678;

tCANMsgObject sTXCANMessageA;
tCANMsgObject TxCANTempMessageA;
tCANMsgObject TxCANDataMessageA;
tCANMsgObject TxPT100DataMessageA;
tCANMsgObject TxDTCDataMessageA;
tCANMsgObject sRXCANMessageA;
//unsigned char ucTXMsgDataA[4] ={ 0x00, 0x00, 0x00, 0x00 };
unsigned char ucTXMsgDataA[8] ={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char tempdata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char operatedata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char PT100data[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char DTCdata[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char ucRXMsgDataA[8];
unsigned long u32CntTXMsgDataA = 0x1234567887654321;

tCANMsgObject sTXCANMessageB;
tCANMsgObject sRXCANMessageB;
unsigned char ucTXMsgDataB[8];
unsigned char ucRXMsgDataB[8];
unsigned long long u32CntTXMsgDataB = 0x1234567887654321;


//char txbuf[]="$test,1,2,3\r\n";
//char rxbuf[256];
//char rxbuf_cnt=0;
//extern unsigned char cmd_ready;

char txbufA[]="$test,1,2,3\r\n";
char rxbufA[256];
char rxbuf_cntA=0;
extern unsigned char cmd_readyA;

char txbufB[]="$test,1,2,3\r\n";
char rxbufB[256];
char rxbuf_cntB=0;
extern unsigned char cmd_readyB;


uint32_t CANAticker = 0;
uint32_t CANBticker = 0;

uint16_t CAN_speed = 0;
//unsigned char rcvcmd_sta=0;
unsigned char rcvcmd_staA=0;
unsigned char rcvcmd_staB=0;
extern uint16_t DTC_code1;
extern uint16_t DTC_code3;


extern int CAN_M1_Iq_OUT;
extern int CAN_M1_Id_OUT;
extern int CAN_M2_Iq_OUT;
extern int CAN_M2_Id_OUT;
extern int CAN_M1_Iq_ref;
extern int CAN_M1_Id_ref;
extern int CAN_M1_Iq_fdb;
extern int CAN_M1_Id_fdb;
extern int CAN_M1_Speed_OUT;
extern int CAN_M1_Ubeta;
extern int CAN_M1_Ualfa;
extern int CAN_M2_Ubeta;
extern int CAN_M2_Ualfa;



void ReadCANB(tCANMsgObject *pMsgObject)
{
    unsigned char len=0;
    for(len=0;len<pMsgObject->ui32MsgLen;len++)
    {
        switch(rcvcmd_staB)
        {
        case 0:
            if (*(pMsgObject->pucMsgData+len) == '$')
            {
                rcvcmd_staB=1;
                rxbuf_cntB=0;
            }
            break;
        case 1:
            if (*(pMsgObject->pucMsgData+len) == '$')
            {
                rcvcmd_staB=1;
                rxbuf_cntB=0;
            }
            if (*(pMsgObject->pucMsgData+len) == 0x0d)
            {
                rcvcmd_staB=2;
            }
            break;
        case 2:
            if (*(pMsgObject->pucMsgData+len) == 0x0a)
            {
                rcvcmd_staB=0;
                cmd_readyB=1;
                rxbufB[rxbuf_cntB]=*(pMsgObject->pucMsgData+len);
                rxbuf_cntB++;
                rxbufB[rxbuf_cntB]=0;
                if (rxbuf_cntB > 255)
                    rxbuf_cntB = 0;
                return;
            }
            break;
        default:
            rcvcmd_staB=0;
            break;
        }
        rxbufB[rxbuf_cntB]=*(pMsgObject->pucMsgData+len);
        rxbuf_cntB++;
        if (rxbuf_cntB > 255)
            rxbuf_cntB = 0;
    }
}

void CANBMsgRead(tCANMsgObject *pMsgObject)
{
    unsigned char len=0;
    for(len=0;len<pMsgObject->ui32MsgLen;len++)
    {
        ucRXMsgDataB[len] = *(pMsgObject->pucMsgData+len);

    }

    CAN_speed = (ucRXMsgDataB[0]&0x00ff)|((ucRXMsgDataB[1]&0x000f)<<8);
}



void ReadCANA(tCANMsgObject *pMsgObject)
{
    unsigned char len=0;
    for(len=0;len<pMsgObject->ui32MsgLen;len++)
    {
        switch(rcvcmd_staA)
        {
        case 0:
            if (*(pMsgObject->pucMsgData+len) == '$')
            {
                rcvcmd_staA=1;
                rxbuf_cntA=0;
            }
            break;
        case 1:
            if (*(pMsgObject->pucMsgData+len) == '$')
            {
                rcvcmd_staA=1;
                rxbuf_cntA=0;
            }
            if (*(pMsgObject->pucMsgData+len) == 0x0d)
            {
                rcvcmd_staA=2;
            }
            break;
        case 2:
            if (*(pMsgObject->pucMsgData+len) == 0x0a)
            {
                rcvcmd_staA=0;
                cmd_readyA=1;
                rxbufA[rxbuf_cntA]=*(pMsgObject->pucMsgData+len);
                rxbuf_cntA++;
                rxbufA[rxbuf_cntA]=0;
                if (rxbuf_cntA > 255)
                    rxbuf_cntA = 0;
                return;
            }
            break;
        default:
            rcvcmd_staA=0;
            break;
        }
        rxbufA[rxbuf_cntA]=*(pMsgObject->pucMsgData+len);
        rxbuf_cntA++;
        if (rxbuf_cntA > 255)
            rxbuf_cntA = 0;
    }
}
//
// CANIntHandler - This function is the interrupt handler for the CAN
//                 peripheral. It checks for the cause of the interrupt, and
//                 maintains a count of all messages that have been
//                 transmitted.
//
interrupt void CANBIntHandler(void)
{
    CANBticker++;
    unsigned long ulStatus;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ulStatus = CANIntStatus(CANB_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ulStatus == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ulStatus = CANStatusGet(CANB_BASE, CAN_STS_CONTROL);

        //
        //Check to see if an error occurred.
        //
        if(((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7) &&
           ((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            g_bErrFlagB = 1;
//            DTC_code1 |=0x80;
        }
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ulStatus == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANB_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulTxMsgCountB++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagB = 0;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // receiving messages.
    //
    else if(ulStatus == 2)
    {
        //
        // Get the received message
        //
        CANMessageGet(CANB_BASE, 2, &sRXCANMessageB, true);
        CANBMsgRead(&sRXCANMessageB);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANB_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountB++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagB = 0;
    }
    else if(ulStatus == 3)
    {
        //
        // Get the received message
        //
        CANMessageGet(CANB_BASE, 3, &sRXCANMessageB, true);
        CANBMsgRead(&sRXCANMessageB);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANB_BASE, 3);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountB++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagB = 0;
    }
    else if(ulStatus == 4)
    {
        //
        // Get the received message
        //
        CANMessageGet(CANB_BASE, 4, &sRXCANMessageB, true);
        CANBMsgRead(&sRXCANMessageB);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANB_BASE, 4);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountB++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagB = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    CANGlobalIntClear(CANB_BASE, CAN_GLB_INT_CANINT0);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
//    CANBMsgSend();

}


interrupt void CANAIntHandler(void)
{
    CANAticker++;
    unsigned long ulStatus;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ulStatus = CANIntStatus(CANA_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ulStatus == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ulStatus = CANStatusGet(CANA_BASE, CAN_STS_CONTROL);

        //
        //Check to see if an error occurred.
        //
        if(((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7) &&
           ((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            g_bErrFlagA = 1;
        }
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ulStatus == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulTxMsgCountA++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagA = 0;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // receiving messages.
    //
    else if(ulStatus == 2)
    {
        //
        // Get the received message
        //
//        CANMessageGet(CANA_BASE, 2, &sRXCANMessageA, true);
//       ReadCANA(&sRXCANMessageA);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountA++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 3)
    {
        //
        // Get the received message
        //
//        CANMessageGet(CANA_BASE, 3, &sRXCANMessageA, true);
//       ReadCANA(&sRXCANMessageA);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 3);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountA++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 4)
    {
        //
        // Get the received message
        //
        CANMessageGet(CANA_BASE, 4, &sRXCANMessageA, true);
//       ReadCANA(&sRXCANMessageA);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 4);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountA++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagA = 0;
    }

    else if(ulStatus == 5)
    {
        //
        // Get the received message
        //
        //CANMessageGet(CANA_BASE, 5, &sRXCANMessageA, true);
//       ReadCANA(&sRXCANMessageA);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 5);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountA++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagA = 0;
    }
    else if(ulStatus == 10)
    {
        //
        // Get the received message
        //
//        CANMessageGet(CANA_BASE, 2, &sRXCANMessageA, true);
//       ReadCANA(&sRXCANMessageA);
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 10);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCountA++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlagA = 0;
    }
    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    CANGlobalIntClear(CANA_BASE, CAN_GLB_INT_CANINT0);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


int transdelay=150;
void Send2CANB(char *buf)
{
    int len=0;
    unsigned char send_cnt=0,remain_cnt=0;
//    unsigned char last_buf[4]={0x00,0x00,0x00,0x00};
    while(*(buf+len) != 0x00) len++;    // test the length of input
    sTXCANMessageB.ui32MsgLen=4;
    for(send_cnt=0;send_cnt<(len/4);send_cnt++)
    {
         ucTXMsgDataB[0] = *(buf+send_cnt*4);
         ucTXMsgDataB[1] = *(buf+send_cnt*4 + 1);
         ucTXMsgDataB[2] = *(buf+send_cnt*4 + 2);
         ucTXMsgDataB[3] = *(buf+send_cnt*4 + 3);
         CANMessageSet(CANB_BASE, 1, &sTXCANMessageB, MSG_OBJ_TYPE_TX);
         DELAY_US(transdelay);
    }
    remain_cnt=len%4;
    sTXCANMessageB.ui32MsgLen=remain_cnt;
    switch(remain_cnt)
    {
    case 0:
        break;
    case 1:
        ucTXMsgDataB[0] = *(buf+send_cnt*4 );
        ucTXMsgDataB[1] = 0;
        ucTXMsgDataB[2] = 0;
        ucTXMsgDataB[3] = 0;
        CANMessageSet(CANB_BASE, 1, &sTXCANMessageB, MSG_OBJ_TYPE_TX);
        break;
    case 2:
        ucTXMsgDataB[0] = *(buf+send_cnt*4 );
        ucTXMsgDataB[1] = *(buf+send_cnt*4 + 1);
        ucTXMsgDataB[2] = 0;
        ucTXMsgDataB[3] = 0;
        CANMessageSet(CANB_BASE, 1, &sTXCANMessageB, MSG_OBJ_TYPE_TX);
        break;
    case 3:
        ucTXMsgDataB[0] = *(buf+send_cnt*4 );
        ucTXMsgDataB[1] = *(buf+send_cnt*4 + 1);
        ucTXMsgDataB[2] = *(buf+send_cnt*4 + 2);
        ucTXMsgDataB[3] = 0;
        CANMessageSet(CANB_BASE, 1, &sTXCANMessageB, MSG_OBJ_TYPE_TX);
        break;
    default:
        break;
    }
    DELAY_US(transdelay);

}

void Send2CANA(char *buf)
{
    int len=0;
    unsigned char send_cnt=0,remain_cnt=0;
//    unsigned char last_buf[4]={0x00,0x00,0x00,0x00};
    while(*(buf+len) != 0x00) len++;    // test the length of input
    sTXCANMessageA.ui32MsgLen=4;
    for(send_cnt=0;send_cnt<(len/4);send_cnt++)
    {
         ucTXMsgDataA[0] = *(buf+send_cnt*4);
         ucTXMsgDataA[1] = *(buf+send_cnt*4 + 1);
         ucTXMsgDataA[2] = *(buf+send_cnt*4 + 2);
         ucTXMsgDataA[3] = *(buf+send_cnt*4 + 3);
         CANMessageSet(CANA_BASE, 1, &sTXCANMessageA, MSG_OBJ_TYPE_TX);
         DELAY_US(transdelay);
    }
    remain_cnt=len%4;
    sTXCANMessageA.ui32MsgLen=remain_cnt;
    switch(remain_cnt)
    {
    case 0:
        break;
    case 1:
        ucTXMsgDataA[0] = *(buf+send_cnt*4 );
        ucTXMsgDataA[1] = 0;
        ucTXMsgDataA[2] = 0;
        ucTXMsgDataA[3] = 0;
        CANMessageSet(CANA_BASE, 1, &sTXCANMessageA, MSG_OBJ_TYPE_TX);
        break;
    case 2:
        ucTXMsgDataA[0] = *(buf+send_cnt*4 );
        ucTXMsgDataA[1] = *(buf+send_cnt*4 + 1);
        ucTXMsgDataA[2] = 0;
        ucTXMsgDataA[3] = 0;
        CANMessageSet(CANA_BASE, 1, &sTXCANMessageA, MSG_OBJ_TYPE_TX);
        break;
    case 3:
        ucTXMsgDataA[0] = *(buf+send_cnt*4 );
        ucTXMsgDataA[1] = *(buf+send_cnt*4 + 1);
        ucTXMsgDataA[2] = *(buf+send_cnt*4 + 2);
        ucTXMsgDataA[3] = 0;
        CANMessageSet(CANA_BASE, 1, &sTXCANMessageA, MSG_OBJ_TYPE_TX);
        break;
    default:
        break;
    }
    DELAY_US(transdelay);

}

void CANA_Initialzation()
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

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    CANIntEnable(CANA_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
}

void CANB_Initialzation()
{

    //CANB
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 6); // GPIO7 - CANRXB
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 6); // GPIO6 - CANTXB
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);

//    GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 2); // GPIO30 - CANRXA
//    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 2); // GPIO31 - CANTXB
//    GPIO_SetupPinOptions(17, GPIO_INPUT, GPIO_ASYNC);
//    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);

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
//    ucTXMsgDataB[0] = (u32CntTXMsgDataB>>56) & 0xFF;
//    ucTXMsgDataB[1] = (u32CntTXMsgDataB>>48) & 0xFF;
//    ucTXMsgDataB[2] = (u32CntTXMsgDataB>>40) & 0xFF;
//    ucTXMsgDataB[3] = (u32CntTXMsgDataB>>32) & 0xFF;
//    ucTXMsgDataB[4] = (u32CntTXMsgDataB>>24) & 0xFF;
//    ucTXMsgDataB[5] = (u32CntTXMsgDataB>>16) & 0xFF;
//    ucTXMsgDataB[6] = (u32CntTXMsgDataB>>8) & 0xFF;
//    ucTXMsgDataB[7] = (u32CntTXMsgDataB) & 0xFF;
//    sTXCANMessageB.ui32MsgID = 0x200110;                      // CAN message ID - use 1
    ucTXMsgDataB[0] = 0x00;
    ucTXMsgDataB[1] = 0x00;
    ucTXMsgDataB[2] = 0x00;
    ucTXMsgDataB[3] = 0x00;
    ucTXMsgDataB[4] = 0x00; //16-bit for 1000rpm
    ucTXMsgDataB[5] = 0x00;
    ucTXMsgDataB[6] = 0x00;//start on
    ucTXMsgDataB[7] = 0x00;

    sTXCANMessageB.ui32MsgID = 0x101000A0;                      // V2.0 oil pump TX ID
    sTXCANMessageB.ui32MsgIDMask = 0;                  // no mask needed for TX
    sTXCANMessageB.ui32Flags = MSG_OBJ_TX_INT_ENABLE;  // enable interrupt on TX
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

    //
    // Send the CAN message using object number 1 (not the same thing as
    // CAN ID, which is also 1 in this exaxmple).  This function will cause
    // the message to be transmitted right away.
    //
//    ucTXMsgDataB[0] = (u32CntTXMsgDataB>>24) & 0xFF;
//    ucTXMsgDataB[1] = (u32CntTXMsgDataB>>16) & 0xFF;
//    ucTXMsgDataB[2] = (u32CntTXMsgDataB>>8) & 0xFF;
//    ucTXMsgDataB[3] = (u32CntTXMsgDataB) & 0xFF;

    //
    // Enter loop to send messages.  A new message will be sent once per
    // second.  The 4 bytes of message content will be treated as an unsigned
    // long and incremented by one each time.
    //

//    Send2CANB("CANB Init OK...");
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
//    ucTXMsgDataA[0] = (u32CntTXMsgDataA>>24) & 0xFF;
//    ucTXMsgDataA[1] = (u32CntTXMsgDataA>>16) & 0xFF;
//    ucTXMsgDataA[2] = (u32CntTXMsgDataA>>8) & 0xFF;
//    ucTXMsgDataA[3] = (u32CntTXMsgDataA) & 0xFF;

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




    //
    // Initialize the message object that will be used for receiving CAN
    // messages.
    //



    *(unsigned long *)ucRXMsgDataA = 0;
    sRXCANMessageA.ui32MsgID = 0x10A001;                      // COMAC Aircraft CMD A
    sRXCANMessageA.ui32MsgIDMask = 0;                  // no mask needed for TX
    sRXCANMessageA.ui32Flags = MSG_OBJ_RX_INT_ENABLE;  // enable interrupt on RX
    sRXCANMessageA.ui32MsgLen = sizeof(ucRXMsgDataA);   // size of message is 8
    sRXCANMessageA.pucMsgData = ucRXMsgDataA;           // ptr to message content



    //
    // Setup the message object being used to receive messages
    //
//    CANMessageSet(CANB_BASE, 2, &sRXCANMessage, MSG_OBJ_TYPE_RX);
//    CANMessageSet(CANB_BASE, 3, &sRXCANMessage, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, 4, &sRXCANMessageA, MSG_OBJ_TYPE_RX);

    //
    // Send the CAN message using object number 1 (not the same thing as
    // CAN ID, which is also 1 in this exaxmple).  This function will cause
    // the message to be transmitted right away.
    //
//    ucTXMsgDataA[0] = (u32CntTXMsgDataA>>24) & 0xFF;
//    ucTXMsgDataA[1] = (u32CntTXMsgDataA>>16) & 0xFF;
//    ucTXMsgDataA[2] = (u32CntTXMsgDataA>>8) & 0xFF;
//    ucTXMsgDataA[3] = (u32CntTXMsgDataA) & 0xFF;

    //
    // Enter loop to send messages.  A new message will be sent once per
    // second.  The 4 bytes of message content will be treated as an unsigned
    // long and incremented by one each time.
    //

    //Send2CANA("CANA Init OK...");
}



extern uint16_t DTC_code2;
extern float NTC_SIC_M1A,NTC_SIC_M1B,NTC_SIC_M1C,NTC_SIC_M2A,NTC_SIC_M2B,NTC_SIC_M2C;
extern float NTC_SIC_M1A1,NTC_SIC_M1B1,NTC_SIC_M1C1,NTC_SIC_M2A1,NTC_SIC_M2B1,NTC_SIC_M2C1;
extern float temp1_value,temp2_value;
extern float M1_real_voltage,M2_real_voltage;
extern int resolver_dirB;
extern float M1_EstimatedSpeed;
extern float M2_EstimatedSpeed;
extern float M1_speed_resolver;
extern float M1_SpeedRef;
extern uint16_t M1_lsw;
extern float Temp_board;
extern float M1_iqref_can;
extern float Mea_PTA1,Mea_PTA2,Mea_PTB1,Mea_PTB2;
float NTC_SIC[];
int motor_temp;
int DC_bus_voltage =0;
int rotation_dir = 0;
int NTC_MAX_TEMP =0;
uint16_t speed_resolver =0;
uint16_t speed_target=0;
uint16_t motor_sta = 0;
float est_torque = 0.0;
extern float speed_out_iq;
int output_power = 0;

void CANBMsgSend()
{
    uint16_t pump_rpm = 1400;//油泵转速

    uint16_t CanTX_pump_data = pump_rpm * 4;//四倍分辨率

    ucTXMsgDataB[0] = 0x00;
    ucTXMsgDataB[1] = 0x00;
    ucTXMsgDataB[2] = 0x00;
    ucTXMsgDataB[3] = (CanTX_pump_data) & 0xff;
    ucTXMsgDataB[4] = (CanTX_pump_data >> 8) & 0xff;
    ucTXMsgDataB[5] = 0x00;
    ucTXMsgDataB[6] = 0x01;
    ucTXMsgDataB[7] = 0x00;
    CANMessageSet(CANB_BASE, 1, &sTXCANMessageB, MSG_OBJ_TYPE_TX);
}

extern float NTC_SIC_avg;
extern int TripZone_flag;
extern float NTC_SIC_avg_filter;
extern float Mea_PTB1_avg,Mea_PTB1_filter,Mea_PTB2_avg,Mea_PTB2_filter;
int over_speed_flag = 0;
extern float NTC_SIC_avg_filter1;

void CANAMsgSend(){
        int NTC_i;
        float NTC_temp=-40.0;

        motor_temp = (int)((temp1_value>temp2_value?temp1_value:temp2_value)+40);

        DC_bus_voltage = (int)(M1_real_voltage/4);
        rotation_dir = resolver_dirB;
        NTC_SIC[0]=NTC_SIC_M1A;
        NTC_SIC[1]=NTC_SIC_M1B;
        NTC_SIC[2]=NTC_SIC_M1C;
        NTC_SIC[3]=NTC_SIC_M2A;
        NTC_SIC[4]=NTC_SIC_M2B;
        NTC_SIC[5]=NTC_SIC_M2C;
        for(NTC_i=0;NTC_i<6;NTC_i++)
        {
            if(NTC_SIC[NTC_i]>NTC_temp)
            {
                NTC_temp = NTC_SIC[NTC_i];
            }
        }

        NTC_MAX_TEMP = (int)(NTC_temp+40);

        if(M1_speed_resolver>3100)
        {
            DTC_code1 |=0x10;
            over_speed_flag = 1;
        }


        if(M1_lsw==0)
        {
            motor_sta =0;
        }
        else
        {
            motor_sta = 2;
        }


        if(speed_out_iq<0.2)
        {
            est_torque = 545.0*speed_out_iq+0.79;
        }
        else
        {
//            est_torque = -159.9*speed_out_iq*speed_out_iq+680.7*speed_out_iq-20.61;
            est_torque = -153.5*speed_out_iq*speed_out_iq+653.47*speed_out_iq-19.78;
        }
        output_power = (int)(est_torque*M1_speed_resolver/9550.0);

        speed_resolver = (uint16_t)M1_speed_resolver;
        speed_target =(uint16_t)(M1_SpeedRef*7200);
        ucTXMsgDataA[0] = (speed_resolver&0x00ff);
        ucTXMsgDataA[1] = (speed_resolver&0xff00)>>8;
        ucTXMsgDataA[2] = DC_bus_voltage;
        ucTXMsgDataA[3] = (int)NTC_SIC_avg_filter1+40;
        ucTXMsgDataA[4] = output_power;
        ucTXMsgDataA[5] = CAN_M1_Speed_OUT;
        ucTXMsgDataA[6] = DTC_code1;
        ucTXMsgDataA[7] = DTC_code2;

    CANMessageSet(CANA_BASE, 1, &sTXCANMessageA, MSG_OBJ_TYPE_TX);


}

int PTA1,PTA2,PTB1,PTB2,PTC,PTA1_AVG,PTA2_AVG,PTB1_AVG,PTB2_AVG = 0;
void TempSend()
{
    tempdata[0] = (int)NTC_SIC_M1A+40;
    tempdata[1] = (int)NTC_SIC_M1B+40;
    tempdata[2] = (int)NTC_SIC_M1C+40;
    tempdata[3] = (int)PTA1;
    tempdata[4] = (int)PTA2;
    tempdata[5] = (int)PTB1;
    tempdata[6] = (int)PTB2;
    tempdata[7] = (int)Temp_board+40;
    CANMessageSet(CANA_BASE, 2, &TxCANTempMessageA, MSG_OBJ_TYPE_TX);
}

extern int Mea_PTA1_avg_1,Mea_PTA2_avg_1;
extern int Mea_PTB1_avg_1,Mea_PTB2_avg_1;
extern int Mea_PTC_avg_1;
void DataSend()
{
    operatedata[0] = CAN_M1_Iq_ref & 0xff;
    operatedata[1] = (CAN_M1_Iq_ref&0xff00)>>8;
    operatedata[2] = CAN_M1_Id_ref & 0xff;
    operatedata[3] = (CAN_M1_Id_ref&0xff00)>>8;
    operatedata[4] = CAN_M1_Iq_fdb & 0xff;
    operatedata[5] = (CAN_M1_Iq_fdb&0xff00)>>8;
    operatedata[6] = CAN_M1_Id_fdb & 0xff;
    operatedata[7] = (CAN_M1_Id_fdb&0xff00)>>8;

    CANMessageSet(CANA_BASE, 3, &TxCANDataMessageA, MSG_OBJ_TYPE_TX);
}


void PT100send()
{
    PT100data[0] = Mea_PTA1_avg_1;
    PT100data[1] = Mea_PTA2_avg_1;
    PT100data[2] = Mea_PTB1_avg_1;
    PT100data[3] = Mea_PTB2_avg_1;
    PT100data[4] = Mea_PTC_avg_1;
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
    uint16_t analog=0;
    analog=analog1*10000;

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



uint16_t speed_set = 0 ;
uint16_t control_command = 0;
uint16_t clear_fault = 0;
uint16_t lock_propeller = 0;
float pid_id_kp =0.0;
float pid_id_ki =0.0;
float pid_spd_kp =0.0;
float pid_spd_ki =0.0;
int PID_adjust = 0;
int auto_iq = 0;
int CAN_iqref =0;
int CAN_idref =0;//dqset

void CANAMsgGet()
{
    speed_set = (ucRXMsgDataA[0]+(ucRXMsgDataA[1]&0x0f)*256)*2.0;
    control_command = (ucRXMsgDataA[1]&0x30)>>4;
    clear_fault = (ucRXMsgDataA[1]&0x40)>>6;
    lock_propeller = (ucRXMsgDataA[1]&0x80)>>7;

    if(g_bErrFlagA == 1)
    {
        speed_set = 0;
    }
    pid_spd_kp = ucRXMsgDataA[2]*0.05;
    pid_spd_ki = ucRXMsgDataA[3]*0.0001;
    pid_id_kp = ucRXMsgDataA[4]*0.01;

    pid_id_ki = (ucRXMsgDataA[5]+(ucRXMsgDataA[6]&0x7F)*255)*0.00001;
    PID_adjust = (ucRXMsgDataA[6]&0x80)>>7;

    CAN_iqref = ucRXMsgDataA[7];
    if(CAN_iqref > 100)  //dqset
    {
        CAN_iqref = 100;
    }
    if(CAN_idref > 100)
    {
        CAN_idref = 100;
    }else if (CAN_idref < -100)
    {
        CAN_idref = -100;
    }                   //dqset
}
