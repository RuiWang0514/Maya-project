#ifndef USER_CAN_HANDLE_H_
#define USER_CAN_HANDLE_H_


void CANA_Initialzation();

void CANA_Ready();

void Send2CANA(char *);

void CANB_Initialzation();

void CANB_Ready();

void Send2CANB(char *);

void CANBMsgSend();

void CANAMsgSend();
void CANAMsgGet();
void TempSend();
void DataSend();
void PT100send();
void DTCSend();
typedef struct
{
    float curI,curI1,curI2;
    float T;
    float diagvar;
    unsigned char OC_flag;
    unsigned char id;
    int Icount;
    int targetcount;
    float lowth,highth;
    int OC_flag_ticker;

} cur_rms;

void cur_rms_Init();

void CleanOverCur_flag(cur_rms *);

void CurRms_Detect(cur_rms *,float ,float , float ,float);
#endif /* USER_CAN_HANDLE_H_ */
