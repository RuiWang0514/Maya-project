#include "RemoteCommand.h"
#include "remotecmd_decode.h"
#include "stdio.h"
#include "canHandle.h"
#include "HPMC_Settings.h"
//#include "HPMC_algo_includes.h"
#include "pid_reg3_027.h"

extern char rxbufA[];
extern char rxbufB[];
extern float ALIGN_ANGLE;

void RunAlgoA();
void RunAlgoB();
void SetLswA();
void SetLswB();
void SetSpeedA();
void SetSpeedB();
void MotorSpeedA();
void MotorSpeedB();
void MotorIqA();
void MotorIqB();
void SetOperationModeA();
void SetOperationModeB();
void SetAlgoRunsA();
void SetAlgoRunsB();
void M1EnableA();
void M1EnableB();
void M2EnableA();
void M2EnableB();
void SpeedPIDA();
void SpeedPIDB();
void M1IdPIDA();
void M1IdPIDB();
void M1IqPIDA();
void M1IqPIDB();
void M2IdPIDA();
void M2IdPIDB();
void M2IqPIDA();
void M2IqPIDB();
void EmgcSTOPA();
void EmgcSTOPB();
void RunstatusUpdateA();
void RunstatusUpdateB();
void angleupdateA();
void angleupdateB();
void ALIGNANGLEA();
void ALIGNANGLEB();


uint32_t ANGLE;


void Command_InitA()
{
    Cmd_registerA("OPSMODES",SetOperationModeA);
    Cmd_registerA("ALGORUNS",SetAlgoRunsA);
    Cmd_registerA("M1ENABLE",M1EnableA);
    Cmd_registerA("M2ENABLE",M2EnableA);
    Cmd_registerA("MOTSPEED",MotorSpeedA);
    Cmd_registerA("MOTIQREF",MotorIqA);
    Cmd_registerA("ALIANGLE",ALIGNANGLEA);
    Cmd_registerA("SPEEDPID",SpeedPIDA);
    Cmd_registerA("M1CIDPID",M1IdPIDA);
    Cmd_registerA("M1CIQPID",M1IqPIDA);
    Cmd_registerA("M2CIDPID",M2IdPIDA);
    Cmd_registerA("M2CIQPID",M2IqPIDA);
    Cmd_registerA("EMGCSTOP",EmgcSTOPA);
    Cmd_registerA("SYSSTATU",RunstatusUpdateA);
    Cmd_registerA("ANGLERED",angleupdateA);
}

void Command_InitB()
{
    Cmd_registerB("OPSMODES",SetOperationModeB);
    Cmd_registerB("ALGORUNS",SetAlgoRunsB);
    Cmd_registerB("M1ENABLE",M1EnableB);
    Cmd_registerB("M2ENABLE",M2EnableB);
    Cmd_registerB("MOTSPEED",MotorSpeedB);
    Cmd_registerB("MOTIQREF",MotorIqB);
    Cmd_registerB("ALIANGLE",ALIGNANGLEB);
    Cmd_registerB("SPEEDPID",SpeedPIDB);
    Cmd_registerB("M1CIDPID",M1IdPIDB);
    Cmd_registerB("M1CIQPID",M1IqPIDB);
    Cmd_registerB("M2CIDPID",M2IdPIDB);
    Cmd_registerB("M2CIQPID",M2IqPIDB);
    Cmd_registerB("EMGCSTOP",EmgcSTOPB);
    Cmd_registerB("SYSSTATU",RunstatusUpdateB);
    Cmd_registerB("ANGLERED",angleupdateB);
}

extern uint16_t M1_lsw;
void SetOperationModeA()
{
    int tmplsw;
    if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%d\r\n",&tmplsw);
        if(tmplsw > 10 || tmplsw < 0)
        {
            Send2CANA("OPSMODES:bad parameters\r\n");
        }
        else
        {
            M1_lsw=tmplsw;
            Send2CANA("OPSMODES:ok\r\n");
        }
    }
    else
    {
        Send2CANA("OPSMODES:BADCOMMAND\r\n");
    }

}

void SetOperationModeB()
{
    int tmplsw;
    if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%d\r\n",&tmplsw);
        if(tmplsw > 10 || tmplsw < 0)
        {
            Send2CANB("OPSMODES:bad parameters\r\n");
        }
        else
        {
            M1_lsw=tmplsw;
            Send2CANB("OPSMODES:ok\r\n");
        }
    }
    else
    {
        Send2CANB("OPSMODES:BADCOMMAND\r\n");
    }

}

extern uint16_t M1_Algo_control_start;
void SetAlgoRunsA()
{
    int tmpalgostart;
    if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%d\r\n",&tmpalgostart);
        if(tmpalgostart > 1 || tmpalgostart < 0)
        {
            Send2CANA("ALGORUNS:bad parameters\r\n");
        }
        else
        {
            M1_Algo_control_start=tmpalgostart;
            Send2CANA("ALGORUNS:ok\r\n");
        }
    }

}

void SetAlgoRunsB()
{
    int tmpalgostart;
    if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%d\r\n",&tmpalgostart);
        if(tmpalgostart > 1 || tmpalgostart < 0)
        {
            Send2CANB("ALGORUNS:bad parameters\r\n");
        }
        else
        {
            M1_Algo_control_start=tmpalgostart;
            Send2CANB("ALGORUNS:ok\r\n");
        }
    }

}

extern uint16_t M1_pwmon;
void M1EnableA()
{
    int tmp;
    if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%d\r\n",&tmp);
        if(tmp > 1 || tmp < 0)
        {
            Send2CANA("M1ENABLE:bad parameters\r\n");
        }
        else
        {
            M1_pwmon=tmp;
            Send2CANA("M1ENABLE:ok\r\n");
        }
    }

}

void M1EnableB()
{
    int tmp;
    if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%d\r\n",&tmp);
        if(tmp > 1 || tmp < 0)
        {
            Send2CANB("M1ENABLE:bad parameters\r\n");
        }
        else
        {
            M1_pwmon=tmp;
            Send2CANB("M1ENABLE:ok\r\n");
        }
    }

}


extern uint16_t M2_pwmon;
void M2EnableA()
{
    int tmp;
    if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%d\r\n",&tmp);
        if(tmp > 1 || tmp < 0)
        {
            Send2CANA("M2ENABLE:bad parameters\r\n");
        }
        else
        {
            M2_pwmon=tmp;
            Send2CANA("M2ENABLE:ok\r\n");
        }
    }

}

void M2EnableB()
{
    int tmp;
    if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%d\r\n",&tmp);
        if(tmp > 1 || tmp < 0)
        {
            Send2CANB("M2ENABLE:bad parameters\r\n");
        }
        else
        {
            M2_pwmon=tmp;
            Send2CANB("M2ENABLE:ok\r\n");
        }
    }

}

extern _iq M1_SpeedRef;
void MotorSpeedA()
{
    float tmp,freq;
    if(*(FIRSTBYTEA) == '?');
        //Send2CAN("$MOTSPEED=0.05\r\n");
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f\r\n",&tmp);
        if(tmp >5000 || tmp< 0)
        {
            Send2CANA("MotorSpeed:bad parameters\r\n");
        }
        else{
            freq=tmp*(POLES/2)/60;
            M1_SpeedRef=freq/BASE_FREQ;
            Send2CANA("MOTSPEED:OK\r\n");
        }
    }
    else
        Send2CANA("MOTSPEED:BADCOMMAND\r\n");
}

void MotorSpeedB()
{
    float tmp,freq;
    if(*(FIRSTBYTEB) == '?');
        //Send2CAN("$MOTSPEED=0.05\r\n");
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f\r\n",&tmp);
        if(tmp >5000 || tmp< 0)
        {
            Send2CANB("MotorSpeed:bad parameters\r\n");
        }
        else{
            freq=tmp*(POLES/2)/60;
            M1_SpeedRef=freq/BASE_FREQ;
            Send2CANB("MOTSPEED:OK\r\n");
        }
    }
    else
        Send2CANB("MOTSPEED:BADCOMMAND\r\n");
}

extern _iq M1_IqRef;
void MotorIqA()
{

    float tmp;
    if(*(FIRSTBYTEA) == '?');
        //Send2CAN("$MOTSPEED=0.05\r\n");
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f\r\n",&tmp);
        if(tmp >1.0 || tmp< 0)
        {
            Send2CANA("MOTIQREF:bad parameters\r\n");
        }
        else{
            M1_IqRef=tmp;
            Send2CANA("MOTIQREF:OK\r\n");
        }
    }
    else
        Send2CANA("MOTIQREF:BADCOMMAND\r\n");
}

void MotorIqB()
{

    float tmp;
    if(*(FIRSTBYTEB) == '?');
        //Send2CAN("$MOTSPEED=0.05\r\n");
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f\r\n",&tmp);
        if(tmp >1.0 || tmp< 0)
        {
            Send2CANB("MOTIQREF:bad parameters\r\n");
        }
        else{
            M1_IqRef=tmp;
            Send2CANB("MOTIQREF:OK\r\n");
        }
    }
    else
        Send2CANB("MOTIQREF:BADCOMMAND\r\n");
}
extern uint16_t sin_enable;
void ALIGNANGLEA()
{
    ANGLE++;
    float tmp;
    if(*(FIRSTBYTEA) == '?');
        //Send2CAN("$MOTSPEED=0.05\r\n");
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f\r\n",&tmp);
//        if(tmp >1.0|| tmp< -1.0)
//        {
//            Send2CAN("ALIGNANGLE:bad parameters\r\n");
//        }
//        else{
        sin_enable = tmp;
            Send2CANA("ALIANGLE:OK\r\n");
//        }
    }
    else
        Send2CANA("ALIANGLE:BADCOMMAND\r\n");
}

void ALIGNANGLEB()
{
    ANGLE++;
    float tmp;
    if(*(FIRSTBYTEB) == '?');
        //Send2CAN("$MOTSPEED=0.05\r\n");
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f\r\n",&tmp);
//        if(tmp >1.0|| tmp< -1.0)
//        {
//            Send2CAN("ALIGNANGLE:bad parameters\r\n");
//        }
//        else{
            ALIGN_ANGLE = tmp;
            Send2CANB("ALIANGLE:OK\r\n");
//        }
    }
    else
        Send2CANB("ALIANGLE:BADCOMMAND\r\n");
}

extern PIDREG3_027 M1_pid1_id;
extern PIDREG3_027 M1_pid1_iq;
extern PIDREG3_027 M1_pid1_spd;

extern PIDREG3_027 M2_pid1_id;
extern PIDREG3_027 M2_pid1_iq;

extern PIDREG3_027 pid1_id;
extern PIDREG3_027 pid1_iq;
//extern TEMP_SENSOR M1_Temp_Measurment;
//extern TEMP_SENSOR M2_Temp_Measurment;

extern float M1_Temp_SiCTemp;;
extern float M2_Temp_SiCTemp;

extern float M1_Board_SiCTemp;
extern float M2_Board_SiCTemp;

extern int OC_MaxspeedRPM;
extern int OC_MinspeedRPM;

extern int M1_OvercurrentView_A;
extern int M1_OvercurrentView_B;
extern int M1_OvercurrentView_C;
extern int M2_OvercurrentView_A;
extern int M2_OvercurrentView_B;
extern int M2_OvercurrentView_C;

extern int M1_undervoltage;
extern int M2_undervoltage;

extern int M1_FT_FLAG;
extern int M2_FT_FLAG;
extern int M1_DESAT_FLAG;
extern int M2_DESAT_FLAG;

void SpeedPIDA()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEA) == '?'){
        sprintf(sendbuf,"$SPEEDPID=%.3f,%.3f,%.3f,%.3f,%.3f\r\n",M1_pid1_spd.Kp*100,M1_pid1_spd.Ki*100,M1_pid1_spd.Kd*100,M1_pid1_spd.OutMax,M1_pid1_spd.OutMin);
        Send2CANA(sendbuf);
    }
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANA("SPEEDPID:bad parameters\r\n");
        }
        else{
            M1_pid1_spd.Kp=kp/100.0;
            M1_pid1_spd.Ki=ki/100.0;
            M1_pid1_spd.Kd=kd/100.0;
//            M1_pid1_spd.OutMax=outmax;
//            M1_pid1_spd.OutMin=outmin;
            Send2CANA("SPEEDPID:OK\r\n");
        }
    }
    else
        Send2CANA("SPEEDPID:BADCOMMAND\r\n");
}

void SpeedPIDB()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEB) == '?'){
        sprintf(sendbuf,"$SPEEDPID=%.3f,%.3f,%.3f,%.3f,%.3f\r\n",M1_pid1_spd.Kp*100,M1_pid1_spd.Ki*100,M1_pid1_spd.Kd*100,M1_pid1_spd.OutMax,M1_pid1_spd.OutMin);
        Send2CANB(sendbuf);
    }
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANB("SPEEDPID:bad parameters\r\n");
        }
        else{
            M1_pid1_spd.Kp=kp/100.0;
            M1_pid1_spd.Ki=ki/100.0;
            M1_pid1_spd.Kd=kd/100.0;
//            M1_pid1_spd.OutMax=outmax;
//            M1_pid1_spd.OutMin=outmin;
            Send2CANB("SPEEDPID:OK\r\n");
        }
    }
    else
        Send2CANB("SPEEDPID:BADCOMMAND\r\n");
}


void M1IdPIDA()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEA) == '?'){
        sprintf(sendbuf,"$M1CIDPID=%.3f,%.3f,%.3f,%.3f,%.3f\r\n",M1_pid1_id.Kp*100,M1_pid1_id.Ki*100,M1_pid1_id.Kd*100,M1_pid1_id.OutMax,M1_pid1_id.OutMin);
        Send2CANA(sendbuf);
    }
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANA("M1CIDPID:bad parameters\r\n");
        }
        else{
            pid1_id.Kp=kp/100.0;
            pid1_id.Ki=ki/100.0;
            pid1_id.Kd=kd/100.0;
//            M1_pid1_id.OutMax=outmax;
//            M1_pid1_id.OutMin=outmin;
            Send2CANA("M1CIDPID:OK\r\n");
        }
    }
    else
        Send2CANA("M1CIDPID:BADCOMMAND\r\n");
}

void M1IdPIDB()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEB) == '?'){
        sprintf(sendbuf,"$M1CIDPID=%.3f,%.3f,%.3f,%.3f,%.3f\r\n",M1_pid1_id.Kp*100,M1_pid1_id.Ki*100,M1_pid1_id.Kd*100,M1_pid1_id.OutMax,M1_pid1_id.OutMin);
        Send2CANB(sendbuf);
    }
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANB("M1CIDPID:bad parameters\r\n");
        }
        else{
            pid1_id.Kp=kp/100.0;
            pid1_id.Ki=ki/100.0;
            pid1_id.Kd=kd/100.0;
//            M1_pid1_id.OutMax=outmax;
//            M1_pid1_id.OutMin=outmin;
            Send2CANB("M1CIDPID:OK\r\n");
        }
    }
    else
        Send2CANB("M1CIDPID:BADCOMMAND\r\n");
}

void M1IqPIDA()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEA) == '?'){
        sprintf(sendbuf,"$M1CIQPID=%.3f,%.3f,%.3f,%.3f,%.3f\r\n",M1_pid1_iq.Kp*100,M1_pid1_iq.Ki*100,M1_pid1_iq.Kd*100,M1_pid1_iq.OutMax,M1_pid1_iq.OutMin);
        Send2CANA(sendbuf);
    }
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANA("M1CIQPID:bad parameters\r\n");
        }
        else{
            pid1_iq.Kp=kp/100.0;
            pid1_iq.Ki=ki/100.0;
            pid1_iq.Kd=kd/100.0;
//            M1_pid1_iq.OutMax=outmax;
//            M1_pid1_iq.OutMin=outmin;
            Send2CANA("M1CIQPID:OK\r\n");
        }
    }
    else
        Send2CANA("M1CIQPID:BADCOMMAND\r\n");
}

void M1IqPIDB()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEB) == '?'){
        sprintf(sendbuf,"$M1CIQPID=%.3f,%.3f,%.3f,%.3f,%.3f\r\n",M1_pid1_iq.Kp*100,M1_pid1_iq.Ki*100,M1_pid1_iq.Kd*100,M1_pid1_iq.OutMax,M1_pid1_iq.OutMin);
        Send2CANB(sendbuf);
    }
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANB("M1CIQPID:bad parameters\r\n");
        }
        else{
            pid1_iq.Kp=kp/100.0;
            pid1_iq.Ki=ki/100.0;
            pid1_iq.Kd=kd/100.0;
//            M1_pid1_iq.OutMax=outmax;
//            M1_pid1_iq.OutMin=outmin;
            Send2CANB("M1CIQPID:OK\r\n");
        }
    }
    else
        Send2CANB("M1CIQPID:BADCOMMAND\r\n");
}

void M2IdPIDA()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEA) == '?'){
        sprintf(sendbuf,"$M2CIDPID=%.f,%.f,%.f,%.f,%.f\r\n",M1_OvercurrentView_A*1.0,M1_OvercurrentView_B*1.0,M1_OvercurrentView_C*1.0,M1_undervoltage*1.0,M1_DESAT_FLAG*1.0);
        Send2CANA(sendbuf);
    }
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANA("M2CIDPID:bad parameters\r\n");
        }
        else{
            M2_pid1_id.Kp=kp/100.0;
            M2_pid1_id.Ki=ki/100.0;
            M2_pid1_id.Kd=kd/100.0;
//           M2_pid1_id.OutMax=outmax;
//            M2_pid1_id.OutMin=outmin;
            Send2CANA("M2CIDPID:OK\r\n");
        }
    }
    else
        Send2CANA("M2CIDPID:BADCOMMAND\r\n");
}

void M2IdPIDB()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEB) == '?'){
        sprintf(sendbuf,"$M2CIDPID=%.f,%.f,%.f,%.f,%.f\r\n",M1_OvercurrentView_A*1.0,M1_OvercurrentView_B*1.0,M1_OvercurrentView_C*1.0,M1_undervoltage*1.0,M1_DESAT_FLAG*1.0);
        Send2CANB(sendbuf);
    }
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANB("M2CIDPID:bad parameters\r\n");
        }
        else{
            M2_pid1_id.Kp=kp/100.0;
            M2_pid1_id.Ki=ki/100.0;
            M2_pid1_id.Kd=kd/100.0;
//           M2_pid1_id.OutMax=outmax;
//            M2_pid1_id.OutMin=outmin;
            Send2CANB("M2CIDPID:OK\r\n");
        }
    }
    else
        Send2CANB("M2CIDPID:BADCOMMAND\r\n");
}

void M2IqPIDA()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEA) == '?'){
        sprintf(sendbuf,"$M2CIQPID=%.f,%.f,%.f,%.f,%.f\r\n",M2_OvercurrentView_A*1.0,M2_OvercurrentView_B*1.0,M2_OvercurrentView_C*1.0,M2_undervoltage*1.0,M2_DESAT_FLAG*1.0);
        Send2CANA(sendbuf);
    }
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANA("M2CIQPID:bad parameters\r\n");
        }
        else{
            M2_pid1_iq.Kp=kp/100.0;
            M2_pid1_iq.Ki=ki/100.0;
            M2_pid1_iq.Kd=kd/100.0;
//            M2_pid1_iq.OutMax=outmax;
//            M2_pid1_iq.OutMin=outmin;
            Send2CANA("M2CIQPID:OK\r\n");
        }
    }
    else
        Send2CANA("M2CIQPID:BADCOMMAND\r\n");
}

void M2IqPIDB()
{
    float kp,ki,kd,outmax,outmin;
    char sendbuf[64];
    if(*(FIRSTBYTEB) == '?'){
        sprintf(sendbuf,"$M2CIQPID=%.f,%.f,%.f,%.f,%.f\r\n",M2_OvercurrentView_A*1.0,M2_OvercurrentView_B*1.0,M2_OvercurrentView_C*1.0,M2_undervoltage*1.0,M2_DESAT_FLAG*1.0);
        Send2CANB(sendbuf);
    }
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,"%f,%f,%f,%f,%f\r\n",&kp,&ki,&kd,&outmax,&outmin);
        if(0)
        {
            Send2CANB("M2CIQPID:bad parameters\r\n");
        }
        else{
            M2_pid1_iq.Kp=kp/100.0;
            M2_pid1_iq.Ki=ki/100.0;
            M2_pid1_iq.Kd=kd/100.0;
//            M2_pid1_iq.OutMax=outmax;
//            M2_pid1_iq.OutMin=outmin;
            Send2CANB("M2CIQPID:OK\r\n");
        }
    }
    else
        Send2CANB("M2CIQPID:BADCOMMAND\r\n");
}

void EmgcSTOPA()
{
    M1_pwmon=0;
    M2_pwmon=0;
    M1_Algo_control_start=0;
}

void EmgcSTOPB()
{
    M1_pwmon=0;
    M2_pwmon=0;
    M1_Algo_control_start=0;
}

extern float real_DCBUS_voltage;
extern float Iderr;
extern float M1_real_voltage,M2_real_voltage;
extern float Temp_board;
extern int M1_M2_FLAG;
extern float M1_EstimatedSpeed;
extern float M2_EstimatedSpeed;
extern float NTC_SIC_M1A,NTC_SIC_M1B,NTC_SIC_M1C,NTC_SIC_M2A,NTC_SIC_M2B,NTC_SIC_M2C;
extern float temp1_value;
extern float temp2_value;
void RunstatusUpdateA()
{
    char sendbuf[64];
    float RPM,M1iq,M2iq,M1_real_DCBUS_voltage,M2_real_DCBUS_voltage;
    RPM=60*(M1_pid1_spd.Fdb*BASE_FREQ)/(POLES/2);
//    M1iq=M1_pid1_iq.Fdb*BASE_CURRENT;
//    M2iq=M2_pid1_iq.Fdb*BASE_CURRENT;

    M1iq=NTC_SIC_M1A;
    M2iq=temp2_value;

    M1_real_DCBUS_voltage=M1_real_voltage;
    M2_real_DCBUS_voltage=M2_real_voltage;

//    M1iq=OC_MaxspeedRPM;
//    M2iq=OC_MinspeedRPM;
    sprintf(sendbuf,"$SYSSTATU=%.1f,%.2f,%.2f,%.1f,%.1f\r\n",RPM,M1iq,M2iq,M1_real_DCBUS_voltage,M2_real_DCBUS_voltage);
    Send2CANA(sendbuf);
}


void RunstatusUpdateB()
{
    char sendbuf[64];
    float RPM,M1iq,M2iq,M1_real_DCBUS_voltage,M2_real_DCBUS_voltage;
    RPM=60*(M1_pid1_spd.Fdb*BASE_FREQ)/(POLES/2);
//    M1iq=M1_pid1_iq.Fdb*BASE_CURRENT;
//    M2iq=M2_pid1_iq.Fdb*BASE_CURRENT;

    M1iq=NTC_SIC_M1A;
    M2iq=temp2_value;


    M1_real_DCBUS_voltage=M1_EstimatedSpeed;
    M2_real_DCBUS_voltage=M2_EstimatedSpeed;

//    M1iq=OC_MaxspeedRPM;
//    M2iq=OC_MinspeedRPM;
    sprintf(sendbuf,"$SYSSTATU=%.1f,%.2f,%.2f,%.1f,%.1f\r\n",RPM,M1iq,M2iq,M1_real_DCBUS_voltage,M2_real_DCBUS_voltage);
    Send2CANB(sendbuf);
}






void angleupdateA()
{
    float angle;
    char sendbuf[64];
    if(*(FIRSTBYTEA) == '?'){
//        angle = ALIGN_ANGLE;
        sprintf(sendbuf,"$ANGLERED=%.3f\r\n",ALIGN_ANGLE);
        Send2CANA(sendbuf);
    }
    else if(*(FIRSTBYTEA) == '=')
    {
        sscanf(FIRSTBYTEA+1,",%f\r\n",&angle);
        if(0)
        {
            Send2CANA("ANGLERED:bad parameters\r\n");
        }
        else{
//            M1_pid1_spd.Kp=kp/100.0;

            ALIGN_ANGLE=angle;
//            M1_pid1_spd.OutMax=outmax;
//            M1_pid1_spd.OutMin=outmin;
            Send2CANA("ANGLERED:OK\r\n");
        }
    }
    else
        Send2CANA("ANGLE:BADCOMMAND\r\n");
}

void angleupdateB()
{
    float angle;
    char sendbuf[64];
    if(*(FIRSTBYTEB) == '?'){
//        angle = ALIGN_ANGLE;
        sprintf(sendbuf,"$ANGLERED=%.3f\r\n",ALIGN_ANGLE);
        Send2CANB(sendbuf);
    }
    else if(*(FIRSTBYTEB) == '=')
    {
        sscanf(FIRSTBYTEB+1,",%f\r\n",&angle);
        if(0)
        {
            Send2CANB("ANGLERED:bad parameters\r\n");
        }
        else{
//            M1_pid1_spd.Kp=kp/100.0;

            ALIGN_ANGLE=angle;
//            M1_pid1_spd.OutMax=outmax;
//            M1_pid1_spd.OutMin=outmin;
            Send2CANB("ANGLERED:OK\r\n");
        }
    }
    else
        Send2CANB("ANGLE:BADCOMMAND\r\n");
}















void RunAlgoA()
{
    int run=0;
    sscanf(rxbufA+1+8+1,"%d\r\n",&run);
}

void SetLswA()
{
    int tlsw=0;
    sscanf(rxbufA+1+8+1,"%d\r\n",&tlsw);

}

void SetSpeedA()
{
    float speed;
    sscanf(rxbufA+1+8+1,"%f\r\n",&speed);
}

void RunAlgoB()
{
    int run=0;
    sscanf(rxbufB+1+8+1,"%d\r\n",&run);
}

void SetLswB()
{
    int tlsw=0;
    sscanf(rxbufB+1+8+1,"%d\r\n",&tlsw);

}

void SetSpeedB()
{
    float speed;
    sscanf(rxbufB+1+8+1,"%f\r\n",&speed);
}
