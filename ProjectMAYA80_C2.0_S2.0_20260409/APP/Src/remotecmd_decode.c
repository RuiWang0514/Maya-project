#include "remotecmd_decode.h"

//unsigned char cmd_count=0;
//extern char rxbuf[];
//unsigned char cmd_ready=0;

unsigned char cmd_countA=0;
extern char rxbufA[];
unsigned char cmd_readyA=0;

unsigned char cmd_countB=0;
extern char rxbufB[];
unsigned char cmd_readyB=0;

cmd_handle CMD_HandleA[MAX_CMD];
cmd_handle CMD_HandleB[MAX_CMD];

int Cmd_registerA(char *cmd,void (*func)())  // register cmd and its function before use
{
    unsigned char i=0;
    if (cmd_countA >= MAX_CMD-1) return 1;
    for(i=0;i<8;i++)
        CMD_HandleA[cmd_countA].cmd[i]=*(cmd+i);
    CMD_HandleA[cmd_countA].func=func;
    cmd_countA++;
    return 0;
}

int Cmd_registerB(char *cmd,void (*func)())  // register cmd and its function before use
{
    unsigned char i=0;
    if (cmd_countB >= MAX_CMD-1) return 1;
    for(i=0;i<8;i++)
        CMD_HandleB[cmd_countB].cmd[i]=*(cmd+i);
    CMD_HandleB[cmd_countB].func=func;
    cmd_countB++;
    return 0;
}

void Cmd_DecodeA() //call after the cmd is received
{
    unsigned char i,j;
    unsigned char incmd[8];
    if(!cmd_readyA)
        return;
    cmd_readyA=0;
    for(i=0;i<8;i++)
    {
//        if(*(rxbuf+1+i) == ',')
//            return;
        incmd[i]=*(rxbufA+1+i);
    }

    for(i=0;i<cmd_countA;i++)
    {
        for(j=0;j<8;j++)
        {
            if(CMD_HandleA[i].cmd[j] != incmd[j])
                goto CMDDISMATCH;
        }

        (*CMD_HandleA[i].func)();
        return;
        CMDDISMATCH:
        continue;
    }
}

void Cmd_DecodeB() //call after the cmd is received
{
    unsigned char i,j;
    unsigned char incmd[8];
    if(!cmd_readyB)
        return;
    cmd_readyB=0;
    for(i=0;i<8;i++)
    {
//        if(*(rxbuf+1+i) == ',')
//            return;
        incmd[i]=*(rxbufB+1+i);
    }

    for(i=0;i<cmd_countB;i++)
    {
        for(j=0;j<8;j++)
        {
            if(CMD_HandleB[i].cmd[j] != incmd[j])
                goto CMDDISMATCH;
        }

        (*CMD_HandleB[i].func)();
        return;
        CMDDISMATCH:
        continue;
    }
}
