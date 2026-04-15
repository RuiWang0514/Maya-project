#ifndef REMOTECMD_DECODE_H_
#define REMOTECMD_DECODE_H_

#define MAX_CMD 256

typedef struct{
    char cmd[8];
    void (*func)();
}cmd_handle;

int Cmd_registerA(char *,void (*)()) ;
int Cmd_registerB(char *,void (*)()) ;
void Cmd_DecodeA(void); //call after the cmd is received
void Cmd_DecodeB(void); //call after the cmd is received


#endif /* REMOTECMD_DECODE_H_ */
