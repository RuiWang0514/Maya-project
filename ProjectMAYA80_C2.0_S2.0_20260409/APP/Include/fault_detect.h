#ifndef USER_FAULT_DETECT_H_
#define USER_FAULT_DETECT_H_


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

} ocfault_diag;

extern ocfault_diag A1_OCdiag,A2_OCdiag,B1_OCdiag,B2_OCdiag,C1_OCdiag,C2_OCdiag;

void OCFault_diag_Init();

void Fault_handling();

void CleanFault_flag(ocfault_diag *);
void SingleOCFault_Detect(ocfault_diag *,float ,float , float ,float); // i the diagnose phase, i1 i2 are two remaining phases

#endif /* USER_FAULT_DETECT_H_ */
