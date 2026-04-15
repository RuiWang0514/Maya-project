#include "fault_detect.h"
#include "HPMC_Settings.h"

ocfault_diag A1_OCdiag,A2_OCdiag,B1_OCdiag,B2_OCdiag,C1_OCdiag,C2_OCdiag;
//uint16_t OC_flag_ticker = 0;
void OCFault_diag_Init()
{
    A1_OCdiag.Icount=0;
    A1_OCdiag.diagvar=1.0;
    A1_OCdiag.targetcount=0;
    A1_OCdiag.OC_flag=0;
    A1_OCdiag.lowth=0.2;

    B1_OCdiag.Icount=0;
    B1_OCdiag.diagvar=1.0;
    B1_OCdiag.targetcount=0;
    B1_OCdiag.OC_flag=0;
    B1_OCdiag.lowth=0.2;

    C1_OCdiag.Icount=0;
    C1_OCdiag.diagvar=1.0;
    C1_OCdiag.targetcount=0;
    C1_OCdiag.OC_flag=0;
    C1_OCdiag.lowth=0.2;

    A2_OCdiag.Icount=0;
    A2_OCdiag.diagvar=1.0;
    A2_OCdiag.targetcount=0;
    A2_OCdiag.OC_flag=0;
    A2_OCdiag.lowth=0.2;

    B2_OCdiag.Icount=0;
    B2_OCdiag.diagvar=1.0;
    B2_OCdiag.targetcount=0;
    B2_OCdiag.OC_flag=0;
    B2_OCdiag.lowth=0.2;

    C2_OCdiag.Icount=0;
    C2_OCdiag.diagvar=1.0;
    C2_OCdiag.targetcount=0;
    C2_OCdiag.OC_flag=0;
    C2_OCdiag.lowth=0.2;
//    GpioDataRegs.GPDCLEAR.bit.GPIO124 = 1;
}

void CleanFault_flag(ocfault_diag *pdiag)
{
    pdiag->diagvar=1.0;
    pdiag->OC_flag=0;
    pdiag->OC_flag_ticker=0;
 //   GpioDataRegs.GPDCLEAR.bit.GPIO125 = 1;
}
void SingleOCFault_Detect(ocfault_diag *pdiag,float i,float i1, float i2,float speed) // i the diagnose phase, i1 i2 are two remaining phases
{
    float tmp1,tmp2,tmp3,freq;
    if(pdiag->Icount < pdiag->targetcount)
    {
        pdiag->curI+=fabsf(i);
        pdiag->curI1+=fabsf(i1);
        pdiag->curI2+=fabsf(i2);
        pdiag->Icount++;
    }
    else
    {
        pdiag->Icount=0;
        tmp1=pdiag->curI/pdiag->T;
        tmp2=pdiag->curI1/pdiag->T;
        tmp3=pdiag->curI2/pdiag->T;
        pdiag->diagvar=(2.0*tmp1)/(tmp2+tmp3);

        pdiag->curI=0.0;
        pdiag->curI1=0.0;
        pdiag->curI2=0.0;

        freq=speed*1.0*BASE_FREQ;
        pdiag->T=1.0/freq;
        pdiag->targetcount=(int)(ISR_FREQUENCY*1000.0/freq);

        /*if(diag.diagvar > diag.highth)
            diag.OC_flag=0;
//        else*/ if(pdiag->diagvar < pdiag->lowth)
        {
          pdiag->OC_flag_ticker++;
        }
        if(pdiag->OC_flag_ticker>1)
        {
            pdiag->OC_flag=1;
        }


//        if(pdiag->OC_flag==1)
//            {
//            OC_flag_ticker++;
//            }
//        else
//            {
//            OC_flag_ticker = 0;
//            }
    }
}
extern uint16_t M1_pwmon;
extern uint16_t M2_pwmon;
extern int M1_FTticker;
extern int M2_FTticker;
int M1_FT_FLAG = 0;
int M2_FT_FLAG = 0;
uint16_t OCFault_Log=0;// 0 no fault , 1-> M1 OC  2-> M2OC
void Fault_handling()
{
    if (A1_OCdiag.OC_flag || B1_OCdiag.OC_flag || C1_OCdiag.OC_flag)
    {
        M1_pwmon=0;
        OCFault_Log|=0X01;
//        GpioDataRegs.GPBSET.bit.GPIO62 = 1;
        M1_FTticker++;
        M1_FT_FLAG = 1;
    }

    if (A2_OCdiag.OC_flag || B2_OCdiag.OC_flag || C2_OCdiag.OC_flag)
    {
        M2_pwmon=0;
        OCFault_Log|=0X02;
//        GpioDataRegs.GPBSET.bit.GPIO62 = 1;
        M2_FTticker++;
        M2_FT_FLAG = 1;
    }

}

