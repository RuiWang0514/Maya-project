#include "Background_task.h"

extern void (*A_Task_Ptr)(void);       // State pointer A branch
extern void (*B_Task_Ptr)(void);       // State pointer B branch
extern void (*C_Task_Ptr)(void);       // State pointer C branch

void A1(void);
void A2(void);
void A3(void);
void B1(void);
void B2(void);
void B3(void);
void C1(void);
void C2(void);
void C3(void);

//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A2
    A_Task_Ptr = &A2;
    //-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A3
    A_Task_Ptr = &A3;
    //-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    A_Task_Ptr = &A1;
    //-----------------
}



//=================================================================================
//  B - TASKS (executed in every 100 usec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    B_Task_Ptr = &B2;
    //-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B3
    B_Task_Ptr = &B3;
    //-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B1;
    //-----------------
}


//=================================================================================
//  C - TASKS (executed in every 150 usec)
//=================================================================================

//--------------------------------- USER ------------------------------------------

extern int LedCnt1;
//----------------------------------------
void C1(void)   // Toggle GPIO-34
//----------------------------------------
{

    if(LedCnt1==0)
    {
//      GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // LED blinking code
        LedCnt1 = 200;
    }
    else
        LedCnt1--;

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C2
    C_Task_Ptr = &C2;
    //-----------------

}

//----------------------------------------
void C2(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C3
    C_Task_Ptr = &C3;
    //-----------------
}


//-----------------------------------------
void C3(void) //  SPARE
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C1
    C_Task_Ptr = &C1;
    //-----------------
}


