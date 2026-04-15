#include "ServSysTick.h"

uint64_t timestamp=0;

__interrupt void cpu_timer2_isr(void);

void InitSystick()
{
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Step 4. Initialize the Device Peripheral. This function can be
    //         found in F2837xD_CpuTimers.c
    InitCpuTimers();   // For this example, only initialize the Cpu Timers

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer2, 200, 10);

    // To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
    // of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in F2837xD_cputimervars.h), the
    // below settings must also be updated.

    CpuTimer2Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

    // Step 5. User specific code, enable interrupts:

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:

    IER |= M_INT14;
}



uint64_t micros() //return timestamp with micro sec
{
    return timestamp;
}

uint32_t mils()   // return timestamp with mil sec
{
    return timestamp/1000;
}


__interrupt void cpu_timer2_isr(void)
{
   CpuTimer2.InterruptCount++;
   timestamp += 10;
   // The CPU acknowledges the interrupt.

}
