#ifndef PROJ_INCLUDE_H
#define PROJ_INCLUDE_H


/*------------------------------------------------------------------------------
 油门输入源（运行时分支；在 HPMC_main_80.c 中修改 throttle_type）
   CAN_THROTTLE ：走 CAN 的 MCU_Command（speed_set、clear_fault、control_command）
   VOLT_THROTTLE：走 analog1 的 0~5V，不经 CAN 指令门控
------------------------------------------------------------------------------*/
typedef enum
{
    CAN_THROTTLE = 0,
    VOLT_THROTTLE = 1
} ThrottleType_e;

extern ThrottleType_e throttle_type;

/* 电压油门：0~5V；线性区 0.5~5V 对应 130~2400rpm；<=0.5V 为怠速 130rpm；>5V 钳位 2400rpm */
#define VOLT_THROTTLE_V_MAX           (5.0f)
#define VOLT_THROTTLE_LINEAR_LO       (0.5f)    /* 线性区低端 */
#define VOLT_THROTTLE_LINEAR_HI       (5.0f)    /* 线性区高端 */
#define VOLT_THROTTLE_IDLE_RPM        (130.0f)  /* 怠速：<=LINEAR_LO 时输出 */
#define VOLT_THROTTLE_MIN_RPM         (130.0f)  /* 映射最低转速（= 怠速） */
#define VOLT_THROTTLE_MAX_RPM         (2400.0f)
/* 油门使能开关：GPIO63 数字输入，低有效（外部默认高，12V 到来拉低 = 使能） */
#define THROTTLE_SW_GPIO              (63u)
/* 开关滤波：仅对“无效侧”（GPIO63 回高）做 10ms 滤波；有效侧（拉低）立即生效 */
#define THROTTLE_SW_OFF_FILTER_TICKS  (200u)    /* 10ms @ 20kHz ISR */

//#define F10
#define MPMOTOR

/*-------------------------------------------------------------------------------
Include project specific include files.
-------------------------------------------------------------------------------*/
// define math type as float(1)
#define   MATH_TYPE      1
#include <stdint.h>
#include "IQmathLib.h"
#include "F28x_Project.h"
/* ---- BSW/Serve layer headers ---- */
#include "ServECap.h"
#include "ServSysTick.h"
#include "ServAD2S.h"
#include "ServMotorTemp.h"
#include "ServNTCTemp.h"
#include "Servi2c.h"
/* ---- BSW/BoardDrv layer headers ---- */
#include "ConfigPWM.h"
#include "Startup.h"
/* ---- APP layer headers (same directory) ---- */
#include "telemetry.h"
#include "pga411.h"
#include "canHandle.h"
#include "fault_detect.h"
/* ---- FWLib headers (via CCS include path) ---- */
#include "F2837xD_gpio.h"
/* ---- Algs layer headers ---- */
#include <math.h>
#include "fast_math.h"


#include "F2837xD_sdfm_drivers.h"
#include "Resolver_Float.h"

#include "F2837x_QEP_Module.h"

#define MS *1000
#define US *1
#define S *1000000


/* ---- SPI Config ---- */
#define SPIB_channel 0
#define SPIC_channel 1


/*------------------------------------------------------------------------------
List of control GND configurations - COLD or HOT
------------------------------------------------------------------------------*/
#define COLD  1           // control GND is COLD
#define HOT   2           // control GND is HOT

/*------------------------------------------------------------------------------
This line sets the CGND configuration to one of the available choices.
------------------------------------------------------------------------------*/
#define  CGND  COLD

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1           // Module check out (do not connect the motors)
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset and speed measurement
#define LEVEL3  3           // Verify closed current(torque) loop and its PIs
#define LEVEL4  4           // Verify speed loop and speed PID
#define LEVEL5  5           // Verify position loop

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL5

/*------------------------------------------------------------------------------
Following is the list of Current sense options
------------------------------------------------------------------------------*/
#define SHUNT_CURRENT_SENSE 1
#define LEM_CURRENT_SENSE   2
#define SD_CURRENT_SENSE    3

/*------------------------------------------------------------------------------
This line sets the CURRENT_SENSE to user's choice from the above.
------------------------------------------------------------------------------*/
#define CURRENT_SENSE LEM_CURRENT_SENSE

/*------------------------------------------------------------------------------
Following is the list of Position Encoder options
------------------------------------------------------------------------------*/
// Select Position Feedback Option
#define QEP_POS_ENCODER       1
#define RESOLVER_POS_ENCODER  2
#define BISS_POS_ENCODER      3
#define ENDAT_POS_ENCODER     4
#define SINCOS_POS_ENCODER    5


/*------------------------------------------------------------------------------
This line sets the POSITION_ENCODER to user's choice from the above.
------------------------------------------------------------------------------*/
#define POSITION_ENCODER QEP_POS_ENCODER

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP280x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 100
#elif (F28_2837xD==1)
#define SYSTEM_FREQUENCY 200
#define TBCLK_FREQUENCY SYSTEM_FREQUENCY/2
#endif

// Timer definitions based on System Clock
// 150 MHz devices
#define     mSec0_5     0.5*SYSTEM_FREQUENCY*1000       // 0.5 mS
#define     mSec1       1*SYSTEM_FREQUENCY*1000     // 1.0 mS
#define     mSec2       2.0*SYSTEM_FREQUENCY*1000       // 2.0 mS
#define     mSec5       5*SYSTEM_FREQUENCY*1000     // 5.0 mS
#define     mSec7_5     7.5*SYSTEM_FREQUENCY*1000       // 7.5 mS
#define     mSec10      10*SYSTEM_FREQUENCY*1000        // 10 mS
#define     mSec20      20*SYSTEM_FREQUENCY*1000        // 20 mS
#define     mSec50      50*SYSTEM_FREQUENCY*1000        // 50 mS
#define     mSec100     100*SYSTEM_FREQUENCY*1000       // 100 mS
#define     mSec500     500*SYSTEM_FREQUENCY*1000   // 500 mS
#define     mSec1000    1000*SYSTEM_FREQUENCY*1000  // 1000 mS

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 20
//#define ISR_FREQUENCY 10
// Define the ISR period (us)
#define ISR_PERIOD (1*1000/ISR_FREQUENCY)
#define INV_PWM_TICKS  ((float)TBCLK_FREQUENCY/ISR_FREQUENCY)*1000

// Define Dead time (us)
#define DEAD_TIME_VALUE 1.0   //0.15
//#define DEAD_TIME_VALUE 1.5
#define DEAD_TIME INV_PWM_TICKS*DEAD_TIME_VALUE/ISR_PERIOD

#define INV_PWM_TBPRD INV_PWM_TICKS/2
#define INV_PWM_HALF_TBPRD INV_PWM_TICKS/4


// Resolver Related defines
#define RESOLVER_EXC_SAMPLE_FREQUENCY   160   // in KHz
#define RESOLVER_EXC_FREQUENCY           10   // in KHz
#define RESOLVER_PWM_TICKS  ((SYSTEM_FREQUENCY/2.0)/RESOLVER_EXC_SAMPLE_FREQUENCY)*1000

#define  TWO_PI                  (2*PI)
#define  DELAY_LENGTH            16

#define  FIR32_SEL    0   /* 1 - Implement 33 tap FIR, samples  over 2 cycles
                             0 - Implement 17 tap FIR, samples  over 1 cycle   */

#define  TUNING_SEL   0   /* 1 - bypass atan value - to tune PI coefficients
                             0 - use atan value*/

// RESOLVER specs
#define RESOLVER_STEPS_PER_TURN         4096       // Resolver's discrete steps/turn
#define RESOLVER_STEPS_PER_POLEPAIR    (RESOLVER_STEPS_PER_TURN/(POLES/2))

#define  POS_KI_LOW_SPD   0.8// _IQ(0.0001)
#define  POS_KI_MED_SPD   1.0//_IQ(0.001)
#define  POS_KI_HI_SPD    1.2//_IQ(0.01)


/*------------------------------------------------------------------------------
set the motor parameters to the one available
------------------------------------------------------------------------------*/

// Define the electrical motor parametes (Estun Servomotor)
#define RS      0.1//0.011//0.011  //0.1524  //0.1992                    // Stator resistance (ohm)
#define RR                              // Rotor resistance (ohm)
#define LS      0.0006//0.00003483//0.00003483  //0.00003 //0.00003594                  // Stator inductance (H)
#define LR                              // Rotor inductance (H)
#define LM                              // Magnatizing inductance (H)
#define POLES   40  //maya80 motor                       // Number of poles
//#define POLES   28 // for mpb motor

// Define the base quantites
#define BASE_VOLTAGE        100      //49    // Base peak phase voltage (volt), Vdc/sqrt(3)
#define BASE_SHUNT_CURRENT    9.95    // Base peak phase current (amp), Max. measurable peak curr.
#define BASE_LEM_CURRENT     12.0     //  ----- do -----
#define BASE_TORQUE                   // Base torque (N.m)
#define BASE_FLUX                     // Base flux linkage (volt.sec/rad)
#define BASE_FREQ       1200           // Base electrical frequency (Hz)

//mpb motor
//#define BASE_FREQ       1000           // Base electrical frequency (Hz)

#define BASE_CURRENT    600//450         // Base peak phase current (amp), Max. measurable peak curr.1.65/6.6mV. Value 246 is used in V1.
#define M_BASE_VOLTAGE  100//433//100            //27.2
#define M_BASE_CURRENT  450//226//450            //6.928


/*-----------------------------------------------------------------------------
 * Sigma Delta Filter Module - frame setting
 * ----------------------------------------------------------------------------
 */
#define  SDFM_TICKS    5
#define  OSR_RATE      OSR_128

/*------------------------------------------------------------------------------
Current sensors scaling
------------------------------------------------------------------------------*/
// LEM    1.0pu current ==> 12.0A -> 2048 counts ==> 8A -> 1365
// SHUNT  1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
#define LEM(A)     2048*A/BASE_LEM_CURRENT
#define SHUNT(A)   2048*A/BASE_SHUNT_CURRENT

// ADC Configuration
//definitions for selecting ADC resolution
#define RESOLUTION_12BIT   0 //12-bit resolution
#define RESOLUTION_16BIT   1 //16-bit resolution (not supported for all variants)

//definitions for selecting ADC signal mode
#define SIGNAL_SINGLE          0 //single-ended channel conversions (12-bit mode only)
#define SIGNAL_DIFFERENTIAL    1 //differential pair channel conversions

//#define ADC_PU_SCALE_FACTOR        0.000244140625     //1/2^12
#define ADC_PU_SCALE_FACTOR        0.000244200244     ///1/(2^12-1)

#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     //1/2^11
#define SD_PU_SCALE_FACTOR         0.000030517578125  // 1/2^15

#define REFERENCE_VDAC     0
#define REFERENCE_VREF     1

// ADC Related defines
#define ADC_REFS        3.3
#define ADC_RES2mVOL     ADC_REFS*ADC_PU_SCALE_FACTOR*1000

//VOLTAGE measurement
#define Volt_upper_R 1245.0 //30.5 //kohm
#define Volt_lower_R 2.0

#define Volt_ADC_PU_SCALE_FACTOR ADC_REFS*(Volt_upper_R+Volt_lower_R)/Volt_lower_R/BASE_VOLTAGE
#define Volt_REALVALUE_FACTOR Volt_ADC_PU_SCALE_FACTOR*BASE_VOLTAGE

#define M1_Volt_DCBUS AdcdResultRegs.ADCRESULT1
#define M2_Volt_DCBUS AdcdResultRegs.ADCRESULT1

//#define M1_Temp_Board_Raw AdcaResultRegs.ADCRESULT4

//#define M1_Temp_PhaseC_Raw AdcaResultRegs.ADCRESULT2
//#define M1_Temp_PhaseB_Raw AdccResultRegs.ADCRESULT2
//#define M1_Temp_PhaseA_Raw AdcaResultRegs.ADCRESULT3
//#define M2_Temp_PhaseA_Raw AdcdResultRegs.ADCRESULT3
//#define M2_Temp_PhaseB_Raw AdcdResultRegs.ADCRESULT2
//#define M2_Temp_PhaseC_Raw AdcbResultRegs.ADCRESULT3

#define M1_Temp_PhaseB_Raw AdcbResultRegs.ADCRESULT1
#define M1_Temp_PhaseC_Raw AdcdResultRegs.ADCRESULT4
#define M1_Temp_PhaseA_Raw AdccResultRegs.ADCRESULT1
//#define M2_Temp_PhaseB_Raw AdcbResultRegs.ADCRESULT1
//#define M2_Temp_PhaseA_Raw AdccResultRegs.ADCRESULT1
//#define M2_Temp_PhaseC_Raw AdcdResultRegs.ADCRESULT4

#define Mea_PTA1_Raw AdccResultRegs.ADCRESULT5
#define Mea_PTA2_Raw AdcaResultRegs.ADCRESULT2
#define Mea_PTB1_Raw AdcbResultRegs.ADCRESULT4
#define Mea_PTB2_Raw AdcaResultRegs.ADCRESULT5
//#define Mea_PTC_Raw AdcbResultRegs.ADCRESULT5
#define M1_Temp_Board_Raw AdcaResultRegs.ADCRESULT1



#define ADC_RES2mVOL     ADC_REFS*ADC_PU_SCALE_FACTOR*1000

//Current Measurement
//#define ShuntR 0.0264  //ohm, for sensor: -50A-50A
#define ShuntRA 0.0002//0.02570594
#define ShuntRB 0.0002//0.02495274
//for 150A current sensor
//#define Cur_Amp_Gain 0.00290178    //Gain of ADC amplifier  INA240A2---GAIN=50

//for 100A current sensor
//#define Cur_Amp_Gain 0.00435268    //Gain of ADC amplifier  INA240A2---GAIN=50

//for 400A current sensor
//#define Cur_Amp_Gain 0.00108817    //Gain of ADC amplifier  INA240A2---GAIN=50

//for 200A current sensor
//#define Cur_Amp_Gain 0.00217634    //Gain of ADC amplifier  INA240A2---GAIN=50

//for maya190 150A current sensor
#define Cur_Amp_Gain 0.00202    //Gain of ADC amplifier  INA240A2---GAIN=50       0.00303*2/3

//for 100A current sensor
//#define Cur_Amp_Gain 0.00533333    //Gain of ADC amplifier  INA240A2---GAIN=50

//#define Cur_ADC_PU_SCALE_FACTOR (ADC_REFS/ShuntR/Cur_Amp_Gain)/BASE_CURRENT
#define Cur_ADC_PU_SCALE_FACTOR (ADC_REFS/Cur_Amp_Gain)/BASE_CURRENT
#define Cur_ADC_PU_SCALE_FACTOR_B (ADC_REFS/ShuntRB/Cur_Amp_Gain)/BASE_CURRENT
#define Cur_REALVALUE_FACTOR Cur_ADC_PU_SCALE_FACTOR*BASE_CURRENT

#define Cur_ADC_PU_SCALE_FACTOR1 (ADC_REFS/ShuntR/Cur_Amp_Gain)

#define Cur_M1_PhaseA_Raw AdcaResultRegs.ADCRESULT0
#define Cur_M1_PhaseB_Raw AdccResultRegs.ADCRESULT0
#define Cur_M1_PhaseC_Raw AdcbResultRegs.ADCRESULT0

#define Cur_M2_PhaseC_Raw AdccResultRegs.ADCRESULT1
#define Cur_M2_PhaseB_Raw AdccResultRegs.ADCRESULT0
#define Cur_M2_PhaseA_Raw AdcaResultRegs.ADCRESULT1

// ************************************************************************
// Scaling factors to bring all current feedbacks to normal scale
//   matching with shunt based measurement
//  With shunt, 1.0pu current  == 9.945A
//       LEM,   1.0pu current  == 12A
//       SDFM,  0.8906pu current  == 12.5A
// ************************************************************************
#define  LEM_TO_SHUNT    1.206637   // (12.0/9.945)
#define  SDFM_TO_SHUNT   1.41131    // (12.5/0.8906)/9.945

/*****************************************************************************
 * ***************************************************************************
 */


#define EPWM_M1_PhaseC_Regs EPwm1Regs
#define EPWM_M1_PhaseB_Regs EPwm2Regs
#define EPWM_M1_PhaseA_Regs EPwm3Regs

#define EPWM_M2_PhaseA_Regs EPwm9Regs
#define EPWM_M2_PhaseB_Regs EPwm8Regs
#define EPWM_M2_PhaseC_Regs EPwm7Regs


#endif
