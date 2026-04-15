
#include "HPMC_Settings.h"
#include "HPMC_algo_includes.h"
#include "TLF35584.h"

#include "CLAShared.h"
#include "SystemInit.h"
#include "i2cHandle.h"
#include "can.h"

#ifdef _FLASH
#pragma CODE_SECTION(MotorControlISR,"ramfuncs");
#pragma CODE_SECTION(M1_AlgoInitStart,"ramfuncs");
#pragma CODE_SECTION(Current_and_voltage_measurement,"ramfuncs");
#pragma CODE_SECTION(SingleOCFault_Detect,"ramfuncs");
#pragma CODE_SECTION(CleanFault_flag,"ramfuncs");
#pragma CODE_SECTION(Fault_handling,"ramfuncs");
#pragma CODE_SECTION(CANAIntHandler,"ramfuncs");
#pragma CODE_SECTION(CANBIntHandler,"ramfuncs");
#pragma CODE_SECTION(CANIntStatus,"ramfuncs");
#pragma CODE_SECTION(CANStatusGet,"ramfuncs");
#pragma CODE_SECTION(CANIntClear,"ramfuncs");
#pragma CODE_SECTION(cpu_timer2_isr,"ramfuncs");
#pragma CODE_SECTION(Fault_GPIO_init,"ramfuncs");
#pragma CODE_SECTION(Fault_GPIO_Stateinit,"ramfuncs");
#pragma CODE_SECTION(F28x_usDelay,"ramfuncs");
#endif

#pragma INTERRUPT (MotorControlISR, LPI)

// Prototype statements for functions found within this file.

// **********************************************************
// ********************Interrupt  Functions *****************
// **********************************************************
__interrupt void MotorControlISR(void);
extern __interrupt void epwm1_tzint_isr(void);
extern __interrupt void epwm2_tzint_isr(void);
extern __interrupt void epwm3_tzint_isr(void);
extern __interrupt void epwm7_tzint_isr(void);
extern __interrupt void epwm8_tzint_isr(void);
extern __interrupt void epwm9_tzint_isr(void);

#ifdef __cplusplus
#pragma DATA_SECTION("CpuToCla1MsgRAM");
uint32_t cal_ecap1;
#pragma DATA_SECTION("CpuToCla1MsgRAM");
uint32_t cal_ecap2;
#pragma DATA_SECTION("CpuToCla1MsgRAM");
float Cla_ALIGN_ANGLE;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
uint32_t cal_ecap3;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
uint32_t cal_ecap4;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
float TimeCap;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
float Cla_angle;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
float Cla_angleB;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
float Cla_mech_angleB;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
uint16_t cla_rcv_countflg;
#else
#pragma DATA_SECTION(cal_ecap1,"CpuToCla1MsgRAM");
uint32_t cal_ecap1;
#pragma DATA_SECTION(cal_ecap2,"CpuToCla1MsgRAM");
uint32_t cal_ecap2;
#pragma DATA_SECTION(Cla_ALIGN_ANGLE,"CpuToCla1MsgRAM");
float Cla_ALIGN_ANGLE;
#pragma DATA_SECTION(cal_ecap3,"Cla1ToCpuMsgRAM");
uint32_t cal_ecap3;
#pragma DATA_SECTION(cal_ecap4,"Cla1ToCpuMsgRAM");
uint32_t cal_ecap4;
#pragma DATA_SECTION(Cla_angle,"Cla1ToCpuMsgRAM");
float Cla_angle;
#pragma DATA_SECTION(Cla_angleB,"Cla1ToCpuMsgRAM");
float Cla_angleB;
#pragma DATA_SECTION(Cla_mech_angleB,"Cla1ToCpuMsgRAM");
float Cla_mech_angleB;
#pragma DATA_SECTION(TimeCap,"Cla1ToCpuMsgRAM");
float TimeCap;


#endif //__cplusplus
// **********************************************************
// ********************* Functions **************************
// **********************************************************

/*------------------------------------------------------------------------------
 默认油门源：CAN_THROTTLE=CAN 指令；VOLT_THROTTLE=analog1 模拟 0~5V
------------------------------------------------------------------------------*/
ThrottleType_e throttle_type = CAN_THROTTLE;

void Scib_xmit(int a);
void Telemetry_Transmit(void);
void Init_SCIB_GPIO(void);
void Init_SCIB(void);

void DeviceInit();
void MemCopy();
void InitFlash();
void HVDMC_Protection(void);
void PWM_1ch_UpDwnCnt_CNF(int16_t n, uint16_t period, int16_t db);
void Configure_PWM(void);
void ConfigureADC(void);
void configureDAC(uint16_t dac_num);
void ADC_Startup(void);

void Parameter_Init(void);
void VoltThrottle_Update(void);

void M1_AlgoInitStart(void);
extern void M1_StopMotor(void);
extern void M1_ClearToStartMotor(void);

void AutoTest(void);
extern __interrupt void cpu_timer2_isr(void);

void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);

__interrupt void cla1Isr1();
__interrupt void cla1Isr2();
__interrupt void cla1Isr3();
__interrupt void cla1Isr4();
__interrupt void cla1Isr5();
__interrupt void cla1Isr6();
__interrupt void cla1Isr7();
__interrupt void cla1Isr8();

_iq refPosGen(_iq out);
_iq ramper(_iq in, _iq out, _iq rampDelta);
extern uint32_t angle_raw;
// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);  //state A0
void B0(void);  //state B0
void C0(void);  //state C0

// A branch states
extern void A1(void);  //state A1
extern void A2(void);  //state A2
extern void A3(void);  //state A3

// B branch states
extern void B1(void);  //state B1
extern void B2(void);  //state B2
extern void B3(void);  //state B3

// C branch states
extern void C1(void);  //state C1
extern void C2(void);  //state C2
extern void C3(void);  //state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch


//*********************** USER Variables *************************************
// Global variables used in this system
//****************************************************************************
uint32_t SoundTime =1;
uint16_t SoundTimeCount=1;
uint16_t angle_channel = 0;//1 for SPIC-1p, 0 for SPIB-2p
uint32_t SoundPeriod = 12000;

uint16_t i2cError;
float Eangle_resolverB1;
float Eangle_resolverB_angle16;
// ****************************************************************************
// Variables for current measurement
// ****************************************************************************
// Offset calibration routine is run to calibrate for any offsets on the opamps
_iq offset_M1_PhaseA,
    offset_M1_PhaseB,         // offset in LEM current V fbk channel @ 0A
    offset_M1_PhaseC;         // offset in LEM current W fbk channel @ 0A


//*********************** DLOG*************************************
// All variables and definition related to DLOG
//****************************************************************************
#define Test_BUF_LEN 1000
uint16_t num_buf = 0 ;
float TestBuf[Test_BUF_LEN];             // Test buffer allocation
float TestBuf2[Test_BUF_LEN];             // Test buffer allocation
float *test=NULL;
float *test2=NULL;
int M1_clearmotor=0;

float MotorLdq=0.0;
float MotorLq=0.0;
float MotorRs=0.011;
float MotorLs=0.00003483;
uint16_t DlogUpdate=160;
uint16_t DlogUpdateV=160;
int count_dlog=0;
float M1_ud = 0.0;
float M1_uq = 0.0;
int down=0, down_sampling=1;
uint16_t Capture_Trigger=0;
extern uint32_t ecapticker;


//*********************** Fundamental *************************************
// All variables and definition related to Fundamental
//****************************************************************************
float T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h
_iq M1_Ipulse;
_iq M1_Lgain=_IQ(1.0);
_iq M2_Ipulse;
_iq M2_Lgain=_IQ(1.0);
uint32_t IsrTicker = 0;
uint32_t cmdticker = 0;
uint32_t OC_Ticker1 = 0;
uint32_t OC_Ticker2 = 0;
int OC_Flag = 0;
float IsrTicker1 = 0;
uint32_t IsrTicker2 = 0;
uint32_t IsrTicker3 = 0;
uint32_t IsrTicker4 = 0;
float TimerInMs = 0.1;

int fsample=500;
int count_pulse=0;
extern uint16_t enTelemetry;
extern uint32_t iThrottle;
int resolver_1st =1;
//*********************** *************************************
// End of Fundamental
//****************************************************************************

//*********************** Basic Measurement and Compensation *************************************
// All variables and definition related to Basic Measurement and Compensation
//****************************************************************************

float TimeCap1=0;
int16_t Direction;
extern float Eangle_resolver, M1_angle_err=0.0;
extern float Eangle_resolverB,Eangle_resolverC;
extern float mech_angle;
extern uint32_t  dshot_rcv_buffer;
extern int eCAPdT;
float Eangle_resolver_err = 0.0;

float M1_PhaseA_Current = 0;
float M1_Phase_Current_A = 0;
float M1_Phase_Current_B = 0;
float M1_Phase_Current_C = 0;
float M1_PhaseB_Current = 0;

uint32_t BaseRpm_1,Rpm_conv,BaseRpm_2;

_iq K1 = _IQ(0.95),          // 40khz Offset filter coefficient K1: 0.05/(T+0.05);
    K2 = _IQ(0.05);

typedef struct {
    float As;      // phase A
    float Bs;      // phase B
    float Cs;      // phase C
    float As1;     // phase A test current
    float Bs1;     // phase B test current
    float Cs1;     // phase C test current
} CURRENT_SENSOR;

CURRENT_SENSOR M1_PhaseCurrent,M2_PhaseCurrent;

typedef struct{
    float MotorTemp;
    float BoardTemp;
    float MCUTemp;
    float SiCTemp;
} TEMP_SENSOR;

TEMP_SENSOR M1_Temp_Measurment,Temp_Measurment;

float M1_Temp_SiCTemp;

float M1_Board_SiCTemp;

float TempProtect = 80;
float TempErr = 0;
extern int over_speed_flag;

//*********************** *************************************
// End of Basic Measurement and Compensation
//****************************************************************************

//*********************** Over all control logic *************************************
// All variables and definition related to Over all control logic
//****************************************************************************
uint16_t BackTicker = 0;
uint16_t M1_lsw=0;

uint16_t M1_SMOswitchflg=1;

float ThrottleSpd=0;

uint32_t M1_StartTicker=0;
uint32_t M1_CloseRunTicker;
uint16_t M1_CloseTicker=0;

float M1_sum_ud=0;
float M1_avg_ud=0;
float M1_sum_uq=0;
float M1_avg_uq=0;
uint16_t M1_avgflg=0;

uint16_t M1_startflg=0;
uint16_t M1_lswflg=0;
uint16_t M1_startfail=0;
uint32_t M1_WaitingForStable=0;
uint32_t M1_HighSpeedDelay = 0;
uint16_t M1_HighSpeedFlg =0;
float M1_AutoTestSpeed = 0.3;

extern float pid_id_kp;
extern float pid_id_ki;
extern float pid_spd_kp;
extern float pid_spd_ki;
extern int PID_adjust;
extern int auto_iq;
float auto_iq_ref = 0.0;
float auto_iq_ref_max = 0.1;//0.475
float auto_iq_ref_min = 0.05;
int temp_iq_state = 1; //1-increase,0-decrease
extern float Mea_PTB_1_2_avg;


uint32_t M1_RestartTicker = 0;
uint32_t M1_WarningTicker = 0;
uint16_t M1_Startingflg = 0;
uint16_t M1_StartingTicker = 0;
uint16_t M1_DeCounter = 1;
_iq M1_DCbusVolts =_IQ(0.0);
uint32_t M1_RampControl = 5;
uint32_t M1_iq_RampControl = 5;

uint32_t M1_id_RampControl = 5;//dqset


uint16_t M1_pwmon = 1;
uint16_t M1_pwmsta = 0; //0-stop 1-start
uint16_t M1_sw_smospeed = 0;
uint16_t M1_Algo_control_start = 0;
uint16_t M1_MotorRunState = 0;
uint16_t M1_MotorRunStateReady = 1;
uint16_t M1_InitState = 0;
uint16_t M1_ThroCheck = 0;
uint32_t Stop_count = 0;
uint32_t Stop_count_M2 = 0;


/*Single Phase OC test*/
uint16_t OCM1A=0,OCM1B=0,OCM1C=0;

void Manual_OCM1A();
void Manual_OCM1B();
void Manual_OCM1C();

volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs};
uint16_t dacval = 2048;
uint16_t dacval_ocflag = 0;
#define REFERENCE_VDAC      0
#define REFERENCE_VREF      1
#define DACA         1
#define DACB         2
#define DACC         3
#define REFERENCE            REFERENCE_VREF
#define DAC_NUM                DACA
extern uint16_t OC_flag_ticker;
/*DC voltage vs Max throttle speed*/
_iq M1_DC_Volts = _IQ(0.0);
_iq M1_DC_VoltsSqr = _IQ(0.0);
_iq M1_DC_Volts_Throttle = _IQ(0.0);
_iq M1_Volts_spd_p1 = _IQ(-6.011);
_iq M1_Volts_spd_p2 = _IQ(8.538);
_iq M1_Volts_spd_p3 = _IQ(-2.212);
_iq M1_Volts_spd_p4 = _IQ(-1.408);
_iq M1_Volts_spd_p5 = _IQ(1.234);
_iq M1_Volts_spd_p6 = _IQ(0.09513);
uint16_t M1_DC_spd_Counter = 2000;

int M1_pwm_step = 0;
int M1_freq = 0;

const uint16_t commutationTable[6][3] = {
    {1, 0, 0},  //
    {1, 0, 1},  //
    {0, 0, 1},  //
    {0, 1, 1},  //
    {0, 1, 0},  //
    {1, 1, 0}   //

};


_iq M1_Ud_volts = _IQ(0.0);
_iq M1_Ud_voltsSqr = _IQ(0.0);
_iq M1_Uq_volts = _IQ(0.0);
_iq M1_Uq_voltsSqr = _IQ(0.0);
_iq M1_Vector_Volts = _IQ(0.0);

int M1_Init=0;
int M1_MotorWarningSound=0;

float M1_testf;
uint32_t M1_test32;
uint16_t M1_test16;

//lsw=1 to lsw=5 switching parameters.
uint32_t M1_Tstop=50000;
uint32_t M1_Tswitch=0;
float M1_Kw=0.00002;
float M1_Kswitch=0;


float M1_Injected_Signal = 0.0;
float M1_Vbus_kTs_1 = 0.0;


int M1_DESAT_FLAG = 0;

float M1_iqref_can = 0.0;
float M1_idref_can = 0.0;//dq
//*********************** *************************************
// End of Over all control logic
//****************************************************************************


//*********************** Speed input *************************************
// All variables and definition related to Speed input
//****************************************************************************
// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL_027 M1_rc1 = RMPCNTL_027_DEFAULTS;
RMPCNTL_027 M1_iq_rc1 = RMPCNTL_027_DEFAULTS;

RMPCNTL_027 M1_id_rc1 = RMPCNTL_027_DEFAULTS;//dqset

RMPCNTL_027 M1_spd_rc1 = RMPCNTL_027_DEFAULTS;

RMPCNTL_027 pos_rc1 = RMPCNTL_027_DEFAULTS;
//  Instance a ramp generator to simulate an Anglele
RAMPGEN_027 M1_rg1 = RAMPGEN_027_DEFAULTS;

//*********************** *************************************
// End of Speed input
//****************************************************************************

//*********************** FOC control related *************************************
// All variables and definition related to FOC control related
//****************************************************************************
_iq M1_IdRef = _IQ(0.0);               // Id reference (pu)
_iq M1_IdRef_lock = _IQ(0.05);
//_iq M1_IqRef = _IQ(0.015);              // Iq reference (pu)
_iq M1_IqRef = _IQ(0.000);              // Iq reference (pu)
_iq M1_IqRef_lock = _IQ(0.000);
_iq M1_SpeedRef = _IQ(0.0);          // For Open Loop tests
float M1_PosRef = 0.0;
float M1_lock_init_angle = 0.0;
int lock_flag = 0;
uint16_t M1_PosAngleRef = 0;
_iq M1_SpeedRef1 = _IQ(0.0);
_iq M1_Angle1;
_iq M1_Vref=_IQ(0.0);
_iq M1_Valpha=_IQ(0.0);
_iq M1_Vbeta=_IQ(0.0);
_iq M1_Ualphatest = _IQ(0.08);

//*********************** *************************************
// End of FOC control related
//****************************************************************************

//*********************** FOC fundamental transforms *************************************
// All variables and definition related to FOC transforms
//****************************************************************************
CLARKE_027 M1_clarke1 = CLARKE_027_DEFAULTS;
PARK_027   M1_park1   = PARK_027_DEFAULTS;
IPARK_027  M1_ipark1  = IPARK_027_DEFAULTS;

PHASEVOLTAGE_027 M1_volt1 = PHASEVOLTAGE_027_DEFAULTS;

//*********************** *************************************
// End of FOC transforms
//****************************************************************************

//*********************** PIDs *************************************
// All variables and definition related to PID
//****************************************************************************
PIDREG3_027 M1_pid1_id = PIDREG3_027_DEFAULTS;
PIDREG3_027 M1_pid1_iq = PIDREG3_027_DEFAULTS;
PIDREG3_027 M1_pid1_spd = PIDREG3_027_DEFAULTS;
PIDREG3_027 M1_pid1_pos = PIDREG3_027_DEFAULTS;

PIDREG3_027 pid1_id = PIDREG3_027_DEFAULTS;
PIDREG3_027 pid1_iq = PIDREG3_027_DEFAULTS;
PIDREG3_027 pid1_pos = PIDREG3_027_DEFAULTS;


PIDREG3_027 Alpha_id = PIDREG3_027_DEFAULTS;
PIDREG3_027 Beta_id = PIDREG3_027_DEFAULTS;

//*********************** *************************************
// End of PID
//****************************************************************************

//*********************** SVPWM  *************************************
// All variables and definition related to SVPWM
//****************************************************************************

SVGEN M1_svgen1 = SVGEN_DEFAULTS;
//*********************** *************************************
// End of SVPWM
//****************************************************************************
telemsg_typedef TelemetryMsg;
//*********************** SMO related *************************************
// All variables and definition related to SMO
//****************************************************************************
#define N_POINTS_PHASE_COMP 16
#define OMEGA_COMP_START _IQ(0.1)

float Id_Integ_1=0.0;
float Id_Integ_2=0.0;
float Angle_err=0.0;
float Angle_com=0.0;

int TZ_flag = 0;
float M1_Uvector=0.0;

_iq M1_SMOangle1=_IQ(0.0);
_iq M1_SMOangle2=_IQ(0.0);

_iq M1_SMOangle_adjust=_IQ(0.0);
_iq M1_SMOangle_manualadjust = _IQ(0.0);
_iq M1_SMOangle_autoadjust = _IQ(0.0);
_iq M1_SMO_F=_IQ(0.99);
_iq M1_SMO_G=_IQ(0.21);

_iq M1_SMOangle_error=_IQ(0.0);
_iq M1_SMOspeed=_IQ(0.0);
_iq M1_p1 = _IQ(0.5);
_iq M1_p2 = _IQ(-0.9184);
_iq M1_p3 = _IQ(-0.0008871);
_iq M1_p4 = _IQ(-0.01524);   //_IQ(0.03262) at 40k ISR


uint32_t SMOshift = 0;
float M1_SMO_BASE_CURRENT = 114;
float M1_SMO_BASE_VOLTAGE = 49;

SMOPOS_027 M1_smo1 = SMOPOS_027_DEFAULTS;

SMOPOS_CONST_027 M1_smo1_const = SMOPOS_CONST_027_DEFAULTS;

const _iq M1_phase_comp[N_POINTS_PHASE_COMP]={_IQ(-0.11), _IQ(-0.115), _IQ(-0.12), _IQ(-0.125), _IQ(-0.13), _IQ(-0.135), _IQ(-0.14), _IQ(-0.145), _IQ(-0.15), _IQ(-0.155), _IQ(-0.16), _IQ(-0.17), _IQ(-0.18), _IQ(-0.19), _IQ(-0.2), _IQ(-0.21)};

int M1_index1;

int M1_Error_Pos=0,M1_Error_Spd;

ESMOPOS  M1_esmo1 = ESMOPOS_DEFAULTS;
_iq  M1_startCnt,
M1_smoK_set = _IQ(0.3);

PI_CONTROLLER  M1_pi_smo = PI_CONTROLLER_DEFAULTS;

// Instance an inverter PWM dead band compensator
//DBC dbc1 = DBC_DEFAULTS;
//*********************** *************************************
// End of SMO
//****************************************************************************

//*********************** Speed Estimate *************************************
// All variables and definition related to Speed Estimate
//****************************************************************************
// Instance a speed calculator based on Encoder position not use
SPEED_MEAS_QEP_027 M1_speed1 = SPEED_MEAS_QEP_027_DEFAULTS;

SPEED_ESTIMATION_027 M1_speed3 = SPEED_ESTIMATION_027_DEFAULTS;

float M1_EstimatedSpeed;

//*********************** *************************************
// End of Speed Estimate
//****************************************************************************



//*********************** Protection and Misc *************************************
// All variables and definition related to Protection and Misc
//****************************************************************************
int32_t Max_SpeedRPM=0;
int32_t Max_StableRPM=0;
int32_t Min_StableRPM=0;

int OC_MaxspeedRPM = 0;
int OC_MinspeedRPM = 0;
int OC_risetime = 0;
int OC_risetimeflag = 0;
int32_t OC_riseticker1 = 0;
int32_t OC_riseticker2 = 0;

int StableRPM_flag=0;
int Risetime_flag=1;
int Risetime_flag1=1;
_iq M1_SpeedRef_Goal=0.0;
int32_t Initial_Isrticker=0;
int32_t Final_Isrticker=0;
int32_t Delta_Isrticker=0;
_iq Max_voltage;
_iq Min_voltage;
_iq Max_current;
_iq M1_anlge_comp=_IQ(0);

_iq M1_anlge_comp_value=_IQ(0.2);
_iq test_Ud_1=_IQ(0.0);
_iq last_test_Ud_1=_IQ(0.0);
_iq last_Ud_in_1=_IQ(0.0);
_iq test_Ud_2=_IQ(0.0);
_iq last_test_Ud_2=_IQ(0.0);
_iq last_Ud_in_2=_IQ(0.0);
float k_Ud=0.8;
int curcount = 5;
int M1_overvoltage=0;
int M1_undervoltage=0;
int M1_overcurrent=0;
int M1_overcurrent_A=0;
uint16_t M1_overcurrent_A_count=0;
uint16_t M1_overcurrent_A_flag=0;
int M1_overcurrent_B=0;
uint16_t M1_overcurrent_B_count=0;
uint16_t M1_overcurrent_B_flag=0;
int M1_overcurrent_C=0;
uint16_t M1_overcurrent_C_count=0;
uint16_t M1_overcurrent_C_flag=0;
int M1_underspeed=0;
int M1_overtemperature=0;
int M1_OvercurrentView=0;
int M1_OvercurrentView_A=0;
int M1_OvercurrentView_B=0;
int M1_OvercurrentView_C=0;
int M1_OvervoltageView=0;
int M1_UndervoltageView=0;
int M1_countvoltage=0;

int M1_Overtempticker=0;

int M1_Overcurticker=0;

int M1_FTticker=0;

int M1_undervoltage_flag = 0;

uint16_t M1_undervoltage_count = 0;

int Max_DC_voltage = 850;
int Over_DC_voltage =0;
int Over_DC_voltage_ticker =0;
//*********************** *************************************
// End of Protection and Misc
//****************************************************************************

int16_t OffsetCalCounter;
int16_t OffsetCalCounter1;

float M1_curLimit = 8.0;
float M2_curLimit = 8.0;

// ****************************************************************************
// Flag variables
// ****************************************************************************
volatile uint16_t EnableFlag = TRUE;

int LedCnt1=0;

uint16_t SpeedLoopPrescaler = 10,      // Speed loop pre scalar
       M1_SpeedLoopCount = 1,           // Speed loop counter
       M2_SpeedLoopCount = 1;           // Speed loop counter


RMPCNTL M1_rc2 = RMPCNTL_DEFAULTS; // for speed


float m1_comp=0.0;
float VinA,VinB;
float Q1_Raw_PU[3],Q2_Raw_PU[3];
int Q1_Raw[3],Q2_Raw[3];
int Diff_BusV_Temp;
float real_DCBUS_voltage;
float real_voltage_high = 0.0;
float real_voltage_low = 600.0;
uint16_t M1_overtemp, M2_overtemp = 0;
uint16_t Test_Delay=0;
float Temp_board = 0.0;
uint16_t Temp_board_ticker = 0;
uint16_t Temp_board_compare = 20000;
float M1_Temp_Sensor_out_mV = 0.0;
float pid1_kpmax = 1.5;
float pid1_kimax = 0.2;
extern int NTC_MAX_TEMP;
extern float NTC_SIC_avg_filter;
extern float NTC_SIC_avg_filter1,NTC_SIC_M1A,NTC_SIC_M1B,NTC_SIC_M1C;
extern float NTC_SIC_avg_filter1_ECAP,NTC_SIC_M1A_ECAP,NTC_SIC_M1B_ECAP,NTC_SIC_M1C_ECAP;
int over_temp_ticker = 0;
int over_avg_temp_flag = 0;
float SIC_MAX_TEMP = 95.0;

void board_temp(void)
{
    M1_Temp_Sensor_out_mV = M1_Temp_Board_Raw*ADC_RES2mVOL;

    Temp_board=((13.582-sqrt((-13.582)*(-13.582)+4*0.00433*(2230.8-M1_Temp_Sensor_out_mV)))/(2*(-0.00433)))+30;
}


float analog1=0.0;   /* ISR 每周期更新，供电压油门门控 v_gate（与主循环映射采样可不同拍） */
float analog2=0.0;
float analog3=0.0;

/* 电压油门目标转速[rpm]：仅 VoltThrottle_Update 写入，ISR 门控读；跨上下文须 volatile */
volatile float analog_cmd_rpm = 0.0f;
uint16_t throttle_sw_on = 0u;            /* GPIO63 开关状态：1=使能（低有效），ISR 每周期刷新 */
uint16_t throttle_sw_off_cnt = 0u;       /* GPIO63 无效侧滤波计数：高电平持续计数 */

uint16_t en_OC_detect=0,OC_detect_ready=0;
uint16_t DTC_code1 = 0x0000;
uint16_t DTC_code2 = 0x0000;
uint16_t DTC_code3 = 0x0000;
float adc_calib = 1.0;
int voltage_err=0;
float RES_ALIGN_ANGLE = 0.115;//对角度
float M1_real_voltage,M2_real_voltage = 0.0;
float Low_Voltage=0.0;
extern int motor_overtemp_flag;

/*------------------------------------------------------------------------------
 电压油门映射（主循环调用，约 100Hz）：0.5~4.5V 线性 0~2600rpm；断线/恢复 50ms 确认。
 发波门控仍在 MotorControlISR，见 analog1 与 analog_cmd_rpm。
------------------------------------------------------------------------------*/
void VoltThrottle_Update(void)
{
    float v;
    float v_lo;
    float v_hi;
    float span;
    float t;
    float rpm_mapped;
    float v_throttle;

    if (throttle_type != VOLT_THROTTLE)
        return;

    /* 与 ISR 中 analog1 同一换算，此处独立采样供映射（减轻 ISR 负担） */
    v_throttle = AdcdResultRegs.ADCRESULT3 * 3.3f * ADC_PU_SCALE_FACTOR / 1.6f * 2.5f;
    v = v_throttle;
    if (v > VOLT_THROTTLE_V_MAX)
        v = VOLT_THROTTLE_V_MAX;
    v_lo = VOLT_THROTTLE_LINEAR_LO;
    v_hi = VOLT_THROTTLE_LINEAR_HI;

    /* 纯映射：<=0.5V 怠速 130rpm；0.5~5V 线性 130~2400rpm；>=5V 钳位 2400rpm */
    if (v <= v_lo)
        rpm_mapped = VOLT_THROTTLE_IDLE_RPM;
    else if (v >= v_hi)
        rpm_mapped = VOLT_THROTTLE_MAX_RPM;
    else
    {
        span = v_hi - v_lo;
        t = (v - v_lo) / span;
        rpm_mapped = VOLT_THROTTLE_IDLE_RPM + t * (VOLT_THROTTLE_MAX_RPM - VOLT_THROTTLE_IDLE_RPM);
    }
    analog_cmd_rpm = rpm_mapped;
}

void Current_and_voltage_measurement(void)
{
    /* analog1：ISR 刷新，供同 ISR 内电压油门门控（滞回/协同关断），非映射主路径 */
    analog1=AdcdResultRegs.ADCRESULT3*3.3*ADC_PU_SCALE_FACTOR/1.6*2.5;
    analog2=AdcdResultRegs.ADCRESULT0*3.3*ADC_PU_SCALE_FACTOR/1.6*2.5;
    analog3=AdccResultRegs.ADCRESULT2*3.3*ADC_PU_SCALE_FACTOR/1.6*2.5;

    M1_PhaseCurrent.As=(offset_M1_PhaseA - (((float)Cur_M1_PhaseA_Raw)*ADC_PU_SCALE_FACTOR))*Cur_ADC_PU_SCALE_FACTOR*adc_calib;
    M1_PhaseCurrent.Bs=(offset_M1_PhaseB - (((float)Cur_M1_PhaseB_Raw)*ADC_PU_SCALE_FACTOR))*Cur_ADC_PU_SCALE_FACTOR*adc_calib;
    M1_PhaseCurrent.Cs=-M1_PhaseCurrent.As-M1_PhaseCurrent.Bs;

    M1_clarke1.As=M1_PhaseCurrent.As*1.0;
    M1_clarke1.Bs=M1_PhaseCurrent.Bs*1.0;

    M1_volt1.DcBusVolt=M1_Volt_DCBUS*ADC_PU_SCALE_FACTOR*Volt_ADC_PU_SCALE_FACTOR*BASE_VOLTAGE/1.6;

    Low_Voltage=AdcdResultRegs.ADCRESULT2*3.3*ADC_PU_SCALE_FACTOR*105.1/5.1;

    M1_real_voltage = M1_volt1.DcBusVolt;

    if(M1_real_voltage>Max_DC_voltage)
    {
        Over_DC_voltage_ticker++;
    }

    if(Over_DC_voltage_ticker>=3)
    {
        Over_DC_voltage = 1;
        DTC_code1 |=0x01;
    }


#ifndef TEST_MODE


    if(((M1_PhaseCurrent.As)>Max_current)||((M1_PhaseCurrent.As)<-Max_current))
    {
        M1_overcurrent_A_count++;
    }
    else M1_overcurrent_A_count=0;
    if(((M1_PhaseCurrent.Bs)>Max_current)||((M1_PhaseCurrent.Bs)<-Max_current))
    {
        M1_overcurrent_B_count++;
    }
    else M1_overcurrent_B_count=0;
    if(((M1_PhaseCurrent.Cs)>Max_current)||((M1_PhaseCurrent.Cs)<-Max_current))
    {
        M1_overcurrent_C_count++;
    }
    else M1_overcurrent_C_count=0;

    if(M1_overcurrent_A_count>= curcount)
    {
        M1_overcurrent_A=1;
        M1_overcurrent_A_count=0;
    }
    if(M1_overcurrent_B_count>= curcount)
    {
        M1_overcurrent_B=1;
        M1_overcurrent_B_count=0;
    }
    if(M1_overcurrent_C_count>= curcount)
    {
        M1_overcurrent_C=1;
        M1_overcurrent_C_count=0;
    }


    if(M1_overcurrent_A==1)
    {

        M1_overcurrent=1;
        M1_OvercurrentView_A=1;

    }
    if(M1_overcurrent_B==1)
    {
        M1_overcurrent=1;
        M1_OvercurrentView_B=1;

    }
    if(M1_overcurrent_C==1)
    {
        M1_overcurrent=1;
        M1_OvercurrentView_C=1;

    }

    if(M1_overcurrent==1)
    {
        DTC_code1 |=0x02;
    }

    if ((Over_DC_voltage==1)||(M1_undervoltage==1)||(M1_overcurrent==1)||(over_speed_flag==1)||(over_avg_temp_flag==1)||(motor_overtemp_flag==1))
    {
        M1_StopMotor();

        OC_Ticker2 = IsrTicker;
        M1_overcurrent = 0;
        M1_overcurrent_A = 0;
        M1_overcurrent_B = 0;
        M1_overcurrent_C = 0;
        M1_Overcurticker++;
    }


#endif

}

//*****************************************************************************
//*****************************************************************************

uint64_t task_50hz_systick = 0;
uint64_t led_tick=0;
uint64_t t_time=0;
float Iderr = 0.0;
float PWM_TEST = 1250.0;
int RE_M1=0;
float M1_Ud,M1_Uq;
float M_Ud = 0.0;
float M_Uq = 0.0;
int M1_M2_FLAG = 0;
int EEPROM_flag = 1;
int EEPROM_read = 0;
int EEPROM_write = 1;
int CAN_M1_Iq_OUT = 100;
int CAN_M1_Id_OUT = 100;
int CAN_M1_Iq_ref = 6000;
int CAN_M1_Id_ref = 6000;
int CAN_M1_Iq_fdb = 6000;
int CAN_M1_Id_fdb = 6000;

int CAN_M2_Iq_OUT = 0;
int CAN_M2_Id_OUT = 0;
int CAN_M1_Speed_OUT = 0;
int CAN_M1_Ubeta = 0;
int CAN_M1_Ualfa = 0;
extern int CAN_iqref;
extern int CAN_idref;//dqset
float Lock_angle = 0.0;
float Lock_angle_resolver = 0.0;
uint16_t DTC_test2 = 0x0000;
uint16_t DTC_test1 = 0x00;
uint16_t DTC_read2 = 0x0000;
uint16_t DTC_read1 = 0x00;
uint16_t DTC_count = 0;
uint16_t DTC_count1 = 0;
int DTC_write = 0;
int cut_off_m2 = 0;
int TripZone_flag = 0;
float speed_out_iq = 0.0;
//*****************************************************************************
//*****************************************************************************
extern void CANAIntHandler();
extern void CANBIntHandler();
extern uint16_t speed_set;
extern uint16_t control_command;
extern uint16_t clear_fault;
extern uint16_t lock_propeller;

uint16_t i2cb;

extern struct I2CMSG I2cMsgOut1;                  // Msg Byte 2

extern struct I2CMSG I2cMsgIn1;

extern struct I2CMSG *CurrentMsgPtr;


float DC_15V=0.0;
uint16_t NTC_loop = 0;
int WWD_ticker = 0;
uint16_t start_reg1 = 0;
uint16_t start_reg = 0;
uint16_t start_reg2 = 0;
uint16_t WWD_err_ticker = 0;
uint16_t WWD_err_ticker7 = 0;
int normal_flag = 0;

void main(void)
{
    volatile int16_t temp;


    //  Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F28M3Xx_SysCtrl.c file.
    InitSysCtrl();

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the compiler

#ifdef _FLASH
    // Copy time critical code and Flash setup code to RAM
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (uint32_t)&RamfuncsLoadSize);
#endif

#ifdef _FLASH
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    InitFlash();    // Call the flash wrapper init function
#endif //(FLASH)

    // Waiting for enable flag set
    while (EnableFlag == FALSE)
    {
        BackTicker++;
    }

    // Clear all interrupts and initialize PIE vector table:

    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F28M3Xx_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F28M3Xx_DefaultIsr.c.
    // This function is found in F28M3Xx_PieVect.c.
    InitPieVectTable();

    CLA_configClaMemory();
    CLA_initCpu1Cla1();

    EALLOW;
    GPIO_SetupPinOptions(43, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(43,0,0);
    GpioDataRegs.GPBCLEAR.bit.GPIO43=1;

    DELAY_US(100000);

    GPIO_SetupPinOptions(50, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(50,0,0);
    GpioDataRegs.GPBCLEAR.bit.GPIO50=1;
    GpioDataRegs.GPBSET.bit.GPIO50 = 1;
    EDIS;

    InitSystick();

    init_ERR_Timer();

    TLF35584_spi_init();

    TLF35584_DisableERRandWWD();

    TLF35584_GoToNormal();

    start_reg2 = TLF35584_ReadReg(0x27);

    // Timing sync for background loops
    // Timer period definitions found in device specific PeripheralHeaderIncludes.h
    //    CpuTimer0Regs.PRD.all =  10000;     // A tasks
        CpuTimer1Regs.PRD.all =  20000;     // B tasks
    //    CpuTimer2Regs.PRD.all =  30000;     // C tasks

    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;
    C_Task_Ptr = &C1;


    // ****************************************************************************
    // ****************************************************************************
    //TODO PWM Configuration
    // ****************************************************************************
    // ****************************************************************************

    // Initialize PWM module
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;


    Fault_GPIO_init();
    Configure_PWM();

    // ****************************************************************************
    // ****************************************************************************
    //TODO ADC Configuration
    // ****************************************************************************
    // ****************************************************************************
    ConfigureADC();

    Parameter_Init();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    offset_M1_PhaseA  = 0;
    offset_M1_PhaseB  = 0;
    offset_M1_PhaseC  = 0;

    DELAY_US(100000);
    for (OffsetCalCounter=0; OffsetCalCounter<30000; )
    {

        if(OffsetCalCounter>5000)
        {
            offset_M1_PhaseA  = K1*offset_M1_PhaseA + K2*(Cur_M1_PhaseA_Raw)*ADC_PU_SCALE_FACTOR;
            offset_M1_PhaseB  = K1*offset_M1_PhaseB + K2*(Cur_M1_PhaseB_Raw)*ADC_PU_SCALE_FACTOR;
            offset_M1_PhaseC  = K1*offset_M1_PhaseC + K2*(Cur_M1_PhaseC_Raw)*ADC_PU_SCALE_FACTOR;
        }
        OffsetCalCounter++;
    }

    OCFault_diag_Init();
    CANA_Initialzation();
    CANB_Initialzation();
    InitI2C();

    InitTZGpio();
    InitFaultGpio();
    InitDigitalGPIO();  /* GPIO63 油门使能开关初始化 */

    SPI_init();
    SPI_fifo_init();

    CANA_Ready();
    CANB_Ready();

    DELAY_US(3000000);

    AD2SB_Regs_Init();

    DELAY_US(3000000);

    Cla_GPIO_Init();

    start_reg = TLF35584_ReadReg(0x27);

    EALLOW;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;   // INT on PRD event
    EPwm1Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event

    PieVectTable.EPWM1_INT = &MotorControlISR;
    PieVectTable.I2CB_INT = &i2c_int1b_isr;

    for (i2cb = 0; i2cb < I2C_MAX_BUFFER_SIZE; i2cb++)
    {
        I2cMsgIn1.MsgBuffer[i2cb] = 0x0000;
    }

    PieCtrlRegs.PIEIER8.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // Enable PWM3INT in PIE group 3

    PieVectTable.CANA0_INT = &CANAIntHandler;
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;
    PieVectTable.CANB0_INT = &CANBIntHandler;
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;

    EPwm1Regs.ETCLR.bit.INT=1;

    PieVectTable.EPWM1_TZ_INT = &epwm1_tzint_isr;
    PieVectTable.EPWM2_TZ_INT = &epwm2_tzint_isr;
    PieVectTable.EPWM3_TZ_INT = &epwm3_tzint_isr;

    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx3 = 1;


    IER |= M_INT3; // Enable group 3 interrupts
    IER |= M_INT1; // Enable group 1 interrupts
    IER |= M_INT8; // Enable group 4 interrupts
    IER |= M_INT9;   // M_INT9
    EINT;          // Enable Global interrupt INTM
    ERTM;          // Enable Global realtime interrupt DBGM

    EDIS;
    DELAY_US(30000);
    Fault_GPIO_Stateinit();

    DELAY_US(3000000);

    CANBMsgSend();

    TLF35584_ConfigWWD(0x00, 0x03);
    TLF35584_EnableWWDandERR();
    TLF35584_flag_error_clear();
    GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
    TLF35584_Trigger_WWD();

    for(;;)  //infinite loop
    {
        t_time = micros();

        NTC_loop++;
        if(NTC_loop==20)
        {
            NTC_temp();
            NTC_loop=0;
        }

        WWD_ticker++;
        if(WWD_ticker==7)
        {
            TLF35584_Trigger_WWD();
            WWD_ticker = 0;
            WWD_err_ticker7 = (TLF35584_ReadReg(0x29)>> 1) & 0xFF;
        }

        PT100_temp();

        DC_15V=AdcdResultRegs.ADCRESULT2*3.3*ADC_PU_SCALE_FACTOR*11.0;


        //write
        if(I2cMsgOut1.MsgStatus == I2C_MSGSTAT_SEND_WITHSTOP)
        {
            i2cError = I2CB_WriteData(&I2cMsgOut1);


            //
            if (i2cError == I2C_SUCCESS)
            {
                CurrentMsgPtr = &I2cMsgOut1;
                I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;
            }
        }
        //read
        if (I2cMsgOut1.MsgStatus == I2C_MSGSTAT_INACTIVE)
        {
            //
            // Check incoming message status.
            //
            if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
            {
                //
                // EEPROM address setup portion
                //
                while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
                {

                }

                CurrentMsgPtr = &I2cMsgIn1;
                I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
            }
            //
            // Once message has progressed past setting up the internal address
            // of the EEPROM, send a restart to read the data bytes from the
            // EEPROM. Complete the communique with a stop bit. MsgStatus is
            // updated in the __interrupt service routine.
            //
            else if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_RESTART)
            {
                //
                // Read data portion
                //
                while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
                {

                }

                CurrentMsgPtr = &I2cMsgIn1;
                I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
            }
        }


        CANAMsgGet();
        CANAMsgSend();

        DELAY_US(1000 * 10);
        /* 电压油门：映射与断线逻辑，约 100Hz（与 DELAY 10ms 一致） */
        VoltThrottle_Update();

        TempSend();
        DataSend();
        DTCSend();

        PT100send();


        if ( micros() > (task_50hz_systick + 20 MS))
        {
            task_50hz_systick = micros();
        }

        if( micros() > (led_tick + 500000 ))
        {
            led_tick=micros();
        }
        // State machine entry & exit point
        //===========================================================
        //(*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
        //===========================================================
    }
} //END MAIN CODE

/******************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 */

//=================================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{

}

void B0(void)
{

}

void C0(void)
{

}


uint32_t temp_ticker = 10000;

float M1_speed_resolver =0.0;

uint16_t sin_enable=0;

// ****************************************************************************
// ****************************************************************************
//TODO Motor Control ISR
// ****************************************************************************
// ****************************************************************************
__interrupt void MotorControlISR(void)
{

    if(normal_flag == 1)
    {
        TLF35584_GoToNormal();
        normal_flag = 0;
        start_reg1 = TLF35584_ReadReg(0x27);
    }

    EINT;

    // Verifying the ISR
    IsrTicker++;
    if(((EPwm1Regs.TZFLG.bit.OST))==1)
    {
        TripZone_flag = 1;
    }
    else
    {
        TripZone_flag = 0;
    }
    if(GpioDataRegs.GPADAT.bit.GPIO10 == 0)//U_Dr_Fault_C（低有效）
    {
        DTC_code2 |=0X01;
    }

    if(GpioDataRegs.GPCDAT.bit.GPIO94 == 0)//V_Dr_Fault_C
    {
        DTC_code2 |=0X02;
    }

    if(GpioDataRegs.GPCDAT.bit.GPIO93 == 0)//W_Dr_Fault_C
    {
        DTC_code2 |=0X04;
    }

    if(GpioDataRegs.GPCDAT.bit.GPIO92 == 0)//Faulut_S1   trip zone
    {
        DTC_code2 |=0X08;
    }

    if(GpioDataRegs.GPCDAT.bit.GPIO90 == 0)//U_Dr_Ready_C
    {
        DTC_code2 |=0X10;
    }

    if(GpioDataRegs.GPCDAT.bit.GPIO85 == 0)//V_Dr_Ready_C
    {
        DTC_code2 |=0X20;
    }

    if(GpioDataRegs.GPCDAT.bit.GPIO86 == 0)//W_Dr_Ready_C
    {
        DTC_code2 |=0X40;
    }

    if(GpioDataRegs.GPCDAT.bit.GPIO88 == 0)//Ready_S1
    {
        DTC_code2 |=0X80;
    }


//    if(GpioDataRegs.GPCDAT.bit.GPIO77 == 0)//U1_12V_Fault（V2.0 由 GPIO 输出驱动，DTC 不检测）
//    {
//        DTC_code3 |=0X1;
//    }
//    if(GpioDataRegs.GPCDAT.bit.GPIO73 == 0)//V1_12V_Fault
//    {
//        DTC_code3 |=0X2;
//    }
//    if(GpioDataRegs.GPCDAT.bit.GPIO76 == 0)//W1_12V_Fault
//    {
//        DTC_code3 |=0X4;
//    }
    if(GpioDataRegs.GPBDAT.bit.GPIO48 == 0)//Fault_DC
    {
        DTC_code3 |=0X8;
    }
    if(GpioDataRegs.GPBDAT.bit.GPIO53 == 0)//Fault_5V_A1
    {
        DTC_code3 |=0X10;
    }
//    if(GpioDataRegs.GPBDAT.bit.GPIO52 == 0)//Fault_5V_A2
//    {
//        DTC_code3 |=0X20;
//    }
//    if(GpioDataRegs.GPBDAT.bit.GPIO51 == 0)//Fault_5V_A3
//    {
//        DTC_code3 |=0X40;
//    }


    /* CAN 油门：需 clear_fault 与 MCU_Command；电压油门不走本段 */
    if(throttle_type == CAN_THROTTLE && clear_fault == 1)
    {
        if(lock_propeller==1)
        {
            M1_Algo_control_start = 1;
        }
        else
        {
            M1_Algo_control_start = 0;
            M1_lsw = 0;
        }
        if(control_command == 0)
        {
            M1_lsw = 0;
        }
        else if(control_command == 2)
        {
            M1_lsw = 2;
            if(auto_iq==1)
            {
                M1_IqRef = auto_iq_ref;
            }
            else
            {
                M1_IqRef = (float)speed_set/4000.0;
                M1_iq_RampControl = 5;
            }
        }
        else if(control_command == 1)
        {
            M1_lsw = 1;
            M1_SpeedRef = (float)speed_set/M1_speed1.BaseRpm;
            M1_IqRef = (float)CAN_iqref/200.0;
            M1_iq_RampControl = 5;
            M1_IdRef = 0;
            M1_id_RampControl = 5;//dqset

        }
        else if(control_command == 3)
        {
            M1_lsw = 3;
            M1_SpeedRef = (float)speed_set/M1_speed1.BaseRpm;
        }

        if(PID_adjust == 1)
        {
            M1_pid1_spd.Kp = pid_spd_kp;
            M1_pid1_spd.Ki = pid_spd_ki;
            pid1_id.Kp = pid_id_kp;
            pid1_id.Ki = pid_id_ki;
        }
    }


    if ((M1_Algo_control_start == 0) && (M1_Init == 0))
    {
        M1_AlgoInitStart();
    }


    if(M1_Algo_control_start == 0)
    {

        Eangle_resolverB_angle16 = _IQ15frac(Cla_angleB);
        Eangle_resolverB1 = _IQ15frac(1.0+Eangle_resolverB_angle16+RES_ALIGN_ANGLE);

        if((Eangle_resolverB1>=_IQ15(0.0))&&(Eangle_resolverB1<=_IQ15(1.0)))
        {
            Eangle_resolverB=Eangle_resolverB1;

            Eangle_resolver = Eangle_resolverB;
        }

        M1_speed1.ElecTheta = Eangle_resolver;

        if(resolver_1st ==1){

            M1_speed1.OldElecTheta = Eangle_resolver;
            resolver_1st = 0;

        }

        SPEED_FR_MACRO_027(M1_speed1)
        M1_speed_resolver = M1_speed1.SpeedRpm;
    }

    if(RE_M1 == 1)
    {
        M1_pwmon = 0;
        SET_U1_RI;
        SET_V1_RI;
        SET_W1_RI;
        DELAY_US(10);
        CLR_U1_RI;
        CLR_V1_RI;
        CLR_W1_RI;
    }

    Current_and_voltage_measurement();

    /* 电压油门门控（20kHz）：analog1 实时电压；analog_cmd_rpm 为主循环 VoltThrottle_Update 给出的目标[rpm] */
    if (throttle_type == VOLT_THROTTLE)
    {
        uint16_t sw_raw = GpioDataRegs.GPBDAT.bit.GPIO63;
        uint16_t throttle_sw_raw_on = (sw_raw == 0u);  /* 原始开关状态：低有效 */

        if (throttle_sw_raw_on)
        {
            /* 有效侧不滤波：检测到低电平后立刻认为开关有效 */
            throttle_sw_on = 1u;
            throttle_sw_off_cnt = 0u;
        }
        else
        {
            /* 无效侧滤波：高电平需持续 THROTTLE_SW_OFF_FILTER_TICKS 才判为真正断开 */
            if (throttle_sw_off_cnt < THROTTLE_SW_OFF_FILTER_TICKS)
                throttle_sw_off_cnt++;
            else
                throttle_sw_on = 0u;
        }

        if (throttle_sw_on)
        {
            M1_Algo_control_start = 1;
            M1_lsw = 3;
            /* analog_cmd_rpm 由主循环 VoltThrottle_Update() 以 ~100Hz 更新 */
            M1_SpeedRef = (float)(analog_cmd_rpm / M1_speed1.BaseRpm);
        }
        else
        {
            M1_Algo_control_start = 0;
            M1_lsw = 0;
            M1_SpeedRef = 0.0f;
        }
    }

    if(Temp_board_ticker>Temp_board_compare)
    {
        board_temp();
        Temp_board_ticker=0;
    }
    else
    {
        Temp_board_ticker++;
    }

    M1_DESAT_FLAG = GpioDataRegs.GPCDAT.bit.GPIO92;

    // =============================== LEVEL 5 ======================================
    //  Level 5 verifies the position control
    //  Position references generated locally
    //  lsw=0: lock the rotor of the motor
    //  lsw=1: - needed only with QEP encoders until first index pulse
    //         - Loops shown for lsw=5 are closed in this stage
    //  lsw=5: close all loops, position, speed and currents Id, Iq
    // ==============================================================================

    //TODO INCRBUILD 5
#if (BUILDLEVEL==LEVEL5)

    // ------------------------------------------------------------------------------
    //  Connect inputs of the RMP module and call the ramp control macro
    // ------------------------------------------------------------------------------

    // ----------------------------------------------------------------------------------------
    //  Connect throttle signal and start automatically, SAFE ZONE reset Protection Error
    // ----------------------------------------------------------------------------------------
#define TEST_MODE
#ifndef TEST_MODE


    if (MotorRunStateReady==1 && TimeCap < (MinThrot+40))
    {
        overvoltage=0;
        undervoltage=0;
        overcurrent=0;
        overtemperature=0;
        StopMotor();
        Startingflg=0;
        StartingTicker=0;
        rc1.RampDelayMax = 80;
        rg1.Angle = _IQ(0);
        ThrottleSpd = 0;
        lsw = 0;
        pid1_id.Out = 0;
        pid1_iq.Out = 0;
        underspeed=0;


        //Reset speed3 parameters
        speed3.EstimatedSpeed=0;
        speed3.OldEstimatedTheta=0;
        speed3.EstimatedTheta=0;

        MotorRunState=1;                        //Motor initialize to start
    }

    if (TimeCap > (MinThrot+98) && MotorRunState==1)
    {
        rc1.SetpointValue=0;                    //Speed Ramp always start from 0 during starting.
        ClearToStartMotor();
        Algo_control_start=1;

        MotorRunState=2;                        //Motor running

    }

    if (MotorRunState==2)
    {
        lswticker++;
        if(lswticker>6000)
        {
            lsw = 1;
            lswticker = 0;
            MotorRunState = 3;
        }
    }

#endif

    if (M1_Algo_control_start==1)
    {

        pos_rc1.TargetValue = M1_PosRef;

        if(M1_lsw==0)
        {
            M1_rc1.TargetValue = 0;
            M1_iq_rc1.TargetValue = 0;
            M1_StartTicker=0;
            M1_startflg=0;
        }
        else if(M1_lsw==1)
        {
            M1_rc1.TargetValue = M1_SpeedRef;
            M1_rc1.RampDelayMax = 20;

            M1_iq_rc1.TargetValue = M1_IqRef;
            M1_iq_rc1.RampDelayMax = M1_iq_RampControl;
            M1_id_rc1.TargetValue = M1_IdRef;//dqset
            M1_id_rc1.RampDelayMax = M1_id_RampControl;//dqset
        }
        else if (M1_lsw == 3|| M1_lsw == 8|| M1_lsw == 11 || M1_lsw == 12)
        {
            M1_rc1.TargetValue = M1_SpeedRef;
            M1_rc1.RampDelayMax = M1_RampControl;
            M1_rc1.RampStep = 20;
        }
        else if (M1_lsw == 2)
        {
            M1_iq_rc1.TargetValue = M1_IqRef;
            M1_iq_rc1.RampDelayMax = M1_iq_RampControl;
            M1_iqref_can = M1_IqRef;
            M1_rc1.RampStep = 1;
        }
        else if(M1_lsw==5|| M1_lsw == 4)
        {
            M1_rc1.TargetValue = M1_SpeedRef;
            M1_rc1.RampDelayMax = 10;
        }

        else if (M1_lsw==7)
        {
            M1_rc1.TargetValue = M1_SpeedRef;
        }
        else if (M1_lsw==9)
        {
            M1_rc1.TargetValue = M1_IqRef;
            M1_rc1.RampDelayMax = M1_RampControl;
        }
        else M1_rc1.TargetValue = M1_Ipulse;


        RC_MACRO_027(M1_rc1)
        RC_MACRO_027(M1_iq_rc1)
        RC_MACRO_027(M1_id_rc1)//dqset

        // ------------------------------------------------------------------------------
        //  Connect inputs of the RAMP GEN module and call the ramp generator macro
        // ------------------------------------------------------------------------------
        M1_rg1.Freq = M1_rc1.SetpointValue;
        RG_MACRO_027(M1_rg1)


        // ------------------------------------------------------------------------------
        //  Connect inputs of the CLARKE module and call the clarke transformation macro
        // ------------------------------------------------------------------------------
        CLARKE_MACRO_027(M1_clarke1)

        // ------------------------------------------------------------------------------
        //  Connect inputs of the PARK module and call the park trans. macro
        // ------------------------------------------------------------------------------

        M1_park1.Alpha = M1_clarke1.Alpha;
        M1_park1.Beta = M1_clarke1.Beta;

        if(angle_channel == SPIB_channel)
        {

            Eangle_resolverB_angle16 = _IQ15frac(Cla_angleB);
            Eangle_resolverB1 = _IQ15frac(1.0+Eangle_resolverB_angle16+RES_ALIGN_ANGLE);
            if((Eangle_resolverB1>=_IQ15(0.0))&&(Eangle_resolverB1<=_IQ15(1.0)))
            {
                Eangle_resolverB=Eangle_resolverB1;

                Eangle_resolver = Eangle_resolverB;
            }

        }
        else if(angle_channel == SPIC_channel)
        {
            Eangle_resolver = Eangle_resolverC;
        }

        Eangle_resolver_err = Eangle_resolverB - M1_rg1.Out;

        if(M1_lsw==0)
        {
            M1_Angle1 = 0;
        }
        else if (M1_lsw==1||M1_lsw==5|| M1_lsw == 4)
        {
            M1_Angle1 = M1_rg1.Out;
        }

        else if (M1_lsw == 3 || M1_lsw == 2|| M1_lsw == 12)
        {
            M1_Angle1 =Eangle_resolver;
        }
        else if (M1_lsw==7||M1_lsw==9)
        {
            M1_Angle1 = M1_SMOangle2;
        }
        else if (M1_lsw==8)
        {
            M1_Angle1 = Lock_angle;
        }
        else if(M1_lsw==11)
        {
            M1_Angle1 = Eangle_resolver+Lock_angle_resolver;
        }


        M1_park1.Angle=M1_Angle1;

        M1_park1.Sine = _IQsinPU(M1_park1.Angle);

        M1_park1.Cosine = _IQcosPU(M1_park1.Angle);

        PARK_MACRO_027(M1_park1)


        // ------------------------------------------------------------------------------
        //    Connect inputs of the PID_REG3 module and call the PID speed controller macro
        // ------------------------------------------------------------------------------

        if(M1_lsw == 12)
        {
            if(lock_flag==0)
            {
                M1_lock_init_angle = Cla_mech_angleB;
                lock_flag = 1;
            }
            M1_pid1_pos.Kp = pid1_pos.Kp;
            M1_pid1_pos.Ki = pid1_pos.Ki;
            M1_pid1_pos.Kd = pid1_pos.Kd;
            M1_pid1_pos.Ref = M1_PosRef;
            if(M1_PosRef>=M1_lock_init_angle)
            {
                if(Cla_mech_angleB <M1_lock_init_angle)
                {
                    M1_pid1_pos.Fdb = Cla_mech_angleB + 1.0;
                }
                else M1_pid1_pos.Fdb = Cla_mech_angleB;
            }
            else
            {
                if(Cla_mech_angleB >M1_lock_init_angle)
                {
                    M1_pid1_pos.Fdb = Cla_mech_angleB - 1.0;
                }
                else M1_pid1_pos.Fdb = Cla_mech_angleB;
            }
            PID_MACRO_027(M1_pid1_pos)

            if(M1_pid1_pos.Err<0.002&&M1_pid1_pos.Err>-0.002)
            {
                M1_lsw = 8;
            }
        }




        if (M1_SpeedLoopCount==SpeedLoopPrescaler)
        {
            if(M1_lsw ==12)
            {
                M1_pid1_spd.Ref = M1_pid1_pos.Out;
            }
            else
            {
                M1_pid1_spd.Ref = M1_rc1.SetpointValue;
            }

            if(M1_lsw==7)
            {
                M1_pid1_spd.Fdb = M1_speed3.EstimatedSpeed;
            }
            else
            {
                M1_pid1_spd.Fdb = M1_speed1.Speed;
            }


            if(M1_SMOswitchflg==1)
            {
                if(M1_speed3.EstimatedSpeed<0.4&&M1_speed3.EstimatedSpeed>0.0)
                {
                    M1_SMOangle_autoadjust = 0.0 - _IQmpy(M1_p1,M1_speed3.EstimatedSpeed);
                }
                else if(M1_speed3.EstimatedSpeed>0.4&&M1_speed3.EstimatedSpeed<0.99)
                {
                    M1_SMOangle_autoadjust = -0.2;
                }

                else
                {
                    M1_SMOangle_autoadjust = 0.0;
                }
                M1_SMOangle_adjust = M1_SMOangle_autoadjust+m1_comp;
            }

            if(M1_SMOswitchflg==0)
            {
                M1_SMOangle_adjust = M1_SMOangle_manualadjust+m1_comp;
            }

            PID_MACRO_027(M1_pid1_spd)
            M1_SpeedLoopCount=1;
            CAN_M1_Speed_OUT = (int)((M1_pid1_spd.Out+1.0)*100);

        }
        else M1_SpeedLoopCount++;

        if(M1_lsw==0 || M1_lsw==1 || M1_lsw==2|| M1_lsw==5||M1_lsw==9)    M1_pid1_spd.Ui=0;

        // ------------------------------------------------------------------------------
        //    Connect inputs of the PID_REG3 module and call the PID IQ controller macro
        // ------------------------------------------------------------------------------

        if(M1_lsw==0)
        {
            M1_pid1_iq.Ref = 0;
            M1_pid1_id.Ref = 0;//dqset
        }
        else if(M1_lsw==1 || M1_lsw==2|| M1_lsw==5||M1_lsw==9||M1_lsw==11)
        {
            M1_pid1_iq.Ref = M1_iq_rc1.SetpointValue;

        }
        else if(M1_lsw==8)
        {
            M1_pid1_iq.Ref = M1_IqRef_lock;
        }
        else if(M1_lsw==12)
        {
            if(M1_pwmon==1)
            {
                M1_pid1_iq.Ref =  M1_pid1_spd.Out;
            }
            else
            {
                M1_pid1_iq.Ref =  M1_pid1_spd.Out;
            }
        }
        else
        {
            if(M1_pid1_spd.Out < 0.0)
            {
                M1_pid1_spd.Out = 0.0;
                M1_pid1_spd.Ui = 0.0;
            }

            if(M1_pwmon==1)
            {
                M1_pid1_iq.Ref =  M1_pid1_spd.Out;
            }
            else
            {
                M1_pid1_iq.Ref =  M1_pid1_spd.Out;
            }
        }

        speed_out_iq = M1_pid1_iq.Ref;
        M1_pid1_id.Kp = pid1_id.Kp;
        M1_pid1_id.Ki = pid1_id.Ki;

        M1_pid1_iq.Fdb = M1_park1.Qs;
        M1_pid1_iq.Kp=M1_pid1_id.Kp;
        M1_pid1_iq.Ki=M1_pid1_id.Ki;
        PID_MACRO_027(M1_pid1_iq)
        CAN_M1_Iq_ref = (int)((M1_pid1_iq.Ref+1)*6000);
        CAN_M1_Iq_fdb = (int)((M1_pid1_iq.Fdb+1)*6000);


        // ------------------------------------------------------------------------------
        //    Connect inputs of the PID_REG3 module and call the PID ID controller macro
        // ------------------------------------------------------------------------------

        if(M1_lsw==0)
        {
            M1_pid1_id.Ref = M1_IdRef;
        }
        else if(M1_lsw==1 || M1_lsw==11 || M1_lsw==12)
        {
            M1_pid1_id.Ref = M1_id_rc1.SetpointValue;//dqset
        }
        else if(M1_lsw==2)
        {
            M1_pid1_id.Ref = M1_IdRef;
        }
        else if(M1_lsw==8)
        {
            M1_pid1_id.Ref = M1_IdRef_lock;
        }
        else {
            M1_pid1_id.Ref = 0;
        }
        M1_pid1_id.Fdb = M1_park1.Ds;
        PID_MACRO_027(M1_pid1_id)

        CAN_M1_Id_ref = (int)((M1_pid1_id.Ref+1)*6000);
        CAN_M1_Id_fdb = (int)((M1_pid1_id.Fdb+1)*6000);

        if(M1_lsw==4)
        {
            M1_Ud = M_Ud;
            M1_Uq = M_Uq;

            M1_ipark1.Ds = M1_Ud;
            M1_ipark1.Qs = M1_Uq;
        }
        else{
            M1_ipark1.Ds = M1_pid1_id.Out;//-MotorLdq*M1_speed3.EstimatedSpeed;
            M1_ipark1.Qs = M1_pid1_iq.Out;//+MotorLdq*M1_speed3.EstimatedSpeed;
        }


        if(M1_lsw==7||M1_lsw==9)
        {
            M1_anlge_comp= _IQmpy(M1_anlge_comp_value,M1_speed3.EstimatedSpeed); //1.5 PWM period delay

            M1_ipark1.Angle= M1_Angle1+M1_anlge_comp; //1.5 PWM period delay
        }
        else
        {
            M1_ipark1.Angle= M1_Angle1;
        }



        M1_ipark1.Sine = _IQsinPU(M1_ipark1.Angle);
        M1_ipark1.Cosine = _IQcosPU(M1_ipark1.Angle);
        IPARK_MACRO_027(M1_ipark1)
        CAN_M1_Ubeta = (int)((M1_ipark1.Beta+1)*600);
        CAN_M1_Ualfa = (int)((M1_ipark1.Alpha+1)*600);

        // ------------------------------------------------------------------------------
        //    Connect inputs of the SPEED_FR module and call the speed calculation macro
        // ------------------------------------------------------------------------------

        M1_speed1.ElecTheta = Eangle_resolver;
        SPEED_FR_MACRO_027(M1_speed1)
        M1_speed_resolver = M1_speed1.SpeedRpm;

        //#define SMO
#ifdef SMO
        // ------------------------------------------------------------------------------
        //    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
        // ------------------------------------------------------------------------------
        // Low Kslide responds better to loop transients
        // Increase Kslide for better torque response after closing the speed loop
        M1_smo1.Fsmopos=M1_SMO_F;
        M1_smo1.Gsmopos=M1_SMO_G;


        if ((M1_lsw==7||M1_lsw==9) && (M1_smo1.Kslide < M1_smoK_set))  M1_smo1.Kslide += _IQ(0.00001);

        M1_smo1.Ialpha = M1_clarke1.Alpha;
        M1_smo1.Ibeta  = M1_clarke1.Beta;
        M1_smo1.Valpha = M1_ipark1.Alpha;
        M1_smo1.Vbeta  = M1_ipark1.Beta;

        SMO_MACRO_027(M1_smo1)

        M1_SMOangle1=M1_smo1.Theta-M1_SMOangle_adjust;

        if (M1_SMOangle1>_IQ(1.0)) M1_SMOangle2=M1_SMOangle1-_IQ(1.0);
        else if (M1_SMOangle1<_IQ(0.0)) M1_SMOangle2=M1_SMOangle1+_IQ(1.0);
        else M1_SMOangle2=M1_SMOangle1;


        // ------------------------------------------------------------------------------
        //    Connect inputs of the SPEED_EST module and call the estimated speed macro
        // ------------------------------------------------------------------------------

        M1_speed3.EstimatedTheta = M1_SMOangle2;
        SE_MACRO_027(M1_speed3)

        M1_EstimatedSpeed = 1.0*M1_speed3.EstimatedSpeedRpm;

#endif
        // ------------------------------------------------------------------------------
        //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
        // ------------------------------------------------------------------------------

        M1_svgen1.Ualpha = M1_ipark1.Alpha;
        M1_svgen1.Ubeta  = M1_ipark1.Beta;

        SVGENDQ_MACRO(M1_svgen1)


        // ------------------------------------------------------------------------------
        //  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
        // ------------------------------------------------------------------------------

        if (M1_pwmsta == 1)
        {
            if (M1_pwmon==0)
            {
                M1_StopMotor();
                M1_pwmsta=0;
            }
            else{
                if (OCM1A == 0)
                {
                    EPWM_M1_PhaseA_Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*M1_svgen1.Ta)+INV_PWM_HALF_TBPRD;
                }
                else
                {
                    Manual_OCM1A();
                    OCM1A=0;
                }
                if (OCM1B == 0)
                    EPWM_M1_PhaseB_Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*M1_svgen1.Tb)+INV_PWM_HALF_TBPRD;
                else
                {
                    Manual_OCM1B();
                    OCM1B=0;
                }
                if(OCM1C == 0)
                    EPWM_M1_PhaseC_Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*M1_svgen1.Tc)+INV_PWM_HALF_TBPRD;
                else
                {
                    Manual_OCM1C();
                    OCM1C=0;
                }
            }
        }
        else
        {
            if (M1_pwmon==1)
            {
                M1_pid1_iq.Ui=0;
                M1_pid1_id.Ui=0;
                M1_ClearToStartMotor();
                M1_pwmsta=1;
            }
        }

        M1_Init=0;

#ifndef Sensorless


#endif //(Sensorless)

    }
    else
    {
        M1_StopMotor();

        M1_pwmsta=0;
    }



#endif // (BUILDLEVEL==LEVEL5)


    if(M1_pwmon == 1)
    {
        M1_M2_FLAG = 1;
    }
    else
    {
        M1_M2_FLAG = 0;
    }

    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all  = PIEACK_GROUP3;


//    GpioDataRegs.GPBCLEAR.bit.GPIO50=1;
}// MainISR Ends Here


void Parameter_Init(void)
{
    NTC_SIC_M1A=0.0;
    NTC_SIC_M1B=0.0;
    NTC_SIC_M1C=0.0;
    NTC_SIC_M1A_ECAP=0.0;
    NTC_SIC_M1B_ECAP=0.0;
    NTC_SIC_M1C_ECAP=0.0;
    Cla_ALIGN_ANGLE = 0.15;
    M1_Algo_control_start = 0;
    M1_lsw = 0;
    M1_pwmon = 1;

    throttle_sw_on = 0u;
    throttle_sw_off_cnt = 0u;


    M1_SMOswitchflg = 1;
    Max_voltage=_IQ(BASE_VOLTAGE);
    Min_voltage=_IQ((float)16.0/BASE_VOLTAGE);
    Max_current=_IQ((float)600/BASE_CURRENT);
    M1_speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    M1_speed1.K2 = _IQ(1/(1+T*2*PI*5));
    M1_speed1.K3 = _IQ(1)-M1_speed1.K2;
    M1_speed1.BaseRpm = 120.0*BASE_FREQ/POLES;
    M1_speed1.Speed = 0.0;
    Rpm_conv=60.0*ISR_FREQUENCY*1000;
    BaseRpm_1 = (uint32_t)120*BASE_FREQ/POLES;
    BaseRpm_2 =  BASE_FREQ*120.0/POLES;
    M1_speed3.K1 = _IQ21(1/(BASE_FREQ*T));
    M1_speed3.K2 = _IQ(1/(1+T*2*PI*5));
    M1_speed3.K3 = _IQ(1)-M1_speed3.K2;
    M1_speed3.BaseRpm = 120.0*BASE_FREQ/POLES;
    M1_rg1.StepAngleMax = _IQ(BASE_FREQ*T);
    M1_smo1_const.Rs = RS;
    M1_smo1_const.Ls = LS;
    M1_smo1_const.Ib = M_BASE_CURRENT;
    M1_smo1_const.Vb = M_BASE_VOLTAGE;
    M1_smo1_const.Ts = T;
    SMO_CONST_MACRO_027(M1_smo1_const)
    M1_smo1.Fsmopos = _IQ(M1_smo1_const.Fsmopos);
    M1_smo1.Gsmopos = _IQ(M1_smo1_const.Gsmopos);
    M1_smo1.Kslide = _IQ(0.05308703613);
    M1_smo1.Kslf = _IQ(0.1);
    M1_esmo1.Fsmopos  = _IQ(M1_smo1_const.Fsmopos);
    M1_esmo1.Gsmopos  = _IQ(M1_smo1_const.Gsmopos);
    M1_esmo1.Kslide   = _IQ(0.05308703613);
    M1_esmo1.base_wTs = _IQ(BASE_FREQ*T);
    M1_pi_smo.Kp   = _IQ(5.0);
    M1_pi_smo.Ki   = _IQdiv(_IQ(100*0.001/ISR_FREQUENCY), M1_pi_smo.Kp);
    M1_pi_smo.Umax = _IQ(1.0);
    M1_pi_smo.Umin = _IQ(-1.0);
    pid1_pos.Kp = _IQ(0.001);
    pid1_pos.Ki = _IQ(0.001);
    pid1_pos.Kd = _IQ(0.001);
    pid1_id.Kp = _IQ(0.62);
    pid1_id.Ki = _IQ(0.004);
    M1_pid1_id.Kp = _IQ(0.62);
    M1_pid1_id.Ki = _IQ(0.004);
    M1_pid1_id.Kd = _IQ(0);
    M1_pid1_id.Kc = _IQ(0);
    M1_pid1_id.OutMax = _IQ(1.0);
    M1_pid1_id.OutMin = _IQ(-1.0);
    M1_pid1_iq.Kp = _IQ(0.62);
    M1_pid1_iq.Ki = _IQ(0.004);
    M1_pid1_iq.Kd = _IQ(0);
    M1_pid1_iq.Kc = _IQ(0);
    M1_pid1_iq.OutMax = _IQ(1.0);
    M1_pid1_iq.OutMin = _IQ(-1.0);
    M1_pid1_pos.Kp = _IQ(0.04);
    M1_pid1_pos.Ki = _IQ(0.001);
    M1_pid1_pos.Kd = _IQ(0);
    M1_pid1_pos.Kc = _IQ(0);
    M1_pid1_pos.OutMax = _IQ(0.002);
    M1_pid1_pos.OutMin = _IQ(-0.002);
    M1_pid1_spd.Kp = _IQ(0.5);
    M1_pid1_spd.Ki = _IQ(0.002);
    M1_pid1_spd.Kd = _IQ(0);
    M1_pid1_spd.Kc = _IQ(0);
    M1_pid1_spd.OutMax = _IQ(1.5);
    M1_pid1_spd.OutMin = _IQ(-1.5);
}

void M1_AlgoInitStart(void)
{
    M1_lsw=0;

    M1_IdRef = _IQ(0.0);
    M1_IqRef = _IQ(0.000);
    M1_SpeedRef = _IQ(0.0);
    M1_PosRef = 0.0;
    M1_PosAngleRef = 0;
    M1_SMOangle1=_IQ(0.0);
    M1_SMOangle2=_IQ(0.0);
    M1_SMOangle_adjust=_IQ(0.0);
    M1_SMOangle_error=_IQ(0.0);
    M1_pid1_id.Kp = _IQ(0.62);
    M1_pid1_id.Ki = _IQ(0.004);
    M1_pid1_id.Kd = _IQ(0);
    M1_pid1_id.Kc = _IQ(0);
    M1_pid1_id.OutMax = _IQ(1.0);
    M1_pid1_id.OutMin = _IQ(-1.0);
    M1_pid1_id.Ref=0;
    M1_pid1_id.Out=0;
    M1_pid1_spd.Ref=0;
    M1_pid1_spd.Fdb=0;
    M1_pid1_spd.Err=0;
    M1_pid1_spd.Up=0;
    M1_pid1_spd.OutPreSat=0;
    M1_pid1_spd.Out=0;
    M1_pid1_spd.Ui=0;
    M1_pid1_iq.Kp = _IQ(0.62);
    M1_pid1_iq.Ki = _IQ(0.004);
    M1_pid1_iq.Kd = _IQ(0);
    M1_pid1_iq.Kc = _IQ(0);
    M1_pid1_iq.OutMax = _IQ(1.0);
    M1_pid1_iq.OutMin = _IQ(-1.0);
    M1_pid1_iq.Out = 0;
    M1_pid1_iq.Ref=0;
    M1_pid1_iq.Err=0;
    M1_pid1_iq.Fdb=0;
    M1_pid1_iq.OutPreSat=0;
    M1_pid1_iq.Up=0;
    M1_pid1_iq.Ui=0;
    M1_pid1_id.Err=0;
    M1_pid1_id.Fdb=0;
    M1_pid1_id.OutPreSat=0;
    M1_pid1_id.Up=0;
    M1_pid1_id.Ui=0;
    M1_pid1_pos.Kp = _IQ(0.04);
    M1_pid1_pos.Ki = _IQ(0.001);
    M1_pid1_spd.Kp = _IQ(0.5);
    M1_pid1_spd.Ki = _IQ(0.002);
    M1_pid1_spd.Kd = _IQ(0);
    M1_pid1_spd.Kc = _IQ(0);
    M1_pid1_spd.OutMax = _IQ(1.5);
    M1_pid1_spd.OutMin = _IQ(-1.5);
    M1_svgen1.Ualpha = 0;
    M1_svgen1.Ubeta = 0;
    M1_overvoltage=0;
    M1_undervoltage=0;
    M1_overcurrent=0;
    M1_overcurrent_A=0;
    M1_overcurrent_B=0;
    M1_overcurrent_C=0;
    M1_overcurrent_A_count=0;
    M1_overcurrent_A_flag=0;
    M1_overcurrent_B_count=0;
    M1_overcurrent_B_flag=0;
    M1_overcurrent_C_count=0;
    M1_overcurrent_C_flag=0;
    M1_overtemperature=0;
    M1_Startingflg=0;
    M1_StartingTicker=0;
    M1_rc1.RampDelayMax = 80;
    M1_rg1.Angle = _IQ(0);
    ThrottleSpd = 0;
    M1_lsw = 0;
    M1_pid1_id.Out = 0;
    M1_pid1_iq.Out = 0;
    M1_speed1.Speed=0;
    M1_speed1.ElecTheta=0;
    M1_speed1.DirectionQep=1;
    M1_speed1.OldElecTheta=0;
    M1_speed3.EstimatedSpeed=0;
    M1_speed3.OldEstimatedTheta=0;
    M1_speed3.EstimatedTheta=0;
    M1_sum_uq=0;

    M1_avg_uq=0;
    M1_CloseRunTicker=0;
    lock_flag = 0;
    M1_Init=1;
}

/**************************************************************************/
/****************************Motor Sound Times*****************************/
/**************************************************************************/

void Telemetry_update()
{
    if( enTelemetry )
    {
        /* Please add data update code here...
         * i.e TelemetryMsg.DCcurrent = 65.2;
         */
        TelemetryMsg.Throttle = iThrottle;
        TelemetryMsg.inputVoltage = M1_volt1.DcBusVolt;
        TelemetryMsg.PhaseCurrentAs = M1_PhaseCurrent.As;
        TelemetryMsg.PhaseCurrentBs = M1_PhaseCurrent.Bs;
        TelemetryMsg.PhaseCurrentCs = M1_PhaseCurrent.Cs;
        TelemetryMsg.TemperatureBoard = M1_Temp_Measurment.BoardTemp;
        TelemetryMsg.SpeedRPM = M1_speed3.EstimatedSpeedRpm;

        Telemetry_Transmit();
    }
}

void Telemetry_Transmit()
{
    uint16_t Xmit_length = sizeof(TelemetryMsg) / 2;  //count in word:16bit
    uint16_t i = 0;
    uint16_t *p= (uint16_t*) & TelemetryMsg;
    for(i = 0 ;i < Xmit_length ; i++)
    {
        Scib_xmit( *(p + i) >> 8); //Transmit High byte first
        Scib_xmit( *(p + i) & 0x00FF); //Then transmit low byte
    }
}

void Telemetry_init()
{
    Init_SCIB_GPIO();
    Init_SCIB();

    TelemetryMsg.head = 0xEB90;
    TelemetryMsg.devID = 0xabcd1234;  // unique device id
    TelemetryMsg.head =0x90EB;
}

/****************************************************************************
 * End of Code *
 * ***************************************************************************
 */

