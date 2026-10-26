/* Maya80系列 Main主函数 */
#include "HPMC_Settings.h"
#include "HPMC_algo_includes.h"
#include "AppInc.h"

#include "TLF35584.h"
#include "CLAShared.h"

#ifdef _FLASH
#pragma CODE_SECTION(MotorControlISR,"ramfuncs");
#pragma CODE_SECTION(M1_AlgoInitStart,"ramfuncs");
#pragma CODE_SECTION(Current_and_voltage_measurement,"ramfuncs");
#pragma CODE_SECTION(TemperatureCurrentWeaken_ClampIqReference,"ramfuncs");
#pragma CODE_SECTION(M1_ResolverApplyAlignOffset,"ramfuncs");
#endif

#pragma INTERRUPT (MotorControlISR, LPI)

// **********************************************************
// ********************Interrupt  Functions *****************
// **********************************************************
__interrupt void MotorControlISR(void);

#pragma DATA_SECTION(Cla_angle,"Cla1ToCpuMsgRAM");
float Cla_angle;
#pragma DATA_SECTION(Cla_ResolverElecTheta_pu,"Cla1ToCpuMsgRAM");
float Cla_ResolverElecTheta_pu;
#pragma DATA_SECTION(Cla_ResolverMechTheta_pu,"Cla1ToCpuMsgRAM");
float Cla_ResolverMechTheta_pu;
#pragma DATA_SECTION(Cla_AD2SB_Fault,"Cla1ToCpuMsgRAM");
volatile uint16_t Cla_AD2SB_Fault;

/*------------------------------------------------------------------------------
                      电机旋变角度零点offset
------------------------------------------------------------------------------*/
float M1_Align_Angle_Offset_pu = 0.356; /* 对角度；历史：M1.0-2号 0.0493 / M2.1-NO2 0.9132 / M1.0-NO2 0.0493 / M1.0-NO1 0.115 / M2.1-NO1 0.35 / 2.2-NO1 0.4395 / 3.1-NO1 0.10 / M3.1-NO2 0.67 */

/*------------------------------------------------------------------------------
 默认油门源（本文件修改 throttle_type，与 HPMC_Settings.h 说明一致）
   HOST_CAN_THROTTLE  = 上位机CAN指令
   RE38_VOLT_THROTTLE = RE38 模拟量
   RE38_CAN_THROTTLE  = RE38 CAN 转速（客户 RuiXiang，0x18EF0010）
------------------------------------------------------------------------------*/
ThrottleType_e throttle_type = HOST_CAN_THROTTLE;

void Parameter_Init(void);
void RE38_VoltThrottle_UpdateFromAdc(void);
void M1_AlgoInitStart(void);
void Current_and_voltage_measurement(void);
void Board_temp(void);
void CalibrateCurrentOffset(void);
void EnableAppInterrupts(void);

// A branch states
extern void A1(void);  // state A1
extern void A2(void);  // state A2
extern void A3(void);  // state A3

// B branch states
extern void B1(void);  // state B1
extern void B2(void);  // state B2
extern void B3(void);  // state B3

// C branch states
extern void C1(void);  // state C1
extern void C2(void);  // state C2
extern void C3(void);  // state C3

// Variable declarations
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

// *********************** USER Variables *************************************
// Global variables used in this system
// ****************************************************************************
uint16_t angle_channel = 0; // 1 for SPIC-1p, 0 for SPIB-2p

uint16_t i2cError;
// ****************************************************************************
// Variables for current measurement
// ****************************************************************************
// Offset calibration routine is run to calibrate for any offsets on the opamps
_iq offset_M1_PhaseA,
    offset_M1_PhaseB,         // offset in LEM current V fbk channel @ 0A
    offset_M1_PhaseC;         // offset in LEM current W fbk channel @ 0A

// *********************** Fundamental *************************************
// All variables and definition related to Fundamental
// ****************************************************************************
float M1_IsrSamplePeriod_s = 0.001/ISR_FREQUENCY;    // Sampling period (sec)

// *********************** *************************************
// End of Fundamental
// ****************************************************************************

// *********************** Basic Measurement and Compensation *************************************
// All variables and definition related to Basic Measurement and Compensation
// ****************************************************************************

extern float M1_ResolverElecTheta_pu;
extern float Eangle_resolverC;

uint32_t BaseRpm_1;

_iq K1 = _IQ(0.95),          // Offset filter coefficient, applied once per iteration of the main() calibration loop (not tied to the ISR rate)
    K2 = _IQ(0.05);

typedef struct {
    float As;      // phase A
    float Bs;      // phase B
    float Cs;      // phase C
    float As1;     // phase A test current
    float Bs1;     // phase B test current
    float Cs1;     // phase C test current
} CURRENT_SENSOR;

#define RECONSTRUCT_PHASE_C 0u
#define RECONSTRUCT_PHASE_A 1u
#define RECONSTRUCT_PHASE_B 2u

// Used by the next ISR current read. The value is selected from the previous
// PWM compare update so it matches the ADC results captured in this cycle.
uint16_t M1_reconstruct_phase = RECONSTRUCT_PHASE_C;

CURRENT_SENSOR M1_PhaseCurrent;

// *********************** *************************************
// End of Basic Measurement and Compensation
// ****************************************************************************

// *********************** Over all control logic *************************************
// All variables and definition related to Over all control logic
// ****************************************************************************
M1_ControlMode_e M1_Control_Mode = M1_MODE_STOP;

uint16_t M1_SMOswitchflg = 1;

extern float CAN_pid_SpdLp_kp;
extern float CAN_pid_SpdLp_ki;
extern float CAN_pid_CurrLp_id_kp;
extern float CAN_pid_CurrLp_id_ki;
extern float CAN_pid_CurrLp_iq_kp;
extern float CAN_pid_CurrLp_iq_ki;

extern int CAN_PID_adjust;
extern uint16_t CAN_RampStep;
extern uint16_t CAN_torque_pwm_reset;

uint32_t M1_iq_RampControl = 5;

uint32_t M1_id_RampControl = 5; // dqset

uint16_t M1_SVPWM_Updating = 0;  /* 0=未写 SVPWM 占空比；1=已清 TZ，ISR 更新 CMPA */
uint16_t M1_Algo_control_start = 0;

int M1_Init = 0;
volatile uint16_t CAN_Feedback_Spd_RPM = 0u;

// *********************** *************************************
// End of Over all control logic
// ****************************************************************************

// *********************** Speed input *************************************
// All variables and definition related to Speed input
// ****************************************************************************
// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL_027 M1_speed_rc1 = RMPCNTL_027_DEFAULTS;
RMPCNTL_027 M1_iq_rc1 = RMPCNTL_027_DEFAULTS;
RMPCNTL_027 M1_id_rc1 = RMPCNTL_027_DEFAULTS;
RMPCNTL_027 Position_Lock = RMPCNTL_027_DEFAULTS;

//  Instance a ramp generator to simulate an Anglele
RAMPGEN_027 M1_rg1 = RAMPGEN_027_DEFAULTS;

// *********************** *************************************
// End of Speed input
// ****************************************************************************

// *********************** FOC control related *************************************
// All variables and definition related to FOC control related
// ****************************************************************************
_iq M1_IdRef = _IQ(0.0);               // Id reference (pu)
_iq M1_IdRef_lock = _IQ(0.05);

_iq M1_IqRef = _IQ(0.000);              // Iq reference (pu)
_iq M1_IqRef_lock = _IQ(0.000);
_iq M1_SpeedRef = _IQ(0.0);          // For Open Loop tests
float M1_PosRef = 0.0;
float M1_lock_init_angle = 0.0;
int lock_flag = 0;
_iq M1_ParkElecTheta_pu;

// *********************** *************************************
// End of FOC control related
// ****************************************************************************

// *********************** FOC fundamental transforms *************************************
// All variables and definition related to FOC transforms
// ****************************************************************************
CLARKE_027 M1_clarke1 = CLARKE_027_DEFAULTS;
PARK_027   M1_park1   = PARK_027_DEFAULTS;
IPARK_027  M1_ipark1  = IPARK_027_DEFAULTS;

PHASEVOLTAGE_027 M1_volt1 = PHASEVOLTAGE_027_DEFAULTS;

// *********************** *************************************
// End of FOC transforms
// ****************************************************************************

// *********************** PIDs *************************************
// All variables and definition related to PID
// ****************************************************************************
PIDREG3_027 M1_pid_CurrLp_id = PIDREG3_027_DEFAULTS;
PIDREG3_027 M1_pid_CurrLp_iq = PIDREG3_027_DEFAULTS;
PIDREG3_027 M1_pid_SpdLp = PIDREG3_027_DEFAULTS;
PIDREG3_027 M1_pid_PosLp = PIDREG3_027_DEFAULTS;

// *********************** *************************************
// End of PID
// ****************************************************************************

// *********************** SVPWM  *************************************
// All variables and definition related to SVPWM
// ****************************************************************************

SVGEN M1_svgen1 = SVGEN_DEFAULTS;
// *********************** *************************************
// End of SVPWM
// ****************************************************************************
// *********************** SMO related *************************************
// All variables and definition related to SMO
// ****************************************************************************
_iq M1_SMOangle1 = _IQ(0.0);
_iq M1_SMOangle2 = _IQ(0.0);

_iq M1_SMOangle_adjust = _IQ(0.0);
_iq M1_SMOangle_manualadjust = _IQ(0.0);
_iq M1_SMOangle_autoadjust = _IQ(0.0);
_iq M1_SMO_F = _IQ(0.95);
_iq M1_SMO_G = _IQ(0.5);

_iq M1_SMOangle_error = _IQ(0.0);
_iq M1_p1 = _IQ(0.4);
SMOPOS_027 M1_smo1 = SMOPOS_027_DEFAULTS;

SMOPOS_CONST_027 M1_smo1_const = SMOPOS_CONST_027_DEFAULTS;

ESMOPOS  M1_esmo1 = ESMOPOS_DEFAULTS;
_iq  M1_smoK_set = _IQ(0.3);

PI_CONTROLLER  M1_pi_smo = PI_CONTROLLER_DEFAULTS;

// Instance an inverter PWM dead band compensator
// *********************** *************************************
// End of SMO
// ****************************************************************************

// *********************** Speed Estimate *************************************
// All variables and definition related to Speed Estimate
// ****************************************************************************
// Instance a speed calculator based on Encoder position not use
SPEED_MEAS_QEP_027 M1_speed1 = SPEED_MEAS_QEP_027_DEFAULTS;

SPEED_ESTIMATION_027 M1_speed3 = SPEED_ESTIMATION_027_DEFAULTS;

float M1_speed_resolver = 0.0;

// *********************** *************************************
// End of Speed Estimate
// ****************************************************************************

// *********************** Protection and Misc *************************************
// All variables and definition related to Protection and Misc
// ****************************************************************************
_iq M1_angle_comp = _IQ(0);

_iq M1_angle_comp_value = _IQ(0.2);
// *********************** *************************************
// End of Protection and Misc
// ****************************************************************************

int16_t OffsetCalCounter;

uint16_t SpeedLoopPrescaler = 10,      // Speed loop pre scalar
         M1_SpeedLoopCount = 1;        // Speed loop counter

float m1_comp = 0.0;
float Temp_board = 0.0;
volatile uint16_t board_overtemp_ticker = 0;
float M1_Temp_Sensor_out_mV = 0.0;
extern float NTC_SIC_M1_AvgC, NTC_SIC_M1A, NTC_SIC_M1B, NTC_SIC_M1C;
extern float NTC_SIC_M1A_ECAP, NTC_SIC_M1B_ECAP, NTC_SIC_M1C_ECAP;

float analog1 = 0.0;   /* ISR 每周期更新 analog1（RE38 模拟油门 ADC 换算同源） */
float analog2 = 0.0;
float analog3 = 0.0;

/* RE38 油门目标转速[rpm]：主循环 Update 写入，MotorCtrl ISR 门控读 */
volatile float re38_volt_target_rpm = RE38_THROTTLE_IDLE_RPM;
static uint16_t re38_volt_overrange_tick = 0u;
static uint16_t re38_volt_lost_tick = 0u;
uint16_t re38_throttle_sw_on = 0u;
uint16_t re38_throttle_sw_off_cnt = 0u;

float adc_calib = 1.0;

float M1_real_voltage = 0.0;

// *****************************************************************************
// *****************************************************************************

float M1_Ud,M1_Uq;
float M_Ud = 0.0;
float M_Uq = 0.0;

float Lock_angle = 0.0;
// *****************************************************************************
// *****************************************************************************
extern void CANAIntHandler();
extern void CANBIntHandler();

static void MotorRestart_ClearFaults(void);
static void MotorRestart_Sequence_Handle(void);
static float TemperatureCurrentWeaken_ClampIqReference(float iq_ref);
static void M1_ResolverApplyAlignOffset(void);
static void M1_ResolverExpectedElectricalAngle(void);

uint16_t i2cb;

extern struct I2CMSG I2cMsgOut1;

extern struct I2CMSG I2cMsgIn1;

extern struct I2CMSG *CurrentMsgPtr;

float DC_15V = 0.0;
float elec_cycle = 0.0;
uint16_t PHASE_RMS_WIN  = 400;   /* 50us*PHASE_RMS_WIN @ 20kHz */
uint16_t PhaseCurrent_datasend_count=0;

volatile uint16_t WWD_ticker = 0;
volatile uint16_t WWD_err_ticker = 0;

void main(void)
{
    InitClockAndPie();

    CLA_configClaMemory();
    CLA_initCpu1Cla1();

    InitGpio43And50();

    InitSystick();

    init_ERR_Timer();
    TLF35584_spi_init();
    TLF35584_DisableERRandWWD();
    TLF35584_GoToNormal();

    BackgroundTask_Init();

    PwmTbclk_Stop();
    GateDriver_InitGpio();
    Configure_PWM();
    ConfigureADC();
    Parameter_Init();
    PwmTbclk_Start();

    CalibrateCurrentOffset();

    CANA_Initialization();
    CANB_Initialization();

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

    CLA_AD2S_Gpio();

    EnableAppInterrupts();

    DELAY_US(3000000);

    CANBMsgSend(); // enable oil pump

    TLF35584_StartWatchdog();

    for (;;)  // infinite loop
    {

        WWD_ticker++;
        if (WWD_ticker >= 7)
        {
            TLF35584_Trigger_WWD();
            WWD_ticker = 0;
            WWD_err_ticker = (TLF35584_ReadReg(0x29)>> 1) & 0xFF;
        }

        NTC_temp();

        PT100_temp();

        Board_temp();

        // write
        if (I2cMsgOut1.MsgStatus == I2C_MSGSTAT_SEND_WITHSTOP)
        {
            i2cError = I2CB_WriteData(&I2cMsgOut1);

            if (i2cError == I2C_SUCCESS)
            {
                CurrentMsgPtr = &I2cMsgOut1;
                I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;
            }
        }
        // read
        if (I2cMsgOut1.MsgStatus == I2C_MSGSTAT_INACTIVE)
        {
            // Check incoming message status.
            if (I2cMsgIn1.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
            {
                // EEPROM address setup portion
                while (I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
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
            else if (I2cMsgIn1.MsgStatus == I2C_MSGSTAT_RESTART)
            {
                // Read data portion
                while (I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
                {

                }

                CurrentMsgPtr = &I2cMsgIn1;
                I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
            }
        }

        if (throttle_type == HOST_CAN_THROTTLE)
        {
            CANAControlCmdGet();
            CANAPidMsgGet();
        }
        else if (throttle_type == RE38_CAN_THROTTLE)
        {
            RE38_CanThrottle_UpdateFromRx();
        }
        else if (throttle_type == RE38_VOLT_THROTTLE)
        {
            RE38_VoltThrottle_UpdateFromAdc();
        }

        CANABootMsgGet();

        CANAMsgSend();

        TempSend();

        DataSend();

        DTCSend();

        PT100send();

        PhaseCurrent_datasend_count++;
        if(PhaseCurrent_datasend_count>20)
        {
           PhaseCurrentRMSSend();
           PhaseCurrent_datasend_count=0;
        }

        elec_cycle = 3/M1_speed_resolver;
        PHASE_RMS_WIN = elec_cycle * ISR_FREQUENCY * 30000;//采30个周期 elec_cycle*20*1000*30

        /* 故障重启：HOST_CAN的torque_reset bit 或 RE38油门switch关闭 触发沿检测 */
        MotorRestart_Sequence_Handle();



        DELAY_US(10000); // 10ms
    }
} // END MAIN CODE

/******************************************************************************
 * CalibrateCurrentOffset
 * 上电相电流零偏标定：PWM 由 TZ 强制封波、栅极驱动已使能，
 * 30000 次迭代，前 5000 次丢弃，其余一阶滤波得到 offset_M1_PhaseA/B/C
 ******************************************************************************/
void CalibrateCurrentOffset(void)
{
    offset_M1_PhaseA  = 0;
    offset_M1_PhaseB  = 0;
    offset_M1_PhaseC  = 0;

    // Enable the gate driver outputs only after Configure_PWM() has already
    // locked all M1 PWM outputs low by Trip Zone. Re-force OST after enabling
    // the driver so offset calibration runs in the real enabled-driver state.
    GateDriver_EnableOutputs();
    M1_ForceAllPwmLowByTZ();

    DELAY_US(100000);

    for (OffsetCalCounter = 0; OffsetCalCounter < 30000; )
    {

        if (OffsetCalCounter > 5000)
        {
            offset_M1_PhaseA  = K1*offset_M1_PhaseA + K2*(Cur_M1_PhaseA_Raw)*ADC_PU_SCALE_FACTOR;
            offset_M1_PhaseB  = K1*offset_M1_PhaseB + K2*(Cur_M1_PhaseB_Raw)*ADC_PU_SCALE_FACTOR;
            offset_M1_PhaseC  = K1*offset_M1_PhaseC + K2*(Cur_M1_PhaseC_Raw)*ADC_PU_SCALE_FACTOR;
        }

        OffsetCalCounter++;
    }
}

/******************************************************************************
 * EnableAppInterrupts
 * 挂接并使能应用中断：EPWM1_INT（FOC ISR）、I2CB、CANA/CANB、EPWM1~3 TZ，
 * 最后打开 CPU 中断组与全局中断（EINT / ERTM）
 ******************************************************************************/
void EnableAppInterrupts(void)
{
    EALLOW;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     // INT on PRD event
    EPwm1Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event

    PieVectTable.EPWM1_INT = &MotorControlISR;
    PieVectTable.I2CB_INT = &i2c_int1b_isr;

    for (i2cb = 0; i2cb < I2C_MAX_BUFFER_SIZE; i2cb++)
    {
        I2cMsgIn1.MsgBuffer[i2cb] = 0x0000;
    }

    PieCtrlRegs.PIEIER8.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // Enable EPWM1_INT in PIE group 3

    PieVectTable.CANA0_INT = &CANAIntHandler;
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;
    PieVectTable.CANB0_INT = &CANBIntHandler;
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;

    EPwm1Regs.ETCLR.bit.INT = 1;

    PieVectTable.EPWM1_TZ_INT = &epwm1_tzint_isr;
    PieVectTable.EPWM2_TZ_INT = &epwm2_tzint_isr;
    PieVectTable.EPWM3_TZ_INT = &epwm3_tzint_isr;

    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx3 = 1;

    IER |= M_INT3; // Enable group 3 interrupts
    IER |= M_INT1; // Enable group 1 interrupts
    IER |= M_INT8; // Enable group 8 interrupts
    IER |= M_INT9; // M_INT9
    EINT;          // Enable Global interrupt INTM
    ERTM;          // Enable Global realtime interrupt DBGM

    EDIS;
}

// ****************************************************************************
// ****************************************************************************
// Motor Control ISR - EPWM
// ****************************************************************************
// ****************************************************************************

__interrupt void MotorControlISR(void)
{
    EINT;

    FaultHardware_Update();

    MotorISR_Throttle_Apply(); // 根据油门类型进行控制模式和给定速度的赋值

#if defined(SMO) && RESOLVER_FAULT_TO_SMO_ENABLE
    {
        uint16_t resolver_fault_active = Cla_AD2SB_Fault & 0x78u; /* LOT/DOS/LOS：D3~D6 */
        RE38_Fault_Start_Update(resolver_fault_active, M1_speed3.EstimatedSpeedRpm);

        if ((M1_Algo_control_start == 1u) && (resolver_fault_active != 0u) && (M1_Control_Mode == M1_MODE_SPEED))
        {
            M1_Control_Mode = M1_MODE_SENSORLESS_SPEED;
        }
    }
#endif

    if ((M1_Algo_control_start == 0) && (M1_Init == 0))
    {
        M1_AlgoInitStart();
    }

    if (M1_Algo_control_start == 0)
    {
        M1_ResolverApplyAlignOffset();

        M1_speed1.ElecTheta = M1_ResolverElecTheta_pu;

        SPEED_FR_MACRO_027(M1_speed1)

        M1_speed_resolver = M1_speed1.SpeedRpm;
    }

    Current_and_voltage_measurement();

    FaultProtect_IsrUpdate(M1_real_voltage,
                           M1_PhaseCurrent.As, M1_PhaseCurrent.Bs, M1_PhaseCurrent.Cs,
                           ((M1_Control_Mode == M1_MODE_SENSORLESS_SPEED) ?
                            (float)M1_speed3.EstimatedSpeedRpm : M1_speed_resolver),
                           DC_15V);

    if (FaultProtect_IsTripped())
    {
        M1_StopMotor();
    }

    if (CAN_BOOTLOADER_STOPMOTOR_FLAG == 1)
    {
        M1_Algo_control_start = 0;
        M1_Control_Mode = M1_MODE_STOP;
        M1_SpeedRef = 0.0f;

        M1_StopMotor();
    }
    /* M1 FOC: M1_Control_Mode 由三源油门（HOST_CAN / RE38）ISR 门控选择控制模式。 */

    if (M1_Algo_control_start == 1)
    {

        if (M1_Control_Mode == M1_MODE_STOP)
        {
            M1_speed_rc1.TargetValue = 0;
            M1_iq_rc1.TargetValue = 0;
        }
        else if (M1_Control_Mode == M1_MODE_OPENLOOP_DRAG)
        {
            M1_speed_rc1.TargetValue = M1_SpeedRef;
            M1_speed_rc1.RampStep = M1_RAMP_STEP;
            M1_speed_rc1.RampDelayMax = 20;

            M1_iq_rc1.TargetValue = M1_IqRef;
            M1_iq_rc1.RampStep = M1_RAMP_STEP;
            M1_iq_rc1.RampDelayMax = M1_iq_RampControl;
            M1_id_rc1.TargetValue = M1_IdRef; // dqset
            M1_id_rc1.RampStep = M1_RAMP_STEP;
            M1_id_rc1.RampDelayMax = M1_id_RampControl; // dqset
        }
        else if (M1_Control_Mode == M1_MODE_TORQUE)
        {
            M1_iq_rc1.TargetValue = M1_IqRef;
            M1_iq_rc1.RampStep = M1_RAMP_STEP; // 默认步长1
            M1_iq_rc1.RampDelayMax = M1_iq_RampControl;
        }
        else if (M1_Control_Mode == M1_MODE_SPEED || M1_Control_Mode == M1_MODE_SENSORLESS_SPEED)
        {
            /* 速度闭环模式：M1_speed_rc1 对 M1_SpeedRef 做 Ramp，输出 SetpointValue → 速环 Ref */
            M1_speed_rc1.TargetValue = M1_SpeedRef;

            if ((throttle_type == HOST_CAN_THROTTLE) && (CAN_RampStep != 0))
            {
                M1_speed_rc1.RampStep = CAN_RampStep;
                M1_speed_rc1.RampDelayMax = M1_RAMP_DELAY_MIN;
            }
            else if (M1_RAMP_SOFT_LAND_ENABLE)
            {
                /* 软着陆：先设最快初值，再由 M1_RampSoftLandingUpdate 按剩余 rpm 改 Delay */
                M1_speed_rc1.RampStep = M1_RAMP_STEP;
                M1_speed_rc1.RampDelayMax = M1_RAMP_DELAY_MIN;
            }
            else
            {
                /* 软着陆关闭且无 CAN 覆盖：全程固定最快 Ramp */
                M1_speed_rc1.RampStep = M1_RAMP_STEP;
                M1_speed_rc1.RampDelayMax = M1_RAMP_DELAY_MIN;
            }
        }
        else if (M1_Control_Mode == M1_MODE_DIRECT_VOLTAGE || M1_Control_Mode == M1_MODE_OPENLOOP_IQ)
        {
            M1_speed_rc1.TargetValue = M1_SpeedRef;
            M1_speed_rc1.RampStep = M1_RAMP_STEP;
            M1_speed_rc1.RampDelayMax = 10;
        }
        else if (M1_Control_Mode == M1_MODE_RESOLVER_ALIGN)
        {
            M1_iq_rc1.TargetValue = M1_IqRef;
            M1_iq_rc1.RampDelayMax = M1_iq_RampControl;
            M1_id_rc1.TargetValue = M1_IdRef; // dqset
            M1_id_rc1.RampDelayMax = M1_id_RampControl; // dqset
        }
        else if (M1_Control_Mode == M1_MODE_SENSORLESS_TORQUE)
        {
            M1_iq_rc1.TargetValue = M1_IqRef;
            M1_iq_rc1.RampStep = M1_RAMP_STEP;
            M1_iq_rc1.RampDelayMax = M1_iq_RampControl;
        }
        else if (M1_Control_Mode == M1_MODE_POSITION_LOCK)
        {
            M1_speed_rc1.TargetValue = 0;
            Position_Lock.RampStep = 1;
            Position_Lock.RampDelayMax = 35;
        }
        else
        {
            M1_speed_rc1.TargetValue = 0;
        }

#if M1_RAMP_SOFT_LAND_ENABLE
        /* Smoothstep 软着陆：RC_MACRO 前更新 RampDelayMax
         * BaseRpm = pu 标定；remain = Target−Setpoint；阈值见 HPMC_Settings.h */
        if ((M1_Control_Mode == M1_MODE_SPEED || M1_Control_Mode == M1_MODE_SENSORLESS_SPEED) && (CAN_RampStep == 0))
        {
            M1_RampSoftLandingUpdate(&M1_speed_rc1, (float)M1_speed1.BaseRpm);
        }
#endif

        /* Ramp 单步：SetpointValue 向 TargetValue 逼近，供速环 / RAMPGEN 使用 */
        RC_MACRO_027(M1_speed_rc1)
        RC_MACRO_027(M1_iq_rc1)
        RC_MACRO_027(M1_id_rc1) // dqset

        //给位置环锁电机Id注入使用，控制模式10转换至8时使用
        RC_MACRO_027(Position_Lock)

        // RAMPGEN
        M1_rg1.Freq = M1_speed_rc1.SetpointValue;
        RG_MACRO_027(M1_rg1)

        // CLARKE
        CLARKE_MACRO_027(M1_clarke1)

        // PARK
        M1_park1.Alpha = M1_clarke1.Alpha;
        M1_park1.Beta = M1_clarke1.Beta;

        if (angle_channel == SPIB_channel)
        {
            M1_ResolverApplyAlignOffset();
        }
        else if (angle_channel == SPIC_channel)
        {
            M1_ResolverElecTheta_pu = Eangle_resolverC;
        }

        if (M1_Control_Mode == M1_MODE_STOP)
        {
            M1_ParkElecTheta_pu = 0;
        }
        else if (M1_Control_Mode == M1_MODE_OPENLOOP_DRAG || M1_Control_Mode == M1_MODE_OPENLOOP_IQ || M1_Control_Mode == M1_MODE_DIRECT_VOLTAGE)
        {
            M1_ParkElecTheta_pu = M1_rg1.Out;
        }
        else if (M1_Control_Mode == M1_MODE_RESOLVER_ALIGN)
        {
            M1_ParkElecTheta_pu = 0;
        }
        else if (M1_Control_Mode == M1_MODE_SPEED || M1_Control_Mode == M1_MODE_TORQUE || M1_Control_Mode == M1_MODE_POSITION)
        {
            M1_ParkElecTheta_pu = M1_ResolverElecTheta_pu;
        }
        else if (M1_Control_Mode == M1_MODE_SENSORLESS_SPEED || M1_Control_Mode == M1_MODE_SENSORLESS_TORQUE)
        {
            M1_ParkElecTheta_pu = M1_SMOangle2;
        }
        else if (M1_Control_Mode == M1_MODE_POSITION_LOCK)
        {
            M1_ParkElecTheta_pu = Position_Lock.SetpointValue;
        }
        M1_park1.Angle = M1_ParkElecTheta_pu;

        M1_park1.Sine = _IQsinPU(M1_park1.Angle);

        M1_park1.Cosine = _IQcosPU(M1_park1.Angle);

        PARK_MACRO_027(M1_park1)

        /* 转速分段 PID 增益插值：在速环/电流环 PID_MACRO 执行前更新 Kp/Ki
         * CAN_PID_adjust = 0：Settings 默认 Low/High 端点，PID_INTERP_RPM_LOW~HIGH（200~500rpm）线性过渡
         * CAN_PID_adjust = 1：CAN 0x10A002 值覆盖两端点，插值结果恒为 CAN 单组 PID */
        if (CAN_PID_adjust == 1)
        {
            PidGainSchedule_ApplyCanToBothEndpoints();
        }
        else
        {
            PidGainSchedule_RestoreDefaultEndpoints();
        }

        if (M1_Control_Mode == M1_MODE_SENSORLESS_SPEED)
        {
            PidGainSchedule_Update(M1_speed3.EstimatedSpeedRpm);
        }
        else
        {
            PidGainSchedule_Update(M1_speed1.SpeedRpm);
        }

        // PID 速度环
        if (M1_Control_Mode == M1_MODE_POSITION)
        {
            float dist = Cla_ResolverMechTheta_pu - M1_PosRef;
//注释的这部分代码功能：用于限制只做逆时针锁电机，不允许顺时针锁电机
//            if (dist < 0.0f)
//            {
//                dist += 1.0f;   /* [0, 1)，逆时针剩余角 */
//            }
            M1_pid_PosLp.Ref = dist;
            M1_pid_PosLp.Fdb = 0.0f;

            PID_MACRO_027(M1_pid_PosLp)

            if (M1_pid_PosLp.Err < 0.002 && M1_pid_PosLp.Err > -0.002)
            {
                M1_ResolverExpectedElectricalAngle();
                Position_Lock.SetpointValue = _IQ(M1_ResolverElecTheta_pu);
                M1_Control_Mode = M1_MODE_POSITION_LOCK;
            }
        }

        if (M1_SpeedLoopCount >= SpeedLoopPrescaler)
        {
            if (M1_Control_Mode == M1_MODE_POSITION)
            {
                M1_pid_SpdLp.Ref = M1_pid_PosLp.Out;
            }
            else
            {
                M1_pid_SpdLp.Ref = M1_speed_rc1.SetpointValue;
            }

            if (M1_Control_Mode == M1_MODE_SENSORLESS_SPEED)
            {
                M1_pid_SpdLp.Fdb = M1_speed3.EstimatedSpeed;
            }
            else
            {
                M1_pid_SpdLp.Fdb = M1_speed1.Speed;
            }

            if (M1_SMOswitchflg == 1)
            {
                if (M1_speed3.EstimatedSpeed < 0.6 && M1_speed3.EstimatedSpeed > 0.0)
                {
                    M1_SMOangle_autoadjust = 0.0 - _IQmpy(M1_p1, M1_speed3.EstimatedSpeed);
                }
                else if (M1_speed3.EstimatedSpeed > 0.4 && M1_speed3.EstimatedSpeed < 0.99)
                {
                    M1_SMOangle_autoadjust = -0.22;
                }
                else
                {
                    M1_SMOangle_autoadjust = 0.0;
                }
                M1_SMOangle_adjust = M1_SMOangle_autoadjust + m1_comp;
            }

            if (M1_SMOswitchflg == 0)
            {
                M1_SMOangle_adjust = M1_SMOangle_manualadjust + m1_comp;
            }

            PID_MACRO_027(M1_pid_SpdLp)
            M1_SpeedLoopCount = 1;

        }
        else
        {
            M1_SpeedLoopCount++;
        }

        if (M1_Control_Mode == M1_MODE_STOP || M1_Control_Mode == M1_MODE_OPENLOOP_DRAG || M1_Control_Mode == M1_MODE_TORQUE || M1_Control_Mode == M1_MODE_OPENLOOP_IQ || M1_Control_Mode == M1_MODE_SENSORLESS_TORQUE)
        {
            M1_pid_SpdLp.Ui = 0;
        }

        // PID Iq
        if (M1_Control_Mode == M1_MODE_STOP)
        {
            M1_pid_CurrLp_iq.Ref = 0;
            M1_pid_CurrLp_id.Ref = 0; // dqset
        }
        else if (M1_Control_Mode == M1_MODE_OPENLOOP_DRAG || M1_Control_Mode == M1_MODE_RESOLVER_ALIGN || M1_Control_Mode == M1_MODE_TORQUE || M1_Control_Mode == M1_MODE_OPENLOOP_IQ || M1_Control_Mode == M1_MODE_SENSORLESS_TORQUE)
        {
            M1_pid_CurrLp_iq.Ref = M1_iq_rc1.SetpointValue;
        }
        else if (M1_Control_Mode == M1_MODE_POSITION_LOCK)
        {
            M1_pid_CurrLp_iq.Ref = M1_IqRef_lock;
        }
        else if (M1_Control_Mode == M1_MODE_POSITION)
        {
            M1_pid_CurrLp_iq.Ref = M1_pid_SpdLp.Out;
        }
        else
        {
            if (M1_pid_SpdLp.Out < 0.0)
            {
                M1_pid_SpdLp.Out = 0.0;
                M1_pid_SpdLp.Ui = 0.0;
            }

            M1_pid_CurrLp_iq.Ref = M1_pid_SpdLp.Out;
        }

        M1_pid_CurrLp_iq.Ref = TemperatureCurrentWeaken_ClampIqReference(M1_pid_CurrLp_iq.Ref); // 过温削减限流

        M1_pid_CurrLp_iq.Fdb = M1_park1.Qs;
        PID_MACRO_027(M1_pid_CurrLp_iq)

        // PID Id
        if (M1_Control_Mode == M1_MODE_STOP)
        {
            M1_pid_CurrLp_id.Ref = M1_IdRef;
        }
        else if (M1_Control_Mode == M1_MODE_OPENLOOP_DRAG || M1_Control_Mode == M1_MODE_RESOLVER_ALIGN || M1_Control_Mode == M1_MODE_POSITION)
        {
            M1_pid_CurrLp_id.Ref = M1_id_rc1.SetpointValue; // dqset
        }
        else if (M1_Control_Mode == M1_MODE_TORQUE)
        {
            M1_pid_CurrLp_id.Ref = M1_IdRef;
        }
        else if (M1_Control_Mode == M1_MODE_POSITION_LOCK)
        {
            M1_pid_CurrLp_id.Ref = M1_IdRef_lock;
        }
        else
        {
            M1_pid_CurrLp_id.Ref = 0;
        }
        M1_pid_CurrLp_id.Fdb = M1_park1.Ds;
        PID_MACRO_027(M1_pid_CurrLp_id)

        if (M1_Control_Mode == M1_MODE_DIRECT_VOLTAGE)
        {
            M1_Ud = M_Ud;
            M1_Uq = M_Uq;

            M1_ipark1.Ds = M1_Ud;
            M1_ipark1.Qs = M1_Uq;
        }
        else
        {
            M1_ipark1.Ds = M1_pid_CurrLp_id.Out;
            M1_ipark1.Qs = M1_pid_CurrLp_iq.Out;
        }

        if (M1_Control_Mode == M1_MODE_SENSORLESS_SPEED || M1_Control_Mode == M1_MODE_SENSORLESS_TORQUE)
        {
            M1_angle_comp = _IQmpy(M1_angle_comp_value, M1_speed3.EstimatedSpeed); // 1.5 PWM period delay

            M1_ipark1.Angle = M1_ParkElecTheta_pu + M1_angle_comp; // 1.5 PWM period delay
        }
        else
        {
            M1_ipark1.Angle = M1_ParkElecTheta_pu;
        }

        M1_ipark1.Sine = _IQsinPU(M1_ipark1.Angle);
        M1_ipark1.Cosine = _IQcosPU(M1_ipark1.Angle);
        IPARK_MACRO_027(M1_ipark1)

        // SPEED_FR
        M1_speed1.ElecTheta = M1_ResolverElecTheta_pu;
        SPEED_FR_MACRO_027(M1_speed1)
        M1_speed_resolver = M1_speed1.SpeedRpm;

#ifdef SMO
        // SMO
        // Low Kslide responds better to loop transients
        // Increase Kslide for better torque response after closing the speed loop
        M1_smo1.Fsmopos = M1_SMO_F;
        M1_smo1.Gsmopos = M1_SMO_G;

        if ((M1_Control_Mode == M1_MODE_SENSORLESS_SPEED || M1_Control_Mode == M1_MODE_SENSORLESS_TORQUE) && (M1_smo1.Kslide < M1_smoK_set))  M1_smo1.Kslide += _IQ(0.00001);

        M1_smo1.Ialpha = M1_clarke1.Alpha;
        M1_smo1.Ibeta  = M1_clarke1.Beta;
        M1_smo1.Valpha = M1_ipark1.Alpha;
        M1_smo1.Vbeta  = M1_ipark1.Beta;

        SMO_MACRO_027(M1_smo1)

        M1_SMOangle1 = M1_smo1.Theta - M1_SMOangle_adjust;

        if (M1_SMOangle1 > _IQ(1.0)) M1_SMOangle2 = M1_SMOangle1 - _IQ(1.0);
        else if (M1_SMOangle1 < _IQ(0.0)) M1_SMOangle2 = M1_SMOangle1 + _IQ(1.0);
        else M1_SMOangle2 = M1_SMOangle1;

        // SPEED_EST
        M1_speed3.EstimatedTheta = M1_SMOangle2;
        SE_MACRO_027(M1_speed3)

#endif
        // SVGEN
        M1_svgen1.Ualpha = M1_ipark1.Alpha;
        M1_svgen1.Ubeta  = M1_ipark1.Beta;

        SVGENDQ_MACRO(M1_svgen1)

        // PWM
        if (M1_SVPWM_Updating == 1)
        {
            float cmpAValue = (INV_PWM_HALF_TBPRD * M1_svgen1.Ta) + INV_PWM_HALF_TBPRD;
            float cmpBValue = (INV_PWM_HALF_TBPRD * M1_svgen1.Tb) + INV_PWM_HALF_TBPRD;
            float cmpCValue = (INV_PWM_HALF_TBPRD * M1_svgen1.Tc) + INV_PWM_HALF_TBPRD;

            // Select which phase to reconstruct on the next ADC read.
            // The shortest low-side window is the least reliable sample.
            if ((cmpAValue <= cmpBValue) && (cmpAValue <= cmpCValue))
            {
                M1_reconstruct_phase = RECONSTRUCT_PHASE_A;
            }
            else if ((cmpBValue <= cmpAValue) && (cmpBValue <= cmpCValue))
            {
                M1_reconstruct_phase = RECONSTRUCT_PHASE_B;
            }
            else
            {
                M1_reconstruct_phase = RECONSTRUCT_PHASE_C;
            }

            EPWM_M1_PhaseA_Regs.CMPA.bit.CMPA = (uint16_t)cmpAValue;
            EPWM_M1_PhaseB_Regs.CMPA.bit.CMPA = (uint16_t)cmpBValue;
            EPWM_M1_PhaseC_Regs.CMPA.bit.CMPA = (uint16_t)cmpCValue;
        }
        else
        {
            if (FaultProtect_IsTripped() == 0)
            {
                M1_pid_CurrLp_iq.Ui = 0;
                M1_pid_CurrLp_id.Ui = 0;
                M1_ClearToStartMotor();
                M1_SVPWM_Updating = 1;
            }
        }

        M1_Init = 0;

    } // End if (M1_Algo_control_start == 1)
    else
    {
        M1_StopMotor();

        M1_SVPWM_Updating = 0;
    }

    /* CAN反馈转速：默认旋变实时值。当出现旋变故障且模式7时, 改用SMO估算值。 */
    if (M1_speed1.SpeedRpm < 0)
    {
        CAN_Feedback_Spd_RPM = 0;
    }
    else
    {
        CAN_Feedback_Spd_RPM = (uint16_t)M1_speed1.SpeedRpm;
    }

#if defined(SMO) && RESOLVER_FAULT_TO_SMO_ENABLE
    if (((Cla_AD2SB_Fault & 0x78u) != 0u) && (M1_Control_Mode == M1_MODE_SENSORLESS_SPEED))
    {
        if (M1_speed3.EstimatedSpeedRpm < 0)
        {
            CAN_Feedback_Spd_RPM = 0;
        }
        else
        {
            CAN_Feedback_Spd_RPM = (uint16_t)M1_speed3.EstimatedSpeedRpm;
        }
    }
#endif

    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all  = PIEACK_GROUP3;

}// MainISR Ends Here

void Parameter_Init(void)
{
    NTC_SIC_M1A = 0.0;
    NTC_SIC_M1B = 0.0;
    NTC_SIC_M1C = 0.0;
    NTC_SIC_M1A_ECAP = 0.0;
    NTC_SIC_M1B_ECAP = 0.0;
    NTC_SIC_M1C_ECAP = 0.0;

    M1_Algo_control_start = 0;
    M1_Control_Mode = M1_MODE_STOP;

    re38_throttle_sw_on = 0u;
    re38_throttle_sw_off_cnt = 0u;
    re38_volt_target_rpm = RE38_THROTTLE_IDLE_RPM;
    re38_can_target_rpm = RE38_THROTTLE_IDLE_RPM;

#ifdef SMO
    RE38_Fault_Start_Reset();
#endif

    M1_SMOswitchflg = 1;

    // Initialize the Speed module for QEP based speed calculation
    M1_speed1.K1 = _IQ21(1/(BASE_FREQ*M1_IsrSamplePeriod_s));
    M1_speed1.K2 = _IQ(1/(1+M1_IsrSamplePeriod_s*2*PI*5));  // Low-pass cut-off frequency
    M1_speed1.K3 = _IQ(1)-M1_speed1.K2;
    M1_speed1.BaseRpm = 120.0*BASE_FREQ/POLES;
    M1_speed1.Speed = 0.0;

    BaseRpm_1 = (Uint32)120*BASE_FREQ/POLES;     // Q24

    // Initialize the SPEED_EST module SMOPOS based speed calculation
    M1_speed3.K1 = _IQ21(1/(BASE_FREQ*M1_IsrSamplePeriod_s));
    M1_speed3.K2 = _IQ(1/(1+M1_IsrSamplePeriod_s*2*PI*5));  // Low-pass cut-off frequency
    M1_speed3.K3 = _IQ(1)-M1_speed3.K2;
    M1_speed3.BaseRpm = 120.0*BASE_FREQ/POLES;

    // Initialize the RAMPGEN module
    M1_rg1.StepAngleMax = _IQ(BASE_FREQ*M1_IsrSamplePeriod_s);

    // Initialize the SMOPOS constant module
    M1_smo1_const.Rs = RS;
    M1_smo1_const.Ls = LS;
    M1_smo1_const.Ib = M_BASE_CURRENT;        // 6.928;                // BASE_CURRENT;
    M1_smo1_const.Vb = M_BASE_VOLTAGE;        // 27.2;                // BASE_VOLTAGE;
    M1_smo1_const.Ts = M1_IsrSamplePeriod_s;
    SMO_CONST_MACRO_027(M1_smo1_const)

    // Initialize the SMOPOS module
    M1_smo1.Fsmopos = _IQ(M1_smo1_const.Fsmopos);
    M1_smo1.Gsmopos = _IQ(M1_smo1_const.Gsmopos);
    M1_smo1.Kslide = _IQ(0.05308703613);
    M1_smo1.Kslf = _IQ(0.09);

    /******************Enhanced SMO calculation******************/
    // Initialize eSMO parameters
    M1_esmo1.Fsmopos  = _IQ(M1_smo1_const.Fsmopos);
    M1_esmo1.Gsmopos  = _IQ(M1_smo1_const.Gsmopos);
    M1_esmo1.Kslide   = _IQ(0.05308703613);
    M1_esmo1.base_wTs = _IQ(BASE_FREQ*M1_IsrSamplePeriod_s);

    // Initialize the PI module for smo angle filter
    M1_pi_smo.Kp   = _IQ(5.0);
    M1_pi_smo.Ki   = _IQdiv(_IQ(100*0.001/ISR_FREQUENCY), M1_pi_smo.Kp);
    M1_pi_smo.Umax = _IQ(1.0);
    M1_pi_smo.Umin = _IQ(-1.0);

    /*pid for Id current loop*/
    M1_pid_CurrLp_id.Kp = _IQ(PID_LOW_ID_KP);
    M1_pid_CurrLp_id.Ki = _IQ(PID_LOW_ID_KI);
    M1_pid_CurrLp_id.Kd = _IQ(0);
    M1_pid_CurrLp_id.Kc = _IQ(0);
    M1_pid_CurrLp_id.OutMax = _IQ(1.0);
    M1_pid_CurrLp_id.OutMin = _IQ(-1.0);

    /*pid for Iq current loop*/
    M1_pid_CurrLp_iq.Kp = _IQ(PID_LOW_IQ_KP);
    M1_pid_CurrLp_iq.Ki = _IQ(PID_LOW_IQ_KI);
    M1_pid_CurrLp_iq.Kd = _IQ(0);
    M1_pid_CurrLp_iq.Kc = _IQ(0);
    M1_pid_CurrLp_iq.OutMax = _IQ(1.0);
    M1_pid_CurrLp_iq.OutMin = _IQ(-1.0);

    /*pid for speed loop*/
    M1_pid_SpdLp.Kp = _IQ(PID_LOW_SPD_KP);
    M1_pid_SpdLp.Ki = _IQ(PID_LOW_SPD_KI);
    M1_pid_SpdLp.Kd = _IQ(0);
    M1_pid_SpdLp.Kc = _IQ(0);
    M1_pid_SpdLp.OutMax = _IQ(1.5);
    M1_pid_SpdLp.OutMin = _IQ(-1.5);

    M1_pid_PosLp.Kp = _IQ(0.04);
    M1_pid_PosLp.Ki = _IQ(0.001);
    M1_pid_PosLp.Kd = _IQ(0);
    M1_pid_PosLp.Kc = _IQ(0);
    M1_pid_PosLp.OutMax = _IQ(0.002);
    M1_pid_PosLp.OutMin = _IQ(-0.002);

    /* 加载 Low/High 默认端点到增益插值模块 */
    PidGainSchedule_Init();

}

void M1_AlgoInitStart(void)
{
    M1_Control_Mode = M1_MODE_STOP;

    M1_SpeedRef = _IQ(0.0);
    M1_IqRef = _IQ(0.0);
    M1_IdRef = _IQ(0.0);

    M1_SMOangle1 = _IQ(0.0);
    M1_SMOangle2 = _IQ(0.0);
    M1_SMOangle_adjust = _IQ(0.0);
    M1_SMOangle_error = _IQ(0.0);

    M1_PosRef = 0.0;

    M1_pid_CurrLp_id.OutMax = _IQ(1.0);
    M1_pid_CurrLp_id.OutMin = _IQ(-1.0);
    M1_pid_CurrLp_id.Ref = 0;
    M1_pid_CurrLp_id.Out = 0;
    M1_pid_CurrLp_id.Err = 0;
    M1_pid_CurrLp_id.Fdb = 0;
    M1_pid_CurrLp_id.OutPreSat = 0;
    M1_pid_CurrLp_id.Up = 0;
    M1_pid_CurrLp_id.Ui = 0;

    M1_pid_CurrLp_iq.OutMax = _IQ(1.0);
    M1_pid_CurrLp_iq.OutMin = _IQ(-1.0);
    M1_pid_CurrLp_iq.Ref = 0;
    M1_pid_CurrLp_iq.Out = 0;
    M1_pid_CurrLp_iq.Err = 0;
    M1_pid_CurrLp_iq.Fdb = 0;
    M1_pid_CurrLp_iq.OutPreSat = 0;
    M1_pid_CurrLp_iq.Up = 0;
    M1_pid_CurrLp_iq.Ui = 0;

    M1_pid_SpdLp.OutMax = _IQ(1.5);
    M1_pid_SpdLp.OutMin = _IQ(-1.5);
    M1_pid_SpdLp.Ref = 0;
    M1_pid_SpdLp.Out = 0;
    M1_pid_SpdLp.Err = 0;
    M1_pid_SpdLp.Fdb = 0;
    M1_pid_SpdLp.OutPreSat = 0;
    M1_pid_SpdLp.Up = 0;
    M1_pid_SpdLp.Ui = 0;

    M1_svgen1.Ualpha = 0;
    M1_svgen1.Ubeta = 0;

    M1_speed_rc1.RampDelayMax = M1_RAMP_DELAY_MIN;
    M1_speed_rc1.SetpointValue = _IQ(0.0);
    M1_speed_rc1.TargetValue = _IQ(0.0);
    M1_speed_rc1.EqualFlag = 0;
    M1_speed_rc1.RampDelayCount = 0;

    M1_iq_rc1.SetpointValue = _IQ(0.0);
    M1_iq_rc1.TargetValue = _IQ(0.0);
    M1_iq_rc1.EqualFlag = 0;
    M1_iq_rc1.RampDelayCount = 0;
    M1_id_rc1.SetpointValue = _IQ(0.0);
    M1_id_rc1.TargetValue = _IQ(0.0);
    M1_id_rc1.EqualFlag = 0;
    M1_id_rc1.RampDelayCount = 0;


    Position_Lock.SetpointValue = _IQ(0.0);
    Position_Lock.TargetValue = _IQ(0.0);
    Position_Lock.EqualFlag = 0;
    Position_Lock.RampDelayCount = 0;



    M1_rg1.Angle = _IQ(0);

    M1_Control_Mode = M1_MODE_STOP;
    M1_speed1.Speed = 0;
    M1_speed1.ElecTheta = M1_ResolverElecTheta_pu;
    M1_speed1.DirectionQep = 1;
    M1_speed1.OldElecTheta = M1_speed1.ElecTheta;
    M1_speed_resolver = 0.0f;
    M1_speed3.EstimatedSpeed = 0;
    M1_speed3.OldEstimatedTheta = 0;
    M1_speed3.EstimatedTheta = 0;
    lock_flag = 0;

    M1_Init = 1;
}

/*------------------------------------------------------------------------------
 RE38 模拟油门映射（主循环约 100Hz）：0.25~5V → RE38_THROTTLE_IDLE~MAX_RPM
 丢失/超量程两路独立检测（钳位前原始电压）；越限冻转速，回窗后继续映射。
 ISR 门控见 MotorCtrl.c RE38_Throttle_ApplyInIsr。
------------------------------------------------------------------------------*/
void RE38_VoltThrottle_UpdateFromAdc(void)
{
    float v;
    float v_lo;
    float v_hi;
    float span;
    float t;
    float rpm_mapped;
    float v_throttle;

    v_throttle = AdcdResultRegs.ADCRESULT3 * 3.3f * ADC_PU_SCALE_FACTOR / 1.6f * 2.5f;

    /* 超量程判断 */
    if ((FAULT_RE38_VOLT_OVERRANGE_V > 0.0f) && (v_throttle > FAULT_RE38_VOLT_OVERRANGE_V))
    {
        re38_volt_overrange_tick++;
        if (re38_volt_overrange_tick >= FAULT_RE38_VOLT_HOLD_TICKS)
        {
            fault_re38_volt_overrange = 1u;
            re38_volt_overrange_tick = FAULT_RE38_VOLT_HOLD_TICKS;
        }
    }
    else
    {
        re38_volt_overrange_tick = 0u;
        fault_re38_volt_overrange = 0u;
    }

    /* 电压丢失判断 */
    if ((FAULT_RE38_VOLT_LOST_V > 0.0f) && (v_throttle < FAULT_RE38_VOLT_LOST_V))
    {
        re38_volt_lost_tick++;
        if (re38_volt_lost_tick >= FAULT_RE38_VOLT_HOLD_TICKS)
        {
            fault_re38_volt_lost = 1u;
            re38_volt_lost_tick = FAULT_RE38_VOLT_HOLD_TICKS;
        }
    }
    else
    {
        re38_volt_lost_tick = 0u;
        fault_re38_volt_lost = 0u;
    }

    if ((re38_volt_overrange_tick == 0u) && (re38_volt_lost_tick == 0u))
    {
        v = v_throttle;
        if (v > RE38_VOLT_THROTTLE_V_MAX)
        {
            v = RE38_VOLT_THROTTLE_V_MAX;
        }

        v_lo = RE38_VOLT_THROTTLE_LINEAR_LO;
        v_hi = RE38_VOLT_THROTTLE_LINEAR_HI;

        if (v <= v_lo)
            rpm_mapped = RE38_THROTTLE_IDLE_RPM;
        else if (v >= v_hi)
            rpm_mapped = RE38_THROTTLE_MAX_RPM;
        else
        {
            span = v_hi - v_lo;
            t = (v - v_lo) / span;
            rpm_mapped = RE38_THROTTLE_IDLE_RPM
                + t * (RE38_THROTTLE_MAX_RPM - RE38_THROTTLE_IDLE_RPM);
        }
        re38_volt_target_rpm = RE38_ClampTargetRpm(rpm_mapped);
    }
}


static float ia2_sum, ib2_sum, ic2_sum;
static uint16_t rms_n;
float Irms_a, Irms_b, Irms_c, Irms_average;   /* 安培，给 CCS / CAN 看 */
void Current_and_voltage_measurement(void)
{
    float ia_raw, ib_raw, ic_raw = 0;

    analog1 = AdcdResultRegs.ADCRESULT3 * ADC_PU_SCALE_FACTOR * 5.15625f; /* = AdcdResultRegs.ADCRESULT3*3.3*ADC_PU_SCALE_FACTOR/1.6*2.5; */
    analog2 = AdcdResultRegs.ADCRESULT0 * ADC_PU_SCALE_FACTOR * 5.15625f; /* = AdcdResultRegs.ADCRESULT0*3.3*ADC_PU_SCALE_FACTOR/1.6*2.5; */
    analog3 = AdccResultRegs.ADCRESULT2 * ADC_PU_SCALE_FACTOR * 5.15625f; /* = AdccResultRegs.ADCRESULT2*3.3*ADC_PU_SCALE_FACTOR/1.6*2.5; */
    DC_15V = AdcdResultRegs.ADCRESULT2 * ADC_PU_SCALE_FACTOR * 36.3f; /* = AdcdResultRegs.ADCRESULT2*3.3*ADC_PU_SCALE_FACTOR*11.0 */

    ia_raw = (offset_M1_PhaseA - (((float)Cur_M1_PhaseA_Raw)*ADC_PU_SCALE_FACTOR))*Cur_ADC_PU_SCALE_FACTOR*adc_calib;
    ib_raw = (offset_M1_PhaseB - (((float)Cur_M1_PhaseB_Raw)*ADC_PU_SCALE_FACTOR))*Cur_ADC_PU_SCALE_FACTOR*adc_calib;
    ic_raw = (offset_M1_PhaseC - (((float)Cur_M1_PhaseC_Raw)*ADC_PU_SCALE_FACTOR))*Cur_ADC_PU_SCALE_FACTOR*adc_calib;

    /* 重建 As/Bs/Cs 之前： */
    ia2_sum += ia_raw * ia_raw;
    ib2_sum += ib_raw * ib_raw;
    ic2_sum += ic_raw * ic_raw;
    if(PHASE_RMS_WIN<20)
    {
        PHASE_RMS_WIN=20;//1ms
    }

    if (++rms_n >= PHASE_RMS_WIN)
    {
        float inv = 1.0f / (float)rms_n;

        Irms_a = sqrtf(ia2_sum * inv) * (float)BASE_CURRENT;
        Irms_b = sqrtf(ib2_sum * inv) * (float)BASE_CURRENT;
        Irms_c = sqrtf(ic2_sum * inv) * (float)BASE_CURRENT;
        Irms_average = (Irms_a+Irms_b+Irms_c)*0.3333;
        ia2_sum = ib2_sum = ic2_sum = 0.0f;
        rms_n = 0u;
    }
    // Reconstruct the phase whose low-side window was shortest for this ADC
    // sample. The selected phase comes from the previous PWM update.
    if (M1_reconstruct_phase == RECONSTRUCT_PHASE_A)
    {
        M1_PhaseCurrent.Bs = ib_raw;
        M1_PhaseCurrent.Cs = ic_raw;
        M1_PhaseCurrent.As = -M1_PhaseCurrent.Bs - M1_PhaseCurrent.Cs;
    }
    else if (M1_reconstruct_phase == RECONSTRUCT_PHASE_B)
    {
        M1_PhaseCurrent.As = ia_raw;
        M1_PhaseCurrent.Cs = ic_raw;
        M1_PhaseCurrent.Bs = -M1_PhaseCurrent.As - M1_PhaseCurrent.Cs;
    }
    else
    {
        M1_PhaseCurrent.As = ia_raw;
        M1_PhaseCurrent.Bs = ib_raw;
        M1_PhaseCurrent.Cs = -M1_PhaseCurrent.As - M1_PhaseCurrent.Bs;
    }

    M1_clarke1.As = M1_PhaseCurrent.As * 1.0;
    M1_clarke1.Bs = M1_PhaseCurrent.Bs * 1.0;

    M1_volt1.DcBusVolt = M1_Volt_DCBUS * ADC_PU_SCALE_FACTOR * Volt_ADC_PU_SCALE_FACTOR * VOLT_ADC_BASE /1.6;
    M1_real_voltage = M1_volt1.DcBusVolt;

}

void Board_temp(void)
{
    M1_Temp_Sensor_out_mV = M1_Temp_Board_Raw * ADC_RES2mVOL;

    Temp_board = ((13.582-sqrt((-13.582)*(-13.582)+4*0.00433*(2230.8-M1_Temp_Sensor_out_mV)))/(2*(-0.00433)))+30;

    /* 控制板过温警告:主循环≈10ms/次,10 tick≈100ms,降温自动清除 */
    if (Temp_board > FAULT_BOARD_OVERTEMP_C)
    {
        board_overtemp_ticker++;
        if (board_overtemp_ticker >= FAULT_BOARD_OVERTEMP_COUNT)
        {
            fault_board_overtemp = 1;
            DTC_code1 |= FAULT_DTC_BOARD_OVERTEMP;
            board_overtemp_ticker = FAULT_BOARD_OVERTEMP_COUNT;
        }
    }
    else
    {
        board_overtemp_ticker = 0;
        fault_board_overtemp = 0;
        DTC_code1 &= (uint16_t)~FAULT_DTC_BOARD_OVERTEMP;
    }
}

/******************************************************************************
 * M1_ResolverApplyAlignOffset
 * 旋变电角度：CLA 原始 pu + 对齐 offset，折算到 [0,1)；停波/运行 ISR 共用
 ******************************************************************************/
static void M1_ResolverApplyAlignOffset(void)
{
    float resolver_elec_frac;
    float resolver_elec_aligned;

    resolver_elec_frac = _IQ15frac(Cla_ResolverElecTheta_pu);
    resolver_elec_aligned = _IQ15frac(1.0 + resolver_elec_frac + M1_Align_Angle_Offset_pu);

    if ((resolver_elec_aligned >= _IQ15(0.0)) && (resolver_elec_aligned <= _IQ15(1.0)))
    {
        M1_ResolverElecTheta_pu = resolver_elec_aligned;
    }
}

/******************************************************************************
 * M1_ResolverExpectedElectricalAngle
 * 期望电角度：期望的机械角度 + 对齐 offset，折算到 [0,1)；停波/运行 ISR 共用
 ******************************************************************************/
static void M1_ResolverExpectedElectricalAngle(void)
{
    float resolver_Expectedelec_frac;
    float resolver_Expectedelec_aligned;
    float resolver_ExpectedElecTheta_pu = (-1.0 * M1_PosRef) * (POLES / 2.0);

    resolver_Expectedelec_frac = _IQ15frac(resolver_ExpectedElecTheta_pu);
    resolver_Expectedelec_aligned = _IQ15frac(1.0 + resolver_Expectedelec_frac + M1_Align_Angle_Offset_pu);

    if ((resolver_Expectedelec_aligned >= _IQ15(0.0)) && (resolver_Expectedelec_aligned <= _IQ15(1.0)))
    {
        Position_Lock.TargetValue = resolver_Expectedelec_aligned;
    }
}


/******************************************************************************
 * TemperatureCurrentWeaken_ClampIqReference
 * 20kHz ISR 内调用限幅：读取后台 SiC/Motor 过温削减的允许电流百分比，取更小值
 * 仅获取主循环中计算好的最大限幅值，函数内无过温削减算法计算来减轻ISR压力。函数返回值为限制后的 Iq Reference（pu）值
 ******************************************************************************/
static float TemperatureCurrentWeaken_ClampIqReference(float iq_ref)
{
#if TEMPERATURE_CURRENT_WEAKEN_ENABLE
    float sic_allowed_percent;
    float motor_allowed_percent;
    float allowed_percent;
    float iq_limit_pu;

    sic_allowed_percent = SicCurrentWeaken_GetAllowedPercent();
    motor_allowed_percent = MotorCurrentWeaken_GetAllowedPercent();

    allowed_percent = sic_allowed_percent;
    if (motor_allowed_percent < allowed_percent)
    {
        allowed_percent = motor_allowed_percent;
    }

    iq_limit_pu = THERMAL_CURRENT_WEAKEN_IQ_MAX_PU * allowed_percent * 0.01f;

    if (iq_ref > iq_limit_pu)
    {
        return iq_limit_pu;
    }
    if (iq_ref < -iq_limit_pu)
    {
        return -iq_limit_pu;
    }
#endif

    return iq_ref;
}

/*------------------------------------------------------------------------------
 MotorRestart_ClearFaults
 清除可恢复软件/硬件 DTC 锁存，并输出驱动 RI 复位脉冲。
 调用路径（主循环 MotorRestart_Sequence_Handle，须存在 DTC）：
   - HOST_CAN：0x10A001 CAN_torque_pwm_reset 上升沿
   - RE38：GPIO63 油门关断确认后 re38_throttle_sw_on 1→0
 清除后由 M1_RestartTicker 配合 RestartHold_ApplyInIsr 封波约 500ms。
------------------------------------------------------------------------------*/
static void MotorRestart_ClearFaults(void)
{
    /* 软件保护域：过压/过流/超速；欠压警告可自清，重启亦清 */
    FaultOvervoltage_Clear();
    FaultOvercurrent_Clear();
    FaultOverspeed_Clear();
    FaultUndervoltage_Clear();

    /* 控制板过温警告可清；旋变故障由 CLA 跨上电周期锁存，不在重启时清除。 */
    fault_board_overtemp = 0;
    board_overtemp_ticker = 0;
    DTC_code1 &= (uint16_t)~FAULT_DTC_BOARD_OVERTEMP;

    /* 硬件诊断域 DTC2/DTC3 */
    FaultHardware_Clear();

    /* 过温：内部判断当前是否允许清除 */
    Overtemp_SicClearFaultsOnRestart();
    Overtemp_MotorClearFaultsOnRestart();

    /* U/V/W 驱动芯片 RI 复位脉冲（低→高→低，10us） */
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(13,0,0);
    GpioDataRegs.GPASET.bit.GPIO13 = 1;
    // GPIO87 --- V1-RI
    GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(87,0,0);
    GpioDataRegs.GPCSET.bit.GPIO87 = 1;
    // GPIO83 --- W1-RI
    GPIO_SetupPinOptions(83, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(83,0,0);
    GpioDataRegs.GPCSET.bit.GPIO83 = 1;

    DELAY_US(100);

    // GPIO13 --- U1-RI
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(13,0,0);
    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
    // GPIO87 --- V1-RI
    GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(87,0,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;
    // GPIO83 --- W1-RI
    GPIO_SetupPinOptions(83, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(83,0,0);
    GpioDataRegs.GPCCLEAR.bit.GPIO83 = 1;
}

/*------------------------------------------------------------------------------
 MotorRestart_Sequence_Handle（主循环调用，非阻塞）
 检测故障重启触发沿 → MotorRestart_ClearFaults → 置 M1_RestartTicker。
 封波时长：50 次主循环递减 ≈ 500ms（主循环 DELAY_US(10000) 约 10ms/次）。
 ISR 侧由 RestartHold_ApplyInIsr 在倒计时内强制 lsw/转速/电流给定为零。
------------------------------------------------------------------------------*/
static void MotorRestart_Sequence_Handle(void)
{
    /* --- HOST_CAN：上位机 CAN_torque_pwm_reset 上升沿 --- */
    if (throttle_type == HOST_CAN_THROTTLE)
    {
        static uint16_t prev_torque_pwm_reset = 0;
        uint16_t now_torque_pwm_reset;

        now_torque_pwm_reset = (uint16_t)(CAN_torque_pwm_reset & 1u);

        /* 未在封波倒计时内，且本帧 bit = 1、上帧=0，且存在 DTC */
        if (M1_RestartTicker == 0 && now_torque_pwm_reset == 1u && prev_torque_pwm_reset == 0u)
        {
            if (DTC_code1 != 0 || DTC_code2 != 0 || DTC_code3 != 0)
            {
                MotorRestart_ClearFaults();
                M1_RestartTicker = 50; /* 主循环约10ms/次，50次≈500ms FOC封波 */
            }
        }

        prev_torque_pwm_reset = now_torque_pwm_reset;
    }
    /* --- RE38：GPIO63 关断确认后 re38_throttle_sw_on 下降沿 --- */
    else if (throttle_type == RE38_VOLT_THROTTLE || throttle_type == RE38_CAN_THROTTLE)
    {
        static uint16_t prev_re38_sw_on = 0u;
        uint16_t now_re38_sw_on = re38_throttle_sw_on;

        /* now == 0 表示 ISR 已完成 RE38_THROTTLE_SW_OFF_FILTER_TICKS 关断滤波 */
        if (M1_RestartTicker == 0 && prev_re38_sw_on == 1u && now_re38_sw_on == 0u)
        {
            if (DTC_code1 != 0 || DTC_code2 != 0 || DTC_code3 != 0)
            {
                MotorRestart_ClearFaults();
                M1_RestartTicker = 50; /* 主循环约10ms/次，50次≈500ms FOC封波 */
            }
        }

        prev_re38_sw_on = now_re38_sw_on;
    }

    /* --- 公共：所有 throttle_type 均递减（封波结束后允许再次触发） --- */
    if (M1_RestartTicker > 0)
    {
        M1_RestartTicker--;
    }
}

/****************************************************************************
 * End of Code *
 * ***************************************************************************
 */

