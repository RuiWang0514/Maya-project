#ifndef HPMC_SETTINGS_H
#define HPMC_SETTINGS_H

/*------------------------------------------------------------------------------
 油门输入源（运行时分支；在 HPMC_main_80.c 中修改 throttle_type，三选一）
   HOST_CAN_THROTTLE  ：上位机 0x10A001（CAN_speed_set、CAN_Enable、CAN_control_command）
   RE38_VOLT_THROTTLE ：GPIO63 开后怠速；LINEAR_LO~LINEAR_HI 电压线性映射到 IDLE~MAX
   RE38_CAN_THROTTLE  ：GPIO63 开后；0x18EF0010 第 7、8 字节转速×SCALE 后钳位 IDLE~MAX
------------------------------------------------------------------------------*/
typedef enum
{
    HOST_CAN_THROTTLE = 0,
    RE38_VOLT_THROTTLE = 1,
    RE38_CAN_THROTTLE = 2
} ThrottleType_e;

extern ThrottleType_e throttle_type;

/*------------------------------------------------------------------------------
 M1 控制模式（控制模式变量 M1_Control_Mode 的具名取值）：由 CAN/电压油门门控选择
------------------------------------------------------------------------------*/
typedef enum
{
    M1_MODE_STOP              = 0,   /* 停止/停机 */
    M1_MODE_OPENLOOP_DRAG     = 1,   /* 开环强拖启动 */
    M1_MODE_TORQUE            = 2,   /* 扭矩闭环 */
    M1_MODE_SPEED             = 3,   /* 速度闭环 */
    M1_MODE_DIRECT_VOLTAGE    = 4,   /* 直接 Ud/Uq 电压注入 */
    M1_MODE_OPENLOOP_IQ       = 5,   /* 开环角度 + iq 电流控制 */
    M1_MODE_RESOLVER_ALIGN    = 6,   /* 旋变对角度模式 */
    M1_MODE_SENSORLESS_SPEED  = 7,   /* 无感(SMO)速度闭环 */
    M1_MODE_POSITION_LOCK     = 8,   /* 位置锁定/保持 */
    M1_MODE_SENSORLESS_TORQUE = 9,   /* 无感(SMO)扭矩控制 */
    M1_MODE_POSITION          = 10   /* 位置闭环 */
} M1_ControlMode_e;

extern M1_ControlMode_e M1_Control_Mode;

/*------------------------------------------------------------------------------
 RE38 油门转速限幅（模拟/CAN 共用）+ CAN 比例系数
 两套设备切换时改下面三处宏（保持 IDLE/MAX 与 SCALE 匹配）：
   Kit A 120~2600：IDLE=120, MAX=2600, SCALE=1.084f  （RE38 发 110→120，发 2400→2600）
   Kit B 120~2400：IDLE=120, MAX=2400, SCALE=1.0f
------------------------------------------------------------------------------*/
#define RE38_THROTTLE_IDLE_RPM        (120.0f)
#define RE38_THROTTLE_MAX_RPM         (2600.0f)

/* RE38 模拟电压线性区：LINEAR_LO~LINEAR_HI 映射 RE38_THROTTLE_IDLE_RPM~RE38_THROTTLE_MAX_RPM；
 * V_MAX 为输入电压上限钳位，高于 LINEAR_HI 的部分保持满速 */
#define RE38_VOLT_THROTTLE_V_MAX      (5.0f)
#define RE38_VOLT_THROTTLE_LINEAR_LO  (0.25f)
#define RE38_VOLT_THROTTLE_LINEAR_HI  (4.5f)

/* RE38 CAN 转速帧：ID 0x18EF0010，Byte7 高字节 / Byte8 低字节，1:1 后再 × SCALE */
#define RE38_CAN_MSG_ID                   (0x18EF0010u)
#define RE38_CAN_RPM_SCALE                (1.084f)

/* RE38 油门使能开关：GPIO63 低有效 */
#define RE38_THROTTLE_SW_GPIO             (63u)
#define RE38_THROTTLE_SW_OFF_FILTER_TICKS (200u)    /* 10ms @ 20kHz ISR */

#define MPMOTOR

/*-------------------------------------------------------------------------------
Include project specific include files.
-------------------------------------------------------------------------------*/
// define math type as float(1)
#define   MATH_TYPE      1
#include <stdint.h>
#include "fault_detect.h"
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
#include "canHandle.h"
/* ---- FWLib headers (via CCS include path) ---- */
#include "F2837xD_gpio.h"
#include "pid_reg3_027.h"
/* ---- Algs layer headers ---- */
#include <math.h>
#include "fast_math.h"

#include "F2837xD_sdfm_drivers.h"
#include "F2837x_QEP_Module.h"
#include "Resolver_Float.h"

/* ---- SPI Config ---- */
#define SPIB_channel 0
#define SPIC_channel 1

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

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 20

// Define the ISR period (us)
#define ISR_PERIOD (1*1000/ISR_FREQUENCY)
#define INV_PWM_TICKS  ((float)TBCLK_FREQUENCY/ISR_FREQUENCY)*1000

// Define Dead time (us)
#define DEAD_TIME_VALUE 0.5 //500 nSec

#define DEAD_TIME INV_PWM_TICKS*DEAD_TIME_VALUE/ISR_PERIOD

// ADC current sampling delay after PWM counter zero, in us.
// For low-side current sensing, sample shortly after dead-time while
// retaining more margin inside the 000 zero-vector window.
#define ADC_SAMPLE_BLANKING_US 0.3
#define ADC_SAMPLE_DELAY_US (DEAD_TIME_VALUE + ADC_SAMPLE_BLANKING_US)
#define ADC_SAMPLE_DELAY_TICKS (INV_PWM_TICKS*ADC_SAMPLE_DELAY_US/ISR_PERIOD)

#define INV_PWM_HALF_TBPRD INV_PWM_TICKS/4

/*------------------------------------------------------------------------------
转速分段线性插值 PID（Low/High 端点 + 中间过渡，见 pid_gain_schedule.c）
  rpm <= PID_INTERP_RPM_LOW                 ：Low 端点
  PID_INTERP_RPM_LOW ~ PID_INTERP_RPM_HIGH  ：Low 与 High 线性插值
  rpm >= PID_INTERP_RPM_HIGH                ：High 端点
------------------------------------------------------------------------------*/
#define PID_INTERP_RPM_LOW          200
#define PID_INTERP_RPM_HIGH         500

/* Low 端点（rpm <= PID_INTERP_RPM_LOW） */
#define PID_LOW_SPD_KP    1.50f
#define PID_LOW_SPD_KI    0.002f
#define PID_LOW_ID_KP     0.635f
#define PID_LOW_ID_KI     0.0255f
#define PID_LOW_IQ_KP     0.635f
#define PID_LOW_IQ_KI     0.0255f

/* High 端点（rpm >= PID_INTERP_RPM_HIGH） */
#define PID_HIGH_SPD_KP   1.50f
#define PID_HIGH_SPD_KI   0.005f
#define PID_HIGH_ID_KP    0.635f
#define PID_HIGH_ID_KI    0.0255f
#define PID_HIGH_IQ_KP    0.635f
#define PID_HIGH_IQ_KI    0.0255f

/* 机械超速保护阈值 [rpm] */
/*------------------------------------------------------------------------------
 速度 Ramp Smoothstep 软着陆（MotorControlISR @ 20kHz 调用 MotorCtrl.c）

 【斜率】∝ RampStep / RampDelayMax；Step=1 时 Delay 越大越慢。
 【分段】remain≥LAND_START → DELAY_MIN；LAND_END~START → Smoothstep；≤END → DELAY_MAX。
 【剩余 rpm】仅 |Target−Setpoint|×BaseRpm，不读 SpeedRpm/Fbk。
 【pu 标定】BaseRpm 为 pu→rpm 满量程标定（非当前转速）。
------------------------------------------------------------------------------*/
#define M1_RAMP_SOFT_LAND_ENABLE    (1u)

#define M1_RAMP_STEP                (1u)
#define M1_RAMP_DELAY_MIN           (2u)      /* 远区最快：等效 1/2 */
#define M1_RAMP_DELAY_MAX           (8u)      /* 近区最慢：等效 1/8 */
#define M1_RAMP_LAND_START_RPM      (150.0f)  /* 剩余 ≥ 此 rpm：DELAY_MIN */
#define M1_RAMP_LAND_END_RPM        (50.0f)   /* 剩余 ≤ 此 rpm：DELAY_MAX */
#define M1_RAMP_DELAY_SLEW_STEP     (1u)      /* 每拍 DelayMax 最多变化 ± 此值，用于顺滑曲线；0=直接跟随 Smoothstep */

// Define the electrical motor parametes (Estun Servomotor)
#define RS      0.1f         // 相电阻 (ohm) — RuiXiang 标定
#define LS      0.0006f      // 相电感 (H) — RuiXiang 标定
#define POLES   40           // Maya 80 Motor

// Define the base quantites
#define VOLT_ADC_BASE   100    // 电压 ADC 采样标幺基准 (V),仅用于母线电压测量缩放
#define BASE_FREQ       1200   // Base electrical frequency (Hz)

#define BASE_CURRENT    700    // Base peak phase current (amp)
#define M_BASE_VOLTAGE  100
#define M_BASE_CURRENT  450

/*------------------------------------------------------------------------------
Current sensors scaling
------------------------------------------------------------------------------*/
#define RESOLUTION_12BIT   0
#define RESOLUTION_16BIT   1
#define SIGNAL_SINGLE   0

#define ADC_PU_SCALE_FACTOR   0.000244200244     ///1/(2^12-1)

#define ADC_REFS       3.3
#define ADC_RES2mVOL   ADC_REFS*ADC_PU_SCALE_FACTOR*1000

#define Volt_upper_R   1245.0 //kohm
#define Volt_lower_R   2.0

#define Volt_ADC_PU_SCALE_FACTOR ADC_REFS*(Volt_upper_R+Volt_lower_R)/Volt_lower_R/VOLT_ADC_BASE

#define M1_Volt_DCBUS AdcdResultRegs.ADCRESULT1

#define M1_Temp_PhaseA_Raw AdccResultRegs.ADCRESULT1
#define M1_Temp_PhaseB_Raw AdcbResultRegs.ADCRESULT1
#define M1_Temp_PhaseC_Raw AdcdResultRegs.ADCRESULT4

#define Mea_PTA1_Raw AdcaResultRegs.ADCRESULT5
#define Mea_PTA2_Raw AdcaResultRegs.ADCRESULT2
#define Mea_PTB1_Raw AdcbResultRegs.ADCRESULT4
#define Mea_PTB2_Raw AdccResultRegs.ADCRESULT5
#define M1_Temp_Board_Raw AdcaResultRegs.ADCRESULT1

//for maya80 800A current sensor
#define Cur_Amp_Gain 0.00166666    //Gain of ADC amplifier  INA240A2---GAIN=50

#define Cur_ADC_PU_SCALE_FACTOR (ADC_REFS/Cur_Amp_Gain)/BASE_CURRENT

/*------------------------------------------------------------------------------
 相 - 硬件资源对应表（按板级走线，PWM 与 ADC 的交叉方式不同）

   逻辑相   EPWM 模块   PWM 输出引脚      电流 ADC 单元
   A        EPwm3       GPIO4 / GPIO5     ADC-A
   B        EPwm2       GPIO2 / GPIO3     ADC-C
   C        EPwm1       GPIO0 / GPIO1     ADC-B

 GPIO0/1 上看到的是 C 相；EPwm1Regs 的 CMPA / TZFLG 也属于 C 相。
 PWM_1ch_UpDwnCnt_CNF(n, ...) 的 n 是 EPWM 模块号，不是相号。
------------------------------------------------------------------------------*/
#define Cur_M1_PhaseA_Raw AdcaResultRegs.ADCRESULT0
#define Cur_M1_PhaseB_Raw AdccResultRegs.ADCRESULT0
#define Cur_M1_PhaseC_Raw AdcbResultRegs.ADCRESULT0

#define EPWM_M1_PhaseC_Regs EPwm1Regs
#define EPWM_M1_PhaseB_Regs EPwm2Regs
#define EPWM_M1_PhaseA_Regs EPwm3Regs

/*------------------------------------------------------------------------------
 Over-temperature trip/clear thresholds (hysteresis, deg C)
 SIC_OVERTEMP_CLEAR_C / MOTOR_OVERTEMP_CLEAR_C：Overtemp_*ClearFaultsOnRestart 门控
------------------------------------------------------------------------------*/
#define SIC_OVERTEMP_TRIP_C       (95.0f)
#define SIC_OVERTEMP_CLEAR_C      (90.0f)
#define MOTOR_OVERTEMP_TRIP_C     (160.0f)
#define MOTOR_OVERTEMP_CLEAR_C    (150.0f)
#define OVERTEMP_SIC_FAULT_CONFIRM    (2u)   /* SiC_over_temp_ticker > 1 */
#define OVERTEMP_MOTOR_FAULT_CONFIRM  (3u)   /* motor_overtemp_ticker > 2 */

/* 电机 PT100 过温监测：1=启用故障/DTC/停机，0=仅保留测温遥测、不判过温 */
#define MOTOR_OVERTEMP_ENABLE         (1)

/*------------------------------------------------------------------------------
 过温电流削减标定（削减阈值单位 °C / %）
 温度升高时在硬故障停机前线性降低允许的最大 Iq（pu 满量程百分比）。
------------------------------------------------------------------------------*/
/* 电机PT100 & 控制器SiC的过温削减使能 */
#define TEMPERATURE_CURRENT_WEAKEN_ENABLE          (0)

#define THERMAL_CURRENT_WEAKEN_IQ_MAX_PU           (1.0f)

/* SiC NTC测温：从 START 到 END 温度，允许电流削减从 START_PERCENT 线性降至 END_PERCENT */
#define SIC_CURRENT_WEAKEN_START_TEMPERATURE_C     (90.0f)
#define SIC_CURRENT_WEAKEN_END_TEMPERATURE_C       (95.0f)
#define SIC_CURRENT_WEAKEN_START_PERCENT           (100.0f)
#define SIC_CURRENT_WEAKEN_END_PERCENT             (0.0f)

/* 电机 PT100测温：从 START 到 END 温度，允许电流削减从 START_PERCENT 线性降至 END_PERCENT */
#define MOTOR_CURRENT_WEAKEN_START_TEMPERATURE_C   (150.0f)
#define MOTOR_CURRENT_WEAKEN_END_TEMPERATURE_C     (160.0f)
#define MOTOR_CURRENT_WEAKEN_START_PERCENT         (100.0f)
#define MOTOR_CURRENT_WEAKEN_END_PERCENT           (0.0f)

/* 旋变故障切无感：LOT/DOS/LOS 故障触发时从旋变速度模式切到 SMO 无感速度模式，CAN 反馈转速也改用 SMO 观测值；0=关闭 */
#define RESOLVER_FAULT_TO_SMO_ENABLE  (0)

#endif
