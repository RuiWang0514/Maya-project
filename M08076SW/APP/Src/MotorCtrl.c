/******************************************************************************
 * @file    MotorCtrl.c
 * @brief   APP - Motor stop and protection functions
 * @layer   APP
 * @note    Motor Control Functions.
 ******************************************************************************/
#include "HPMC_Settings.h"
#include "MotorCtrl.h"

/* 故障清除后 FOC 封波倒计时（主循环递减；>0 时 RestartHold_ApplyInIsr 压控制模式/给定） */
volatile uint32_t M1_RestartTicker = 0;

#ifdef _FLASH
#pragma CODE_SECTION(M1_StopMotor,"ramfuncs");
#pragma CODE_SECTION(M1_RampSoftLandingUpdate,"ramfuncs");
#pragma CODE_SECTION(M1_RampApplyDelayMax,"ramfuncs");
#pragma CODE_SECTION(MotorISR_Throttle_Apply,"ramfuncs");
#pragma CODE_SECTION(HostCanThrottle_ApplyInIsr,"ramfuncs");
#pragma CODE_SECTION(RE38_Throttle_ApplyInIsr,"ramfuncs");
#ifdef SMO
#pragma CODE_SECTION(RE38_Fault_Start_Update,"ramfuncs");
#endif
#endif

/* --- Extern declarations for globals defined in HPMC_main_80.c --- */
extern ThrottleType_e throttle_type;
extern volatile float re38_volt_target_rpm;
extern volatile float re38_can_target_rpm;
extern uint16_t re38_throttle_sw_on;
extern uint16_t re38_throttle_sw_off_cnt;

extern uint16_t M1_Algo_control_start;
extern M1_ControlMode_e M1_Control_Mode;
extern uint16_t M1_SVPWM_Updating;

extern float M1_SpeedRef;
extern float M1_IqRef;
extern float M1_IdRef;
extern uint32_t M1_iq_RampControl;
extern uint32_t M1_id_RampControl;
extern uint32_t BaseRpm_1;
extern uint16_t CAN_speed_set;
extern uint16_t CAN_control_command;
extern uint16_t CAN_Enable;
extern uint16_t CAN_PWM_Enable;
extern int CAN_iqref;
extern PIDREG3_027 M1_pid_CurrLp_iq;

#ifdef SMO
typedef enum
{
    RE38_FAULT_START_NOT_ACTIVE = 0,
    RE38_FAULT_START_OPENLOOP_DRAG,
    RE38_FAULT_START_SMO_CLOSED_LOOP
} RE38_FAULT_START_STATE_e;

static RE38_FAULT_START_STATE_e re38_fault_start_state = RE38_FAULT_START_NOT_ACTIVE;
static uint16_t re38_throttle_sw_was_on = 0u;
static uint16_t re38_smo_speed_stable_ticks = 0u;

static float RE38_Fault_Start_GetTargetSpeedPu(void)
{
    float target_rpm;

    if (throttle_type == RE38_VOLT_THROTTLE)
    {
        target_rpm = re38_volt_target_rpm;
    }
    else
    {
        target_rpm = re38_can_target_rpm;
    }

    return RE38_ClampTargetRpm(target_rpm) / (float)BaseRpm_1;
}

void RE38_Fault_Start_Reset(void)
{
    re38_fault_start_state = RE38_FAULT_START_NOT_ACTIVE;
    re38_throttle_sw_was_on = 0u;
    re38_smo_speed_stable_ticks = 0u;
}

void RE38_Fault_Start_Update(uint16_t resolver_fault_active,
                             int32_t smo_estimated_speed_rpm)
{
    if ((throttle_type != RE38_VOLT_THROTTLE) && (throttle_type != RE38_CAN_THROTTLE))
    {
        RE38_Fault_Start_Reset();
        return;
    }

    if (resolver_fault_active == 0u)
    {
        re38_fault_start_state = RE38_FAULT_START_NOT_ACTIVE;
        re38_smo_speed_stable_ticks = 0u;
        re38_throttle_sw_was_on = re38_throttle_sw_on;
        return;
    }

    if (re38_throttle_sw_on == 0u)
    {
        re38_fault_start_state = RE38_FAULT_START_NOT_ACTIVE;
        re38_smo_speed_stable_ticks = 0u;
        re38_throttle_sw_was_on = 0u;
        return;
    }

    if (re38_fault_start_state == RE38_FAULT_START_NOT_ACTIVE)
    {
        if ((re38_throttle_sw_was_on != 0u) && (M1_SVPWM_Updating != 0u))
        {
            /* 高速运行中首次发生故障：维持模式3→7飞行切换。 */
            re38_fault_start_state = RE38_FAULT_START_SMO_CLOSED_LOOP;
        }
        else
        {
            /* 故障锁存后重新开启，或PWM尚未运行：从模式1强拖启动。 */
            re38_fault_start_state = RE38_FAULT_START_OPENLOOP_DRAG;
            re38_smo_speed_stable_ticks = 0u;
        }
    }

    if (re38_fault_start_state == RE38_FAULT_START_OPENLOOP_DRAG)
    {
        if (M1_Algo_control_start != 0u)
        {
            M1_Control_Mode = M1_MODE_OPENLOOP_DRAG;
            M1_IqRef = 0.075f;
            M1_IdRef = 0.0f;
            /* Iq 没起来之前转速给 0，电流到位后再给 120 */
            if (M1_pid_CurrLp_iq.Fdb < 0.07f)
            {
                M1_SpeedRef = 0.0f;
            }
            else
            {
                M1_SpeedRef = 120.0f / (float)BaseRpm_1;
            }

            if ((smo_estimated_speed_rpm >= 100) &&
                (smo_estimated_speed_rpm <= 140))
            {
                if (re38_smo_speed_stable_ticks < 8000u)
                {
                    re38_smo_speed_stable_ticks++;
                }
            }
            else
            {
                re38_smo_speed_stable_ticks = 0u;
            }

            if (re38_smo_speed_stable_ticks >= 8000u) /* 400ms @ 20kHz */
            {
                re38_fault_start_state = RE38_FAULT_START_SMO_CLOSED_LOOP;
                M1_Control_Mode = M1_MODE_SENSORLESS_SPEED;
                M1_SpeedRef = RE38_Fault_Start_GetTargetSpeedPu();
            }
        }
    }
    else if (re38_fault_start_state == RE38_FAULT_START_SMO_CLOSED_LOOP)
    {
        /* RE38油门每拍写模式3；强制保留已接管的模式7。 */
        if (M1_Algo_control_start != 0u)
        {
            M1_Control_Mode = M1_MODE_SENSORLESS_SPEED;
        }
    }

    re38_throttle_sw_was_on = 1u;
}
#endif

/******************************************************************************
 * @brief   Force Trip Zone to stop all M1 PWM outputs (emergency stop)
 ******************************************************************************/
void M1_ForceAllPwmLowByTZ(void)
{
    EALLOW;
    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseA_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseB_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPWM_M1_PhaseC_Regs.TZCTL.bit.TZB = TZ_FORCE_LO;

    EPWM_M1_PhaseA_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseB_Regs.TZFRC.bit.OST = 0x1;
    EPWM_M1_PhaseC_Regs.TZFRC.bit.OST = 0x1;
    EDIS;
}

/******************************************************************************
 * @brief   Force Trip Zone to stop all M1 PWM outputs (emergency stop)
 ******************************************************************************/
void M1_StopMotor(void)
{
    M1_ForceAllPwmLowByTZ();
}

/******************************************************************************
 * @brief   Clear Trip Zone flags to re-enable M1 PWM outputs
 ******************************************************************************/
void M1_ClearToStartMotor(void)
{
    EALLOW;
    EPWM_M1_PhaseC_Regs.TZFRC.bit.OST = 0x0;
    EPWM_M1_PhaseB_Regs.TZFRC.bit.OST = 0x0;
    EPWM_M1_PhaseA_Regs.TZFRC.bit.OST = 0x0;
    EPWM_M1_PhaseC_Regs.TZCLR.bit.OST = 0x1;
    EPWM_M1_PhaseB_Regs.TZCLR.bit.OST = 0x1;
    EPWM_M1_PhaseA_Regs.TZCLR.bit.OST = 0x1;
    EDIS;
}

/* 上一拍写入的 RampDelayMax，供 SLEW 限速使用 */
static uint32_t M1_RampDelayPrev = M1_RAMP_DELAY_MIN;

/******************************************************************************
 * @brief   将目标 Delay 钳位、SLEW 限速后写入 rc->RampDelayMax
 * @param   delay_target  Smoothstep 或远/近区边界计算出的目标 Delay
 * @note    SLEW 避免 Delay 整数量化时在单 tick 内跳变过大，减小参考加加速度尖峰
 ******************************************************************************/
static void M1_RampApplyDelayMax(RMPCNTL_027 *rc, uint32_t delay_target)
{
    uint32_t delay_out;

    /* 钳位到 [DELAY_MIN, DELAY_MAX] */
    if (delay_target < M1_RAMP_DELAY_MIN)
    {
        delay_target = M1_RAMP_DELAY_MIN;
    }
    else if (delay_target > M1_RAMP_DELAY_MAX)
    {
        delay_target = M1_RAMP_DELAY_MAX;
    }

#if M1_RAMP_DELAY_SLEW_STEP
    /* 相对上一拍最多变化 ±SLEW_STEP */
    delay_out = M1_RampDelayPrev;
    if (delay_target > delay_out)
    {
        delay_out += M1_RAMP_DELAY_SLEW_STEP;
        if (delay_out > delay_target)
        {
            delay_out = delay_target;
        }
    }
    else if (delay_target < delay_out)
    {
        delay_out -= M1_RAMP_DELAY_SLEW_STEP;
        if (delay_out < delay_target)
        {
            delay_out = delay_target;
        }
    }
    M1_RampDelayPrev = delay_out;
#else
    delay_out = delay_target;
    M1_RampDelayPrev = delay_out;
#endif

    rc->RampDelayMax = delay_out;
}

/******************************************************************************
 * @brief   速度 Ramp Smoothstep 软着陆：按剩余 rpm 连续调节 RampDelayMax
 * @param   rc               速度 Ramp 对象（M1_speed_rc1）
 * @param   pu_fullscale_rpm M1_speed1.BaseRpm，pu→rpm 标定
 * @note    20kHz MotorControlISR 内、RC_MACRO_027 之前调用
 *          来shaping reference：remain = |Target − Setpoint|
 *
 * 分区：
 *   remain ≥ LAND_START → DELAY_MIN
 *   remain ≤ LAND_END   → DELAY_MAX
 *   中间                → Smoothstep 插值
 ******************************************************************************/
void M1_RampSoftLandingUpdate(RMPCNTL_027 *rc, float pu_fullscale_rpm)
{
    float target_rpm;
    float setpoint_rpm;
    float remain_rpm;
    float alpha;
    float delay_f;
    float land_span_rpm;
    uint32_t delay_target;

    target_rpm = _IQtoF(rc->TargetValue) * pu_fullscale_rpm;
    setpoint_rpm = _IQtoF(rc->SetpointValue) * pu_fullscale_rpm;
    remain_rpm = fabsf(target_rpm - setpoint_rpm);

    rc->RampStep = M1_RAMP_STEP;

    if (remain_rpm <= 0.0f)
    {
        return;
    }

    if (remain_rpm >= M1_RAMP_LAND_START_RPM)
    {
        M1_RampApplyDelayMax(rc, M1_RAMP_DELAY_MIN);
        return;
    }

    if (remain_rpm <= M1_RAMP_LAND_END_RPM)
    {
        M1_RampApplyDelayMax(rc, M1_RAMP_DELAY_MAX);
        return;
    }

    land_span_rpm = M1_RAMP_LAND_START_RPM - M1_RAMP_LAND_END_RPM;
    alpha = (M1_RAMP_LAND_START_RPM - remain_rpm) / land_span_rpm;
    alpha = alpha * alpha * (3.0f - 2.0f * alpha);
    delay_f = (float)M1_RAMP_DELAY_MIN
            + alpha * (float)(M1_RAMP_DELAY_MAX - M1_RAMP_DELAY_MIN);
    delay_target = (uint32_t)(delay_f + 0.5f);
    M1_RampApplyDelayMax(rc, delay_target);
}

/******************************************************************************
 * RE38 目标转速钳位：怠速 ~ 满速
 ******************************************************************************/
float RE38_ClampTargetRpm(float rpm)
{
    if (rpm < RE38_THROTTLE_IDLE_RPM)
        rpm = RE38_THROTTLE_IDLE_RPM;
    if (rpm > RE38_THROTTLE_MAX_RPM)
        rpm = RE38_THROTTLE_MAX_RPM;
    return rpm;
}

/******************************************************************************
 * RestartHold_ApplyInIsr
 * M1_RestartTicker > 0 时覆盖油门门控结果，强制关闭 FOC 与所有给定。
 * 用于故障清除后约 500ms 稳定期；HOST_CAN 与 RE38 共用。
 ******************************************************************************/
static void RestartHold_ApplyInIsr(void)
{
    if (M1_RestartTicker > 0)
    {
        M1_Algo_control_start = 0;
        M1_Control_Mode = M1_MODE_STOP;
        M1_SpeedRef = 0.0f;
        M1_IqRef = 0.0f;
        M1_IdRef = 0.0f;
    }
}

/******************************************************************************
 * 上位机 CAN 油门 ISR 门控（0x10A001）
 ******************************************************************************/
void HostCanThrottle_ApplyInIsr(void)
{
    if (CAN_Enable != 1u)
    {
        return;
    }

    if (CAN_PWM_Enable == 1u)
    {
        M1_Algo_control_start = 1;
    }
    else
    {
        M1_Algo_control_start = 0;
        M1_Control_Mode = M1_MODE_STOP;
    }

    if (CAN_control_command == 0u)
    {
        M1_Control_Mode = M1_MODE_STOP;
    }
    else if (CAN_control_command == 1u)
    {
        M1_Control_Mode = M1_MODE_OPENLOOP_DRAG;
        M1_SpeedRef = (float)CAN_speed_set / (float)BaseRpm_1;
        M1_IqRef = (float)CAN_iqref / 200.0f;
        M1_iq_RampControl = 5;
        M1_IdRef = 0.0f;
        M1_id_RampControl = 5;
    }
    else if (CAN_control_command == 2u)
    {
        M1_Control_Mode = M1_MODE_TORQUE;
        M1_IqRef = (float)CAN_speed_set / 3000.0f;
        M1_iq_RampControl = 5;
    }
    else if (CAN_control_command == 3u)
    {
        M1_Control_Mode = M1_MODE_SPEED;
        M1_SpeedRef = (float)CAN_speed_set / (float)BaseRpm_1;
    }
    else if (CAN_control_command == 7u)
    {
        M1_Control_Mode = M1_MODE_SENSORLESS_SPEED;
        M1_SpeedRef = (float)CAN_speed_set / (float)BaseRpm_1;
    }

    /* 故障重启封波：倒计时内忽略 CAN 控制指令 */
    RestartHold_ApplyInIsr();
}

/******************************************************************************
 * RE38 模拟/CAN 油门 ISR 门控（GPIO63 + M1_MODE_SPEED）
 ******************************************************************************/
void RE38_Throttle_ApplyInIsr(void)
{
    uint16_t sw_raw;
    uint16_t throttle_sw_raw_on;
    float rpm;

    sw_raw = GpioDataRegs.GPBDAT.bit.GPIO63;
    throttle_sw_raw_on = (sw_raw == 0u);

    if (throttle_sw_raw_on)
    {
        re38_throttle_sw_on = 1u;
        re38_throttle_sw_off_cnt = 0u;
    }
    else
    {
        if (re38_throttle_sw_off_cnt < RE38_THROTTLE_SW_OFF_FILTER_TICKS)
            re38_throttle_sw_off_cnt++;
        else
            re38_throttle_sw_on = 0u;
    }

    if (re38_throttle_sw_on)
    {
        M1_Algo_control_start = 1;
        M1_Control_Mode = M1_MODE_SPEED;
        if (throttle_type == RE38_VOLT_THROTTLE)
            rpm = re38_volt_target_rpm;
        else
            rpm = re38_can_target_rpm;

        rpm = RE38_ClampTargetRpm(rpm);
        M1_SpeedRef = rpm / (float)BaseRpm_1;
    }
    else
    {
        M1_Algo_control_start = 0;
        M1_Control_Mode = M1_MODE_STOP;
        M1_SpeedRef = 0.0f;
    }

    /* 故障重启封波：倒计时内即使 GPIO63 再次打开也不给速 */
    RestartHold_ApplyInIsr();
}

void MotorISR_Throttle_Apply(void)
{
    if (throttle_type == HOST_CAN_THROTTLE)
    {
        HostCanThrottle_ApplyInIsr();
        return;
    }
    if (throttle_type == RE38_VOLT_THROTTLE || throttle_type == RE38_CAN_THROTTLE)
    {
        RE38_Throttle_ApplyInIsr();
        return;
    }

    M1_Algo_control_start = 0;
    M1_Control_Mode = M1_MODE_STOP;
    M1_SpeedRef = 0.0f;
}
