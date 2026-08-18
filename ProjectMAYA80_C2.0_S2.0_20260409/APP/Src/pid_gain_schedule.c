#include "pid_gain_schedule.h"
#include "HPMC_Settings.h"
/* 仅包含 PID 结构体头文件；不可 include HPMC_algo_includes.h，
 * 其中 smopos/rmp_cntl/speed_fr 等宏头会在 .c 中定义全局临时变量，重复编译会链接报错 */
#include "pid_reg3_027.h"

extern PIDREG3_027 M1_pid_SpdLp;
extern PIDREG3_027 M1_pid_CurrLp_id;
extern PIDREG3_027 M1_pid_CurrLp_iq;

extern float CAN_pid_SpdLp_kp;
extern float CAN_pid_SpdLp_ki;
extern float CAN_pid_CurrLp_id_kp;
extern float CAN_pid_CurrLp_id_ki;
extern float CAN_pid_CurrLp_iq_kp;
extern float CAN_pid_CurrLp_iq_ki;

/* Settings 默认端点备份，CAN 调参不会修改此表 */
static PidEndpointParams_t g_endpoint_default[2];
/* 运行时插值用的 Low[0] / High[1] 端点 */
static PidEndpointParams_t g_endpoint[2];
/* 插值系数 0~1：0=Low 端点，1=High 端点，中间为过渡区 */
static float g_alpha = 0.0f;

static void load_defaults_from_settings(PidEndpointParams_t *lo, PidEndpointParams_t *hi)
{
    lo->spd_kp = PID_LOW_SPD_KP;
    lo->spd_ki = PID_LOW_SPD_KI;
    lo->id_kp  = PID_LOW_ID_KP;
    lo->id_ki  = PID_LOW_ID_KI;
    lo->iq_kp  = PID_LOW_IQ_KP;
    lo->iq_ki  = PID_LOW_IQ_KI;

    hi->spd_kp = PID_HIGH_SPD_KP;
    hi->spd_ki = PID_HIGH_SPD_KI;
    hi->id_kp  = PID_HIGH_ID_KP;
    hi->id_ki  = PID_HIGH_ID_KI;
    hi->iq_kp  = PID_HIGH_IQ_KP;
    hi->iq_ki  = PID_HIGH_IQ_KI;
}

static float interp_f(float lo, float hi, float alpha)
{
    return lo + alpha * (hi - lo);
}

static void apply_gains(float spd_kp, float spd_ki,
                        float id_kp, float id_ki,
                        float iq_kp, float iq_ki)
{
    M1_pid_SpdLp.Kp = spd_kp;
    M1_pid_SpdLp.Ki = spd_ki;
    M1_pid_CurrLp_id.Kp = id_kp;
    M1_pid_CurrLp_id.Ki = id_ki;
    M1_pid_CurrLp_iq.Kp = iq_kp;
    M1_pid_CurrLp_iq.Ki = iq_ki;
}

void PidGainSchedule_Init(void)
{
    load_defaults_from_settings(&g_endpoint_default[0], &g_endpoint_default[1]);
    g_endpoint[0] = g_endpoint_default[0];
    g_endpoint[1] = g_endpoint_default[1];
    g_alpha = 0.0f;
}

void PidGainSchedule_RestoreDefaultEndpoints(void)
{
    /* CAN_PID_adjust=0：恢复 HPMC_Settings.h 中的 Low/High 默认端点 */
    g_endpoint[0] = g_endpoint_default[0];
    g_endpoint[1] = g_endpoint_default[1];
}

void PidGainSchedule_ApplyCanToBothEndpoints(void)
{
    PidEndpointParams_t can_params;

    /* 来自 CAN 0x10A002，由 CANAPidMsgGet() 解析到 CAN_pid_* 变量 */
    can_params.spd_kp = CAN_pid_SpdLp_kp;
    can_params.spd_ki = CAN_pid_SpdLp_ki;
    can_params.id_kp  = CAN_pid_CurrLp_id_kp;
    can_params.id_ki  = CAN_pid_CurrLp_id_ki;
    can_params.iq_kp  = CAN_pid_CurrLp_iq_kp;
    can_params.iq_ki  = CAN_pid_CurrLp_iq_ki;

    /* Low 与 High 设为相同 → 插值结果恒为 CAN 值，等效全段固定 PID */
    g_endpoint[0] = can_params;
    g_endpoint[1] = can_params;
}

void PidGainSchedule_Update(int32_t speed_rpm)
{
    int32_t rpm = speed_rpm;
    const PidEndpointParams_t *lo = &g_endpoint[0];
    const PidEndpointParams_t *hi = &g_endpoint[1];

    if (rpm < 0)
    {
        rpm = -rpm;
    }

    /* 根据实际转速（旋变 SpeedRpm）计算插值系数 */
    if (rpm <= PID_INTERP_RPM_LOW)
    {
        g_alpha = 0.0f;
    }
    else if (rpm >= PID_INTERP_RPM_HIGH)
    {
        g_alpha = 1.0f;
    }
    else
    {
        g_alpha = (float)(rpm - PID_INTERP_RPM_LOW)
                / (float)(PID_INTERP_RPM_HIGH - PID_INTERP_RPM_LOW);
    }

    /* 6 个 Kp/Ki 各自插值后写入运行 PID 结构体 */
    apply_gains(
        interp_f(lo->spd_kp, hi->spd_kp, g_alpha),
        interp_f(lo->spd_ki, hi->spd_ki, g_alpha),
        interp_f(lo->id_kp,  hi->id_kp,  g_alpha),
        interp_f(lo->id_ki,  hi->id_ki,  g_alpha),
        interp_f(lo->iq_kp,  hi->iq_kp,  g_alpha),
        interp_f(lo->iq_ki,  hi->iq_ki,  g_alpha)
    );
}
