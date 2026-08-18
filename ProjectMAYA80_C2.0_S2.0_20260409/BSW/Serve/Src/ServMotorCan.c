#include "HPMC_Settings.h"
#include "ServMotorCan.h"
#include "canHandle.h"
#include "pid_reg3_027.h"

extern PIDREG3_027 M1_pid_CurrLp_iq;

void Motor310_UpdateCurrentRms(void)
{
    static int iq_ref_avg_buf[10];

    static uint16_t iq_ref_avg_idx = 0u;

    int32_t sum = 0;

    uint16_t i;

    int32_t avg;

    int iq_ref_sample;

    iq_ref_sample = (int)(_IQtoF(M1_pid_CurrLp_iq.Ref) * (float)BASE_CURRENT);

    iq_ref_avg_buf[iq_ref_avg_idx] = iq_ref_sample;

    iq_ref_avg_idx = (iq_ref_avg_idx + 1u) % MOTOR310_IQ_REF_AVG_LEN;

    for(i = 0; i < MOTOR310_IQ_REF_AVG_LEN; i++)
    {
        sum += iq_ref_avg_buf[i];
    }

    avg = sum / (int32_t)MOTOR310_IQ_REF_AVG_LEN;

    motor310_iq_ref_avg = (int)avg;

    motor310_current_rms = motor310_iq_ref_avg * 0.707106781f;
}

