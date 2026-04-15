/*
 * spi_resolver.c
 *

 *      Author: quop_
 */
#include "HPMC_Settings.h"
#include "ServNTCTemp.h"

float NTC_SIC_M1A,NTC_SIC_M1B,NTC_SIC_M1C,NTC_SIC_M2A,NTC_SIC_M2B,NTC_SIC_M2C = 0;
float NTC_SIC_M1A_ECAP,NTC_SIC_M1B_ECAP,NTC_SIC_M1C_ECAP = 0;
float NTC_SIC_M1A1,NTC_SIC_M1B1,NTC_SIC_M1C1,NTC_SIC_M2A1,NTC_SIC_M2B1,NTC_SIC_M2C1 = 0.0;

int NTC_filter_buf = 10;
int NTC_motor_avg_buf = 100;
int NTC_motor_filter_buf = 100;
unsigned char NTC_temp_buf[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char NTC_temp_buf_A1[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char NTC_temp_buf_B1[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char NTC_temp_buf_C1[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char NTC_temp_buf_A2[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char NTC_temp_buf_B2[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char NTC_temp_buf_C2[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char NTC_temp_avg[10]={0,0,0,0,0,0,0,0,0,0};

extern uint16_t DTC_code1;

int NTC_f_i_A1 = 0;
int NTC_f_i_B1 = 0;
int NTC_f_i_C1 = 0;
int NTC_f_i_A2 = 0;
int NTC_f_i_B2 = 0;
int NTC_f_i_C2 = 0;
int NTC_f_i_avg = 0;
float NTC_SIC_avg = 0.0;
float NTC_SIC_avg_filter = 0.0;
float NTC_SIC_avg_filter1 = 0.0;
float NTC_SIC_avg_ECAP = 0.0;
float NTC_SIC_avg_filter_ECAP = 0.0;
float NTC_SIC_avg_filter1_ECAP = 0.0;
extern int over_avg_temp_flag;
extern int over_temp_ticker;
extern float SIC_MAX_TEMP;
void NTC_temp()
{

    NTC_SIC_M1A = (int)(NTC_SIC_temp5(M1_Temp_PhaseA_Raw));
    NTC_SIC_M1B = (int)(NTC_SIC_temp5(M1_Temp_PhaseB_Raw));
    NTC_SIC_M1C = (int)(NTC_SIC_temp5(M1_Temp_PhaseC_Raw));


    NTC_SIC_M1A_ECAP = (int)(NTC_duty_measurement80(ECAP1_duty));
    NTC_SIC_M1B_ECAP = (int)(NTC_duty_measurement80(ECAP2_duty));
    NTC_SIC_M1C_ECAP = (int)(NTC_duty_measurement80(ECAP3_duty));

    NTC_SIC_avg = (NTC_SIC_M1A+NTC_SIC_M1B+NTC_SIC_M1C)/3.0;
    NTC_SIC_avg_filter1 = NTC_SIC_avg;

    NTC_SIC_avg_ECAP = (NTC_SIC_M1A_ECAP+NTC_SIC_M1B_ECAP+NTC_SIC_M1C_ECAP)/3.0+40;
    NTC_SIC_avg_filter1_ECAP = NTC_SIC_avg_ECAP;


    if(NTC_SIC_M1A>SIC_MAX_TEMP||NTC_SIC_M1B>SIC_MAX_TEMP||NTC_SIC_M1C>SIC_MAX_TEMP||
       NTC_SIC_M1A_ECAP>SIC_MAX_TEMP||NTC_SIC_M1B_ECAP>SIC_MAX_TEMP||NTC_SIC_M1C_ECAP>SIC_MAX_TEMP)
    {
        over_temp_ticker++;
    }
    else
    {
        over_temp_ticker = 0;
    }
    if(over_temp_ticker > 1)
    {
        DTC_code1 |= 0x08;
        over_avg_temp_flag = 1;
    }

}

float NTC_duty_measurement(uint16_t adcresult)
{
    float duty = 0.0;
    float pt100_duty_temp =0.0;
    duty = adcresult/4095.0;
    pt100_duty_temp = -471.3*duty*duty*duty+729.3*duty*duty-516.8*duty+238.8;
    return pt100_duty_temp;
}

float NTC_duty_measurement80(float duty)
{
    float duty_temp =0.0;
    float NTC_RES = 0.0;
    float term2=0.0;
    float term1 = 0.003354;
    float temp_vol=0.0;
    temp_vol=0.8*duty+0.1;

    NTC_RES=((330*0.8846)/(0.8846-temp_vol)-330)*0.001;
    term2 = (1.0 / 3436) * log(NTC_RES / 5);

    duty_temp = 1.0 / (term1 + term2)-273.15;
    return duty_temp;
}


//80NTC
float NTC_SIC_temp5(uint16_t adcresult)
{
    float NTC_SIC = 0.0;
    float NTC_RES = 0.0;

    float term2=0.0;
    float term1 = 0.003354;

    NTC_RES=((330*0.8846)/(0.8846-(adcresult*3.3*ADC_PU_SCALE_FACTOR))-330)*0.001;


    term2 = (1.0 / 3436) * log(NTC_RES / 5);

    NTC_SIC = 1.0 / (term1 + term2)-273.15;
    return NTC_SIC;

}


float NTC_SIC_temp(uint16_t adcresult)
{
    float NTC_SIC = 0.0;
    if(adcresult<=2090)
    {
        NTC_SIC= 0.04262 * adcresult -17.33;
    }
    else if(adcresult<=2670)
    {
        NTC_SIC= 2.873e-7*adcresult*adcresult*adcresult-0.001946*adcresult*adcresult+4.449*adcresult-3350;
    }
    else
    {
        NTC_SIC = 0.001147*adcresult*adcresult-6.027*adcresult+8041;
    }

    return NTC_SIC;

}

float NTC_SIC_temp1(uint16_t adcresult)
{
    float NTC_SIC = 0.0;
    if(adcresult<=402)
    {
        adcresult = 402;
    }
    if(adcresult<=1702)
    {
        NTC_SIC= 0.00002588 * adcresult * adcresult +0.01293*adcresult-7.495;
    }
    else if(adcresult<=1767)
    {
        NTC_SIC= (adcresult - 1702)/65.0*10.6+89.5;
    }
    else
    {
        NTC_SIC = 0.001394 * adcresult * adcresult - 4.805*adcresult+4238;
    }

    return NTC_SIC;

}


float NTC_SIC_temp2(uint16_t adcresult)
{
    float NTC_SIC = 0.0;
    if(adcresult<=647)
    {
        adcresult = 647;
    }
    if(adcresult<=2400)
    {
        NTC_SIC= 0.00000513 * adcresult * adcresult +0.0221*adcresult-15.87;
    }
//    else if(adcresult<=1767)
//    {
//        NTC_SIC= (adcresult - 1702)/65.0*10.6+89.5;
//    }
    else
    {
        NTC_SIC = 0.0000003022 * adcresult * adcresult * adcresult- 0.00232*adcresult*adcresult+5.989*adcresult-5120;
    }

    return NTC_SIC;

}


float NTC_SIC_temp3(uint16_t adcresult)
{
    float NTC_SIC = 0.0;
    if(adcresult<=364)
    {
        adcresult = 364;
    }
    if(adcresult<=1100)
    {
        NTC_SIC= 0.00005417 * adcresult * adcresult +0.010911*adcresult-9.905;
    }
    else if(adcresult<=1229)
    {
        NTC_SIC= (adcresult - 1100)/129.0*16.5+65.6;//75.6
    }
    else
    {
        NTC_SIC = 0.001376 * adcresult * adcresult - 3.271*adcresult+2022;
    }

    return NTC_SIC;

}

float NTC_SIC_temp4(uint16_t adcresult)
{
    float NTC_SIC = 0.0;
    if(adcresult<=599)
    {
        adcresult = 599;
    }
    if(adcresult<=1800)
    {
        NTC_SIC= 0.00002003 * adcresult * adcresult +0.00554*adcresult-9.905;
    }
    else if(adcresult<=1900)
    {
        NTC_SIC= (adcresult - 1800)/100.0*11.6+65.0;
    }
    else
    {
        NTC_SIC = 0.0006428 * adcresult * adcresult - 2.43*adcresult+2373;
    }

    return NTC_SIC;

}





//int NTC_count = 0;
int NTC_count_A1 = 0;
int NTC_count_B1 = 0;
int NTC_count_C1 = 0;
int NTC_count_A2 = 0;
int NTC_count_B2 = 0;
int NTC_count_C2 = 0;

float NTC_filter_A1(int NTC_SIC_temp_int)
{
    int NTC_sum = 0;
    int NTC_count;
    NTC_temp_buf_A1[NTC_f_i_A1] = NTC_SIC_temp_int;
    NTC_f_i_A1++;

    if(NTC_f_i_A1 == NTC_filter_buf)
    {
        NTC_f_i_A1 = 0;
    }
    for(NTC_count=0;NTC_count<NTC_filter_buf;NTC_count++)
    {
        NTC_sum =NTC_sum + NTC_temp_buf_A1[NTC_count];
    }

    return (float)(NTC_sum)/NTC_filter_buf;
}
float NTC_avg_A1_temp =0.0;
float NTC_sum_avg_A1 = 0.0;
float NTC_avg_A1(int NTC_SIC_temp_int)
{

//    int NTC_count;
    NTC_temp_buf_A1[NTC_f_i_A1] = NTC_SIC_temp_int;
    NTC_sum_avg_A1 =NTC_sum_avg_A1 + NTC_temp_buf_A1[NTC_f_i_A1];
    NTC_f_i_A1++;

    if(NTC_f_i_A1 == NTC_filter_buf)
    {
        NTC_f_i_A1 = 0;
        NTC_avg_A1_temp = NTC_sum_avg_A1/NTC_filter_buf;
        NTC_sum_avg_A1 = 0;
    }
    return NTC_avg_A1_temp;
}

float NTC_filter_B1(int NTC_SIC_temp_int)
{
    int NTC_sum = 0;
    int NTC_count;
    NTC_temp_buf_B1[NTC_f_i_B1] = NTC_SIC_temp_int;
    NTC_f_i_B1++;

    if(NTC_f_i_B1 == NTC_filter_buf)
    {
        NTC_f_i_B1 = 0;
    }
    for(NTC_count=0;NTC_count<NTC_filter_buf;NTC_count++)
    {
        NTC_sum =NTC_sum + NTC_temp_buf_B1[NTC_count];
    }

    return (float)(NTC_sum)/NTC_filter_buf;
}
float NTC_filter_C1(int NTC_SIC_temp_int)
{
    int NTC_sum = 0;
    int NTC_count;
    NTC_temp_buf_C1[NTC_f_i_C1] = NTC_SIC_temp_int;
    NTC_f_i_C1++;

    if(NTC_f_i_C1 == NTC_filter_buf)
    {
        NTC_f_i_C1 = 0;
    }
    for(NTC_count=0;NTC_count<NTC_filter_buf;NTC_count++)
    {
        NTC_sum =NTC_sum + NTC_temp_buf_C1[NTC_count];
    }

    return (float)(NTC_sum)/NTC_filter_buf;
}

float NTC_avg_A2_temp =0.0;
float NTC_sum_avg_A2 = 0.0;
float NTC_avg_A2(int NTC_SIC_temp_int)
{

//    int NTC_count;
    NTC_temp_buf_A2[NTC_f_i_A2] = NTC_SIC_temp_int;
    NTC_sum_avg_A2 =NTC_sum_avg_A2 + NTC_temp_buf_A2[NTC_f_i_A2];
    NTC_f_i_A2++;

    if(NTC_f_i_A2 == NTC_filter_buf)
    {
        NTC_f_i_A2 = 0;
        NTC_avg_A2_temp = NTC_sum_avg_A2/NTC_filter_buf;
        NTC_sum_avg_A2 = 0;
    }
    return NTC_avg_A2_temp;
}

float NTC_avg_B2_temp =0.0;
float NTC_sum_avg_B2 = 0.0;
float NTC_avg_B2(int NTC_SIC_temp_int)
{

//    int NTC_count;
    NTC_temp_buf_B2[NTC_f_i_B2] = NTC_SIC_temp_int;
    NTC_sum_avg_B2 =NTC_sum_avg_B2 + NTC_temp_buf_B2[NTC_f_i_B2];
    NTC_f_i_B2++;

    if(NTC_f_i_B2 == NTC_filter_buf)
    {
        NTC_f_i_B2 = 0;
        NTC_avg_B2_temp = NTC_sum_avg_B2/NTC_filter_buf;
        NTC_sum_avg_B2 = 0;
    }
    return NTC_avg_B2_temp;
}

float NTC_avg_C2_temp =0.0;
float NTC_sum_avg_C2 = 0.0;
float NTC_avg_C2(int NTC_SIC_temp_int)
{

//    int NTC_count;
    NTC_temp_buf_C2[NTC_f_i_C2] = NTC_SIC_temp_int;
    NTC_sum_avg_C2 =NTC_sum_avg_C2 + NTC_temp_buf_C2[NTC_f_i_C2];
    NTC_f_i_C2++;

    if(NTC_f_i_C2 == NTC_filter_buf)
    {
        NTC_f_i_C2 = 0;
        NTC_avg_C2_temp = NTC_sum_avg_C2/NTC_filter_buf;
        NTC_sum_avg_C2 = 0;
    }
    return NTC_avg_C2_temp;
}
float NTC_avg_temp =0.0;
float NTC_sum_avg = 0.0;
float NTC_avg(int NTC_SIC_temp_int)
{

//    int NTC_count;
    NTC_temp_avg[NTC_f_i_avg] = NTC_SIC_temp_int;
    NTC_sum_avg =NTC_sum_avg + NTC_temp_avg[NTC_f_i_avg];
    NTC_f_i_avg++;

    if(NTC_f_i_avg == NTC_filter_buf)
    {
        NTC_f_i_avg = 0;
        NTC_avg_temp = NTC_sum_avg/NTC_filter_buf;
        NTC_sum_avg = 0;
    }
    return NTC_avg_temp;
}

