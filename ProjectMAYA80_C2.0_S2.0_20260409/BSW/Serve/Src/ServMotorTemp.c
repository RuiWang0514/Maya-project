/*
 * spi_resolver.c
 *
 *  Created on:
 *      Author: quop_
 */
#include "HPMC_Settings.h"
#include "ServMotorTemp.h"

extern uint16_t DTC_code1;
extern int NTC_motor_avg_buf;
extern int NTC_motor_filter_buf;

float Mea_PTA1,Mea_PTA2,Mea_PTB1,Mea_PTB2,Mea_PTC = 0.0;
float Mea_PTB1_avg,Mea_PTB1_filter,Mea_PTB2_avg,Mea_PTB2_filter = 0.0;
float Mea_PTA1_avg,Mea_PTC_avg,Mea_PTA1_filter,Mea_PTA2_avg,Mea_PTA2_filter,Mea_PTC_avg_filter = 0.0;

unsigned char NTC_temp_avg_buf_PTB1[100]={0};
unsigned char NTC_temp_avg_buf_PTA1[100]={0};
unsigned char NTC_temp_avg_buf_PTB2[100]={0};
unsigned char NTC_temp_avg_buf_PTA2[100]={0};
unsigned char NTC_temp_avg_buf_PTC[100]={0};
unsigned char NTC_temp_filter_buf_PTB1[100]={0};
unsigned char NTC_temp_filter_buf_PTB2[100]={0};
float Mea_PTB_1_2_avg,Mea_PTA_1_2_avg,Mea_PTAB_1_2_avg,Mea_PTABC_avg;
int motor_overtemp_ticker = 0;
int motor_overtemp_flag = 0;
int Mea_PTB1_avg_1,Mea_PTB2_avg_1 =0;
int Mea_PTA1_avg_1,Mea_PTA2_avg_1 ,Mea_PTC_avg_1=0;

uint16_t temp_motor_B;
float tempB_value;
uint32_t tempB_overtivker = 0;
extern uint16_t AD2SB_Test_Ticker;



uint16_t MAX6755C_READ(uint16_t addr)
{
    uint16_t tempC;

    SpicRegs.SPITXBUF=addr;
    while(SpicRegs.SPIFFRX.bit.RXFFST!=1){};
    tempC = (SpicRegs.SPIRXBUF);

    return tempC;

}
uint16_t MAX6755A_READ1(uint16_t addr)
{
    uint16_t tempA;

    SpiaRegs.SPITXBUF=(addr<<8);
    while(SpiaRegs.SPIFFRX.bit.RXFFST!=1){};
    tempA = (SpiaRegs.SPIRXBUF);
    tempA <<=8;

    SpiaRegs.SPITXBUF=(addr<<8);
    while(SpiaRegs.SPIFFRX.bit.RXFFST!=1){};
    tempA |= (SpiaRegs.SPIRXBUF);

    return tempA;

}



uint16_t MAX6755C_READ2(uint16_t addr)
{
    uint16_t tempC;

    SpicRegs.SPITXBUF=(addr);
    while(SpicRegs.SPIFFRX.bit.RXFFST!=1){};
    tempC = (SpicRegs.SPIRXBUF);
    tempC <<=8;

    SpicRegs.SPITXBUF=(addr);
    while(SpicRegs.SPIFFRX.bit.RXFFST!=1){};
    tempC |= (SpicRegs.SPIRXBUF);
    tempC = (SpicRegs.SPIRXBUF);
    return tempC;

}


uint16_t temp_motor_1;
uint16_t temp_motor_2;
float temp1_value;
float temp2_value;
extern uint16_t AD2SC_Test_Ticker;
uint16_t AD2SA_Test_Ticker;
void MAX6755A_temp_cal()
{
    AD2SA_Test_Ticker++;
    //TEMP1
    TEMP1_CS_L;

    temp_motor_1 = MAX6755A_READ1(0xff);

    TEMP1_CS_H;

    if(temp_motor_1 & 0x0004)
    {
        temp_motor_1 = 4095;
    }

    else
    {
        temp_motor_1 = temp_motor_1 & 0x3FFF;
        temp_motor_1 = temp_motor_1>>3;
    }

    temp_motor_1 = temp_motor_1 &0x0fff;

    temp1_value = (float)(temp_motor_1*1024.0)/4095.0;
    //TEMP2
    TEMP2_CS_L;

    temp_motor_2 = MAX6755A_READ1(0xff);

    TEMP2_CS_H;

    if(temp_motor_2 & 0x0004)
    {
        temp_motor_2 = 4095;
    }

    else
    {
        temp_motor_2 = temp_motor_2>>3;
    }

    temp_motor_2 = temp_motor_2 &0x0fff;

    temp2_value = (float)(temp_motor_2*1024.0)/4095.0;

}

extern int PTA1, PTA2, PTB1, PTB2, PTC;

float PT100_measurement(uint16_t adcresult)
{
    float pt100_temp = 0.0;
    float pt100_res  = 0.0;
    pt100_res=33000.0/(16.5-5.0*0.09456047-adcresult*3.3*ADC_PU_SCALE_FACTOR)-2000;
    pt100_temp=(pt100_res-100)*2.568845047266748869;
    return pt100_temp;
}

 int NTC_a_i_PTB1 = 0;
 float NTC_sum_avg_PTB1 = 0.0;
 float NTC_avg_PTB1_temp =0.0;
float NTC_avg_PTB1(int NTC_SIC_temp_int)
{
    NTC_temp_avg_buf_PTB1[NTC_a_i_PTB1] = NTC_SIC_temp_int;
    NTC_sum_avg_PTB1 =NTC_sum_avg_PTB1 + NTC_temp_avg_buf_PTB1[NTC_a_i_PTB1];
    NTC_a_i_PTB1++;

    if(NTC_a_i_PTB1 == NTC_motor_avg_buf)
    {
        NTC_a_i_PTB1 = 0;
        NTC_avg_PTB1_temp = NTC_sum_avg_PTB1/NTC_motor_avg_buf;
        NTC_sum_avg_PTB1 = 0;
    }
    return NTC_avg_PTB1_temp;
}

int NTC_f_i_PTB1 = 0;
float NTC_filter_PTB1(int NTC_SIC_temp_int)
{
    int NTC_sum_PTB1 = 0;
    int NTC_count_PTB1;
    NTC_temp_filter_buf_PTB1[NTC_f_i_PTB1] = NTC_SIC_temp_int;
    NTC_f_i_PTB1++;

    if(NTC_f_i_PTB1 == NTC_motor_filter_buf)
    {
        NTC_f_i_PTB1 = 0;
    }
    for(NTC_count_PTB1=0;NTC_count_PTB1<NTC_motor_filter_buf;NTC_count_PTB1++)
    {
        NTC_sum_PTB1 =NTC_sum_PTB1 + NTC_temp_filter_buf_PTB1[NTC_count_PTB1];
    }

    return (float)(NTC_sum_PTB1)/NTC_motor_filter_buf;
}

int NTC_a_i_PTB2 = 0;
float NTC_sum_avg_PTB2 = 0.0;
float NTC_avg_PTB2_temp =0.0;
float NTC_avg_PTB2(int NTC_SIC_temp_int)
{
   NTC_temp_avg_buf_PTB2[NTC_a_i_PTB2] = NTC_SIC_temp_int;
   NTC_sum_avg_PTB2 =NTC_sum_avg_PTB2 + NTC_temp_avg_buf_PTB2[NTC_a_i_PTB2];
   NTC_a_i_PTB2++;

   if(NTC_a_i_PTB2 == NTC_motor_avg_buf)
   {
       NTC_a_i_PTB2 = 0;
       NTC_avg_PTB2_temp = NTC_sum_avg_PTB2/NTC_motor_avg_buf;
       NTC_sum_avg_PTB2 = 0;
   }
   return NTC_avg_PTB2_temp;
}

int NTC_f_i_PTB2 = 0;
float NTC_filter_PTB2(int NTC_SIC_temp_int)
{
   int NTC_sum_PTB2 = 0;
   int NTC_count_PTB2;
   NTC_temp_filter_buf_PTB2[NTC_f_i_PTB2] = NTC_SIC_temp_int;
   NTC_f_i_PTB2++;

   if(NTC_f_i_PTB2 == NTC_motor_filter_buf)
   {
       NTC_f_i_PTB2 = 0;
   }
   for(NTC_count_PTB2=0;NTC_count_PTB2<NTC_motor_filter_buf;NTC_count_PTB2++)
   {
       NTC_sum_PTB2 =NTC_sum_PTB2 + NTC_temp_filter_buf_PTB2[NTC_count_PTB2];
   }

   return (float)(NTC_sum_PTB2)/NTC_motor_filter_buf;
}
int NTC_a_i_PTA1 = 0;
float NTC_sum_avg_PTA1 = 0.0;
float NTC_avg_PTA1_temp =0.0;
float NTC_avg_PTA1(int NTC_SIC_temp_int)
{
    NTC_temp_avg_buf_PTA1[NTC_a_i_PTA1] = NTC_SIC_temp_int;
    NTC_sum_avg_PTA1 =NTC_sum_avg_PTA1 + NTC_temp_avg_buf_PTA1[NTC_a_i_PTA1];
    NTC_a_i_PTA1++;

    if(NTC_a_i_PTA1 == NTC_motor_avg_buf)
    {
        NTC_a_i_PTA1 = 0;
        NTC_avg_PTA1_temp = NTC_sum_avg_PTA1/NTC_motor_avg_buf;
        NTC_sum_avg_PTA1 = 0;
    }
    return NTC_avg_PTA1_temp;
}

int NTC_a_i_PTA2 = 0;
float NTC_sum_avg_PTA2 = 0.0;
float NTC_avg_PTA2_temp =0.0;
float NTC_avg_PTA2(int NTC_SIC_temp_int)
{
    NTC_temp_avg_buf_PTA2[NTC_a_i_PTA2] = NTC_SIC_temp_int;
    NTC_sum_avg_PTA2 =NTC_sum_avg_PTA2 + NTC_temp_avg_buf_PTA2[NTC_a_i_PTA2];
    NTC_a_i_PTA2++;

    if(NTC_a_i_PTA2 == NTC_motor_avg_buf)
    {
        NTC_a_i_PTA2 = 0;
        NTC_avg_PTA2_temp = NTC_sum_avg_PTA2/NTC_motor_avg_buf;
        NTC_sum_avg_PTA2 = 0;
    }
    return NTC_avg_PTA2_temp;
}


int NTC_a_i_PTC = 0;
float NTC_sum_avg_PTC = 0.0;
float NTC_avg_PTC_temp =0.0;
float NTC_avg_PTC(int NTC_SIC_temp_int)
{
   NTC_temp_avg_buf_PTC[NTC_a_i_PTC] = NTC_SIC_temp_int;
   NTC_sum_avg_PTC =NTC_sum_avg_PTC + NTC_temp_avg_buf_PTC[NTC_a_i_PTC];
   NTC_a_i_PTC++;

   if(NTC_a_i_PTC == NTC_motor_avg_buf)
   {
       NTC_a_i_PTC = 0;
       NTC_avg_PTC_temp = NTC_sum_avg_PTC/NTC_motor_avg_buf;
       NTC_sum_avg_PTC = 0;
   }
   return NTC_avg_PTC_temp;
}

void PT100_temp(void)
{
    Mea_PTA1 = (int)(PT100_measurement(Mea_PTA1_Raw))+40;
    PTA1 = Mea_PTA1;
    Mea_PTA2 = (int)(PT100_measurement(Mea_PTA2_Raw))+40;
    PTA2 = Mea_PTA2;
    Mea_PTB1 = (int)(PT100_measurement(Mea_PTB1_Raw))+40;
    PTB1 = Mea_PTB1;
    Mea_PTB2 = (int)(PT100_measurement(Mea_PTB2_Raw))+40;
    PTB2 = Mea_PTB2;

    Mea_PTA1_avg = NTC_avg_PTA1(Mea_PTA1) - 40;
    Mea_PTA1_avg_1 = (int)(Mea_PTA1_avg + 40);

    Mea_PTB1_avg = NTC_avg_PTB1(Mea_PTB1) - 40;
    Mea_PTB1_avg_1 = (int)(Mea_PTB1_avg + 40);

    Mea_PTA2_avg = NTC_avg_PTA2(Mea_PTA2) - 40;
    Mea_PTA2_avg_1 = (int)(Mea_PTA2_avg + 40);

    Mea_PTB2_avg = NTC_avg_PTB2(Mea_PTB2) - 40;
    Mea_PTB2_avg_1 = (int)(Mea_PTB2_avg + 40);

    Mea_PTC_avg_1 = 0;

    Mea_PTB_1_2_avg = Mea_PTB1_avg_1 > Mea_PTB2_avg_1 ? Mea_PTB1_avg_1 : Mea_PTB2_avg_1;
    Mea_PTA_1_2_avg = Mea_PTA1_avg_1 > Mea_PTA2_avg_1 ? Mea_PTA1_avg_1 : Mea_PTA2_avg_1;
    Mea_PTAB_1_2_avg = Mea_PTB_1_2_avg > Mea_PTA_1_2_avg ? Mea_PTB_1_2_avg : Mea_PTA_1_2_avg;
    Mea_PTABC_avg = Mea_PTAB_1_2_avg > Mea_PTC_avg_1 ? Mea_PTAB_1_2_avg : Mea_PTC_avg_1;

    if(Mea_PTABC_avg > 190.0)
    {
        motor_overtemp_ticker++;
    }
    else
    {
        motor_overtemp_ticker = 0;
    }
    if(motor_overtemp_ticker>2)
    {
        motor_overtemp_flag = 1;
        DTC_code1 |=0x20;
    }
}

