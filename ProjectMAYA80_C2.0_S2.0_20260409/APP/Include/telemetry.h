#ifndef USER_TELEMETRY_H_
#define USER_TELEMETRY_H_

#include "HPMC_Settings.h"


typedef struct
{
    uint16_t head;

    uint32_t devID;

    uint16_t Throttle;

    float inputVoltage;
//    float DCcurrent;
//    float PhaseCurrent[3];
    float PhaseCurrentAs;
    float PhaseCurrentBs;
    float PhaseCurrentCs;
//    float Temperature[3];
    float TemperatureBoard;
    float SpeedRPM;

    uint16_t faultFlag;



    uint16_t end;
} telemsg_typedef;

void Telemetry_update( void );
void Telemetry_init(void);

#endif /* USER_TELEMETRY_H_ */
