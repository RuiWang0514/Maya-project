#include "F28x_Project.h"

#define MinThrot 1100
#define MaxThrot 1920
#define MinSpd 3000
#define MaxSpd 28000

int32 LinearConvertion(int32 Value, int32 MinIn, int32 MaxIn, int32 MinOut, int32 MaxOut);
float LinearLimConvertion(float Value, float MinIn, float MaxIn, float MinOut, float MaxOut);
//int32 LinearLimConvertion(int32 Value,int32 MinIn,int32 MaxIn,int32 MinOut,int32 MaxOut);
