// Include header files used in the main function
#include "Linear_Conversion.h"


//Linear convertion 
int32 LinearConvertion(int32 Value,int32 MinIn,int32 MaxIn,int32 MinOut,int32 MaxOut)
{
  int32 Res;
  if (MaxIn != MinIn)
  {
      Res = (Value - MinIn)*(MaxOut - MinOut)/(MaxIn - MinIn) + MinOut;
  }
  else
  {
      Res = MinOut;
  }

  return Res;
}
//-----------------------------------------------------------------------
//int32 LinearLimConvertion(int32 Value,int32 MinIn,int32 MaxIn,int32 MinOut,int32 MaxOut)
//{
//  int32 Res;
//  int32 MinVal;
//  int32 MaxVal;
//
//  if (MinOut != MaxOut)
//  {
//      MinVal = MinOut;
//      MaxVal = MaxOut;
//      if(MinOut > MaxOut)
//      {
//         MinVal = MaxOut;
//         MaxVal = MinOut;
//      }
//  }
//  else
//  {
//      MinVal = MaxOut;
//      MaxVal = MinOut;
//  }
//
//  Res = LinearConvertion(Value, MinIn, MaxIn, MinOut, MaxOut);
//
//  if (Res < MinVal)
//  {
//      Res = MinVal;
//  }
//  else if (Res > MaxVal)
//  {
//      Res = MaxVal;
//  }
//
//  return Res;
//}
float LinearLimConvertion(float Value,float MinIn,float MaxIn,float MinOut,float MaxOut)
{
  float Res;

  Res = (Value - MinIn)*(MaxOut - MinOut)/(MaxIn - MinIn) + MinOut;

  if (Res < MinOut)
  {
      Res = MinOut;
  }
  else if (Res > MaxOut)
  {
      Res = MaxOut;
  }

  return Res;
}
