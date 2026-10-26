/* =================================================================================
File name:        RMP_CNTL.H  (IQ version)                  
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type, and macro  definitions for the RMPCNTL module.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 10-15-2009	Version 1.0                                                 
------------------------------------------------------------------------------*/
#ifndef __RMP_CNTL_027_H__
#define __RMP_CNTL_027_H__

typedef struct { _iq    TargetValue; 	// Input: Target input (pu)
				 Uint32 RampDelayMax;	// Parameter: Maximum delay rate (Q0) - independently with global Q			
		 	 	 _iq    RampLowLimit;	// Parameter: Minimum limit (pu)				  
				 _iq    RampHighLimit;	// Parameter: Maximum limit (pu)
				 Uint32 RampDelayCount; // Variable: Incremental delay (Q0) - independently with global Q	
				 _iq    SetpointValue;	// Output: Target output (pu)				 
				 Uint32 EqualFlag;		// Output: Flag output (Q0) - independently with global Q
				 Uint32 RampStep;        // Parameter: Change step per sampling time
		  	   } RMPCNTL_027;	            

typedef RMPCNTL_027 *RMPCNTL_027_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/                     
#define RMPCNTL_027_DEFAULTS { 0, 		 \
                           80,		 \
                          	_IQ(-1), \
                          	_IQ(1),  \
                          	0,       \
                          	0,       \
                          	0,       \
                          	1,   \
                   		  }

/*------------------------------------------------------------------------------
 	RAMP Controller Macro Definition
------------------------------------------------------------------------------*/

#define RC_MACRO_027(v)										\
{															\
	_iq rc_tmp;												\
	_iq rc_step;											\
	rc_tmp = v.TargetValue - v.SetpointValue;			\
/*  0.0000305 is resolution of Q15 */					\
    rc_step = _IQmpy(_IQ(0.0000305),v.RampStep);          \
if (_IQabs(rc_tmp) > rc_step)					\
{														\
	v.RampDelayCount += 1;								\
		if (v.RampDelayCount >= v.RampDelayMax)			\
		{												\
			if (v.TargetValue >= v.SetpointValue)		\
			{											\
				v.SetpointValue += rc_step;		\
				if (v.SetpointValue > v.RampHighLimit)	\
					v.SetpointValue = v.RampHighLimit;	\
				v.RampDelayCount = 0;					\
			}											\
			else										\
			{											\
			v.SetpointValue -= rc_step;			\
			if (v.SetpointValue < v.RampLowLimit)		\
				v.SetpointValue = v.RampLowLimit;		\
			v.RampDelayCount = 0;						\
			}											\
		}												\
}														\
else v.EqualFlag = 0x7FFFFFFF;								\
}

#endif // __RMP_CNTL_H__
