/* =================================================================================
File name:       CLARKE.H  (IQ version)                  
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type, and macro definition for the CLARKE.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2009	Version 1.0                                                  
------------------------------------------------------------------------------*/
#ifndef __CLARKE_027_H__
#define __CLARKE_027_H__

typedef struct {  _iq  As;  		// Input: phase-a stator variable
				  _iq  Bs;			// Input: phase-b stator variable 
				  _iq  Alpha;		// Output: stationary d-axis stator variable 
				  _iq  Beta;		// Output: stationary q-axis stator variable
		 	 	} CLARKE_027;	            

typedef CLARKE_027 *CLARKE_027_handle;
/*-----------------------------------------------------------------------------
	Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_027_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
              			} 

/*------------------------------------------------------------------------------
	CLARKE Transformation Macro Definition
------------------------------------------------------------------------------*/
	

#define CLARKE_MACRO_027(v)											\
																\
v.Alpha = v.As;													\
v.Beta = _IQmpy((v.As +_IQmpy2(v.Bs)),_IQ(0.57735026918963));
																
//  1/sqrt(3) = 0.57735026918963								

#endif // __CLARKE_H__
