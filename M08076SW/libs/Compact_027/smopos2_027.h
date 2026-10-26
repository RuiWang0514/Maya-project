/* =================================================================================
File name:       SMOPOS.H   (IQ version)                  
                     
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the SMOPOS.
====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2009	Version 1.0                                                 
------------------------------------------------------------------------------*/
#ifndef __SMOPOS2_027_H_
#define __SMOPOS2_027_H_

typedef struct {  _iq  Valpha;   	// Input: Stationary alfa-axis stator voltage 
                  _iq  Ealpha;   	// Variable: Stationary alfa-axis back EMF 
                  _iq  Zalpha;      // Output: Stationary alfa-axis sliding control 
                  _iq  Gsmopos;    	// Parameter: Motor dependent control gain 
                  _iq  EstIalpha;   // Variable: Estimated stationary alfa-axis stator current 
                  _iq  Fsmopos;    	// Parameter: Motor dependent plant matrix 
                  _iq  Vbeta;   	// Input: Stationary beta-axis stator voltage 
                  _iq  Ebeta;  		// Variable: Stationary beta-axis back EMF 
                  _iq  Zbeta;      	// Output: Stationary beta-axis sliding control 
                  _iq  EstIbeta;    // Variable: Estimated stationary beta-axis stator current 
                  _iq  Ialpha;  	// Input: Stationary alfa-axis stator current 
                  _iq  IalphaError; // Variable: Stationary alfa-axis current error                 
                  _iq  Kslide;     	// Parameter: Sliding control gain 
                  _iq  Ibeta;  		// Input: Stationary beta-axis stator current 
                  _iq  IbetaError;  // Variable: Stationary beta-axis current error                 
                  _iq  Kslf;       	// Parameter: Sliding control filter gain 
                  _iq  Theta;     	// Output: Compensated rotor angle
                  _iq  K2;               // Parameter: Constant for low-pass filter (pu)
                  _iq  K3;
                 
				 } SMOPOS2_027;

typedef SMOPOS2_027 *SMOPOS2_027_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SMOPOS object.
-----------------------------------------------------------------------------*/                     
#define SMOPOS2_027_DEFAULTS {  0,0,0,0,0,0,0,0,0,0,   \
	                       0,0,0,0,0,0,0,0,0,   \
              			 }

/*------------------------------------------------------------------------------
Prototypes for the functions in SMOPOS.C
------------------------------------------------------------------------------*/

_iq E02 =_IQ(0.5);
_iq invE02=_IQ(2.0);
_iq Ealpha_temp;
_iq Ebeta_temp;
_iq Tmp_lpf;
_iq Tmp_lpf2;
#define SMO_MACRO2_027(v)																					\
																										\
    /*	Sliding mode current observer	*/																\
    v.EstIalpha = _IQmpy(v.Fsmopos,v.EstIalpha) + _IQmpy(v.Gsmopos,(v.Valpha-v.Zalpha));		        \
    v.EstIbeta = _IQmpy(v.Fsmopos,v.EstIbeta) + _IQmpy(v.Gsmopos,(v.Vbeta-v.Zbeta));			        \
																										\
	/*	Current errors	*/																				\
    v.IalphaError = v.EstIalpha - v.Ialpha;																\
    v.IbetaError= v.EstIbeta - v.Ibeta;																	\
    																									\
	/*  Sliding control calculator	*/																	\
    Ealpha_temp = _IQmpy(v.Kslide,v.IalphaError);                                                       \
	if (_IQabs(Ealpha_temp) < E02)																		\
    	v.Zalpha = Ealpha_temp;										                                    \
	else if (Ealpha_temp >= E02)																		\
		v.Zalpha = E02;																			        \
	else if (Ealpha_temp <= -E02)																		\
		v.Zalpha = -E02;																			    \
    Ebeta_temp=_IQmpy(v.Kslide,v.IbetaError);                                                           \
	if (_IQabs(Ebeta_temp) < E02)																		\
		v.Zbeta = Ebeta_temp;											                                \
	else if (Ebeta_temp >= E02)																		    \
		v.Zbeta = E02;																				    \
	else if (Ebeta_temp <= -E02)																		\
		v.Zbeta = -E02;																			        \
																										\
	/* Low-pass filter*/                                                                                \
        Tmp_lpf = v.Zalpha;                                                                             \
        Tmp_lpf = _IQmpy(v.K2,v.Zalpha)+_IQmpy(v.K3,Tmp_lpf);                                           \
        v.Ealpha =Tmp_lpf;                                                                              \
                                                                                                        \
        Tmp_lpf2 = v.Zbeta;                                                                             \
        Tmp_lpf2 = _IQmpy(v.K2,v.Zbeta)+_IQmpy(v.K3,Tmp_lpf2);                                          \
        v.Ebeta =Tmp_lpf2;                                                                              \
                                                                                                        \
	/*	Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)	*/										\
	v.Theta = _IQatan2PU(-v.Ealpha,v.Ebeta); 

#endif
