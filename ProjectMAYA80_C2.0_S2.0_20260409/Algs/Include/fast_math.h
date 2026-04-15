#ifndef FAST_MATH_H_
#define FAST_MATH_H_

/* Private configuration defines -------------------------------------------- */

// Please select, for each math function below, the most suitable implementation.
// Generally speaking, a math function can have several implementations such as
// TMU, CLA, FPU, fastRTS, RTS (in order of execution speed).

// For example calls to math.h functions are redirected automatically to the TMU only
// if an optimized implementation for TMU exists and only if relaxed FPU mode is selected
// in the project options; otherwise (slow) implementations from the standard RTS are used.

// The compiler may then use (in order of speed) TMU optimized functions, fastRTS library functions
// and standard library (math.h) functions.

/**
 * To mode the angle.
 * here, X is a float angle;
 * the result is within [0, 2pi];
 * Time consuming: 16 sysclks
 */
#define MODE_2PI(X)						__mpy2pif32(__fracf32(__fracf32(__div2pif32(X)) + 1));
#define MODE_ARBITRARY(X,LOW,HIGH)		((__fracf32(__fracf32(__divf32(X-LOW, HIGH-LOW)) + 1))*(HIGH - LOW) + LOW);

#define	EXP_FRAC(X)		(1 + X*(1 + (X*ONE_SECOND)*(1 + (X*ONE_THIRD)*( 1 +(X*ONE_FOURTH)*(1 + (X*ONE_FIFTH)*(1 + (X*ONE_SIXTH)*(1 + (X*ONE_SEVENTH)*(1 + (X*ONE_EIGHTH)*(1 + X*ONE_NINTH)))))))))

#define MYEXP(X)           (EXP_FRAC(__fracf32(X)) * exp_table[(int)(X)+20])

#define LPF_SG1(Y, U, G, T)			Y = (Y + T*U)/(1 + G*T);
#define LPF_SG2(Y, U, U1, G, T)		Y = ((2-G*T)*Y + T*(U+U1))/(2 + G*T);
#define Pure_int(Y,U,T)             Y = Y + T*U;
#define Norm(X,Y)                   X * fastinverse(fast_math_hypotf(X,Y));

/**
 * To extract the sign of a real number, IEEE-754 floating point.
 * here, X is the input variable, can be defined as float or integer;
 * SIGN is the output sign, should be defined as long;
 */
// 5 sysclks
#define SIGN_EXTRACT(X)    		(1 + 2*((* ((long *) &(X)))>>31))
// 4 sysclks
#define EXPONENT_EXTRACT(X)    	(((* ((long *) &(X))) & 0x7f800000)>>23)
// 3 sysclks
#define MANTISSA_EXTRACT(X)    	((* ((long *) &(X))) & 0x7fffff)

#define fast_math_custom_floorf(X)		((long)(X)>(X) ? (long)(X)-1:(long)(X))
#define fast_math_custom_ceilf(X)		((long)(X)<(X) ? (long)(X)+1:(long)(X))
#define fast_math_custom_roundf(X)		((X) < 0.0F ? fast_math_custom_ceilf((X) - 0.5F) : fast_math_custom_floorf((X) + 0.5F))

#define fastsin(x)			__sin(x)
#define fastsinpu(x)		__sinpuf32(x)
#define fastcos(x)			__cos(x)
#define fastcospu(x)		__cospuf32(x)
#define fastsqrt(x)			__sqrt(x)
#define fastdiv2pi(x)		__div2pif32(x)
#define fastmpy2pi(x)		__mpy2pif32(x)
#define fastmax(x,y)		__fmax(x,y)
#define fastmin(x,y)		__fmin(x,y)
#define fastfrac(x)		__fracf32(x)
#define fastsat(x,max,min)	__fsat(x,max,min)
#define fastinverse(x)		__divf32(1.0,x)
#define fastdiv(num,den)	__divf32(num,den)
#define fastmax_index(dst, src, index_dst, index_src)        __f32_max_idx(dst, src, index_dst, index_src)        // 9 sysclks
#define fastmin_index(dst, src, index_dst, index_src)        __f32_min_idx(dst, src, index_dst, index_src)        // 9 sysclks


#define fast_math_fmodf 			fmodf
#define fast_math_fmod2pif(x) 		__mpy2pif32(__fracf32(__fracf32(__div2pif32(x)) + 1))

#define fast_math_sqrtf 			__sqrt
#define fast_math_isqrtf 			__eisqrtf32

#define fast_math_cbrtf 			fast_math_custom_cbrtf

#define fast_math_sinf 						__sin
#define fast_math_cosf 						__cos
#define fast_math_sincosf(x, psin, pcos)  	*(psin) = __sin(x);\
									 	 	 *(pcos) = __cos(x)

#define fast_math_tanf(x) 			(__sin(x) / __cos(x))

#define fast_math_asinf 			fast_math_asine
#define fast_math_acosf 			fast_math_acosine
#define fast_math_atanf 			__atan
#define fast_math_atan2f 			__atan2
#define fast_math_atan2fpu 			__atan2puf32

#define fast_math_expf 				MYEXP
#define fast_math_expRf 			fast_math_exp2
#define fast_math_exp10f 			fast_math_exp10
//#define fast_math_powf 				fast_math_custom_pow	/*	fast_math_expf(e*fast_math_logf(e))	*/
#define fast_math_powf(x,y)			pow(x,y)

#define fast_math_logf 				fast_math_ln
#define fast_math_log10f 			fast_math_log10
#define fast_math_logNf				fast_math_logN
#define fast_math_log2f 			fast_math_log2
//
#define fast_math_hypotf(x,y)		fast_math_sqrtf((x)*(x) + (y)*(y))
#define fast_math_coshf 			fast_math_custom_coshf
#define fast_math_sinhf 			fast_math_custom_sinhf
#define fast_math_acoshf(x)			fast_math_logf((x) + fast_math_sqrtf((x)*(x) - 1))
#define fast_math_asinhf(x)			fast_math_logf((x) + fast_math_sqrtf((x)*(x) + 1))

#define fast_math_floorf 			fast_math_custom_floorf
#define fast_math_ceilf 			fast_math_custom_ceilf
#define fast_math_roundf 			fast_math_custom_roundf


#define FAST_MATH_LOGe_MACRO(X, TEMP)			TEMP = fast_math_ln(X);
#define FAST_MATH_LOG2_MACRO(X, TEMP)			TEMP = fast_math_ln(X)*iLN2;
#define FAST_MATH_LOG10_MACRO(X, TEMP)			TEMP = fast_math_ln(X)*iLN10;
#define FAST_MATH_LOGN_MACRO(N, X, TEMP)		TEMP = fast_math_ln(X)/fast_math_ln(N);
#define FAST_MATH_EXPe_MACRO(X, TEMP)			TEMP = X; TEMP = MYEXP(TEMP);
#define FAST_MATH_EXP2_MACRO(X, TEMP)			TEMP = X*LN2; TEMP = MYEXP(TEMP);
#define FAST_MATH_EXP10_MACRO(X, TEMP)			TEMP = X*LN10; TEMP = MYEXP(TEMP);
#define FAST_MATH_EXPN_MACRO(N, X, TEMP)		TEMP = X*fast_math_ln(N); TEMP = MYEXP(TEMP);


//
///* Exported functions ------------------------------------------------------- */
//
//
float fast_math_custom_coshf(float x);
float fast_math_custom_sinhf(float x);
//float fast_math_custom_floorf(float x);
//float fast_math_custom_ceilf(float x);
float fast_math_custom_cbrtf(float x);
//float fast_math_custom_roundf(float number);

float fast_math_ln(float x);
float fast_math_log2(float x);
float fast_math_log10(float x);
float fast_math_logN(float N, float x);
float fast_math_exp2(float x);
float fast_math_exp10(float x);
float fast_math_expN(float N, float x);
float fast_math_asine(float x);
float fast_math_acosine(float x);
//float fast_math_custom_pow(float x, float y);

void fast_td(float h, float c, float r, float x_ref, float *x_f, float *v_f);


#endif /* FAST_MATH_H_ */
