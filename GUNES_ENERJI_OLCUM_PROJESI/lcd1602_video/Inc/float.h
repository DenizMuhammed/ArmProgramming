#ifndef _FLOAT_H
#define _FLOAT_H 

/* float */
#define FLT_RADIX     2
#define FLT_MANT_DIG  24
#define FLT_DIG       6
#define FLT_ROUNDS    1
#define FLT_EPSILON   1.19209290e-07F
#define FLT_MIN_EXP   (-125)
#define FLT_MIN       1.17549435e-38F
#define FLT_MIN_10_EXP (-37)
#define FLT_MAX_EXP   128
#define FLT_MAX       3.40282347e+38F
#define FLT_MAX_10_EXP 38

/* double */
#define DBL_MANT_DIG    FLT_MANT_DIG
#define DBL_DIG         FLT_DIG
#define DBL_EPSILON     FLT_EPSILON
#define DBL_MIN_EXP     FLT_MIN_EXP
#define DBL_MIN         FLT_MIN
#define DBL_MIN_10_EXP  FLT_MIN_10_EXP
#define DBL_MAX_EXP     FLT_MAX_EXP
#define DBL_MAX         FLT_MAX
#define DBL_MAX_10_EXP  FLT_MAX_10_EXP

/* long double */
#define LDBL_MANT_DIG 53
#define LDBL_DIG 15
#define LDBL_EPSILON 2.2204460492503131e-16L
#define LDBL_MIN_EXP (-1021)
#define LDBL_MIN 2.2250738585072014e-308L
#define LDBL_MIN_10_EXP (-307)
#define LDBL_MAX_EXP 1024
#define LDBL_MAX 1.7976931348623157e+308L
#define LDBL_MAX_10_EXP 308

#endif
