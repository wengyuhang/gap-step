/* C-only entry points for the unchanged SciPy 1.15.3 L-BFGS-B implementation.
 * Python/NumPy binding declarations are omitted. Upstream license is in lbfgsb.c.
 */
#pragma once
#include <math.h>
#define daxpy_ scipy_daxpy_
#define dscal_ scipy_dscal_
#define dcopy_ scipy_dcopy_
#define dnrm2_ scipy_dnrm2_
#define ddot_ scipy_ddot_
#define dpotrf_ scipy_dpotrf_
#define dtrtrs_ scipy_dtrtrs_
void daxpy_(int*,double*,double*,int*,double*,int*);
void dscal_(int*,double*,double*,int*);
void dcopy_(int*,double*,int*,double*,int*);
double dnrm2_(int*,double*,int*);
double ddot_(int*,double*,int*,double*,int*);
void dpotrf_(char*,int*,double*,int*,int*);
void dtrtrs_(char*,char*,char*,int*,int*,double*,int*,double*,int*,int*);
