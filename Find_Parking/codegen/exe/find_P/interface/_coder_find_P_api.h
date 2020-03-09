/*
 * File: _coder_find_P_api.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 03-Apr-2019 16:57:22
 */

#ifndef _CODER_FIND_P_API_H
#define _CODER_FIND_P_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_find_P_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void find_P(real_T block[8], real_T Point_Target[3], real_T *Type, real_T
                   size[2]);
extern void find_P_api(const mxArray * const prhs[1], int32_T nlhs, const
  mxArray *plhs[3]);
extern void find_P_atexit(void);
extern void find_P_initialize(void);
extern void find_P_terminate(void);
extern void find_P_xil_terminate(void);

#endif

/*
 * File trailer for _coder_find_P_api.h
 *
 * [EOF]
 */
