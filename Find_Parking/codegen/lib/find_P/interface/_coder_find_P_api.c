/*
 * File: _coder_find_P_api.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 03-Apr-2019 16:45:30
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_find_P_api.h"
#include "_coder_find_P_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131467U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "find_P",                            /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[8];
static const mxArray *b_emlrt_marshallOut(const real_T u);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8];
static const mxArray *c_emlrt_marshallOut(const real_T u[2]);
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *block,
  const char_T *identifier))[8];
static const mxArray *emlrt_marshallOut(const real_T u[3]);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[8]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[8]
{
  real_T (*y)[8];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
  static const mxArray *b_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[8]
 */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8]
{
  real_T (*ret)[8];
  static const int32_T dims[2] = { 4, 2 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  ret = (real_T (*)[8])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const real_T u[2]
 * Return Type  : const mxArray *
 */
  static const mxArray *c_emlrt_marshallOut(const real_T u[2])
{
  const mxArray *y;
  const mxArray *m2;
  static const int32_T iv2[2] = { 0, 0 };

  static const int32_T iv3[2] = { 1, 2 };

  y = NULL;
  m2 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m2, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m2, *(int32_T (*)[2])&iv3[0], 2);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *block
 *                const char_T *identifier
 * Return Type  : real_T (*)[8]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *block,
  const char_T *identifier))[8]
{
  real_T (*y)[8];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(block), &thisId);
  emlrtDestroyArray(&block);
  return y;
}
/*
 * Arguments    : const real_T u[3]
 * Return Type  : const mxArray *
 */
  static const mxArray *emlrt_marshallOut(const real_T u[3])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 0 };

  static const int32_T iv1[1] = { 3 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[1])&iv1[0], 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[1]
 *                int32_T nlhs
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
void find_P_api(const mxArray * const prhs[1], int32_T nlhs, const mxArray *
                plhs[3])
{
  real_T (*Point_Target)[3];
  real_T (*size)[2];
  real_T (*block)[8];
  real_T Type;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  Point_Target = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));
  size = (real_T (*)[2])mxMalloc(sizeof(real_T [2]));

  /* Marshall function inputs */
  block = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "block");

  /* Invoke the target function */
  find_P(*block, *Point_Target, &Type, *size);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*Point_Target);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(Type);
  }

  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(*size);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void find_P_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  find_P_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void find_P_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void find_P_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_find_P_api.c
 *
 * [EOF]
 */
