/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_solveCFTOC_api.c
 *
 * Code generation for function '_coder_solveCFTOC_api'
 *
 */

/* Include files */
#include "_coder_solveCFTOC_api.h"
#include "rt_nonfinite.h"
#include "solveCFTOC.h"
#include "solveCFTOC_data.h"
#include "solveCFTOC_types.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2500];

static real_T (*c_emlrt_marshallIn(const mxArray *f,
                                   const char_T *identifier))[50];

static real_T (*d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[50];

static real_T (*e_emlrt_marshallIn(const mxArray *A,
                                   const char_T *identifier))[2000];

static real_T (*emlrt_marshallIn(const mxArray *H,
                                 const char_T *identifier))[2500];

static const mxArray *emlrt_marshallOut(const real_T u[50]);

static real_T (*f_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2000];

static real_T (*g_emlrt_marshallIn(const mxArray *b,
                                   const char_T *identifier))[40];

static real_T (*h_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[40];

static real_T (*i_emlrt_marshallIn(const mxArray *lb,
                                   const char_T *identifier))[50];

static real_T (*j_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[50];

static real_T (*k_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2500];

static real_T (*l_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[50];

static real_T (*m_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2000];

static real_T (*n_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[40];

static real_T (*o_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[50];

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2500]
{
  real_T(*y)[2500];
  y = k_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*c_emlrt_marshallIn(const mxArray *f,
                                   const char_T *identifier))[50]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[50];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(f), &thisId);
  emlrtDestroyArray(&f);
  return y;
}

static real_T (*d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[50]
{
  real_T(*y)[50];
  y = l_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*e_emlrt_marshallIn(const mxArray *A,
                                   const char_T *identifier))[2000]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[2000];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(emlrtAlias(A), &thisId);
  emlrtDestroyArray(&A);
  return y;
}

static real_T (*emlrt_marshallIn(const mxArray *H,
                                 const char_T *identifier))[2500]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[2500];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(H), &thisId);
  emlrtDestroyArray(&H);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[50])
{
  static const int32_T i = 0;
  static const int32_T i1 = 50;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2000]
{
  real_T(*y)[2000];
  y = m_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*g_emlrt_marshallIn(const mxArray *b,
                                   const char_T *identifier))[40]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[40];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(emlrtAlias(b), &thisId);
  emlrtDestroyArray(&b);
  return y;
}

static real_T (*h_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[40]
{
  real_T(*y)[40];
  y = n_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*i_emlrt_marshallIn(const mxArray *lb,
                                   const char_T *identifier))[50]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[50];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(emlrtAlias(lb), &thisId);
  emlrtDestroyArray(&lb);
  return y;
}

static real_T (*j_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[50]
{
  real_T(*y)[50];
  y = o_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*k_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2500]
{
  static const int32_T dims[2] = {50, 50};
  real_T(*ret)[2500];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                          (const char_T *)"double", false, 2U,
                          (void *)&dims[0]);
  ret = (real_T(*)[2500])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*l_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[50]
{
  static const int32_T dims[2] = {1, 50};
  real_T(*ret)[50];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                          (const char_T *)"double", false, 2U,
                          (void *)&dims[0]);
  ret = (real_T(*)[50])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*m_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2000]
{
  static const int32_T dims[2] = {40, 50};
  real_T(*ret)[2000];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                          (const char_T *)"double", false, 2U,
                          (void *)&dims[0]);
  ret = (real_T(*)[2000])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*n_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[40]
{
  static const int32_T dims = 40;
  real_T(*ret)[40];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                          (const char_T *)"double", false, 1U, (void *)&dims);
  ret = (real_T(*)[40])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*o_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[50]
{
  static const int32_T dims = 50;
  real_T(*ret)[50];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src,
                          (const char_T *)"double", false, 1U, (void *)&dims);
  ret = (real_T(*)[50])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void solveCFTOC_api(solveCFTOCStackData *SD, const mxArray *const prhs[7],
                    const mxArray **plhs)
{
  real_T(*H)[2500];
  real_T(*A)[2000];
  real_T(*f)[50];
  real_T(*lb)[50];
  real_T(*ub)[50];
  real_T(*x)[50];
  real_T(*x0)[50];
  real_T(*b)[40];
  x = (real_T(*)[50])mxMalloc(sizeof(real_T[50]));
  /* Marshall function inputs */
  H = emlrt_marshallIn(emlrtAlias(prhs[0]), "H");
  f = c_emlrt_marshallIn(emlrtAlias(prhs[1]), "f");
  A = e_emlrt_marshallIn(emlrtAlias(prhs[2]), "A");
  b = g_emlrt_marshallIn(emlrtAlias(prhs[3]), "b");
  lb = i_emlrt_marshallIn(emlrtAlias(prhs[4]), "lb");
  ub = i_emlrt_marshallIn(emlrtAlias(prhs[5]), "ub");
  x0 = i_emlrt_marshallIn(emlrtAlias(prhs[6]), "x0");
  /* Invoke the target function */
  solveCFTOC(SD, *H, *f, *A, *b, *lb, *ub, *x0, *x);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*x);
}

/* End of code generation (_coder_solveCFTOC_api.c) */
