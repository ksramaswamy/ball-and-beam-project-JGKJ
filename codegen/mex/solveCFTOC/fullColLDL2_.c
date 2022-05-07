/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fullColLDL2_.c
 *
 * Code generation for function 'fullColLDL2_'
 *
 */

/* Include files */
#include "fullColLDL2_.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_types.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Function Definitions */
void fullColLDL2_(c_struct_T *obj, int32_T NColsRemain, real_T REG_PRIMAL)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  int32_T LD_diagOffset;
  int32_T i;
  int32_T idx;
  int32_T k;
  int32_T subMatrixDim;
  for (k = 0; k < NColsRemain; k++) {
    LD_diagOffset = 91 * k;
    if (muDoubleScalarAbs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
      obj->FMat[LD_diagOffset] += REG_PRIMAL;
    }
    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 1;
    i = subMatrixDim - 1;
    for (idx = 0; idx <= i; idx++) {
      obj->workspace_[idx] = obj->FMat[(LD_diagOffset + idx) + 1];
    }
    if (subMatrixDim >= 1) {
      m_t = (ptrdiff_t)subMatrixDim;
      n_t = (ptrdiff_t)subMatrixDim;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      lda_t = (ptrdiff_t)90;
      dger(&m_t, &n_t, &alpha1, &obj->workspace_[0], &incx_t,
           &obj->workspace_[0], &incy_t, &obj->FMat[LD_diagOffset + 91],
           &lda_t);
    }
    for (idx = 0; idx < subMatrixDim; idx++) {
      i = (LD_diagOffset + idx) + 1;
      obj->FMat[i] /= obj->FMat[LD_diagOffset];
    }
  }
  LD_diagOffset = 91 * (NColsRemain - 1);
  if (muDoubleScalarAbs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
    obj->FMat[LD_diagOffset] += REG_PRIMAL;
  }
}

/* End of code generation (fullColLDL2_.c) */
