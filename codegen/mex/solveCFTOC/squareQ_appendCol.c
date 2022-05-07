/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * squareQ_appendCol.c
 *
 * Code generation for function 'squareQ_appendCol'
 *
 */

/* Include files */
#include "squareQ_appendCol.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_types.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>

/* Function Definitions */
void squareQ_appendCol(f_struct_T *obj, const real_T vec[7191], int32_T iv0)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  real_T c;
  real_T s;
  int32_T Qk0;
  int32_T idx;
  int32_T k;
  int32_T n;
  int32_T temp_tmp;
  char_T TRANSA;
  Qk0 = obj->ncols + 1;
  obj->minRowCol = muIntScalarMin_sint32(obj->mrows, Qk0);
  if (obj->mrows >= 1) {
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)obj->mrows;
    n_t = (ptrdiff_t)obj->mrows;
    lda_t = (ptrdiff_t)90;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &obj->Q[0], &lda_t, &vec[iv0 - 1],
          &incx_t, &beta1, &obj->QR[90 * obj->ncols], &incy_t);
  }
  obj->ncols++;
  obj->jpvt[obj->ncols - 1] = obj->ncols;
  for (idx = obj->mrows - 2; idx + 2 > obj->ncols; idx--) {
    Qk0 = idx + 90 * (obj->ncols - 1);
    alpha1 = obj->QR[Qk0];
    beta1 = obj->QR[Qk0 + 1];
    c = 0.0;
    s = 0.0;
    drotg(&alpha1, &beta1, &c, &s);
    obj->QR[Qk0] = alpha1;
    obj->QR[Qk0 + 1] = beta1;
    Qk0 = 90 * idx;
    n = obj->mrows;
    if (obj->mrows >= 1) {
      for (k = 0; k < n; k++) {
        temp_tmp = Qk0 + k;
        alpha1 = obj->Q[temp_tmp + 90];
        beta1 = obj->Q[temp_tmp];
        obj->Q[temp_tmp + 90] = c * alpha1 - s * beta1;
        obj->Q[temp_tmp] = c * beta1 + s * alpha1;
      }
    }
  }
}

/* End of code generation (squareQ_appendCol.c) */
