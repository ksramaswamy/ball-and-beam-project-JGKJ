/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGrad_StoreHx.c
 *
 * Code generation for function 'computeGrad_StoreHx'
 *
 */

/* Include files */
#include "computeGrad_StoreHx.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeGrad_StoreHx(b_struct_T *obj, const real_T H[2500],
                         const real_T f[50], const real_T x[51])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  int32_T i;
  int32_T idx;
  char_T TRANSA;
  switch (obj->objtype) {
  case 5:
    i = obj->nvar;
    if (0 <= i - 2) {
      memset(&obj->grad[0], 0, (i - 1) * sizeof(real_T));
    }
    obj->grad[obj->nvar - 1] = obj->gammaScalar;
    break;
  case 3:
    if (obj->nvar >= 1) {
      alpha1 = 1.0;
      beta1 = 0.0;
      TRANSA = 'N';
      m_t = (ptrdiff_t)obj->nvar;
      n_t = (ptrdiff_t)obj->nvar;
      lda_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &alpha1, &H[0], &lda_t, &x[0], &incx_t, &beta1,
            &obj->Hx[0], &incy_t);
    }
    i = obj->nvar;
    if (0 <= i - 1) {
      memcpy(&obj->grad[0], &obj->Hx[0], i * sizeof(real_T));
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      alpha1 = 1.0;
      n_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      daxpy(&n_t, &alpha1, &f[0], &incx_t, &obj->grad[0], &incy_t);
    }
    break;
  default:
    if (obj->nvar >= 1) {
      alpha1 = 1.0;
      beta1 = 0.0;
      TRANSA = 'N';
      m_t = (ptrdiff_t)obj->nvar;
      n_t = (ptrdiff_t)obj->nvar;
      lda_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &alpha1, &H[0], &lda_t, &x[0], &incx_t, &beta1,
            &obj->Hx[0], &incy_t);
    }
    i = obj->nvar + 1;
    for (idx = i; idx < 51; idx++) {
      obj->Hx[idx - 1] = 0.0 * x[idx - 1];
    }
    memcpy(&obj->grad[0], &obj->Hx[0], 50U * sizeof(real_T));
    if (obj->hasLinear && (obj->nvar >= 1)) {
      alpha1 = 1.0;
      n_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      daxpy(&n_t, &alpha1, &f[0], &incx_t, &obj->grad[0], &incy_t);
    }
    break;
  }
}

/* End of code generation (computeGrad_StoreHx.c) */
