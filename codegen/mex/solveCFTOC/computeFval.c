/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFval.c
 *
 * Code generation for function 'computeFval'
 *
 */

/* Include files */
#include "computeFval.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T computeFval(const b_struct_T *obj, real_T workspace[7191],
                   const real_T H[2500], const real_T f[50], const real_T x[51])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  real_T val;
  int32_T ixlast;
  int32_T k;
  char_T TRANSA;
  switch (obj->objtype) {
  case 5:
    val = x[obj->nvar - 1];
    break;
  case 3:
    ixlast = obj->nvar;
    if (0 <= ixlast - 1) {
      memcpy(&workspace[0], &f[0], ixlast * sizeof(real_T));
    }
    alpha1 = 0.5;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)obj->nvar;
    n_t = (ptrdiff_t)obj->nvar;
    lda_t = (ptrdiff_t)obj->nvar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &H[0], &lda_t, &x[0], &incx_t, &beta1,
          &workspace[0], &incy_t);
    val = 0.0;
    if (obj->nvar >= 1) {
      ixlast = obj->nvar;
      for (k = 0; k < ixlast; k++) {
        val += x[k] * workspace[k];
      }
    }
    break;
  default:
    ixlast = obj->nvar;
    if (0 <= ixlast - 1) {
      memcpy(&workspace[0], &f[0], ixlast * sizeof(real_T));
    }
    alpha1 = 0.5;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)obj->nvar;
    n_t = (ptrdiff_t)obj->nvar;
    lda_t = (ptrdiff_t)obj->nvar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &H[0], &lda_t, &x[0], &incx_t, &beta1,
          &workspace[0], &incy_t);
    val = 0.0;
    for (k = 0; k < 50; k++) {
      val += x[k] * workspace[k];
    }
    break;
  }
  return val;
}

/* End of code generation (computeFval.c) */
