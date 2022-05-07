/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeQ_.c
 *
 * Code generation for function 'computeQ_'
 *
 */

/* Include files */
#include "computeQ_.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_types.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeQ_(f_struct_T *obj, int32_T nrows)
{
  ptrdiff_t info_t;
  int32_T i;
  int32_T iQR0;
  int32_T idx;
  int32_T n;
  i = obj->minRowCol;
  for (idx = 0; idx < i; idx++) {
    iQR0 = 90 * idx + idx;
    n = obj->mrows - idx;
    if (0 <= n - 2) {
      memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1],
             (((n + iQR0) - iQR0) - 1) * sizeof(real_T));
    }
  }
  info_t = LAPACKE_dorgqr(102, (ptrdiff_t)obj->mrows, (ptrdiff_t)nrows,
                          (ptrdiff_t)obj->minRowCol, &obj->Q[0], (ptrdiff_t)90,
                          &obj->tau[0]);
  if ((int32_T)info_t != 0) {
    for (i = 0; i < 8100; i++) {
      obj->Q[i] = rtNaN;
    }
  }
}

/* End of code generation (computeQ_.c) */
