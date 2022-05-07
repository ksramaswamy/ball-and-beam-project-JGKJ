/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgeqrf.c
 *
 * Code generation for function 'xgeqrf'
 *
 */

/* Include files */
#include "xgeqrf.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void xgeqrf(real_T A[12690], int32_T m, int32_T n, real_T tau[90])
{
  ptrdiff_t info_t;
  int32_T i;
  int32_T minmn;
  if (n == 0) {
    memset(&tau[0], 0, 90U * sizeof(real_T));
  } else {
    info_t = LAPACKE_dgeqrf(102, (ptrdiff_t)m, (ptrdiff_t)n, &A[0],
                            (ptrdiff_t)90, &tau[0]);
    if ((int32_T)info_t != 0) {
      for (minmn = 0; minmn < n; minmn++) {
        for (i = 0; i < m; i++) {
          A[minmn * 90 + i] = rtNaN;
        }
      }
      minmn = muIntScalarMin_sint32(m, n) - 1;
      for (i = 0; i <= minmn; i++) {
        tau[i] = rtNaN;
      }
      minmn += 2;
      for (i = minmn; i < 91; i++) {
        tau[i - 1] = 0.0;
      }
    }
  }
}

/* End of code generation (xgeqrf.c) */
