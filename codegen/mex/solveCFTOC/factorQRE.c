/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factorQRE.c
 *
 * Code generation for function 'factorQRE'
 *
 */

/* Include files */
#include "factorQRE.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_types.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void factorQRE(f_struct_T *obj, int32_T mrows, int32_T ncols)
{
  ptrdiff_t jpvt_t[141];
  ptrdiff_t info_t;
  int32_T b_i;
  int32_T i;
  int32_T k;
  if (mrows * ncols == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    obj->usedPivoting = true;
    obj->mrows = mrows;
    obj->ncols = ncols;
    i = muIntScalarMin_sint32(mrows, ncols);
    obj->minRowCol = i;
    if (ncols < 1) {
      memset(&obj->tau[0], 0, 90U * sizeof(real_T));
      for (k = 0; k < ncols; k++) {
        obj->jpvt[k] = k + 1;
      }
    } else {
      for (b_i = 0; b_i < 141; b_i++) {
        jpvt_t[b_i] = (ptrdiff_t)obj->jpvt[b_i];
      }
      info_t =
          LAPACKE_dgeqp3(102, (ptrdiff_t)mrows, (ptrdiff_t)ncols, &obj->QR[0],
                         (ptrdiff_t)90, &jpvt_t[0], &obj->tau[0]);
      if ((int32_T)info_t != 0) {
        for (k = 0; k < ncols; k++) {
          for (b_i = 0; b_i < mrows; b_i++) {
            obj->QR[k * 90 + b_i] = rtNaN;
          }
        }
        b_i = i - 1;
        for (k = 0; k <= b_i; k++) {
          obj->tau[k] = rtNaN;
        }
        i++;
        for (k = i; k < 91; k++) {
          obj->tau[k - 1] = 0.0;
        }
        for (k = 0; k < ncols; k++) {
          obj->jpvt[k] = k + 1;
        }
      } else {
        for (k = 0; k < ncols; k++) {
          obj->jpvt[k] = (int32_T)jpvt_t[k];
        }
      }
    }
  }
}

/* End of code generation (factorQRE.c) */
