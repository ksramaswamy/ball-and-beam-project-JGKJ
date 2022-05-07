/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factorQR.c
 *
 * Code generation for function 'factorQR'
 *
 */

/* Include files */
#include "factorQR.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_types.h"
#include "xgeqrf.h"
#include "mwmathutil.h"

/* Function Definitions */
void factorQR(f_struct_T *obj, const real_T A[7191], int32_T mrows,
              int32_T ncols)
{
  int32_T idx;
  int32_T ix0;
  int32_T iy0;
  int32_T k;
  boolean_T guard1 = false;
  ix0 = mrows * ncols;
  guard1 = false;
  if (ix0 > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = 51 * idx;
      iy0 = 90 * idx;
      for (k = 0; k < mrows; k++) {
        obj->QR[iy0 + k] = A[ix0 + k];
      }
    }
    guard1 = true;
  } else if (ix0 == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt[idx] = idx + 1;
    }
    obj->minRowCol = muIntScalarMin_sint32(mrows, ncols);
    xgeqrf(obj->QR, mrows, ncols, obj->tau);
  }
}

/* End of code generation (factorQR.c) */
