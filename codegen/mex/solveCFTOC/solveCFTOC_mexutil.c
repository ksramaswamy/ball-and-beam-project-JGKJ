/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveCFTOC_mexutil.c
 *
 * Code generation for function 'solveCFTOC_mexutil'
 *
 */

/* Include files */
#include "solveCFTOC_mexutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
int32_T div_nde_s32_floor(int32_T numerator)
{
  int32_T b_numerator;
  if ((numerator < 0) && (numerator % 51 != 0)) {
    b_numerator = -1;
  } else {
    b_numerator = 0;
  }
  return numerator / 51 + b_numerator;
}

/* End of code generation (solveCFTOC_mexutil.c) */
