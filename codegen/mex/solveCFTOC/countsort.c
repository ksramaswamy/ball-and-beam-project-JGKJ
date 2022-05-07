/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * countsort.c
 *
 * Code generation for function 'countsort'
 *
 */

/* Include files */
#include "countsort.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void countsort(int32_T x[141], int32_T xLen, int32_T workspace[141],
               int32_T xMin, int32_T xMax)
{
  int32_T idx;
  int32_T idxEnd;
  int32_T idxFill;
  int32_T idxStart;
  int32_T maxOffset;
  if ((xLen > 1) && (xMax > xMin)) {
    idxStart = xMax - xMin;
    if (0 <= idxStart) {
      memset(&workspace[0], 0, (idxStart + 1) * sizeof(int32_T));
    }
    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart]++;
    }
    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] += workspace[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }
    for (idx = idxStart; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

/* End of code generation (countsort.c) */
