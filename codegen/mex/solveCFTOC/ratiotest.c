/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ratiotest.c
 *
 * Code generation for function 'ratiotest'
 *
 */

/* Include files */
#include "ratiotest.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "mwmathutil.h"

/* Function Definitions */
void ratiotest(const real_T solution_xstar[51],
               const real_T solution_searchDir[51], int32_T workingset_nVar,
               const real_T workingset_lb[51], const real_T workingset_ub[51],
               const int32_T workingset_indexLB[51],
               const int32_T workingset_indexUB[51],
               const int32_T workingset_sizes[5],
               const int32_T workingset_isActiveIdx[6],
               const boolean_T workingset_isActiveConstr[141],
               const int32_T workingset_nWConstr[5], real_T *toldelta,
               real_T *alpha, boolean_T *newBlocking, int32_T *constrType,
               int32_T *constrIdx)
{
  real_T denomTol;
  real_T p_max;
  real_T phaseOneCorrectionP;
  real_T phaseOneCorrectionX;
  real_T pk_corrected;
  real_T ratio;
  real_T ratio_tmp;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  p_max = 0.0;
  denomTol =
      2.2204460492503131E-13 * xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      i1 = workingset_indexLB[idx];
      pk_corrected = -solution_searchDir[i1 - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio_tmp = -solution_xstar[i1 - 1] - workingset_lb[i1 - 1];
        ratio = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        ratio = muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-5 - ratio) /
                pk_corrected;
        if ((ratio <= *alpha) && (muDoubleScalarAbs(pk_corrected) > p_max)) {
          *alpha = ratio;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        ratio = ratio_tmp - phaseOneCorrectionX;
        ratio = muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-5 - ratio) /
                pk_corrected;
        if (ratio < *alpha) {
          *alpha = ratio;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = muDoubleScalarAbs(pk_corrected);
        }
      }
    }
    i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    phaseOneCorrectionX = solution_searchDir[i];
    if ((-phaseOneCorrectionX > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio_tmp = -solution_xstar[i] - workingset_lb[i];
      ratio = ratio_tmp - *toldelta;
      ratio = muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-5 - ratio) /
              -phaseOneCorrectionX;
      if ((ratio <= *alpha) &&
          (muDoubleScalarAbs(phaseOneCorrectionX) > p_max)) {
        *alpha = ratio;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
      ratio =
          muDoubleScalarMin(muDoubleScalarAbs(ratio_tmp), 1.0E-5 - ratio_tmp) /
          -phaseOneCorrectionX;
      if (ratio < *alpha) {
        *alpha = ratio;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
        p_max = muDoubleScalarAbs(solution_searchDir[i]);
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    for (idx = 0; idx < totalUB; idx++) {
      i = workingset_indexUB[idx];
      pk_corrected = solution_searchDir[i - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio_tmp = solution_xstar[i - 1] - workingset_ub[i - 1];
        ratio = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        ratio = muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-5 - ratio) /
                pk_corrected;
        if ((ratio <= *alpha) && (muDoubleScalarAbs(pk_corrected) > p_max)) {
          *alpha = ratio;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
        ratio = ratio_tmp - phaseOneCorrectionX;
        ratio = muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-5 - ratio) /
                pk_corrected;
        if (ratio < *alpha) {
          *alpha = ratio;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = muDoubleScalarAbs(pk_corrected);
        }
      }
    }
  }
  *toldelta += 6.608625846508183E-7;
  if (p_max > 0.0) {
    *alpha = muDoubleScalarMax(*alpha, 6.608625846508183E-7 / p_max);
  }
  if ((*newBlocking) && (*alpha > 1.0)) {
    *newBlocking = false;
  }
  *alpha = muDoubleScalarMin(*alpha, 1.0);
}

/* End of code generation (ratiotest.c) */
