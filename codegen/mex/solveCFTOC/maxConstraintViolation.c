/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * maxConstraintViolation.c
 *
 * Code generation for function 'maxConstraintViolation'
 *
 */

/* Include files */
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_mexutil.h"
#include "mwmathutil.h"

/* Function Definitions */
real_T maxConstraintViolation(g_struct_T *obj, const real_T x[51])
{
  real_T c;
  real_T v;
  int32_T ia;
  int32_T iac;
  int32_T idxLB;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  if (obj->probType == 2) {
    for (idxLB = 0; idxLB < 40; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 1989; iac += 51) {
      c = 0.0;
      idxLB = iac + 50;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    v = 0.0;
  } else {
    v = 0.0;
    for (idxLB = 0; idxLB < 40; idxLB++) {
      obj->maxConstrWorkspace[idxLB] = obj->beq[idxLB];
      obj->maxConstrWorkspace[idxLB] = -obj->maxConstrWorkspace[idxLB];
    }
    for (iac = 0; iac <= 1989; iac += 51) {
      c = 0.0;
      idxLB = iac + obj->nVar;
      for (ia = iac + 1; ia <= idxLB; ia++) {
        c += obj->Aeq[ia - 1] * x[(ia - iac) - 1];
      }
      idxLB = div_nde_s32_floor(iac);
      obj->maxConstrWorkspace[idxLB] += c;
    }
    for (iac = 0; iac < 40; iac++) {
      v = muDoubleScalarMax(v, muDoubleScalarAbs(obj->maxConstrWorkspace[iac]));
    }
  }
  if (obj->sizes[3] > 0) {
    for (iac = 0; iac < mLB; iac++) {
      idxLB = obj->indexLB[iac] - 1;
      v = muDoubleScalarMax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }
  if (obj->sizes[4] > 0) {
    for (iac = 0; iac < mUB; iac++) {
      idxLB = obj->indexUB[iac] - 1;
      v = muDoubleScalarMax(v, x[idxLB] - obj->ub[idxLB]);
    }
  }
  if (obj->sizes[0] > 0) {
    for (iac = 0; iac < mFixed; iac++) {
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(x[obj->indexFixed[iac] - 1] -
                               obj->ub[obj->indexFixed[iac] - 1]));
    }
  }
  return v;
}

/* End of code generation (maxConstraintViolation.c) */
