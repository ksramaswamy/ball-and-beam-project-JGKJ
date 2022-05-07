/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFirstOrderOpt.c
 *
 * Code generation for function 'computeFirstOrderOpt'
 *
 */

/* Include files */
#include "computeFirstOrderOpt.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void computeFirstOrderOpt(struct_T *solution, const b_struct_T *objective,
                          int32_T workingset_nVar,
                          const real_T workingset_ATwset[7191],
                          int32_T workingset_nActiveConstr,
                          real_T workspace[7191])
{
  real_T s;
  real_T smax;
  int32_T i;
  int32_T i1;
  int32_T ia;
  int32_T iac;
  int32_T ix;
  int32_T k;
  if (0 <= workingset_nVar - 1) {
    memcpy(&workspace[0], &objective->grad[0],
           workingset_nVar * sizeof(real_T));
  }
  if (workingset_nActiveConstr != 0) {
    ix = 0;
    k = 51 * (workingset_nActiveConstr - 1) + 1;
    for (iac = 1; iac <= k; iac += 51) {
      i = (iac + workingset_nVar) - 1;
      for (ia = iac; ia <= i; ia++) {
        i1 = ia - iac;
        workspace[i1] += workingset_ATwset[ia - 1] * solution->lambda[ix];
      }
      ix++;
    }
  }
  ix = 1;
  smax = muDoubleScalarAbs(workspace[0]);
  for (k = 2; k <= workingset_nVar; k++) {
    s = muDoubleScalarAbs(workspace[k - 1]);
    if (s > smax) {
      ix = k;
      smax = s;
    }
  }
  solution->firstorderopt = muDoubleScalarAbs(workspace[ix - 1]);
}

/* End of code generation (computeFirstOrderOpt.c) */
