/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * addBoundToActiveSetMatrix_.c
 *
 * Code generation for function 'addBoundToActiveSetMatrix_'
 *
 */

/* Include files */
#include "addBoundToActiveSetMatrix_.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include <string.h>

/* Function Definitions */
void addBoundToActiveSetMatrix_(g_struct_T *obj, int32_T TYPE,
                                int32_T idx_local)
{
  int32_T colOffset;
  int32_T i;
  int32_T idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  colOffset = 51 * (obj->nActiveConstr - 1) - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = obj->ub[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB[idx_local - 1];
    obj->bwset[obj->nActiveConstr - 1] = obj->lb[idx_bnd_local - 1];
  }
  if (0 <= idx_bnd_local - 2) {
    memset(&obj->ATwset[colOffset + 1], 0,
           (((idx_bnd_local + colOffset) - colOffset) - 1) * sizeof(real_T));
  }
  obj->ATwset[idx_bnd_local + colOffset] = 2.0 * (real_T)(TYPE == 5) - 1.0;
  idx_bnd_local++;
  i = obj->nVar;
  if (idx_bnd_local <= i) {
    memset(&obj->ATwset[idx_bnd_local + colOffset], 0,
           ((((i + colOffset) - idx_bnd_local) - colOffset) + 1) *
               sizeof(real_T));
  }
  switch (obj->probType) {
  case 3:
  case 2:
    break;
  default:
    obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/* End of code generation (addBoundToActiveSetMatrix_.c) */
