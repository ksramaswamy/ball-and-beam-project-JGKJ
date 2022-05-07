/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * modifyOverheadPhaseOne_.c
 *
 * Code generation for function 'modifyOverheadPhaseOne_'
 *
 */

/* Include files */
#include "modifyOverheadPhaseOne_.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"

/* Function Definitions */
void modifyOverheadPhaseOne_(g_struct_T *obj)
{
  int32_T idx;
  int32_T idxEq;
  int32_T idxStartIneq;
  idxEq = obj->sizes[0];
  for (idx = 0; idx < idxEq; idx++) {
    obj->ATwset[51 * idx + 50] = 0.0;
  }
  for (idx = 0; idx < 40; idx++) {
    idxEq = 51 * idx + 50;
    obj->Aeq[idxEq] = 0.0;
    obj->ATwset[idxEq + 51 * (obj->isActiveIdx[1] - 1)] = 0.0;
  }
  obj->indexLB[obj->sizes[3] - 1] = 51;
  obj->lb[50] = obj->SLACK0;
  idxStartIneq = obj->isActiveIdx[2];
  idxEq = obj->nActiveConstr;
  for (idx = idxStartIneq; idx <= idxEq; idx++) {
    obj->ATwset[51 * (idx - 1) + 50] = -1.0;
  }
  if (obj->nWConstr[4] > 0) {
    idxEq = obj->sizesNormal[4];
    for (idx = 0; idx <= idxEq; idx++) {
      obj->isActiveConstr[(obj->isActiveIdx[4] + idx) - 1] = false;
    }
  }
  obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
}

/* End of code generation (modifyOverheadPhaseOne_.c) */
