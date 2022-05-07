/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * removeEqConstr.c
 *
 * Code generation for function 'removeEqConstr'
 *
 */

/* Include files */
#include "removeEqConstr.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"

/* Function Definitions */
void removeEqConstr(g_struct_T *obj, int32_T idx_global)
{
  int32_T TYPE_tmp_tmp;
  int32_T i;
  int32_T idx;
  int32_T totalEq;
  totalEq = (obj->nWConstr[0] + obj->nWConstr[1]) - 1;
  if ((totalEq + 1 != 0) && (idx_global <= totalEq + 1)) {
    if ((obj->nActiveConstr == totalEq + 1) || (idx_global == totalEq + 1)) {
      obj->mEqRemoved++;
      obj->indexEqRemoved[obj->mEqRemoved - 1] = obj->Wlocalidx[idx_global - 1];
      totalEq = obj->Wid[idx_global - 1] - 1;
      obj->isActiveConstr[(obj->isActiveIdx[totalEq] +
                           obj->Wlocalidx[idx_global - 1]) -
                          2] = false;
      obj->Wid[idx_global - 1] = obj->Wid[obj->nActiveConstr - 1];
      obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[obj->nActiveConstr - 1];
      i = obj->nVar;
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[idx + 51 * (idx_global - 1)] =
            obj->ATwset[idx + 51 * (obj->nActiveConstr - 1)];
      }
      obj->bwset[idx_global - 1] = obj->bwset[obj->nActiveConstr - 1];
      obj->nActiveConstr--;
      obj->nWConstr[totalEq]--;
    } else {
      obj->mEqRemoved++;
      TYPE_tmp_tmp = obj->Wid[idx_global - 1] - 1;
      obj->indexEqRemoved[obj->mEqRemoved - 1] = obj->Wlocalidx[idx_global - 1];
      obj->isActiveConstr[(obj->isActiveIdx[obj->Wid[idx_global - 1] - 1] +
                           obj->Wlocalidx[idx_global - 1]) -
                          2] = false;
      obj->Wid[idx_global - 1] = obj->Wid[totalEq];
      obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[totalEq];
      i = obj->nVar;
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[idx + 51 * (idx_global - 1)] =
            obj->ATwset[idx + 51 * totalEq];
      }
      obj->bwset[idx_global - 1] = obj->bwset[totalEq];
      obj->Wid[totalEq] = obj->Wid[obj->nActiveConstr - 1];
      obj->Wlocalidx[totalEq] = obj->Wlocalidx[obj->nActiveConstr - 1];
      i = obj->nVar;
      for (idx = 0; idx < i; idx++) {
        obj->ATwset[idx + 51 * totalEq] =
            obj->ATwset[idx + 51 * (obj->nActiveConstr - 1)];
      }
      obj->bwset[totalEq] = obj->bwset[obj->nActiveConstr - 1];
      obj->nActiveConstr--;
      obj->nWConstr[TYPE_tmp_tmp]--;
    }
  }
}

/* End of code generation (removeEqConstr.c) */
