/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * setProblemType.c
 *
 * Code generation for function 'setProblemType'
 *
 */

/* Include files */
#include "setProblemType.h"
#include "modifyOverheadPhaseOne_.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include <string.h>

/* Function Definitions */
void setProblemType(g_struct_T *obj, int32_T PROBLEM_TYPE)
{
  int32_T colOffsetATw;
  int32_T colOffsetAeq;
  int32_T i;
  int32_T i1;
  int32_T idx_col;
  int32_T idx_lb;
  switch (PROBLEM_TYPE) {
  case 3:
    obj->nVar = 50;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (colOffsetAeq = 0; colOffsetAeq < i; colOffsetAeq++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + colOffsetAeq) - 1] =
            obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetAeq) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;
  case 1:
    obj->nVar = 51;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    break;
  case 2:
    obj->nVar = 50;
    obj->mConstr = 140;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      for (idx_col = 0; idx_col < 40; idx_col++) {
        idx_lb = idx_col + 50;
        colOffsetAeq = 51 * idx_col - 1;
        colOffsetATw = colOffsetAeq + 51 * (obj->isActiveIdx[1] - 1);
        if (51 <= idx_lb) {
          memset(&obj->Aeq[colOffsetAeq + 51], 0,
                 (((idx_lb + colOffsetAeq) - colOffsetAeq) - 50) *
                     sizeof(real_T));
          memset(&obj->ATwset[colOffsetATw + 51], 0,
                 (((idx_lb + colOffsetATw) - colOffsetATw) - 50) *
                     sizeof(real_T));
        }
        i = idx_col + colOffsetAeq;
        obj->Aeq[i + 51] = -1.0;
        i1 = idx_col + colOffsetATw;
        obj->ATwset[i1 + 51] = -1.0;
        idx_lb = idx_col + 52;
        if (idx_lb <= 90) {
          memset(&obj->Aeq[idx_lb + colOffsetAeq], 0,
                 (((colOffsetAeq - idx_lb) - colOffsetAeq) + 91) *
                     sizeof(real_T));
          memset(&obj->ATwset[idx_lb + colOffsetATw], 0,
                 (((colOffsetATw - idx_lb) - colOffsetATw) + 91) *
                     sizeof(real_T));
        }
        idx_lb = idx_col + 90;
        if (91 <= idx_lb) {
          memset(&obj->Aeq[colOffsetAeq + 91], 0,
                 (((idx_lb + colOffsetAeq) - colOffsetAeq) - 90) *
                     sizeof(real_T));
          memset(&obj->ATwset[colOffsetATw + 91], 0,
                 (((idx_lb + colOffsetATw) - colOffsetATw) - 90) *
                     sizeof(real_T));
        }
        obj->Aeq[i + 91] = 1.0;
        obj->ATwset[i1 + 91] = 1.0;
      }
      idx_lb = 50;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (colOffsetAeq = i; colOffsetAeq <= i1; colOffsetAeq++) {
        idx_lb++;
        obj->indexLB[colOffsetAeq - 1] = idx_lb;
      }
      if (obj->nWConstr[4] > 0) {
        i = obj->sizesRegularized[4];
        for (colOffsetAeq = 0; colOffsetAeq < i; colOffsetAeq++) {
          obj->isActiveConstr[obj->isActiveIdxRegularized[4] + colOffsetAeq] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetAeq) - 1];
        }
      }
      i = obj->isActiveIdx[4];
      i1 = obj->isActiveIdxRegularized[4] - 1;
      if (i <= i1) {
        memset(&obj->isActiveConstr[i + -1], 0,
               ((i1 - i) + 1) * sizeof(boolean_T));
      }
      obj->lb[50] = 0.0;
      idx_lb = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = idx_lb; idx_col <= i; idx_col++) {
        colOffsetATw = 51 * (idx_col - 1) - 1;
        if (obj->Wid[idx_col - 1] == 3) {
          i1 = obj->Wlocalidx[idx_col - 1] + 49;
          if (51 <= i1) {
            memset(&obj->ATwset[colOffsetATw + 51], 0,
                   (((i1 + colOffsetATw) - colOffsetATw) - 50) *
                       sizeof(real_T));
          }
          obj->ATwset[(obj->Wlocalidx[idx_col - 1] + colOffsetATw) + 50] = -1.0;
          i1 = obj->Wlocalidx[idx_col - 1] + 51;
          if (i1 <= 50) {
            memset(&obj->ATwset[i1 + colOffsetATw], 0,
                   (((colOffsetATw - i1) - colOffsetATw) + 51) *
                       sizeof(real_T));
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
    break;
  default:
    obj->nVar = 51;
    obj->mConstr = 141;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    break;
  }
  obj->probType = PROBLEM_TYPE;
}

/* End of code generation (setProblemType.c) */
