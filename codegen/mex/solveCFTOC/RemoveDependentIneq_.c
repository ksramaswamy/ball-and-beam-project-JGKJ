/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RemoveDependentIneq_.c
 *
 * Code generation for function 'RemoveDependentIneq_'
 *
 */

/* Include files */
#include "RemoveDependentIneq_.h"
#include "countsort.h"
#include "factorQRE.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void RemoveDependentIneq_(g_struct_T *workingset, f_struct_T *qrmanager,
                          e_struct_T *memspace, real_T tolfactor)
{
  real_T tol;
  int32_T idx;
  int32_T idxDiag;
  int32_T idx_col;
  int32_T ix0;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  int32_T nVar_tmp_tmp;
  nDepIneq = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar_tmp_tmp = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
          workingset->nWConstr[4] >
      0) {
    tol = tolfactor * (real_T)workingset->nVar * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt[idx] = 1;
    }
    idx_col = nFixedConstr + 1;
    if (idx_col <= nDepIneq) {
      memset(&qrmanager->jpvt[idx_col + -1], 0,
             ((nDepIneq - idx_col) + 1) * sizeof(int32_T));
    }
    for (idx_col = 0; idx_col < nDepIneq; idx_col++) {
      idxDiag = 90 * idx_col;
      ix0 = 51 * idx_col;
      for (idx = 0; idx < nVar_tmp_tmp; idx++) {
        qrmanager->QR[idxDiag + idx] = workingset->ATwset[ix0 + idx];
      }
    }
    factorQRE(qrmanager, workingset->nVar, workingset->nActiveConstr);
    nDepIneq = 0;
    for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar_tmp_tmp; idx--) {
      nDepIneq++;
      memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
    }
    if (idx + 1 <= workingset->nVar) {
      idxDiag = idx + 90 * idx;
      while ((idx + 1 > nFixedConstr) &&
             (muDoubleScalarAbs(qrmanager->QR[idxDiag]) < tol)) {
        nDepIneq++;
        memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx];
        idx--;
        idxDiag -= 91;
      }
    }
    countsort(memspace->workspace_int, nDepIneq, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      idxDiag = memspace->workspace_int[idx - 1] - 1;
      ix0 = workingset->Wid[idxDiag] - 1;
      workingset->isActiveConstr[(workingset->isActiveIdx[ix0] +
                                  workingset->Wlocalidx[idxDiag]) -
                                 2] = false;
      workingset->Wid[idxDiag] = workingset->Wid[workingset->nActiveConstr - 1];
      workingset->Wlocalidx[idxDiag] =
          workingset->Wlocalidx[workingset->nActiveConstr - 1];
      idx_col = workingset->nVar;
      for (nVar_tmp_tmp = 0; nVar_tmp_tmp < idx_col; nVar_tmp_tmp++) {
        workingset->ATwset[nVar_tmp_tmp + 51 * idxDiag] =
            workingset
                ->ATwset[nVar_tmp_tmp + 51 * (workingset->nActiveConstr - 1)];
      }
      workingset->bwset[idxDiag] =
          workingset->bwset[workingset->nActiveConstr - 1];
      workingset->nActiveConstr--;
      workingset->nWConstr[ix0]--;
    }
  }
}

/* End of code generation (RemoveDependentIneq_.c) */
