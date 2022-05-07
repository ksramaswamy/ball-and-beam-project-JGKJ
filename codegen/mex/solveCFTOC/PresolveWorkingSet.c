/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PresolveWorkingSet.c
 *
 * Code generation for function 'PresolveWorkingSet'
 *
 */

/* Include files */
#include "PresolveWorkingSet.h"
#include "RemoveDependentIneq_.h"
#include "computeQ_.h"
#include "countsort.h"
#include "factorQRE.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "removeEqConstr.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                        g_struct_T *workingset, f_struct_T *qrmanager)
{
  ptrdiff_t info_t;
  real_T qtb;
  real_T tol;
  int32_T idx;
  int32_T idxDiag;
  int32_T idx_col;
  int32_T k;
  int32_T mTotalWorkingEq;
  int32_T mWorkingFixed;
  int32_T nDepInd;
  int32_T nVar;
  int32_T totalRank;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T okWorkingSet;
  solution->state = 82;
  qrmanager->ldq = 90;
  memset(&qrmanager->QR[0], 0, 12690U * sizeof(real_T));
  memset(&qrmanager->Q[0], 0, 8100U * sizeof(real_T));
  memset(&qrmanager->jpvt[0], 0, 141U * sizeof(int32_T));
  qrmanager->mrows = 0;
  qrmanager->ncols = 0;
  memset(&qrmanager->tau[0], 0, 90U * sizeof(real_T));
  qrmanager->minRowCol = 0;
  qrmanager->usedPivoting = false;
  nVar = workingset->nVar - 1;
  mWorkingFixed = workingset->nWConstr[0] - 1;
  mTotalWorkingEq = workingset->nWConstr[0] + 39;
  for (totalRank = 0; totalRank <= mTotalWorkingEq; totalRank++) {
    for (idx_col = 0; idx_col <= nVar; idx_col++) {
      qrmanager->QR[totalRank + 90 * idx_col] =
          workingset->ATwset[idx_col + 51 * totalRank];
    }
  }
  totalRank = (workingset->nWConstr[0] - workingset->nVar) + 40;
  nDepInd = muIntScalarMax_sint32(0, totalRank);
  if (0 <= nVar) {
    memset(&qrmanager->jpvt[0], 0, (nVar + 1) * sizeof(int32_T));
  }
  factorQRE(qrmanager, workingset->nWConstr[0] + 40, workingset->nVar);
  tol = 100.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
  idxDiag = mTotalWorkingEq + 1;
  totalRank = muIntScalarMin_sint32(workingset->nVar, idxDiag);
  idxDiag = totalRank + 90 * (totalRank - 1);
  while ((idxDiag > 0) &&
         (muDoubleScalarAbs(qrmanager->QR[idxDiag - 1]) < tol)) {
    idxDiag -= 91;
    nDepInd++;
  }
  if (nDepInd > 0) {
    idx_col = qrmanager->minRowCol;
    for (idx = 0; idx < idx_col; idx++) {
      totalRank = 90 * idx + idx;
      idxDiag = qrmanager->mrows - idx;
      if (0 <= idxDiag - 2) {
        memcpy(&qrmanager->Q[totalRank + 1], &qrmanager->QR[totalRank + 1],
               (((idxDiag + totalRank) - totalRank) - 1) * sizeof(real_T));
      }
    }
    info_t = LAPACKE_dorgqr(102, (ptrdiff_t)qrmanager->mrows,
                            (ptrdiff_t)qrmanager->mrows,
                            (ptrdiff_t)qrmanager->minRowCol, &qrmanager->Q[0],
                            (ptrdiff_t)90, &qrmanager->tau[0]);
    if ((int32_T)info_t != 0) {
      for (idx_col = 0; idx_col < 8100; idx_col++) {
        qrmanager->Q[idx_col] = rtNaN;
      }
    }
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx <= nDepInd - 1)) {
      totalRank = 90 * ((mWorkingFixed - idx) + 40);
      qtb = 0.0;
      for (k = 0; k <= mTotalWorkingEq; k++) {
        qtb += qrmanager->Q[totalRank + k] * workingset->bwset[k];
      }
      if (muDoubleScalarAbs(qtb) >= tol) {
        nDepInd = -1;
        exitg1 = true;
      } else {
        idx++;
      }
    }
  }
  if (nDepInd > 0) {
    for (idx_col = 0; idx_col <= mTotalWorkingEq; idx_col++) {
      totalRank = 90 * idx_col;
      idxDiag = 51 * idx_col;
      for (k = 0; k <= nVar; k++) {
        qrmanager->QR[totalRank + k] = workingset->ATwset[idxDiag + k];
      }
    }
    for (idx = 0; idx <= mWorkingFixed; idx++) {
      qrmanager->jpvt[idx] = 1;
    }
    idx_col = workingset->nWConstr[0] + 1;
    if (idx_col <= mTotalWorkingEq + 1) {
      memset(&qrmanager->jpvt[idx_col + -1], 0,
             ((mTotalWorkingEq - idx_col) + 2) * sizeof(int32_T));
    }
    factorQRE(qrmanager, workingset->nVar, workingset->nWConstr[0] + 40);
    for (idx = 0; idx < nDepInd; idx++) {
      memspace->workspace_int[idx] =
          qrmanager->jpvt[((mWorkingFixed - nDepInd) + idx) + 41];
    }
    countsort(memspace->workspace_int, nDepInd, memspace->workspace_sort, 1,
              workingset->nWConstr[0] + 40);
    for (idx = nDepInd; idx >= 1; idx--) {
      removeEqConstr(workingset, memspace->workspace_int[idx - 1]);
    }
  }
  if ((nDepInd != -1) && (workingset->nActiveConstr <= 90)) {
    RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace->workspace_double, solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = maxConstraintViolation(workingset, solution->xstar);
      if (tol > 1.0E-5) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    totalRank = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxDiag = workingset->nActiveConstr;
    for (idx_col = totalRank; idx_col <= idxDiag; idx_col++) {
      workingset->isActiveConstr
          [(workingset->isActiveIdx[workingset->Wid[idx_col - 1] - 1] +
            workingset->Wlocalidx[idx_col - 1]) -
           2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr =
        workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

void b_PresolveWorkingSet(struct_T *solution, e_struct_T *memspace,
                          g_struct_T *workingset, f_struct_T *qrmanager)
{
  real_T qtb;
  real_T tol;
  int32_T idxDiag;
  int32_T idx_col;
  int32_T k;
  int32_T mTotalWorkingEq_tmp_tmp;
  int32_T mWorkingFixed;
  int32_T nDepInd;
  int32_T nVar;
  int32_T totalRank;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T okWorkingSet;
  solution->state = 82;
  nVar = workingset->nVar - 1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    for (totalRank = 0; totalRank < mTotalWorkingEq_tmp_tmp; totalRank++) {
      for (idx_col = 0; idx_col <= nVar; idx_col++) {
        qrmanager->QR[totalRank + 90 * idx_col] =
            workingset->ATwset[idx_col + 51 * totalRank];
      }
    }
    totalRank = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    nDepInd = muIntScalarMax_sint32(0, totalRank);
    if (0 <= nVar) {
      memset(&qrmanager->jpvt[0], 0, (nVar + 1) * sizeof(int32_T));
    }
    factorQRE(qrmanager, mTotalWorkingEq_tmp_tmp, workingset->nVar);
    tol = 100.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
    totalRank =
        muIntScalarMin_sint32(workingset->nVar, mTotalWorkingEq_tmp_tmp);
    idxDiag = totalRank + 90 * (totalRank - 1);
    while ((idxDiag > 0) &&
           (muDoubleScalarAbs(qrmanager->QR[idxDiag - 1]) < tol)) {
      idxDiag -= 91;
      nDepInd++;
    }
    if (nDepInd > 0) {
      computeQ_(qrmanager, qrmanager->mrows);
      idxDiag = 0;
      exitg1 = false;
      while ((!exitg1) && (idxDiag <= nDepInd - 1)) {
        totalRank = 90 * ((mTotalWorkingEq_tmp_tmp - idxDiag) - 1);
        qtb = 0.0;
        for (k = 0; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager->Q[totalRank + k] * workingset->bwset[k];
        }
        if (muDoubleScalarAbs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idxDiag++;
        }
      }
    }
    if (nDepInd > 0) {
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        totalRank = 90 * idx_col;
        idxDiag = 51 * idx_col;
        for (k = 0; k <= nVar; k++) {
          qrmanager->QR[totalRank + k] = workingset->ATwset[idxDiag + k];
        }
      }
      for (idxDiag = 0; idxDiag < mWorkingFixed; idxDiag++) {
        qrmanager->jpvt[idxDiag] = 1;
      }
      totalRank = workingset->nWConstr[0] + 1;
      if (totalRank <= mTotalWorkingEq_tmp_tmp) {
        memset(&qrmanager->jpvt[totalRank + -1], 0,
               ((mTotalWorkingEq_tmp_tmp - totalRank) + 1) * sizeof(int32_T));
      }
      factorQRE(qrmanager, workingset->nVar, mTotalWorkingEq_tmp_tmp);
      for (idxDiag = 0; idxDiag < nDepInd; idxDiag++) {
        memspace->workspace_int[idxDiag] =
            qrmanager->jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idxDiag];
      }
      countsort(memspace->workspace_int, nDepInd, memspace->workspace_sort, 1,
                mTotalWorkingEq_tmp_tmp);
      for (idxDiag = nDepInd; idxDiag >= 1; idxDiag--) {
        removeEqConstr(workingset, memspace->workspace_int[idxDiag - 1]);
      }
    }
  }
  if ((nDepInd != -1) && (workingset->nActiveConstr <= 90)) {
    RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace->workspace_double, solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = maxConstraintViolation(workingset, solution->xstar);
      if (tol > 1.0E-5) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    totalRank = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxDiag = workingset->nActiveConstr;
    for (idx_col = totalRank; idx_col <= idxDiag; idx_col++) {
      workingset->isActiveConstr
          [(workingset->isActiveIdx[workingset->Wid[idx_col - 1] - 1] +
            workingset->Wlocalidx[idx_col - 1]) -
           2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr =
        workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

/* End of code generation (PresolveWorkingSet.c) */
