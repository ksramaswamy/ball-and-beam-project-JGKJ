/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * feasibleX0ForWorkingSet.c
 *
 * Code generation for function 'feasibleX0ForWorkingSet'
 *
 */

/* Include files */
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_mexutil.h"
#include "solveCFTOC_types.h"
#include "xgeqrf.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
boolean_T feasibleX0ForWorkingSet(real_T workspace[7191], real_T xCurrent[51],
                                  g_struct_T *workingset, f_struct_T *qrmanager)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T B[7191];
  real_T beta1;
  real_T c;
  real_T v;
  int32_T exitg1;
  int32_T ia;
  int32_T iac;
  int32_T idx;
  int32_T mFixed;
  int32_T mLB;
  int32_T mWConstr;
  int32_T nVar;
  int32_T offsetQR;
  char_T DIAGA1;
  char_T SIDE1;
  char_T TRANSA1;
  char_T UPLO1;
  boolean_T nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    for (idx = 0; idx < mWConstr; idx++) {
      workspace[idx] = workingset->bwset[idx];
      workspace[idx + 141] = workingset->bwset[idx];
    }
    if (mWConstr != 0) {
      idx = 51 * (mWConstr - 1) + 1;
      for (iac = 1; iac <= idx; iac += 51) {
        c = 0.0;
        offsetQR = (iac + nVar) - 1;
        for (ia = iac; ia <= offsetQR; ia++) {
          c += workingset->ATwset[ia - 1] * xCurrent[ia - iac];
        }
        offsetQR = div_nde_s32_floor(iac - 1);
        workspace[offsetQR] += -c;
      }
    }
    if (mWConstr >= nVar) {
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (idx = 0; idx < nVar; idx++) {
        offsetQR = 90 * idx;
        for (mLB = 0; mLB < mWConstr; mLB++) {
          qrmanager->QR[mLB + offsetQR] = workingset->ATwset[idx + 51 * mLB];
        }
        qrmanager->jpvt[idx] = idx + 1;
      }
      qrmanager->minRowCol = muIntScalarMin_sint32(mWConstr, nVar);
      xgeqrf(qrmanager->QR, mWConstr, nVar, qrmanager->tau);
      computeQ_(qrmanager, mWConstr);
      c = 1.0;
      memcpy(&B[0], &workspace[0], 7191U * sizeof(real_T));
      beta1 = 0.0;
      DIAGA1 = 'N';
      TRANSA1 = 'T';
      m_t = (ptrdiff_t)nVar;
      n_t = (ptrdiff_t)2;
      k_t = (ptrdiff_t)mWConstr;
      lda_t = (ptrdiff_t)90;
      ldb_t = (ptrdiff_t)141;
      ldc_t = (ptrdiff_t)141;
      dgemm(&TRANSA1, &DIAGA1, &m_t, &n_t, &k_t, &c, &qrmanager->Q[0], &lda_t,
            &B[0], &ldb_t, &beta1, &workspace[0], &ldc_t);
      c = 1.0;
      DIAGA1 = 'N';
      TRANSA1 = 'N';
      UPLO1 = 'U';
      SIDE1 = 'L';
      m_t = (ptrdiff_t)nVar;
      n_t = (ptrdiff_t)2;
      lda_t = (ptrdiff_t)90;
      ldb_t = (ptrdiff_t)141;
      dtrsm(&SIDE1, &UPLO1, &TRANSA1, &DIAGA1, &m_t, &n_t, &c,
            &qrmanager->QR[0], &lda_t, &workspace[0], &ldb_t);
    } else {
      factorQR(qrmanager, workingset->ATwset, nVar, mWConstr);
      computeQ_(qrmanager, qrmanager->minRowCol);
      if (mWConstr >= 1) {
        c = 1.0;
        DIAGA1 = 'N';
        TRANSA1 = 'T';
        UPLO1 = 'U';
        SIDE1 = 'L';
        m_t = (ptrdiff_t)mWConstr;
        n_t = (ptrdiff_t)2;
        lda_t = (ptrdiff_t)90;
        ldb_t = (ptrdiff_t)141;
        dtrsm(&SIDE1, &UPLO1, &TRANSA1, &DIAGA1, &m_t, &n_t, &c,
              &qrmanager->QR[0], &lda_t, &workspace[0], &ldb_t);
        c = 1.0;
        memcpy(&B[0], &workspace[0], 7191U * sizeof(real_T));
        beta1 = 0.0;
        DIAGA1 = 'N';
        TRANSA1 = 'N';
        m_t = (ptrdiff_t)nVar;
        n_t = (ptrdiff_t)2;
        k_t = (ptrdiff_t)mWConstr;
        lda_t = (ptrdiff_t)90;
        ldb_t = (ptrdiff_t)141;
        ldc_t = (ptrdiff_t)141;
        dgemm(&TRANSA1, &DIAGA1, &m_t, &n_t, &k_t, &c, &qrmanager->Q[0], &lda_t,
              &B[0], &ldb_t, &beta1, &workspace[0], &ldc_t);
      }
    }
    idx = 0;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        if (muDoubleScalarIsInf(workspace[idx]) ||
            muDoubleScalarIsNaN(workspace[idx])) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[idx + 141];
          if (muDoubleScalarIsInf(c) || muDoubleScalarIsNaN(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        c = 1.0;
        n_t = (ptrdiff_t)nVar;
        m_t = (ptrdiff_t)1;
        lda_t = (ptrdiff_t)1;
        daxpy(&n_t, &c, &xCurrent[0], &m_t, &workspace[0], &lda_t);
        mLB = workingset->sizes[3];
        mWConstr = workingset->sizes[4];
        mFixed = workingset->sizes[0];
        if (workingset->probType == 2) {
          beta1 = 0.0;
          for (offsetQR = 0; offsetQR < 40; offsetQR++) {
            workingset->maxConstrWorkspace[offsetQR] =
                workingset->beq[offsetQR];
            workingset->maxConstrWorkspace[offsetQR] =
                -workingset->maxConstrWorkspace[offsetQR];
          }
          for (iac = 0; iac <= 1989; iac += 51) {
            c = 0.0;
            idx = iac + 50;
            for (ia = iac + 1; ia <= idx; ia++) {
              c += workingset->Aeq[ia - 1] * workspace[(ia - iac) - 1];
            }
            idx = div_nde_s32_floor(iac);
            workingset->maxConstrWorkspace[idx] += c;
          }
          for (idx = 0; idx < 40; idx++) {
            workingset->maxConstrWorkspace[idx] =
                (workingset->maxConstrWorkspace[idx] - workspace[idx + 50]) +
                workspace[idx + 90];
            beta1 = muDoubleScalarMax(
                beta1, muDoubleScalarAbs(workingset->maxConstrWorkspace[idx]));
          }
        } else {
          beta1 = 0.0;
          for (offsetQR = 0; offsetQR < 40; offsetQR++) {
            workingset->maxConstrWorkspace[offsetQR] =
                workingset->beq[offsetQR];
            workingset->maxConstrWorkspace[offsetQR] =
                -workingset->maxConstrWorkspace[offsetQR];
          }
          for (iac = 0; iac <= 1989; iac += 51) {
            c = 0.0;
            idx = iac + workingset->nVar;
            for (ia = iac + 1; ia <= idx; ia++) {
              c += workingset->Aeq[ia - 1] * workspace[(ia - iac) - 1];
            }
            idx = div_nde_s32_floor(iac);
            workingset->maxConstrWorkspace[idx] += c;
          }
          for (idx = 0; idx < 40; idx++) {
            beta1 = muDoubleScalarMax(
                beta1, muDoubleScalarAbs(workingset->maxConstrWorkspace[idx]));
          }
        }
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            offsetQR = workingset->indexLB[idx] - 1;
            beta1 = muDoubleScalarMax(beta1, -workspace[offsetQR] -
                                                 workingset->lb[offsetQR]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < mWConstr; idx++) {
            offsetQR = workingset->indexUB[idx] - 1;
            beta1 = muDoubleScalarMax(beta1, workspace[offsetQR] -
                                                 workingset->ub[offsetQR]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            beta1 = muDoubleScalarMax(
                beta1, muDoubleScalarAbs(
                           workspace[workingset->indexFixed[idx] - 1] -
                           workingset->ub[workingset->indexFixed[idx] - 1]));
          }
        }
        mLB = workingset->sizes[3];
        mWConstr = workingset->sizes[4];
        mFixed = workingset->sizes[0];
        if (workingset->probType == 2) {
          v = 0.0;
          for (offsetQR = 0; offsetQR < 40; offsetQR++) {
            workingset->maxConstrWorkspace[offsetQR] =
                workingset->beq[offsetQR];
            workingset->maxConstrWorkspace[offsetQR] =
                -workingset->maxConstrWorkspace[offsetQR];
          }
          for (iac = 0; iac <= 1989; iac += 51) {
            c = 0.0;
            idx = iac + 50;
            for (ia = iac + 1; ia <= idx; ia++) {
              c += workingset->Aeq[ia - 1] * workspace[(ia - iac) + 140];
            }
            idx = div_nde_s32_floor(iac);
            workingset->maxConstrWorkspace[idx] += c;
          }
          for (idx = 0; idx < 40; idx++) {
            workingset->maxConstrWorkspace[idx] =
                (workingset->maxConstrWorkspace[idx] - workspace[idx + 191]) +
                workspace[idx + 231];
            v = muDoubleScalarMax(
                v, muDoubleScalarAbs(workingset->maxConstrWorkspace[idx]));
          }
        } else {
          v = 0.0;
          for (offsetQR = 0; offsetQR < 40; offsetQR++) {
            workingset->maxConstrWorkspace[offsetQR] =
                workingset->beq[offsetQR];
            workingset->maxConstrWorkspace[offsetQR] =
                -workingset->maxConstrWorkspace[offsetQR];
          }
          for (iac = 0; iac <= 1989; iac += 51) {
            c = 0.0;
            idx = iac + workingset->nVar;
            for (ia = iac + 1; ia <= idx; ia++) {
              c += workingset->Aeq[ia - 1] * workspace[(ia - iac) + 140];
            }
            idx = div_nde_s32_floor(iac);
            workingset->maxConstrWorkspace[idx] += c;
          }
          for (idx = 0; idx < 40; idx++) {
            v = muDoubleScalarMax(
                v, muDoubleScalarAbs(workingset->maxConstrWorkspace[idx]));
          }
        }
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            offsetQR = workingset->indexLB[idx];
            v = muDoubleScalarMax(v, -workspace[offsetQR + 140] -
                                         workingset->lb[offsetQR - 1]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < mWConstr; idx++) {
            offsetQR = workingset->indexUB[idx];
            v = muDoubleScalarMax(v, workspace[offsetQR + 140] -
                                         workingset->ub[offsetQR - 1]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            v = muDoubleScalarMax(
                v, muDoubleScalarAbs(
                       workspace[workingset->indexFixed[idx] + 140] -
                       workingset->ub[workingset->indexFixed[idx] - 1]));
          }
        }
        if ((beta1 <= 2.2204460492503131E-16) || (beta1 < v)) {
          if (0 <= nVar - 1) {
            memcpy(&xCurrent[0], &workspace[0], nVar * sizeof(real_T));
          }
        } else if (0 <= nVar - 1) {
          memcpy(&xCurrent[0], &workspace[141], nVar * sizeof(real_T));
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

/* End of code generation (feasibleX0ForWorkingSet.c) */
