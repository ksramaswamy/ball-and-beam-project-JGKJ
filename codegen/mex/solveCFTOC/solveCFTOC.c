/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveCFTOC.c
 *
 * Code generation for function 'solveCFTOC'
 *
 */

/* Include files */
#include "solveCFTOC.h"
#include "driver.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void solveCFTOC(solveCFTOCStackData *SD, const real_T H[2500],
                const real_T f[50], const real_T A[2000], const real_T b[40],
                const real_T lb[50], const real_T ub[50], const real_T x0[50],
                real_T x[50])
{
  b_struct_T QPObjective;
  d_struct_T expl_temp;
  e_struct_T memspace;
  g_struct_T WorkingSet;
  struct_T solution;
  real_T H_infnrm;
  real_T colSum;
  real_T f_infnrm;
  real_T tol;
  int32_T b_i;
  int32_T i;
  int32_T iATw0;
  int32_T iAeq0_tmp;
  int32_T idxFillStart;
  int32_T idx_local;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  int32_T nWFixed;
  int8_T obj_tmp[5];
  int8_T i1;
  boolean_T guard1 = false;
  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  memset(&solution.lambda[0], 0, 141U * sizeof(real_T));
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  memset(&solution.searchDir[0], 0, 51U * sizeof(real_T));
  memcpy(&solution.xstar[0], &x0[0], 50U * sizeof(real_T));
  SD->f0.CholRegManager.ldm = 90;
  SD->f0.CholRegManager.ndims = 0;
  SD->f0.CholRegManager.info = 0;
  SD->f0.CholRegManager.ConvexCheck = true;
  SD->f0.CholRegManager.regTol_ = 0.0;
  SD->f0.CholRegManager.scaleFactor = 100.0;
  WorkingSet.nVar = 50;
  WorkingSet.nVarOrig = 50;
  WorkingSet.nVarMax = 51;
  WorkingSet.ldA = 51;
  memset(&WorkingSet.Aeq[0], 0, 2040U * sizeof(real_T));
  memset(&WorkingSet.beq[0], 0, 40U * sizeof(real_T));
  memset(&WorkingSet.lb[0], 0, 51U * sizeof(real_T));
  memset(&WorkingSet.ub[0], 0, 51U * sizeof(real_T));
  WorkingSet.mEqRemoved = 0;
  memset(&WorkingSet.indexEqRemoved[0], 0, 40U * sizeof(int32_T));
  memset(&WorkingSet.ATwset[0], 0, 7191U * sizeof(real_T));
  WorkingSet.nActiveConstr = 0;
  memset(&WorkingSet.bwset[0], 0, 141U * sizeof(real_T));
  memset(&WorkingSet.maxConstrWorkspace[0], 0, 141U * sizeof(real_T));
  memset(&WorkingSet.isActiveConstr[0], 0, 141U * sizeof(boolean_T));
  memset(&WorkingSet.Wid[0], 0, 141U * sizeof(int32_T));
  memset(&WorkingSet.Wlocalidx[0], 0, 141U * sizeof(int32_T));
  for (i = 0; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  memset(&WorkingSet.indexLB[0], 0, 51U * sizeof(int32_T));
  memset(&WorkingSet.indexUB[0], 0, 51U * sizeof(int32_T));
  memset(&WorkingSet.indexFixed[0], 0, 51U * sizeof(int32_T));
  mLB = 0;
  mUB = 0;
  mFixed = 0;
  for (i = 0; i < 50; i++) {
    tol = lb[i];
    guard1 = false;
    if ((!muDoubleScalarIsInf(tol)) && (!muDoubleScalarIsNaN(tol))) {
      if (muDoubleScalarAbs(tol - ub[i]) < 1.0E-5) {
        mFixed++;
        WorkingSet.indexFixed[mFixed - 1] = i + 1;
      } else {
        mLB++;
        WorkingSet.indexLB[mLB - 1] = i + 1;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      tol = ub[i];
      if ((!muDoubleScalarIsInf(tol)) && (!muDoubleScalarIsNaN(tol))) {
        mUB++;
        WorkingSet.indexUB[mUB - 1] = i + 1;
      }
    }
  }
  i = ((mLB + mUB) + mFixed) + 40;
  WorkingSet.mConstr = i;
  WorkingSet.mConstrOrig = i;
  WorkingSet.mConstrMax = 141;
  obj_tmp[0] = (int8_T)mFixed;
  obj_tmp[1] = 40;
  obj_tmp[2] = 0;
  obj_tmp[3] = (int8_T)mLB;
  obj_tmp[4] = (int8_T)mUB;
  WorkingSet.sizesPhaseOne[0] = mFixed;
  WorkingSet.sizesPhaseOne[1] = 40;
  WorkingSet.sizesPhaseOne[2] = 0;
  WorkingSet.sizesPhaseOne[3] = mLB + 1;
  WorkingSet.sizesPhaseOne[4] = mUB;
  WorkingSet.sizesRegularized[0] = mFixed;
  WorkingSet.sizesRegularized[1] = 40;
  WorkingSet.sizesRegularized[2] = 0;
  WorkingSet.sizesRegularized[3] = mLB + 80;
  WorkingSet.sizesRegularized[4] = mUB;
  WorkingSet.sizesRegPhaseOne[0] = mFixed;
  WorkingSet.sizesRegPhaseOne[1] = 40;
  WorkingSet.sizesRegPhaseOne[2] = 0;
  WorkingSet.sizesRegPhaseOne[3] = mLB + 81;
  WorkingSet.sizesRegPhaseOne[4] = mUB;
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 40;
  WorkingSet.isActiveIdxRegPhaseOne[3] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    i1 = obj_tmp[i];
    WorkingSet.sizes[i] = i1;
    WorkingSet.sizesNormal[i] = i1;
    WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  for (b_i = 0; b_i < 6; b_i++) {
    i = WorkingSet.isActiveIdxRegPhaseOne[b_i];
    WorkingSet.isActiveIdx[b_i] = i;
    WorkingSet.isActiveIdxNormal[b_i] = i;
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 40;
  WorkingSet.isActiveIdxRegPhaseOne[3] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB + 1;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  for (b_i = 0; b_i < 6; b_i++) {
    WorkingSet.isActiveIdxPhaseOne[b_i] =
        WorkingSet.isActiveIdxRegPhaseOne[b_i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 40;
  WorkingSet.isActiveIdxRegPhaseOne[3] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB + 80;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  for (b_i = 0; b_i < 6; b_i++) {
    WorkingSet.isActiveIdxRegularized[b_i] =
        WorkingSet.isActiveIdxRegPhaseOne[b_i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 40;
  WorkingSet.isActiveIdxRegPhaseOne[3] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB + 81;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  for (i = 0; i < 40; i++) {
    for (idxFillStart = 0; idxFillStart < 50; idxFillStart++) {
      WorkingSet.Aeq[idxFillStart + 51 * i] = A[i + 40 * idxFillStart];
    }
    WorkingSet.beq[i] = b[i];
  }
  for (i = 0; i < 50; i++) {
    WorkingSet.lb[i] = -lb[i];
    WorkingSet.ub[i] = ub[i];
  }
  setProblemType(&WorkingSet, 3);
  idxFillStart = WorkingSet.isActiveIdx[2];
  for (i = idxFillStart; i < 142; i++) {
    WorkingSet.isActiveConstr[i - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 40;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0] + 40;
  nWFixed = WorkingSet.sizes[0];
  for (idx_local = 0; idx_local < nWFixed; idx_local++) {
    WorkingSet.Wid[idx_local] = 1;
    WorkingSet.Wlocalidx[idx_local] = idx_local + 1;
    WorkingSet.isActiveConstr[idx_local] = true;
    idxFillStart = 51 * idx_local;
    b_i = WorkingSet.indexFixed[idx_local];
    if (0 <= b_i - 2) {
      memset(&WorkingSet.ATwset[idxFillStart], 0,
             (((b_i + idxFillStart) - idxFillStart) - 1) * sizeof(real_T));
    }
    WorkingSet.ATwset[(WorkingSet.indexFixed[idx_local] + idxFillStart) - 1] =
        1.0;
    b_i = WorkingSet.indexFixed[idx_local] + 1;
    i = WorkingSet.nVar;
    if (b_i <= i) {
      memset(&WorkingSet.ATwset[(b_i + idxFillStart) + -1], 0,
             ((((i + idxFillStart) - b_i) - idxFillStart) + 1) *
                 sizeof(real_T));
    }
    WorkingSet.bwset[idx_local] =
        WorkingSet.ub[WorkingSet.indexFixed[idx_local] - 1];
  }
  WorkingSet.SLACK0 = 0.0;
  tol = 1.0;
  b_i = WorkingSet.nVar - 1;
  for (idx_local = 0; idx_local < 40; idx_local++) {
    idxFillStart = nWFixed + idx_local;
    WorkingSet.Wid[idxFillStart] = 2;
    WorkingSet.Wlocalidx[idxFillStart] = idx_local + 1;
    WorkingSet.isActiveConstr[idxFillStart] = true;
    iAeq0_tmp = 51 * idx_local;
    iATw0 = 51 * idxFillStart;
    for (i = 0; i <= b_i; i++) {
      WorkingSet.ATwset[iATw0 + i] = WorkingSet.Aeq[iAeq0_tmp + i];
    }
    WorkingSet.bwset[idxFillStart] = WorkingSet.beq[idx_local];
    colSum = 0.0;
    for (idxFillStart = 0; idxFillStart < 50; idxFillStart++) {
      colSum += muDoubleScalarAbs(WorkingSet.Aeq[idxFillStart + iAeq0_tmp]);
    }
    tol = muDoubleScalarMax(tol, colSum);
  }
  H_infnrm = 0.0;
  f_infnrm = 0.0;
  for (i = 0; i < 50; i++) {
    colSum = 0.0;
    for (idxFillStart = 0; idxFillStart < 50; idxFillStart++) {
      colSum += muDoubleScalarAbs(H[idxFillStart + 50 * i]);
    }
    H_infnrm = muDoubleScalarMax(H_infnrm, colSum);
    f_infnrm = muDoubleScalarMax(f_infnrm, muDoubleScalarAbs(f[i]));
  }
  expl_temp.RemainFeasible = false;
  expl_temp.ProbRelTolFactor =
      muDoubleScalarMax(muDoubleScalarMax(tol, f_infnrm), H_infnrm);
  expl_temp.ConstrRelTolFactor = tol;
  expl_temp.MaxIterations = 10 * (((mFixed + mLB) + mUB) + 90);
  driver(H, f, &solution, &memspace, &WorkingSet, &SD->f0.CholRegManager,
         expl_temp, &SD->f0.QRManager, &QPObjective);
  memcpy(&x[0], &solution.xstar[0], 50U * sizeof(real_T));
}

/* End of code generation (solveCFTOC.c) */
