/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * iterate.c
 *
 * Code generation for function 'iterate'
 *
 */

/* Include files */
#include "iterate.h"
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "compute_lambda.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "ratiotest.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "squareQ_appendCol.h"
#include "xnrm2.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void iterate(const real_T H[2500], const real_T f[50], struct_T *solution,
             e_struct_T *memspace, g_struct_T *workingset,
             f_struct_T *qrmanager, c_struct_T *cholmanager,
             b_struct_T *objective, real_T options_ObjectiveLimit,
             real_T options_StepTolerance, int32_T runTimeOptions_MaxIterations,
             real_T c_runTimeOptions_ConstrRelTolFa,
             real_T runTimeOptions_ProbRelTolFactor,
             boolean_T runTimeOptions_RemainFeasible)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  real_T alpha;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T phaseOneCorrectionX;
  real_T pk_corrected;
  real_T ratio;
  real_T tolDelta;
  int32_T TYPE;
  int32_T activeConstrChangedType;
  int32_T activeSetChangeID;
  int32_T exitg1;
  int32_T globalActiveConstrIdx;
  int32_T idx;
  int32_T localActiveConstrIdx;
  int32_T nVar_tmp_tmp;
  int32_T phaseOneCorrectionX_tmp;
  int32_T totalUB;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T newBlocking;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar_tmp_tmp = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double,
                                        f, solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }
  memset(&solution->lambda[0], 0, 141U * sizeof(real_T));
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1:
          squareQ_appendCol(qrmanager, workingset->ATwset,
                            51 * (workingset->nActiveConstr - 1) + 1);
          break;
        case -1:
          deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;
        default:
          factorQR(qrmanager, workingset->ATwset, nVar_tmp_tmp,
                   workingset->nActiveConstr);
          computeQ_(qrmanager, qrmanager->mrows);
          break;
        }
        compute_deltax(H, solution, memspace, qrmanager, cholmanager,
                       objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((xnrm2(nVar_tmp_tmp, solution->searchDir) <
                    options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar_tmp_tmp)) {
          guard2 = true;
        } else {
          updateFval = (TYPE == 5);
          if (updateFval || runTimeOptions_RemainFeasible) {
            totalUB = workingset->sizes[4];
            alpha = 1.0E+30;
            newBlocking = false;
            activeConstrChangedType = 0;
            localActiveConstrIdx = 0;
            denomTol = 2.2204460492503131E-13 *
                       xnrm2(workingset->nVar, solution->searchDir);
            if (workingset->nWConstr[3] < workingset->sizes[3]) {
              phaseOneCorrectionX_tmp = workingset->nVar - 1;
              phaseOneCorrectionX =
                  (real_T)updateFval * solution->xstar[phaseOneCorrectionX_tmp];
              phaseOneCorrectionP =
                  (real_T)updateFval *
                  solution->searchDir[phaseOneCorrectionX_tmp];
              phaseOneCorrectionX_tmp = workingset->sizes[3];
              for (idx = 0; idx <= phaseOneCorrectionX_tmp - 2; idx++) {
                pk_corrected =
                    -solution->searchDir[workingset->indexLB[idx] - 1] -
                    phaseOneCorrectionP;
                if ((pk_corrected > denomTol) &&
                    (!workingset
                          ->isActiveConstr[(workingset->isActiveIdx[3] + idx) -
                                           1])) {
                  ratio = (-solution->xstar[workingset->indexLB[idx] - 1] -
                           workingset->lb[workingset->indexLB[idx] - 1]) -
                          phaseOneCorrectionX;
                  pk_corrected = muDoubleScalarMin(muDoubleScalarAbs(ratio),
                                                   1.0E-5 - ratio) /
                                 pk_corrected;
                  if (pk_corrected < alpha) {
                    alpha = pk_corrected;
                    activeConstrChangedType = 4;
                    localActiveConstrIdx = idx + 1;
                    newBlocking = true;
                  }
                }
              }
              phaseOneCorrectionX_tmp =
                  workingset->indexLB[workingset->sizes[3] - 1] - 1;
              phaseOneCorrectionX =
                  -solution->searchDir[phaseOneCorrectionX_tmp];
              if ((phaseOneCorrectionX > denomTol) &&
                  (!workingset->isActiveConstr[(workingset->isActiveIdx[3] +
                                                workingset->sizes[3]) -
                                               2])) {
                ratio = -solution->xstar[phaseOneCorrectionX_tmp] -
                        workingset->lb[phaseOneCorrectionX_tmp];
                pk_corrected = muDoubleScalarMin(muDoubleScalarAbs(ratio),
                                                 1.0E-5 - ratio) /
                               phaseOneCorrectionX;
                if (pk_corrected < alpha) {
                  alpha = pk_corrected;
                  activeConstrChangedType = 4;
                  localActiveConstrIdx = workingset->sizes[3];
                  newBlocking = true;
                }
              }
            }
            if (workingset->nWConstr[4] < workingset->sizes[4]) {
              phaseOneCorrectionX_tmp = workingset->nVar - 1;
              phaseOneCorrectionX =
                  (real_T)updateFval * solution->xstar[phaseOneCorrectionX_tmp];
              phaseOneCorrectionP =
                  (real_T)updateFval *
                  solution->searchDir[phaseOneCorrectionX_tmp];
              for (idx = 0; idx < totalUB; idx++) {
                pk_corrected =
                    solution->searchDir[workingset->indexUB[idx] - 1] -
                    phaseOneCorrectionP;
                if ((pk_corrected > denomTol) &&
                    (!workingset
                          ->isActiveConstr[(workingset->isActiveIdx[4] + idx) -
                                           1])) {
                  ratio = (solution->xstar[workingset->indexUB[idx] - 1] -
                           workingset->ub[workingset->indexUB[idx] - 1]) -
                          phaseOneCorrectionX;
                  pk_corrected = muDoubleScalarMin(muDoubleScalarAbs(ratio),
                                                   1.0E-5 - ratio) /
                                 pk_corrected;
                  if (pk_corrected < alpha) {
                    alpha = pk_corrected;
                    activeConstrChangedType = 5;
                    localActiveConstrIdx = idx + 1;
                    newBlocking = true;
                  }
                }
              }
            }
            if (!updateFval) {
              if (newBlocking && (alpha > 1.0)) {
                newBlocking = false;
              }
              alpha = muDoubleScalarMin(alpha, 1.0);
            }
          } else {
            ratiotest(solution->xstar, solution->searchDir, workingset->nVar,
                      workingset->lb, workingset->ub, workingset->indexLB,
                      workingset->indexUB, workingset->sizes,
                      workingset->isActiveIdx, workingset->isActiveConstr,
                      workingset->nWConstr, &tolDelta, &alpha, &newBlocking,
                      &activeConstrChangedType, &localActiveConstrIdx);
          }
          if (newBlocking) {
            switch (activeConstrChangedType) {
            case 3:
              workingset->nWConstr[2]++;
              workingset->isActiveConstr[(workingset->isActiveIdx[2] +
                                          localActiveConstrIdx) -
                                         2] = true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] =
                  localActiveConstrIdx;
              /* A check that is always false is detected at compile-time.
               * Eliminating code that follows. */
              break;
            case 4:
              addBoundToActiveSetMatrix_(workingset, 4, localActiveConstrIdx);
              break;
            default:
              addBoundToActiveSetMatrix_(workingset, 5, localActiveConstrIdx);
              break;
            }
            activeSetChangeID = 1;
          } else {
            if (objective->objtype == 5) {
              if (xnrm2(objective->nvar, solution->searchDir) >
                  100.0 * (real_T)objective->nvar * 1.4901161193847656E-8) {
                solution->state = 3;
              } else {
                solution->state = 4;
              }
            }
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }
          n_t = (ptrdiff_t)nVar_tmp_tmp;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          daxpy(&n_t, &alpha, &solution->searchDir[0], &incx_t,
                &solution->xstar[0], &incy_t);
          computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
          guard1 = true;
        }
      } else {
        if (0 <= nVar_tmp_tmp - 1) {
          memset(&solution->searchDir[0], 0, nVar_tmp_tmp * sizeof(real_T));
        }
        guard2 = true;
      }
      if (guard2) {
        compute_lambda(memspace->workspace_double, solution, objective,
                       qrmanager);
        if ((solution->state != -7) ||
            (workingset->nActiveConstr > nVar_tmp_tmp)) {
          localActiveConstrIdx = -1;
          pk_corrected =
              0.0 * runTimeOptions_ProbRelTolFactor * (real_T)(TYPE != 5);
          phaseOneCorrectionX_tmp =
              (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          activeConstrChangedType = workingset->nActiveConstr;
          for (idx = phaseOneCorrectionX_tmp; idx <= activeConstrChangedType;
               idx++) {
            phaseOneCorrectionX = solution->lambda[idx - 1];
            if (phaseOneCorrectionX < pk_corrected) {
              pk_corrected = phaseOneCorrectionX;
              localActiveConstrIdx = idx - 1;
            }
          }
          if (localActiveConstrIdx + 1 == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = localActiveConstrIdx + 1;
            subProblemChanged = true;
            activeConstrChangedType = workingset->Wid[localActiveConstrIdx] - 1;
            workingset->isActiveConstr
                [(workingset
                      ->isActiveIdx[workingset->Wid[localActiveConstrIdx] - 1] +
                  workingset->Wlocalidx[localActiveConstrIdx]) -
                 2] = false;
            workingset->Wid[localActiveConstrIdx] =
                workingset->Wid[workingset->nActiveConstr - 1];
            workingset->Wlocalidx[localActiveConstrIdx] =
                workingset->Wlocalidx[workingset->nActiveConstr - 1];
            phaseOneCorrectionX_tmp = workingset->nVar;
            for (idx = 0; idx < phaseOneCorrectionX_tmp; idx++) {
              workingset->ATwset[idx + 51 * localActiveConstrIdx] =
                  workingset
                      ->ATwset[idx + 51 * (workingset->nActiveConstr - 1)];
            }
            workingset->bwset[localActiveConstrIdx] =
                workingset->bwset[workingset->nActiveConstr - 1];
            workingset->nActiveConstr--;
            workingset->nWConstr[activeConstrChangedType]--;
            solution->lambda[localActiveConstrIdx] = 0.0;
          }
        } else {
          localActiveConstrIdx = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          phaseOneCorrectionX_tmp = workingset->nActiveConstr - 1;
          activeConstrChangedType =
              workingset->Wid[phaseOneCorrectionX_tmp] - 1;
          workingset->isActiveConstr
              [(workingset->isActiveIdx[activeConstrChangedType] +
                workingset->Wlocalidx[phaseOneCorrectionX_tmp]) -
               2] = false;
          workingset->nActiveConstr--;
          workingset->nWConstr[activeConstrChangedType]--;
          solution->lambda[localActiveConstrIdx - 1] = 0.0;
        }
        updateFval = false;
        guard1 = true;
      }
      if (guard1) {
        solution->iterations++;
        phaseOneCorrectionX_tmp = objective->nvar - 1;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }
        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr =
              maxConstraintViolation(workingset, solution->xstar);
          pk_corrected = solution->maxConstr;
          if (objective->objtype == 5) {
            pk_corrected =
                solution->maxConstr - solution->xstar[phaseOneCorrectionX_tmp];
          }
          if (pk_corrected > 1.0E-5 * c_runTimeOptions_ConstrRelTolFa) {
            if (0 <= phaseOneCorrectionX_tmp) {
              memcpy(&solution->searchDir[0], &solution->xstar[0],
                     (phaseOneCorrectionX_tmp + 1) * sizeof(real_T));
            }
            newBlocking = feasibleX0ForWorkingSet(memspace->workspace_double,
                                                  solution->searchDir,
                                                  workingset, qrmanager);
            if ((!newBlocking) && (solution->state != 0)) {
              solution->state = -2;
            }
            activeSetChangeID = 0;
            pk_corrected =
                maxConstraintViolation(workingset, solution->searchDir);
            if (pk_corrected < solution->maxConstr) {
              if (0 <= phaseOneCorrectionX_tmp) {
                memcpy(&solution->xstar[0], &solution->searchDir[0],
                       (phaseOneCorrectionX_tmp + 1) * sizeof(real_T));
              }
              solution->maxConstr = pk_corrected;
            }
          }
        }
        if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
          solution->fstar = computeFval_ReuseHx(
              objective, memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) &&
              ((solution->state != 0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = computeFval_ReuseHx(
            objective, memspace->workspace_double, f, solution->xstar);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* End of code generation (iterate.c) */
