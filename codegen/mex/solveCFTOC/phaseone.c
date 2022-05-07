/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * phaseone.c
 *
 * Code generation for function 'phaseone'
 *
 */

/* Include files */
#include "phaseone.h"
#include "computeFval.h"
#include "iterate.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "solveCFTOC_data.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include <string.h>

/* Function Definitions */
void b_phaseone(const real_T H[2500], const real_T f[50], struct_T *solution,
                e_struct_T *memspace, g_struct_T *workingset,
                f_struct_T *qrmanager, c_struct_T *cholmanager,
                b_struct_T *objective, h_struct_T *options,
                const d_struct_T *runTimeOptions)
{
  int32_T PROBTYPE_ORIG;
  int32_T idx;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idx_global;
  int32_T nVar_tmp_tmp;
  boolean_T exitg1;
  PROBTYPE_ORIG = workingset->probType;
  nVar_tmp_tmp = workingset->nVar;
  solution->xstar[50] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    idxEndIneq = 1;
  } else {
    idxEndIneq = 4;
  }
  setProblemType(workingset, idxEndIneq);
  idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
          workingset->Wlocalidx[idx_global - 1]) -
         2] = false;
  }
  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = 51;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-5 * runTimeOptions->ConstrRelTolFactor;
  options->StepTolerance = 1.4901161193847657E-10;
  solution->fstar =
      computeFval(objective, memspace->workspace_double, H, f, solution->xstar);
  solution->state = 5;
  iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
          objective, options->ObjectiveLimit, options->StepTolerance,
          runTimeOptions->MaxIterations, runTimeOptions->ConstrRelTolFactor,
          runTimeOptions->ProbRelTolFactor, runTimeOptions->RemainFeasible);
  if (workingset
          ->isActiveConstr[(workingset->isActiveIdx[3] + workingset->sizes[3]) -
                           2]) {
    idx = workingset->sizes[0] + 40;
    exitg1 = false;
    while ((!exitg1) && (idx + 1 <= workingset->nActiveConstr)) {
      if ((workingset->Wid[idx] == 4) &&
          (workingset->Wlocalidx[idx] == workingset->sizes[3])) {
        idxEndIneq = workingset->Wid[idx] - 1;
        workingset->isActiveConstr
            [(workingset->isActiveIdx[workingset->Wid[idx] - 1] +
              workingset->Wlocalidx[idx]) -
             2] = false;
        workingset->Wid[idx] = workingset->Wid[workingset->nActiveConstr - 1];
        workingset->Wlocalidx[idx] =
            workingset->Wlocalidx[workingset->nActiveConstr - 1];
        idx_global = workingset->nVar;
        for (idxStartIneq = 0; idxStartIneq < idx_global; idxStartIneq++) {
          workingset->ATwset[idxStartIneq + 51 * idx] =
              workingset
                  ->ATwset[idxStartIneq + 51 * (workingset->nActiveConstr - 1)];
        }
        workingset->bwset[idx] =
            workingset->bwset[workingset->nActiveConstr - 1];
        workingset->nActiveConstr--;
        workingset->nWConstr[idxEndIneq]--;
        exitg1 = true;
      } else {
        idx++;
      }
    }
  }
  idxStartIneq = workingset->nActiveConstr - 1;
  while ((idxStartIneq + 1 > workingset->sizes[0] + 40) &&
         (idxStartIneq + 1 > nVar_tmp_tmp)) {
    idxEndIneq = workingset->Wid[idxStartIneq] - 1;
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[idxStartIneq] - 1] +
          workingset->Wlocalidx[idxStartIneq]) -
         2] = false;
    workingset->Wid[idxStartIneq] =
        workingset->Wid[workingset->nActiveConstr - 1];
    workingset->Wlocalidx[idxStartIneq] =
        workingset->Wlocalidx[workingset->nActiveConstr - 1];
    idx_global = workingset->nVar;
    for (idx = 0; idx < idx_global; idx++) {
      workingset->ATwset[idx + 51 * idxStartIneq] =
          workingset->ATwset[idx + 51 * (workingset->nActiveConstr - 1)];
    }
    workingset->bwset[idxStartIneq] =
        workingset->bwset[workingset->nActiveConstr - 1];
    workingset->nActiveConstr--;
    workingset->nWConstr[idxEndIneq]--;
    idxStartIneq--;
  }
  solution->maxConstr = solution->xstar[50];
  setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 1.0E-5;
}

void phaseone(const real_T H[2500], const real_T f[50], struct_T *solution,
              e_struct_T *memspace, g_struct_T *workingset,
              f_struct_T *qrmanager, c_struct_T *cholmanager,
              const d_struct_T *runTimeOptions, b_struct_T *objective,
              h_struct_T *options)
{
  static const char_T t1_SolverName[8] = {'q', 'u', 'a', 'd',
                                          'p', 'r', 'o', 'g'};
  static const char_T t1_FiniteDifferenceType[7] = {'f', 'o', 'r', 'w',
                                                    'a', 'r', 'd'};
  int32_T idx;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idx_global;
  int32_T nVar_tmp_tmp;
  boolean_T exitg1;
  options->InitDamping = 0.01;
  for (idx_global = 0; idx_global < 7; idx_global++) {
    options->FiniteDifferenceType[idx_global] =
        t1_FiniteDifferenceType[idx_global];
  }
  options->SpecifyObjectiveGradient = false;
  options->ScaleProblem = false;
  options->SpecifyConstraintGradient = false;
  options->NonFiniteSupport = true;
  options->IterDisplaySQP = false;
  options->FiniteDifferenceStepSize = -1.0;
  options->MaxFunctionEvaluations = -1.0;
  options->IterDisplayQP = false;
  options->PricingTolerance = 0.0;
  for (idx_global = 0; idx_global < 10; idx_global++) {
    options->Algorithm[idx_global] = cv[idx_global];
  }
  options->ConstraintTolerance = 1.0E-5;
  options->OptimalityTolerance = 1.0E-5;
  options->MaxIterations = -1.0;
  options->FunctionTolerance = rtInf;
  for (idx_global = 0; idx_global < 8; idx_global++) {
    options->SolverName[idx_global] = t1_SolverName[idx_global];
  }
  options->CheckGradients = false;
  options->DiffMaxChange = rtInf;
  options->DiffMinChange = 0.0;
  options->Diagnostics[0] = 'o';
  options->Display[0] = 'o';
  options->FunValCheck[0] = 'o';
  options->Diagnostics[1] = 'f';
  options->Display[1] = 'f';
  options->FunValCheck[1] = 'f';
  options->Diagnostics[2] = 'f';
  options->Display[2] = 'f';
  options->FunValCheck[2] = 'f';
  options->UseParallel = false;
  options->LinearSolver[0] = 'a';
  options->LinearSolver[1] = 'u';
  options->LinearSolver[2] = 't';
  options->LinearSolver[3] = 'o';
  options->SubproblemAlgorithm[0] = 'c';
  options->SubproblemAlgorithm[1] = 'g';
  nVar_tmp_tmp = workingset->nVar;
  solution->xstar[50] = solution->maxConstr + 1.0;
  setProblemType(workingset, 1);
  idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
          workingset->Wlocalidx[idx_global - 1]) -
         2] = false;
  }
  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  memset(&objective->grad[0], 0, 51U * sizeof(real_T));
  memset(&objective->Hx[0], 0, 50U * sizeof(real_T));
  objective->maxVar = 51;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->prev_objtype = 3;
  objective->prev_nvar = 50;
  objective->prev_hasLinear = true;
  objective->objtype = 5;
  objective->nvar = 51;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-5 * runTimeOptions->ConstrRelTolFactor;
  solution->fstar =
      computeFval(objective, memspace->workspace_double, H, f, solution->xstar);
  solution->state = 5;
  iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
          objective, options->ObjectiveLimit, 1.4901161193847657E-10,
          runTimeOptions->MaxIterations, runTimeOptions->ConstrRelTolFactor,
          runTimeOptions->ProbRelTolFactor, runTimeOptions->RemainFeasible);
  if (workingset
          ->isActiveConstr[(workingset->isActiveIdx[3] + workingset->sizes[3]) -
                           2]) {
    idx = workingset->sizes[0] + 40;
    exitg1 = false;
    while ((!exitg1) && (idx + 1 <= workingset->nActiveConstr)) {
      if ((workingset->Wid[idx] == 4) &&
          (workingset->Wlocalidx[idx] == workingset->sizes[3])) {
        idxEndIneq = workingset->Wid[idx] - 1;
        workingset->isActiveConstr
            [(workingset->isActiveIdx[workingset->Wid[idx] - 1] +
              workingset->Wlocalidx[idx]) -
             2] = false;
        workingset->Wid[idx] = workingset->Wid[workingset->nActiveConstr - 1];
        workingset->Wlocalidx[idx] =
            workingset->Wlocalidx[workingset->nActiveConstr - 1];
        idx_global = workingset->nVar;
        for (idxStartIneq = 0; idxStartIneq < idx_global; idxStartIneq++) {
          workingset->ATwset[idxStartIneq + 51 * idx] =
              workingset
                  ->ATwset[idxStartIneq + 51 * (workingset->nActiveConstr - 1)];
        }
        workingset->bwset[idx] =
            workingset->bwset[workingset->nActiveConstr - 1];
        workingset->nActiveConstr--;
        workingset->nWConstr[idxEndIneq]--;
        exitg1 = true;
      } else {
        idx++;
      }
    }
  }
  idxStartIneq = workingset->nActiveConstr - 1;
  while ((idxStartIneq + 1 > workingset->sizes[0] + 40) &&
         (idxStartIneq + 1 > nVar_tmp_tmp)) {
    idxEndIneq = workingset->Wid[idxStartIneq] - 1;
    workingset->isActiveConstr
        [(workingset->isActiveIdx[workingset->Wid[idxStartIneq] - 1] +
          workingset->Wlocalidx[idxStartIneq]) -
         2] = false;
    workingset->Wid[idxStartIneq] =
        workingset->Wid[workingset->nActiveConstr - 1];
    workingset->Wlocalidx[idxStartIneq] =
        workingset->Wlocalidx[workingset->nActiveConstr - 1];
    idx_global = workingset->nVar;
    for (idx = 0; idx < idx_global; idx++) {
      workingset->ATwset[idx + 51 * idxStartIneq] =
          workingset->ATwset[idx + 51 * (workingset->nActiveConstr - 1)];
    }
    workingset->bwset[idxStartIneq] =
        workingset->bwset[workingset->nActiveConstr - 1];
    workingset->nActiveConstr--;
    workingset->nWConstr[idxEndIneq]--;
    idxStartIneq--;
  }
  solution->maxConstr = solution->xstar[50];
  setProblemType(workingset, 3);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 1.0E-5;
}

/* End of code generation (phaseone.c) */
