/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * driver.c
 *
 * Code generation for function 'driver'
 *
 */

/* Include files */
#include "driver.h"
#include "PresolveWorkingSet.h"
#include "computeFirstOrderOpt.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "phaseone.h"
#include "rt_nonfinite.h"
#include "solveCFTOC_data.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include <string.h>

/* Function Definitions */
void driver(const real_T H[2500], const real_T f[50], struct_T *solution,
            e_struct_T *memspace, g_struct_T *workingset,
            c_struct_T *cholmanager, d_struct_T runTimeOptions,
            f_struct_T *qrmanager, b_struct_T *objective)
{
  static const char_T b_cv[128] = {
      '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\x07', '\x08',
      '	',    '\x0a', '\x0b', '\x0c', '\x0d', '\x0e', '\x0f', '\x10', '\x11',
      '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
      '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
      '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
      '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
      '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
      '?',    '@',    'a',    'b',    'c',    'd',    'e',    'f',    'g',
      'h',    'i',    'j',    'k',    'l',    'm',    'n',    'o',    'p',
      'q',    'r',    's',    't',    'u',    'v',    'w',    'x',    'y',
      'z',    '[',    '\\',   ']',    '^',    '_',    '`',    'a',    'b',
      'c',    'd',    'e',    'f',    'g',    'h',    'i',    'j',    'k',
      'l',    'm',    'n',    'o',    'p',    'q',    'r',    's',    't',
      'u',    'v',    'w',    'x',    'y',    'z',    '{',    '|',    '}',
      '~',    '\x7f'};
  static const char_T cv1[8] = {'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g'};
  static const char_T t3_SolverName[8] = {'q', 'u', 'a', 'd',
                                          'p', 'r', 'o', 'g'};
  static const char_T t3_FiniteDifferenceType[7] = {'f', 'o', 'r', 'w',
                                                    'a', 'r', 'd'};
  h_struct_T options;
  real_T maxConstr_new;
  int32_T exitg1;
  int32_T i;
  int32_T idx;
  int32_T nVar;
  boolean_T b_bool;
  boolean_T guard1 = false;
  memset(&objective->grad[0], 0, 51U * sizeof(real_T));
  memset(&objective->Hx[0], 0, 50U * sizeof(real_T));
  objective->hasLinear = true;
  objective->nvar = 50;
  objective->maxVar = 51;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->objtype = 3;
  objective->prev_objtype = 3;
  objective->prev_nvar = 0;
  objective->prev_hasLinear = false;
  objective->gammaScalar = 0.0;
  solution->iterations = 0;
  runTimeOptions.RemainFeasible = true;
  nVar = workingset->nVar - 1;
  i = workingset->sizes[0];
  for (idx = 0; idx < i; idx++) {
    solution->xstar[workingset->indexFixed[idx] - 1] =
        workingset->ub[workingset->indexFixed[idx] - 1];
  }
  i = workingset->sizes[3];
  for (idx = 0; idx < i; idx++) {
    if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + idx) - 1]) {
      solution->xstar[workingset->indexLB[idx] - 1] =
          -workingset->lb[workingset->indexLB[idx] - 1];
    }
  }
  i = workingset->sizes[4];
  for (idx = 0; idx < i; idx++) {
    if (workingset->isActiveConstr[(workingset->isActiveIdx[4] + idx) - 1]) {
      solution->xstar[workingset->indexUB[idx] - 1] =
          workingset->ub[workingset->indexUB[idx] - 1];
    }
  }
  PresolveWorkingSet(solution, memspace, workingset, qrmanager);
  options.InitDamping = 0.01;
  for (i = 0; i < 7; i++) {
    options.FiniteDifferenceType[i] = t3_FiniteDifferenceType[i];
  }
  options.SpecifyObjectiveGradient = false;
  options.ScaleProblem = false;
  options.SpecifyConstraintGradient = false;
  options.NonFiniteSupport = true;
  options.IterDisplaySQP = false;
  options.FiniteDifferenceStepSize = -1.0;
  options.MaxFunctionEvaluations = -1.0;
  options.IterDisplayQP = false;
  options.PricingTolerance = 0.0;
  for (i = 0; i < 10; i++) {
    options.Algorithm[i] = cv[i];
  }
  options.ObjectiveLimit = -1.0E+20;
  options.ConstraintTolerance = 1.0E-5;
  options.OptimalityTolerance = 1.0E-5;
  options.StepTolerance = 1.0E-5;
  options.MaxIterations = -1.0;
  options.FunctionTolerance = rtInf;
  for (i = 0; i < 8; i++) {
    options.SolverName[i] = t3_SolverName[i];
  }
  options.CheckGradients = false;
  options.DiffMaxChange = rtInf;
  options.DiffMinChange = 0.0;
  options.Diagnostics[0] = 'o';
  options.Display[0] = 'o';
  options.FunValCheck[0] = 'o';
  options.Diagnostics[1] = 'f';
  options.Display[1] = 'f';
  options.FunValCheck[1] = 'f';
  options.Diagnostics[2] = 'f';
  options.Display[2] = 'f';
  options.FunValCheck[2] = 'f';
  options.UseParallel = false;
  options.LinearSolver[0] = 'a';
  options.LinearSolver[1] = 'u';
  options.LinearSolver[2] = 't';
  options.LinearSolver[3] = 'o';
  options.SubproblemAlgorithm[0] = 'c';
  options.SubproblemAlgorithm[1] = 'g';
  if (solution->state >= 0) {
    solution->iterations = 0;
    solution->maxConstr = maxConstraintViolation(workingset, solution->xstar);
    maxConstr_new = 1.0E-5 * runTimeOptions.ConstrRelTolFactor;
    guard1 = false;
    if (solution->maxConstr > maxConstr_new) {
      phaseone(H, f, solution, memspace, workingset, qrmanager, cholmanager,
               &runTimeOptions, objective, &options);
      if (solution->state != 0) {
        solution->maxConstr =
            maxConstraintViolation(workingset, solution->xstar);
        if (solution->maxConstr > maxConstr_new) {
          memset(&solution->lambda[0], 0, 141U * sizeof(real_T));
          solution->fstar = computeFval(objective, memspace->workspace_double,
                                        H, f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            if (0 <= nVar) {
              memcpy(&solution->searchDir[0], &solution->xstar[0],
                     (nVar + 1) * sizeof(real_T));
            }
            b_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = maxConstraintViolation(workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              if (0 <= nVar) {
                memcpy(&solution->xstar[0], &solution->searchDir[0],
                       (nVar + 1) * sizeof(real_T));
              }
            }
          }
          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.ObjectiveLimit, options.StepTolerance,
              runTimeOptions.MaxIterations, runTimeOptions.ConstrRelTolFactor,
              runTimeOptions.ProbRelTolFactor, true);
      b_bool = false;
      nVar = 0;
      do {
        exitg1 = 0;
        if (nVar < 8) {
          if (b_cv[(uint8_T)options.SolverName[nVar]] !=
              b_cv[(int32_T)cv1[nVar]]) {
            exitg1 = 1;
          } else {
            nVar++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if (b_bool && (solution->state != -6)) {
        solution->maxConstr =
            maxConstraintViolation(workingset, solution->xstar);
        computeFirstOrderOpt(solution, objective, workingset->nVar,
                             workingset->ATwset, workingset->nActiveConstr,
                             memspace->workspace_double);
        runTimeOptions.RemainFeasible = false;
        while ((solution->iterations < runTimeOptions.MaxIterations) &&
               ((solution->state == -7) ||
                ((solution->state == 1) &&
                 ((solution->maxConstr >
                   1.0E-5 * runTimeOptions.ConstrRelTolFactor) ||
                  (solution->firstorderopt >
                   1.0E-5 * runTimeOptions.ProbRelTolFactor))))) {
          feasibleX0ForWorkingSet(memspace->workspace_double, solution->xstar,
                                  workingset, qrmanager);
          b_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
          b_phaseone(H, f, solution, memspace, workingset, qrmanager,
                     cholmanager, objective, &options, &runTimeOptions);
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options.ObjectiveLimit, options.StepTolerance,
                  runTimeOptions.MaxIterations,
                  runTimeOptions.ConstrRelTolFactor,
                  runTimeOptions.ProbRelTolFactor, false);
          solution->maxConstr =
              maxConstraintViolation(workingset, solution->xstar);
          computeFirstOrderOpt(solution, objective, workingset->nVar,
                               workingset->ATwset, workingset->nActiveConstr,
                               memspace->workspace_double);
        }
      }
    }
  }
}

/* End of code generation (driver.c) */
