/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveCFTOC_internal_types.h
 *
 * Code generation for function 'solveCFTOC'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "solveCFTOC_types.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  real_T xstar[51];
  real_T fstar;
  real_T firstorderopt;
  real_T lambda[141];
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  real_T searchDir[51];
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  real_T grad[51];
  real_T Hx[50];
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  int32_T MaxIterations;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
  boolean_T RemainFeasible;
} d_struct_T;
#endif /* typedef_d_struct_T */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T
typedef struct {
  real_T workspace_double[7191];
  int32_T workspace_int[141];
  int32_T workspace_sort[141];
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  real_T Aeq[2040];
  real_T beq[40];
  real_T lb[51];
  real_T ub[51];
  int32_T indexLB[51];
  int32_T indexUB[51];
  int32_T indexFixed[51];
  int32_T mEqRemoved;
  int32_T indexEqRemoved[40];
  real_T ATwset[7191];
  real_T bwset[141];
  int32_T nActiveConstr;
  real_T maxConstrWorkspace[141];
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  boolean_T isActiveConstr[141];
  int32_T Wid[141];
  int32_T Wlocalidx[141];
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T
typedef struct {
  real_T InitDamping;
  char_T FiniteDifferenceType[7];
  boolean_T SpecifyObjectiveGradient;
  boolean_T ScaleProblem;
  boolean_T SpecifyConstraintGradient;
  boolean_T NonFiniteSupport;
  boolean_T IterDisplaySQP;
  real_T FiniteDifferenceStepSize;
  real_T MaxFunctionEvaluations;
  boolean_T IterDisplayQP;
  real_T PricingTolerance;
  char_T Algorithm[10];
  real_T ObjectiveLimit;
  real_T ConstraintTolerance;
  real_T OptimalityTolerance;
  real_T StepTolerance;
  real_T MaxIterations;
  real_T FunctionTolerance;
  char_T SolverName[8];
  boolean_T CheckGradients;
  char_T Diagnostics[3];
  real_T DiffMaxChange;
  real_T DiffMinChange;
  char_T Display[3];
  char_T FunValCheck[3];
  boolean_T UseParallel;
  char_T LinearSolver[4];
  char_T SubproblemAlgorithm[2];
} h_struct_T;
#endif /* typedef_h_struct_T */

/* End of code generation (solveCFTOC_internal_types.h) */
