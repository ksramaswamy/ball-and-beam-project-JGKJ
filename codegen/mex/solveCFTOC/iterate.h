/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * iterate.h
 *
 * Code generation for function 'iterate'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "solveCFTOC_internal_types.h"
#include "solveCFTOC_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void iterate(const real_T H[2500], const real_T f[50], struct_T *solution,
             e_struct_T *memspace, g_struct_T *workingset,
             f_struct_T *qrmanager, c_struct_T *cholmanager,
             b_struct_T *objective, real_T options_ObjectiveLimit,
             real_T options_StepTolerance, int32_T runTimeOptions_MaxIterations,
             real_T c_runTimeOptions_ConstrRelTolFa,
             real_T runTimeOptions_ProbRelTolFactor,
             boolean_T runTimeOptions_RemainFeasible);

/* End of code generation (iterate.h) */
