/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ratiotest.h
 *
 * Code generation for function 'ratiotest'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void ratiotest(const real_T solution_xstar[51],
               const real_T solution_searchDir[51], int32_T workingset_nVar,
               const real_T workingset_lb[51], const real_T workingset_ub[51],
               const int32_T workingset_indexLB[51],
               const int32_T workingset_indexUB[51],
               const int32_T workingset_sizes[5],
               const int32_T workingset_isActiveIdx[6],
               const boolean_T workingset_isActiveConstr[141],
               const int32_T workingset_nWConstr[5], real_T *toldelta,
               real_T *alpha, boolean_T *newBlocking, int32_T *constrType,
               int32_T *constrIdx);

/* End of code generation (ratiotest.h) */
